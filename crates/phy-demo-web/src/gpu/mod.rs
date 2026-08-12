//! W1: Web Demo GPU 加速后端(§5.7.4)。
//!
//! `#[cfg(feature = "gpu")]` 下编译:wasm32(浏览器 WebGPU)是正式路径;native(桌面 wgpu
//! 原生后端驱动真实 adapter)在 G1 真机验证中使用,需 MSVC 工具链(MinGW 链接 wgpu 崩溃 M3)。
//! 原生回读须在 `map_async` 后 `device.poll(PollType::Wait)` 触发回调,否则死锁/超时。
//!
//! 本文件打通 **WebGPU compute 全链路**:申请 device/queue → 上传 → wgsl kernel →
//! dispatch → 回读。W1 最小原型(square)、W2 光学逐像素、W3 焦散、W4 SPH 逐粒子、W5 颗粒
//! PBD 都在此上下文之上叠加。

use wgpu::util::DeviceExt;

/// GPU 加速策略(API 层默认 Auto)。
///
/// - `Auto`:有可用 adapter 就用 GPU,否则自动回退 CPU(默认)。
/// - `ForceGpu`:强制走 GPU;无 adapter 时 `plan()` 直接报错(便于 CI/调测暴露问题)。
/// - `ForceCpu`:强制走 CPU,任何情况下都不初始化 GPU(便于在共享 GPU 环境/无头机器上跳过)。
///
/// 用途:在 web/桌面 demo 入口、各 `render_*_gpu` 内部根据此枚举决定 GPU/CPU 路由,
/// 实现 §2.3 的"GpuStrategy 默认 Auto"。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GpuStrategy {
    Auto,
    ForceGpu,
    ForceCpu,
}

impl Default for GpuStrategy {
    fn default() -> Self {
        GpuStrategy::Auto
    }
}

impl GpuStrategy {
    /// 给定 GPU adapter 是否可用,返回是否应当走 GPU 路径。
    /// `Auto`:adapter 可用即 true;`ForceGpu`:要求 adapter 可用,否则 Err;`ForceCpu`:恒 false。
    pub fn plan(self, gpu_available: bool) -> Result<bool, String> {
        match self {
            GpuStrategy::Auto => Ok(gpu_available),
            GpuStrategy::ForceCpu => Ok(false),
            GpuStrategy::ForceGpu => {
                if gpu_available {
                    Ok(true)
                } else {
                    Err("GpuStrategy::ForceGpu 要求 GPU adapter 可用,但当前不可用".to_string())
                }
            }
        }
    }
}

/// 预编译并常驻的 GPU 管线集合。
///
/// 所有核(square/optic/caustic/sph/granular)的 shader 模块与 compute pipeline
/// 在 `GpuContext::init()` 时一次性编译,之后每个渲染/步进调用只重建"按输入尺寸变化"
/// 的 data buffer 与 bind group,复用这里的 pipeline —— 这是 §2.3 的"GPU 管线持久化"
/// (消除每帧重新编译 pipeline 的开销)。
pub struct GpuPipelines {
    pub square: wgpu::ComputePipeline,
    pub optic: wgpu::ComputePipeline,
    pub caustic: wgpu::ComputePipeline,
    // SPH 用自定义 bind group layout + pipeline layout(两个 entry point 共享),需一并常驻。
    pub sph_bgl: wgpu::BindGroupLayout,
    pub sph_pl: wgpu::PipelineLayout,
    pub sph_dens: wgpu::ComputePipeline,
    pub sph_force: wgpu::ComputePipeline,
    // granular 用自定义 bind group layout(8 绑定),需一并常驻。
    pub granular_bgl: wgpu::BindGroupLayout,
    pub granular_pl: wgpu::PipelineLayout,
    pub granular_dens: wgpu::ComputePipeline,
    pub granular_pred: wgpu::ComputePipeline,
    pub granular_solve: wgpu::ComputePipeline,
}

/// WebGPU 上下文:持有 device/queue + 常驻管线缓存,提供 compute 原型。
///
/// `init()` 只应调用一次(上层用 `Rc<RefCell<Option<GpuContext>>>` 缓存);其返回的
/// `pipelines` 永久有效,后续每帧渲染/步进直接复用,不再重编译。
pub struct GpuContext {
    pub adapter: wgpu::Adapter,
    device: wgpu::Device,
    queue: wgpu::Queue,
    /// 一次性预编译的常驻管线,所有核共享。
    pub pipelines: GpuPipelines,
}

impl GpuContext {
    /// 异步申请 adapter/device/queue,并预编译全部 compute 管线。
    /// 浏览器里必须走 async(await navigator.gpu.requestAdapter)。
    ///
    /// 注意:本函数有 pipeline 编译开销,仅应在启动时调用一次;返回的 `pipelines` 常驻,
    /// 之后每帧渲染/步进复用,不再重编译(§2.3 GPU 管线持久化)。
    pub async fn init() -> Result<GpuContext, String> {
        Self::init_with(GpuStrategy::Auto).await
    }

    /// 带策略的初始化:根据 `strategy` 决定是否走 GPU 路径(§2.3 GpuStrategy 默认 Auto)。
    ///
    /// - `Auto`:有 adapter 就用 GPU,否则 `request_adapter` 失败(上层回退 CPU)。
    /// - `ForceCpu`:直接返回专门错误,调用方应据此走 CPU,不初始化 GPU。
    /// - `ForceGpu`:要求 adapter 必须可用,否则返回明确错误(便于 CI/调测暴露问题)。
    pub async fn init_with(strategy: GpuStrategy) -> Result<GpuContext, String> {
        // ForceCpu:不碰 adapter,直接报错让上层回退。
        if let GpuStrategy::ForceCpu = strategy {
            return Err("GpuStrategy::ForceCpu:已强制 CPU,跳过 GPU 初始化".to_string());
        }
        let instance = wgpu::Instance::new(wgpu::InstanceDescriptor::new_without_display_handle());
        let adapter = instance
            .request_adapter(&wgpu::RequestAdapterOptions {
                power_preference: wgpu::PowerPreference::HighPerformance,
                compatible_surface: None,
                force_fallback_adapter: false,
                ..Default::default()
            })
            .await
            .map_err(|e| format!("request_adapter 失败: {:?}", e))?;
        // ForceGpu 要求 adapter 可用,这里 adapter 已拿到,plan 必为 Ok(true);
        // 若将来 adapter 拿到但策略判定不可用(极端),给出明确错误。
        if let Err(msg) = strategy.plan(true) {
            return Err(msg);
        }
        // 关键:M1 验收在真实浏览器发现 wgpu 0.20 的 `Limits::default()` /
        // `downlevel_defaults()` 会把 `max_inter_stage_shader_components` 等字段设成
        // 非 None 值,而浏览器端 WebGPU 规范已移除/重命名该 limit,`requestDevice`
        // 直接报错 "limit ... is not recognized",导致整条 GPU 路径在真机上崩。
        // wgpu 30 已彻底移除该废弃字段,故直接用 adapter 的 limits 基线即可。
        let mut limits = adapter.limits().clone();
        // 我们的 compute 内核用到 storage buffer / 大 binding,确保下限即可。
        limits.max_storage_buffers_per_shader_stage =
            limits.max_storage_buffers_per_shader_stage.max(8);
        limits.max_buffer_size = limits.max_buffer_size.max(1 << 24);
        limits.max_storage_buffer_binding_size =
            limits.max_storage_buffer_binding_size.max(1 << 24);
        let (device, queue) = adapter
            .request_device(
                &wgpu::DeviceDescriptor {
                    required_features: wgpu::Features::empty(),
                    required_limits: limits,
                    label: Some("phy-gpu"),
                    memory_hints: wgpu::MemoryHints::MemoryUsage,
                    ..Default::default()
                },
            )
            .await
            .map_err(|e| format!("request_device 失败: {:?}", e))?;

        let mk_pipeline = |device: &wgpu::Device, label: &str, src: &'static str| {
            let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
                label: Some(label),
                source: wgpu::ShaderSource::Wgsl(std::borrow::Cow::Borrowed(src)),
            });
            device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
                label: Some(&format!("{}_pipe", label)),
                layout: None,
                module: &shader,
                entry_point: Some("main"),
                compilation_options: wgpu::PipelineCompilationOptions::default(),
                cache: None,
            })
        };

        let (sph_bgl, sph_pl, sph_dens, sph_force) = build_sph_pipelines(&device);
        let (granular_bgl, granular_pl, granular_dens, granular_pred, granular_solve) =
            build_granular_pipelines(&device);

        let pipelines = GpuPipelines {
            square: mk_pipeline(&device, "square", SQUARE_WGSL),
            optic: mk_pipeline(&device, "optic", OPTIC_WGSL),
            caustic: mk_pipeline(&device, "caustic", CAUSTIC_WGSL),
            sph_bgl,
            sph_pl,
            sph_dens,
            sph_force,
            granular_bgl,
            granular_pl,
            granular_dens,
            granular_pred,
            granular_solve,
        };
        Ok(GpuContext {
            adapter,
            device,
            queue,
            pipelines,
        })
    }

    /// 最小 compute 原型:对 `data` 中每个 f32 求平方,返回新 buffer。
    /// 验证链路:staging 上传 → bind group → pipeline → dispatch(workgroups) → 回读 map。
    pub async fn square_self_test(&self, data: &[f32]) -> Result<Vec<f32>, String> {
        let n = data.len();
        let bytes = bytemuck::cast_slice::<f32, u8>(data);

        // 输入 buffer(只读 storage)+ 输出 buffer(读写 storage)。
        let in_buf = self
            .device
            .create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("in"),
                contents: bytes,
                usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
            });
        let out_buf = self.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("out"),
            size: (n * std::mem::size_of::<f32>()) as u64,
            usage: wgpu::BufferUsages::STORAGE
                | wgpu::BufferUsages::COPY_SRC
                | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        // 回读 staging buffer(CPU 可见)。
        let read_buf = self.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("read"),
            size: (n * std::mem::size_of::<f32>()) as u64,
            usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        let pipeline = &self.pipelines.square;
        let bind_group = self.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("square_bg"),
            layout: &pipeline.get_bind_group_layout(0),
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: in_buf.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: out_buf.as_entire_binding(),
                },
            ],
        });

        let mut encoder =
            self.device
                .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                    label: Some("square_enc"),
                });
        {
            let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("square_pass"),
                ..Default::default()
            });
            pass.set_pipeline(pipeline);
            pass.set_bind_group(0, &bind_group, &[]);
            // one thread per element, 64 workgroup size。
            pass.dispatch_workgroups(((n + 63) / 64) as u32, 1, 1);
        }
        encoder.copy_buffer_to_buffer(&out_buf, 0, &read_buf, 0, read_buf.size());
        self.queue.submit(std::iter::once(encoder.finish()));

        // 回读:map_async 在浏览器为 async。
        let buf_slice = read_buf.slice(..);
        let (tx, rx) = futures::channel::oneshot::channel();
        buf_slice.map_async(wgpu::MapMode::Read, move |r| {
            let _ = tx.send(r);
        });
        // 原生(桌面)wgpu 需 poll 才触发 map_async 回调。
        let _ = self.device.poll(wgpu::PollType::Wait { submission_index: None, timeout: None });
        rx.await
            .map_err(|e| format!("map 通道失败: {:?}", e))?
            .map_err(|e| format!("map_async 失败: {:?}", e))?;
        let view = buf_slice.get_mapped_range()
            .map_err(|e| format!("get_mapped_range 失败: {:?}", e))?;
        let out: Vec<f32> = bytemuck::cast_slice(&view).to_vec();
        drop(view);
        read_buf.unmap();
        Ok(out)
    }
}

/// SPH 核的常驻管线构造:8 绑定的 bind group layout + 共享 pipeline layout +
/// 两个 entry point(density_main / force_main)。在 `GpuContext::init()` 中调用一次,
/// 之后每个 `render_sph_gpu` 调用复用,消除每帧重编译(§2.3 GPU 管线持久化)。
fn build_sph_pipelines(
    device: &wgpu::Device,
) -> (
    wgpu::BindGroupLayout,
    wgpu::PipelineLayout,
    wgpu::ComputePipeline,
    wgpu::ComputePipeline,
) {
    let sph_bgl = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some("sph_bgl"),
        entries: &[
            wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::COMPUTE,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            },
            wgpu::BindGroupLayoutEntry {
                binding: 1,
                visibility: wgpu::ShaderStages::COMPUTE,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Storage { read_only: true },
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            },
            wgpu::BindGroupLayoutEntry {
                binding: 2,
                visibility: wgpu::ShaderStages::COMPUTE,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Storage { read_only: true },
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            },
            wgpu::BindGroupLayoutEntry {
                binding: 3,
                visibility: wgpu::ShaderStages::COMPUTE,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Storage { read_only: true },
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            },
            wgpu::BindGroupLayoutEntry {
                binding: 4,
                visibility: wgpu::ShaderStages::COMPUTE,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Storage { read_only: true },
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            },
            wgpu::BindGroupLayoutEntry {
                binding: 5,
                visibility: wgpu::ShaderStages::COMPUTE,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Storage { read_only: true },
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            },
            wgpu::BindGroupLayoutEntry {
                binding: 6,
                visibility: wgpu::ShaderStages::COMPUTE,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Storage { read_only: false },
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            },
            wgpu::BindGroupLayoutEntry {
                binding: 7,
                visibility: wgpu::ShaderStages::COMPUTE,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Storage { read_only: false },
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            },
        ],
    });
    let sph_pl = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
        label: Some("sph_pl"),
        bind_group_layouts: &[Some(&sph_bgl)],
        immediate_size: 0,
    });
    let module = device.create_shader_module(wgpu::ShaderModuleDescriptor {
        label: Some("sph"),
        source: wgpu::ShaderSource::Wgsl(std::borrow::Cow::Borrowed(SPH_WGSL)),
    });
    let mk = |entry: &str| -> wgpu::ComputePipeline {
        device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
            label: Some(entry),
            layout: Some(&sph_pl),
            module: &module,
            entry_point: Some(entry),
            compilation_options: wgpu::PipelineCompilationOptions::default(),
            cache: None,
        })
    };
    let sph_dens = mk("density_main");
    let sph_force = mk("force_main");
    (sph_bgl, sph_pl, sph_dens, sph_force)
}

/// granular 核的常驻管线构造:5 绑定的 bind group layout + 共享 pipeline layout +
/// 三个 entry point(clear_main / contact_main / apply_main)。在 `GpuContext::init()` 中
/// 调用一次,之后每个 `render_granular_gpu` 调用复用(§2.3 GPU 管线持久化)。
fn build_granular_pipelines(
    device: &wgpu::Device,
) -> (
    wgpu::BindGroupLayout,
    wgpu::PipelineLayout,
    wgpu::ComputePipeline,
    wgpu::ComputePipeline,
    wgpu::ComputePipeline,
) {
    // 显式 bind group layout:auto-layout 只会包含各 entry point 实际用到的
    // binding(clear_main 只用 deltas=>仅 binding4),导致共用 bind group 失败。
    let bgl = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some("gran_bgl"),
        entries: &[
            wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::COMPUTE,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            },
            wgpu::BindGroupLayoutEntry {
                binding: 1,
                visibility: wgpu::ShaderStages::COMPUTE,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Storage { read_only: false },
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            },
            wgpu::BindGroupLayoutEntry {
                binding: 2,
                visibility: wgpu::ShaderStages::COMPUTE,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Storage { read_only: true },
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            },
            wgpu::BindGroupLayoutEntry {
                binding: 3,
                visibility: wgpu::ShaderStages::COMPUTE,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Storage { read_only: true },
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            },
            wgpu::BindGroupLayoutEntry {
                binding: 4,
                visibility: wgpu::ShaderStages::COMPUTE,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Storage { read_only: false },
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            },
        ],
    });
    let pl = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
        label: Some("gran_pl"),
        bind_group_layouts: &[Some(&bgl)],
        immediate_size: 0,
    });
    let module = device.create_shader_module(wgpu::ShaderModuleDescriptor {
        label: Some("gran_wgsl"),
        source: wgpu::ShaderSource::Wgsl(GRANULAR_WGSL.into()),
    });
    let mk = |entry: &str| -> wgpu::ComputePipeline {
        device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
            label: Some(entry),
            layout: Some(&pl),
            module: &module,
            entry_point: Some(entry),
            compilation_options: wgpu::PipelineCompilationOptions::default(),
            cache: None,
        })
    };
    let granular_dens = mk("clear_main");
    let granular_pred = mk("contact_main");
    let granular_solve = mk("apply_main");
    (bgl, pl, granular_dens, granular_pred, granular_solve)
}

/// W1 原型 kernel:逐元素平方(one thread per element)。
/// 后续 W2 光学 / W3 焦散 / W4 SPH / W5 PBD 都复用这一"逐元素 map"骨架,
/// 只替换 @compute 函数体,保证数值内核与 CPU 端(wgsl 一一对应)可对照。
const SQUARE_WGSL: &str = r#"
@group(0) @binding(0) var<storage, read> inp: array<f32>;
@group(0) @binding(1) var<storage, read_write> outp: array<f32>;

@compute @workgroup_size(64)
fn main(@builtin(global_invocation_id) gid: vec3<u32>) {
    let i = gid.x;
    if (i >= arrayLength(&inp)) { return; }
    outp[i] = inp[i] * inp[i];
}
"#;

/// M1 验收:真实 WebGPU adapter 申请 + 返回适配器身份字符串。
///
/// 直接走 `request_adapter`(不 fallback),成功则调用 `adapter.info()` 取回
/// 真实 vendor / architecture / device / description,序列化成一个可读 JSON。
/// 这是"真实 WebGPU adapter 验收"的核心证据:无 adapter 时返回 Err(不会假装通过)。
/// 浏览器侧 `app.adapter_info().then(s => console.log(s))` 应看到:
/// `{"ok":true,"vendor":"...","architecture":"...","device":"...","description":"..."}`
pub async fn adapter_info() -> Result<String, String> {
    let instance = wgpu::Instance::new(wgpu::InstanceDescriptor::new_without_display_handle());
    let adapter = instance
        .request_adapter(&wgpu::RequestAdapterOptions {
            power_preference: wgpu::PowerPreference::HighPerformance,
            compatible_surface: None,
            force_fallback_adapter: false,
            ..Default::default()
        })
        .await
        .map_err(|e| {
            format!("request_adapter 失败(无可用 WebGPU adapter, M1 验收失败): {:?}", e)
        })?;
    // wgpu: adapter.get_info() 返回 AdapterInfo { name, vendor, device,
    // device_type, driver, driver_info, backend }(vendor/device 为 u32,其余为 String/枚举)。
    let info = adapter.get_info();
    Ok(format!(
        "{{\"ok\":true,\"name\":\"{}\",\"vendor\":\"{}\",\"device\":\"{}\",\"device_type\":\"{:?}\",\"driver\":\"{}\",\"driver_info\":\"{}\",\"backend\":\"{:?}\"}}",
        info.name, info.vendor, info.device, info.device_type, info.driver, info.driver_info, info.backend
    ))
}

/// 高层自测:构造 [1,2,3,4,5],跑 square_self_test,返回结果字符串。
/// 验证链路通:device 申请成功 + kernel 正确执行 + 回读准确。
/// 浏览器侧可在 console 看到 "W1 gpu self-test: [1,4,9,16,25]"。
pub async fn gpu_self_test() -> Result<String, String> {
    let ctx = GpuContext::init().await?;
    let src = [1.0f32, 2.0, 3.0, 4.0, 5.0];
    let out = ctx.square_self_test(&src).await?;
    let ok = out
        .iter()
        .zip(src.iter())
        .all(|(o, s)| (o - s * s).abs() < 1e-5);
    Ok(format!(
        "W1 gpu self-test ok={} out=[{}]",
        ok,
        out.iter()
            .map(|v| format!("{:.1}", v))
            .collect::<Vec<_>>()
            .join(",")
    ))
}

// ===========================================================================
// W2: 光学实时近似后端(Approx)走 GPU。
//
// 逐像素 one-thread,复刻 CPU 端 `Approx::trace` + `OpticScene::intersect`
// (sphere / box 两种形状;convex 不在 GPU 实现,调用方需回退 CPU)。
// 场景降为 f32 扁平缓冲(每个 body 5×vec4 = 80B),相机参数走 uniform。
// 数值内核与 wgsl 一一对应,便于与 CPU 渲染结果对照。
// ===========================================================================

use phy_math::Vec3 as V3;
use phy_optics::OpticScene;
use phy_rigid::{Body, Shape};

/// GPU 光学体扁平布局(80B = 5×vec4,std430 对齐)。
/// 与 wgsl `BodyGpu` 完全一致。
#[repr(C, align(16))]
#[derive(Copy, Clone, bytemuck::Pod, bytemuck::Zeroable)]
struct BodyGpu {
    /// pos.xyz, kind(0=sphere,1=box)
    pos_kind: [f32; 4],
    /// quat (w,x,y,z) —— 把世界向量旋到局部用 q*v*q^-1。
    quat: [f32; 4],
    /// sphere: r; box: half.x/half.y/half.z
    geo: [f32; 4],
    /// albedo.rgb, ior
    albedo_ior: [f32; 4],
    /// transparent(0/1), roughness, _, _
    misc: [f32; 4],
}

/// 把 `OpticScene<f32>` 扁平化为 GPU body buffer;返回 None 表示含 GPU 不支持的形状(convex)。
fn flatten_scene(scene: &OpticScene<f32>) -> Option<Vec<BodyGpu>> {
    let mut out = Vec::with_capacity(scene.bodies.len());
    for ob in &scene.bodies {
        let p = ob.body.pos;
        let q = *ob.body.rot.quaternion(); // (w,x,y,z)
        let (geo, kind) = match &ob.body.shape {
            Shape::Sphere { r } => ([*r, 0.0, 0.0, 0.0], 0u32),
            Shape::Box { half } => ([half.x, half.y, half.z, 0.0], 1u32),
            // GPU 暂不支持凸体 / 胶囊 / 高度场 / 复合体:直接回退 CPU 渲染。
            Shape::Convex { .. }
            | Shape::Capsule { .. }
            | Shape::Heightfield { .. }
            | Shape::Compound { .. } => return None,
        };
        out.push(BodyGpu {
            pos_kind: [p.x, p.y, p.z, kind as f32],
            quat: [q.w, q.i, q.j, q.k],
            geo,
            albedo_ior: [ob.surface.albedo.x, ob.surface.albedo.y, ob.surface.albedo.z, ob.surface.ior],
            misc: [
                if ob.surface.transparent { 1.0 } else { 0.0 },
                ob.surface.roughness,
                0.0,
                0.0,
            ],
        });
    }
    Some(out)
}

/// W2 主入口:用 GPU 渲染一帧到 `buf`(宽×高 RGB f32)。
/// 返回 Err 表示本场景 GPU 不支持(调用方应回退 CPU `render_camera`)。
pub async fn render_camera_gpu(
    ctx: &GpuContext,
    scene: &OpticScene<f32>,
    buf: &mut [V3<f32>],
    width: usize,
    height: usize,
    eye: &V3<f32>,
    target: &V3<f32>,
    up: &V3<f32>,
    fov: f32,
) -> Result<(), String> {
    let bodies = flatten_scene(scene).ok_or_else(|| {
        "render_camera_gpu: 场景含 convex 形状,GPU 不支持,回退 CPU".to_string()
    })?;
    let aspect = (width as f32) / (height as f32);
    let tan_h = (fov * 0.5).tan();
    let tan_w = tan_h * aspect;
    let cam: [[f32; 4]; 5] = [
        [eye.x, eye.y, eye.z, tan_w],
        [target.x, target.y, target.z, tan_h],
        [up.x, up.y, up.z, aspect],
        [width as f32, height as f32, scene.env_ior, 0.0],
        [scene.background.x, scene.background.y, scene.background.z, 0.0],
    ];

    // body storage buffer。
    let body_bytes = bytemuck::cast_slice::<BodyGpu, u8>(&bodies);
    let body_buf = ctx
        .device
        .create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("bodies"),
            contents: body_bytes,
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
        });
    // camera uniform。
    let cam_bytes = bytemuck::cast_slice::<[f32; 4], u8>(&cam);
    let cam_buf = ctx
        .device
        .create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("cam"),
            contents: cam_bytes,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        });
    // out color buffer(rgba f32)。
    let px = width * height;
    let out_buf = ctx.device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("out"),
        size: (px * 4 * std::mem::size_of::<f32>()) as u64,
        usage: wgpu::BufferUsages::STORAGE
            | wgpu::BufferUsages::COPY_SRC
            | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    });
    let read_buf = ctx.device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("read"),
        size: out_buf.size(),
        usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    });

    let pipeline = &ctx.pipelines.optic;
    let bind_group = ctx.device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("optic_bg"),
        layout: &pipeline.get_bind_group_layout(0),
        entries: &[
            wgpu::BindGroupEntry {
                binding: 0,
                resource: cam_buf.as_entire_binding(),
            },
            wgpu::BindGroupEntry {
                binding: 1,
                resource: body_buf.as_entire_binding(),
            },
            wgpu::BindGroupEntry {
                binding: 2,
                resource: out_buf.as_entire_binding(),
            },
        ],
    });

    let mut encoder =
        ctx.device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("optic_enc"),
            });
    {
        let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
            label: Some("optic_pass"),
            ..Default::default()
        });
        pass.set_pipeline(pipeline);
        pass.set_bind_group(0, &bind_group, &[]);
        pass.dispatch_workgroups(((px + 63) / 64) as u32, 1, 1);
    }
    encoder.copy_buffer_to_buffer(&out_buf, 0, &read_buf, 0, read_buf.size());
    ctx.queue.submit(std::iter::once(encoder.finish()));

    let slice = read_buf.slice(..);
    let (tx, rx) = futures::channel::oneshot::channel();
    slice.map_async(wgpu::MapMode::Read, move |r| {
        let _ = tx.send(r);
    });
    // 原生(桌面)wgpu 需 poll 才触发 map_async 回调。
    let _ = ctx.device.poll(wgpu::PollType::Wait { submission_index: None, timeout: None });
    rx.await
        .map_err(|e| format!("map 通道失败: {:?}", e))?
        .map_err(|e| format!("map_async 失败: {:?}", e))?;
    let view = slice.get_mapped_range()
        .map_err(|e| format!("get_mapped_range 失败: {:?}", e))?;
    let rgba: &[f32] = bytemuck::cast_slice(&view);
    for i in 0..px {
        buf[i] = V3::new(rgba[i * 4], rgba[i * 4 + 1], rgba[i * 4 + 2]);
    }
    drop(view);
    read_buf.unmap();
    Ok(())
}

/// W2 光学 wgsl:复刻 `Approx::trace` + `OpticScene::intersect`(sphere/box)。
/// 逐像素 one-thread(像素间独立,与 CPU 串行逐像素结果一致)。
const OPTIC_WGSL: &str = r#"
struct BodyGpu {
    pos_kind : vec4<f32>,
    quat     : vec4<f32>,
    geo      : vec4<f32>,
    albedo   : vec4<f32>,
    misc     : vec4<f32>,
};
struct CamGpu {
    eye_tanw   : vec4<f32>,
    target_tanh: vec4<f32>,
    up_aspect  : vec4<f32>,
    wh_env     : vec4<f32>,
    bg         : vec4<f32>,
};

@group(0) @binding(0) var<uniform> cam : CamGpu;
@group(0) @binding(1) var<storage, read> bodies : array<BodyGpu>;
@group(0) @binding(2) var<storage, read_write> outp : array<vec4<f32>>;

const EPS = 1e-4;

fn quat_rot(q: vec4<f32>, v: vec3<f32>) -> vec3<f32> {
    // q = (w, x, y, z); return q * v * q^-1
    let t = 2.0 * cross(q.yzw, v);
    return v + q.x * t + cross(q.yzw, t);
}
fn quat_rot_inv(q: vec4<f32>, v: vec3<f32>) -> vec3<f32> {
    let t = 2.0 * cross(q.yzw, v);
    return v - q.x * t + cross(q.yzw, t);
}
fn norm(v: vec3<f32>) -> vec3<f32> {
    let n = length(v);
    if (n > 1e-12) { return v / n; }
    return vec3<f32>(1.0, 0.0, 0.0);
}
fn reflect(i: vec3<f32>, n: vec3<f32>) -> vec3<f32> {
    return i - n * (2.0 * dot(i, n));
}
fn refract(i: vec3<f32>, n: vec3<f32>, eta: f32) -> vec3<f32> {
    let cosi = -dot(i, n);
    let k = 1.0 - eta * eta * (1.0 - cosi * cosi);
    if (k < 0.0) { return vec3<f32>(0.0); }
    return norm(i * eta + n * (eta * cosi - sqrt(k)));
}
fn fresnel(cosi: f32, f0: f32) -> f32 {
    let t = 1.0 - cosi;
    let t2 = t * t;
    return f0 + (1.0 - f0) * t2 * t2 * t;
}

// 命中结构: (t, 世界法线, 命中体索引)。idx<0 表示无交。
struct Hit { t: f32, n: vec3<f32>, idx: f32 };
fn intersect2(ro: vec3<f32>, rd: vec3<f32>) -> Hit {
    var best_t = 1e30;
    var best_n = vec3<f32>(0.0);
    var best_i = -1.0;
    for (var idx = 0u; idx < arrayLength(&bodies); idx = idx + 1u) {
        let b = bodies[idx];
        let kind = b.pos_kind.w;
        let q = b.quat;
        let ro_l = quat_rot_inv(q, ro - b.pos_kind.xyz);
        let rd_l = quat_rot_inv(q, rd);
        let rdl = length(rd_l);
        if (rdl <= 0.0) { continue; }
        let rd_n = rd_l / rdl;
        var t_local = -1.0;
        var n_local = vec3<f32>(0.0);
        if (kind < 0.5) {
            let r = b.geo.x;
            let bb = dot(ro_l, rd_n);
            let cc = dot(ro_l, ro_l) - r * r;
            let disc = bb * bb - cc;
            if (disc < 0.0) { continue; }
            let sq = sqrt(disc);
            var t0 = -bb - sq;
            let t = select(-bb + sq, t0, t0 > 0.0);
            if (t <= 0.0) { continue; }
            let pl = ro_l + rd_n * t;
            t_local = t * rdl;
            n_local = norm(pl / r);
        } else {
            let half = b.geo.xyz;
            var tmin = 0.0;
            var tmax = 1e30;
            var nrm = vec3<f32>(0.0);
            var ok = true;
            for (var ax = 0u; ax < 3u; ax = ax + 1u) {
                let o = ro_l[ax];
                let d = rd_n[ax];
                let h = half[ax];
                var inv_d = 0.0;
                var sign = 1.0;
                if (abs(d) > 1e-12) { inv_d = 1.0 / d; } else { if (o < -h || o > h) { ok = false; break; } inv_d = 0.0; }
                var t1 = (-h - o) * inv_d;
                var t2 = (h - o) * inv_d;
                var nax = vec3<f32>(0.0);
                nax[ax] = -1.0;
                if (t1 > t2) { let tmp = t1; t1 = t2; t2 = tmp; nax[ax] = 1.0; sign = -1.0; }
                if (t1 > tmin) { tmin = t1; nrm = nax * sign; }
                if (t2 < tmax) { tmax = t2; }
                if (tmin > tmax) { ok = false; break; }
            }
            if (!ok || tmin <= 0.0) { continue; }
            t_local = tmin * rdl;
            n_local = norm(nrm);
        }
        let n_world = quat_rot(q, n_local);
        if (t_local < best_t) { best_t = t_local; best_n = n_world; best_i = f32(idx); }
    }
    return Hit(best_t, best_n, best_i);
}

fn in_shadow(p: vec3<f32>, light_dir: vec3<f32>) -> bool {
    let ro = p + light_dir * EPS;
    let h = intersect2(ro, light_dir);
    if (h.idx < 0.0) { return false; }
    return bodies[u32(h.idx)].misc.x < 0.5; // transparent==false => shadow
}

fn approx_trace(ro: vec3<f32>, rd: vec3<f32>) -> vec3<f32> {
    let hit = intersect2(ro, rd);
    if (hit.idx < 0.0) { return cam.bg.xyz; }
    let body = bodies[u32(hit.idx)];
    let surf_albedo = body.albedo.xyz;
    let ior = body.albedo.w;
    let transparent = body.misc.x > 0.5;
    let p = ro + rd * hit.t;
    let cosi = -dot(rd, hit.n);
    let abs_cosi = abs(cosi);
    if (transparent) {
        let eta = cam.wh_env.z / ior;
        let f0 = (cam.wh_env.z - ior) / (cam.wh_env.z + ior);
        let f0v = f0 * f0;
        let fr = fresnel(abs_cosi, f0v);
        let rfl = reflect(rd, hit.n);
        let ro_r = p + rfl * EPS;
        var refl_col = cam.bg.xyz;
        let h2 = intersect2(ro_r, rfl);
        if (h2.idx >= 0.0) {
            let s2 = bodies[u32(h2.idx)];
            let lambert = max(dot(h2.n, vec3<f32>(0.0,1.0,0.0)), 0.0) * 0.5;
            refl_col = s2.albedo.xyz * (0.5 + lambert);
        }
        var trans_col = cam.bg.xyz * surf_albedo;
        let tdir = refract(rd, hit.n, eta);
        if (length(tdir) > 0.0) {
            let ro_t = p + tdir * EPS;
            let h3 = intersect2(ro_t, tdir);
            if (h3.idx >= 0.0) {
                trans_col = bodies[u32(h3.idx)].albedo.xyz * surf_albedo;
            }
        } else {
            trans_col = refl_col;
        }
        let ft = 1.0 - fr;
        return clamp(refl_col * fr + trans_col * ft, vec3<f32>(0.0), vec3<f32>(1.0));
    } else {
        let light_dir = norm(vec3<f32>(0.5, 1.0, 0.3));
        let diff = max(dot(hit.n, light_dir), 0.0);
        var shade = 0.2;
        if (!in_shadow(p, light_dir)) { shade = 0.2 + diff * 0.8; }
        return clamp(surf_albedo * shade, vec3<f32>(0.0), vec3<f32>(1.0));
    }
}

@compute @workgroup_size(64)
fn main(@builtin(global_invocation_id) gid: vec3<u32>) {
    let idx = gid.x;
    let total = u32(cam.wh_env.x * cam.wh_env.y);
    if (idx >= total) { return; }
    let width = u32(cam.wh_env.x);
    let height = u32(cam.wh_env.y);
    let x = idx % width;
    let y = idx / width;
    let eye = cam.eye_tanw.xyz;
    let tgt = cam.target_tanh.xyz;
    let up = cam.up_aspect.xyz;
    let tan_w = cam.eye_tanw.w;
    let tan_h = cam.target_tanh.w;
    var fwd = tgt - eye;
    let fl = length(fwd);
    if (fl > 1e-12) { fwd = fwd / fl; } else { fwd = vec3<f32>(0.0,0.0,-1.0); }
    var right = cross(fwd, up);
    if (length(right) > 1e-12) { right = right / length(right); } else { right = vec3<f32>(1.0,0.0,0.0); }
    let true_up = cross(right, fwd);
    let u = (2.0 * f32(x) / (cam.wh_env.x - 1.0) - 1.0) * tan_w;
    let v = (1.0 - 2.0 * f32(y) / (cam.wh_env.y - 1.0)) * tan_h;
    var dir = fwd + right * u + true_up * v;
    dir = norm(dir);
    let col = approx_trace(eye, dir);
    outp[idx] = vec4<f32>(col, 1.0);
}
"#;

/// W2 自测:构造最小场景(1 个玻璃球 + 背景),GPU 渲染 8×8,
/// 返回首像素颜色字符串,验证 wgsl 求交/折射链路正确。
pub async fn optic_self_test() -> Result<String, String> {
    use phy_optics::{OpticBody, OpticScene, Surface};
    use phy_rigid::{Body, Shape};

    let mut scene = OpticScene::<f32>::new();
    scene.background = V3::new(0.05f32, 0.07, 0.1);
    scene.env_ior = 1.0;
    // 玻璃球 r=1 在原点前(静态 inv_mass=0)。
    let body = Body::new(Shape::Sphere { r: 1.0 }, V3::new(0.0f32, 0.0, -3.0), 0.0);
    scene.add(OpticBody::new(
        body,
        Surface::glass(1.5, V3::new(1.0, 1.0, 1.0)),
    ));

    let ctx = GpuContext::init().await?;
    let w = 8u32;
    let h = 8u32;
    let mut buf = vec![V3::new(0.0f32, 0.0, 0.0); (w * h) as usize];
    render_camera_gpu(
        &ctx,
        &scene,
        &mut buf,
        w as usize,
        h as usize,
        &V3::new(0.0f32, 0.0, 0.0),   // eye
        &V3::new(0.0f32, 0.0, -1.0),  // target
        &V3::new(0.0f32, 1.0, 0.0),   // up
        1.0f32,                       // fov
    )
    .await?;
    // 报告中心像素(应受玻璃球折射影响,非纯背景)。
    let cx = (w / 2) as usize;
    let cy = (h / 2) as usize;
    let c = buf[cy * w as usize + cx];
    Ok(format!(
        "W2 optic ok: center=({:.3},{:.3},{:.3}) bg=({:.3},{:.3},{:.3})",
        c.x, c.y, c.z, scene.background.x, scene.background.y, scene.background.z
    ))
}

// ===========================================================================
// W3: 焦散(caustics)逐射线 march 走 GPU。
//
// 复用 W2 的 `BodyGpu` 扁平缓冲与 sphere/box 求交,逐射线 one-thread 独立 march
// (每条射线只写自己的 grid 单元,无 atomic 竞争),复刻 CPU 端 `Caustics::march`。
// ===========================================================================

/// W3 主入口:GPU 计算接收面焦散强度网格,返回扁平 `grid_n*grid_n` 强度。
pub async fn render_caustics_gpu(
    ctx: &GpuContext,
    scene: &OpticScene<f32>,
    light_dir: &V3<f32>,
    plane_y: f32,
    half_extent: f32,
    grid_n: usize,
) -> Result<Vec<f32>, String> {
    let bodies = flatten_scene(scene).ok_or_else(|| {
        "render_caustics_gpu: 场景含 convex 形状,GPU 不支持,回退 CPU".to_string()
    })?;
    let n = grid_n * grid_n;
    let ldir = *light_dir;
    let ll = (ldir.x * ldir.x + ldir.y * ldir.y + ldir.z * ldir.z).max(1e-12).sqrt();
    let travel = V3::new(-ldir.x / ll, -ldir.y / ll, -ldir.z / ll);
    let params: [[f32; 4]; 4] = [
        [travel.x, travel.y, travel.z, plane_y],
        [half_extent, grid_n as f32, scene.env_ior, 0.0],
        [0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0],
    ];

    let body_bytes = bytemuck::cast_slice::<BodyGpu, u8>(&bodies);
    let body_buf = ctx
        .device
        .create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("bodies"),
            contents: body_bytes,
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
        });
    let param_bytes = bytemuck::cast_slice::<[f32; 4], u8>(&params);
    let param_buf = ctx
        .device
        .create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("params"),
            contents: param_bytes,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        });
    let out_buf = ctx.device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("out"),
        size: (n * std::mem::size_of::<f32>()) as u64,
        usage: wgpu::BufferUsages::STORAGE
            | wgpu::BufferUsages::COPY_SRC
            | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    });
    let read_buf = ctx.device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("read"),
        size: out_buf.size(),
        usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    });

    let pipeline = &ctx.pipelines.caustic;
    let bind_group = ctx.device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("caustic_bg"),
        layout: &pipeline.get_bind_group_layout(0),
        entries: &[
            wgpu::BindGroupEntry {
                binding: 0,
                resource: param_buf.as_entire_binding(),
            },
            wgpu::BindGroupEntry {
                binding: 1,
                resource: body_buf.as_entire_binding(),
            },
            wgpu::BindGroupEntry {
                binding: 2,
                resource: out_buf.as_entire_binding(),
            },
        ],
    });

    let mut encoder =
        ctx.device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("caustic_enc"),
            });
    {
        let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
            label: Some("caustic_pass"),
            ..Default::default()
        });
        pass.set_pipeline(pipeline);
        pass.set_bind_group(0, &bind_group, &[]);
        pass.dispatch_workgroups(((n + 63) / 64) as u32, 1, 1);
    }
    encoder.copy_buffer_to_buffer(&out_buf, 0, &read_buf, 0, read_buf.size());
    ctx.queue.submit(std::iter::once(encoder.finish()));

    let slice = read_buf.slice(..);
    let (tx, rx) = futures::channel::oneshot::channel();
    slice.map_async(wgpu::MapMode::Read, move |r| {
        let _ = tx.send(r);
    });
    // 原生(桌面)wgpu 需 poll 才触发 map_async 回调。
    let _ = ctx.device.poll(wgpu::PollType::Wait { submission_index: None, timeout: None });
    rx.await
        .map_err(|e| format!("map 通道失败: {:?}", e))?
        .map_err(|e| format!("map_async 失败: {:?}", e))?;
    let view = slice.get_mapped_range()
        .map_err(|e| format!("get_mapped_range 失败: {:?}", e))?;
    let out: Vec<f32> = bytemuck::cast_slice(&view).to_vec();
    drop(view);
    read_buf.unmap();
    Ok(out)
}

/// W3 自测:1 个玻璃球在接收面上方,平行光射入,渲染 16×16 焦散网格,
/// 返回 max 强度(应 >0,且中心聚焦)。
pub async fn caustics_self_test() -> Result<String, String> {
    use phy_optics::{OpticBody, OpticScene, Surface};
    use phy_rigid::Shape;

    let mut scene = OpticScene::<f32>::new();
    scene.env_ior = 1.0;
    let body = Body::new(Shape::Sphere { r: 1.0 }, V3::new(0.0f32, 0.0, -3.0), 0.0);
    scene.add(OpticBody::new(
        body,
        Surface::glass(1.5, V3::new(1.0, 1.0, 1.0)),
    ));
    let ctx = GpuContext::init().await?;
    let grid_n = 16usize;
    let grid = render_caustics_gpu(
        &ctx,
        &scene,
        &V3::new(0.3f32, 1.0, 0.2), // 平行光方向(指向光源)
        0.0,                        // 接收面 y
        6.0,                        // half_extent
        grid_n,
    )
    .await?;
    let max = grid.iter().cloned().fold(0.0f32, f32::max);
    let sum: f32 = grid.iter().sum();
    Ok(format!(
        "W3 caustics ok: grid_n={} max={:.4} sum={:.3}",
        grid_n, max, sum
    ))
}

/// W3 焦散 wgsl:逐射线 one-thread,复刻 `Caustics::march`(sphere/box 求交)。
const CAUSTIC_WGSL: &str = r#"
struct BodyGpu {
    pos_kind : vec4<f32>,
    quat     : vec4<f32>,
    geo      : vec4<f32>,
    albedo   : vec4<f32>,
    misc     : vec4<f32>,
};
struct CausticParams {
    travel_plane : vec4<f32>,
    he_gn_env    : vec4<f32>,
    p2           : vec4<f32>,
    p3           : vec4<f32>,
};

@group(0) @binding(0) var<uniform> prm : CausticParams;
@group(0) @binding(1) var<storage, read> bodies : array<BodyGpu>;
@group(0) @binding(2) var<storage, read_write> outp : array<f32>;

const EPS = 1e-4;

fn quat_rot(q: vec4<f32>, v: vec3<f32>) -> vec3<f32> {
    let t = 2.0 * cross(q.yzw, v);
    return v + q.x * t + cross(q.yzw, t);
}
fn quat_rot_inv(q: vec4<f32>, v: vec3<f32>) -> vec3<f32> {
    let t = 2.0 * cross(q.yzw, v);
    return v - q.x * t + cross(q.yzw, t);
}
fn norm(v: vec3<f32>) -> vec3<f32> {
    let n = length(v);
    if (n > 1e-12) { return v / n; }
    return vec3<f32>(1.0, 0.0, 0.0);
}
fn reflect1(i: vec3<f32>, n: vec3<f32>) -> vec3<f32> { return i - n * (2.0 * dot(i, n)); }
fn refract1(i: vec3<f32>, n: vec3<f32>, eta: f32) -> vec3<f32> {
    let cosi = -dot(i, n);
    let k = 1.0 - eta * eta * (1.0 - cosi * cosi);
    if (k < 0.0) { return vec3<f32>(0.0); }
    return norm(i * eta + n * (eta * cosi - sqrt(k)));
}
fn fresnel1(cosi: f32, f0: f32) -> f32 {
    let t = 1.0 - cosi;
    let t2 = t * t;
    return f0 + (1.0 - f0) * t2 * t2 * t;
}

struct Hit { t: f32, n: vec3<f32>, idx: f32 };
fn intersect2(ro: vec3<f32>, rd: vec3<f32>) -> Hit {
    var best_t = 1e30;
    var best_n = vec3<f32>(0.0);
    var best_i = -1.0;
    for (var idx = 0u; idx < arrayLength(&bodies); idx = idx + 1u) {
        let b = bodies[idx];
        let kind = b.pos_kind.w;
        let q = b.quat;
        let ro_l = quat_rot_inv(q, ro - b.pos_kind.xyz);
        let rd_l = quat_rot_inv(q, rd);
        let rdl = length(rd_l);
        if (rdl <= 0.0) { continue; }
        let rd_n = rd_l / rdl;
        var t_local = -1.0;
        var n_local = vec3<f32>(0.0);
        if (kind < 0.5) {
            let r = b.geo.x;
            let bb = dot(ro_l, rd_n);
            let cc = dot(ro_l, ro_l) - r * r;
            let disc = bb * bb - cc;
            if (disc < 0.0) { continue; }
            let sq = sqrt(disc);
            var t0 = -bb - sq;
            let t = select(-bb + sq, t0, t0 > 0.0);
            if (t <= 0.0) { continue; }
            let pl = ro_l + rd_n * t;
            t_local = t * rdl;
            n_local = norm(pl / r);
        } else {
            let half = b.geo.xyz;
            var tmin = 0.0;
            var tmax = 1e30;
            var nrm = vec3<f32>(0.0);
            var ok = true;
            for (var ax = 0u; ax < 3u; ax = ax + 1u) {
                let o = ro_l[ax];
                let d = rd_n[ax];
                let h = half[ax];
                var inv_d = 0.0;
                var sign = 1.0;
                if (abs(d) > 1e-12) { inv_d = 1.0 / d; } else { if (o < -h || o > h) { ok = false; break; } inv_d = 0.0; }
                var t1 = (-h - o) * inv_d;
                var t2 = (h - o) * inv_d;
                var nax = vec3<f32>(0.0);
                nax[ax] = -1.0;
                if (t1 > t2) { let tmp = t1; t1 = t2; t2 = tmp; nax[ax] = 1.0; sign = -1.0; }
                if (t1 > tmin) { tmin = t1; nrm = nax * sign; }
                if (t2 < tmax) { tmax = t2; }
                if (tmin > tmax) { ok = false; break; }
            }
            if (!ok || tmin <= 0.0) { continue; }
            t_local = tmin * rdl;
            n_local = norm(nrm);
        }
        let n_world = quat_rot(q, n_local);
        if (t_local < best_t) { best_t = t_local; best_n = n_world; best_i = f32(idx); }
    }
    return Hit(best_t, best_n, best_i);
}

fn f0_of(n_from: f32, n_to: f32) -> f32 {
    let r = (n_from - n_to) / (n_from + n_to);
    return r * r;
}

fn march(ro0: vec3<f32>, travel: vec3<f32>, env_ior: f32) -> f32 {
    var ro = ro0;
    var rd = travel;
    var cur_ior = env_ior;
    var flux = 1.0;
    for (var seg = 0u; seg < 8u; seg = seg + 1u) {
        let h = intersect2(ro, rd);
        if (h.idx < 0.0) { return 0.0; }
        let surf_albedo = bodies[u32(h.idx)].albedo.xyz;
        let ior = bodies[u32(h.idx)].albedo.w;
        let transparent = bodies[u32(h.idx)].misc.x > 0.5;
        let p = ro + rd * h.t;
        let cosi = -dot(rd, h.n);
        var n_face = h.n;
        var entering = true;
        var eta = cur_ior / ior;
        var n_from = cur_ior;
        var n_to = ior;
        if (cosi <= 0.0) {
            n_face = -h.n;
            entering = false;
            eta = ior / cur_ior;
            n_from = ior;
            n_to = cur_ior;
        }
        let abs_cosi = abs(cosi);
        let f0 = f0_of(n_from, n_to);
        if (transparent) {
            let ft = 1.0 - fresnel1(abs_cosi, f0);
            flux = flux * ft;
            let tdir = refract1(rd, n_face, eta);
            if (length(tdir) <= 0.0) { return 0.0; }
            if (entering) { cur_ior = ior; } else { cur_ior = env_ior; }
            ro = p + tdir * EPS;
            rd = tdir;
        } else {
            return flux;
        }
    }
    return 0.0;
}

@compute @workgroup_size(64)
fn main(@builtin(global_invocation_id) gid: vec3<u32>) {
    let idx = gid.x;
    let grid_n = u32(prm.he_gn_env.y);
    let total = grid_n * grid_n;
    if (idx >= total) { return; }
    let plane_y = prm.travel_plane.w;
    let top = plane_y + 20.0;
    let half = prm.he_gn_env.x;
    let cell = (2.0 * half) / prm.he_gn_env.y;
    let i = idx % grid_n;
    let j = idx / grid_n;
    let x = -half + (f32(i) + 0.5) * cell;
    let z = -half + (f32(j) + 0.5) * cell;
    let ro0 = vec3<f32>(x, top, z);
    let travel = norm(prm.travel_plane.xyz);
    let env_ior = prm.he_gn_env.z;
    outp[idx] = march(ro0, travel, env_ior);
}
"#;

// ===========================================================================
// W4: SPH 逐粒子密度/受力走 GPU。
//
// 复用 phy-fluid 的 `SphFlatData`(W4 前置:Grid::to_flat 扁平网格)。
// 两个 entry point 同 shader: density_main(写 rho/p) → force_main(读 rho/p 算加速度)。
// 复刻 CPU 端 `compute_density_pressure` + `compute_forces`(含 power-law 非牛顿粘度)。
// ===========================================================================

use phy_fluid::{FluidSubsystem, FluidWorld, SphFlatData};
use phy_granular::subsystem::GranularSubsystem;
use phy_granular::world::GranularWorld;
use phy_core::World;
use phy_granular::GranularFlatData;

/// W4 主入口:GPU 算密度/压力 + 受力,返回 (rho_p 扁平, acc_mu 扁平),
/// 每个粒子一个 vec4:rho_p=(rho,p,mass,mat),acc_mu=(ax,ay,az,mu)。
pub async fn render_sph_gpu(
    ctx: &GpuContext,
    data: &SphFlatData,
) -> Result<(Vec<[f32; 4]>, Vec<[f32; 4]>), String> {
    let n = data.n;
    // 物性 uniform(固定 12 个 vec4)。
    let mut visc_k = [0.0f32; 4];
    let mut visc_n = [0.0f32; 4];
    for (i, v) in data.visc_k.iter().enumerate().take(4) {
        visc_k[i] = *v;
    }
    for (i, v) in data.visc_n.iter().enumerate().take(4) {
        visc_n[i] = *v;
    }
    let uni: [[f32; 4]; 12] = [
        [data.h, data.rest_density, data.stiffness, data.shear_min],
        [data.gravity[0], data.gravity[1], data.gravity[2], 0.0],
        [data.nc[0] as f32, data.nc[1] as f32, data.nc[2] as f32, 0.0],
        [data.grid_min[0] as f32, data.grid_min[1] as f32, data.grid_min[2] as f32, 0.0],
        visc_k,
        visc_n,
        [0.0; 4],
        [0.0; 4],
        [0.0; 4],
        [0.0; 4],
        [0.0; 4],
        [0.0; 4],
    ];

    let pos_bytes = bytemuck::cast_slice::<[f32; 4], u8>(&data.pos);
    let pos_buf = ctx
        .device
        .create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("pos"),
            contents: pos_bytes,
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
        });
    let vel_bytes = bytemuck::cast_slice::<[f32; 4], u8>(&data.vel);
    let vel_buf = ctx
        .device
        .create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("vel"),
            contents: vel_bytes,
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
        });
    let scl_bytes = bytemuck::cast_slice::<[f32; 4], u8>(&data.scalar);
    let scl_buf = ctx
        .device
        .create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("scl"),
            contents: scl_bytes,
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
        });
    let cs_bytes = bytemuck::cast_slice::<i32, u8>(&data.cell_start);
    let cs_buf = ctx
        .device
        .create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("cs"),
            contents: cs_bytes,
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
        });
    let sd_bytes = bytemuck::cast_slice::<i32, u8>(&data.sorted);
    let sd_buf = ctx
        .device
        .create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("sd"),
            contents: sd_bytes,
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
        });
    let uni_bytes = bytemuck::cast_slice::<[f32; 4], u8>(&uni);
    let uni_buf = ctx
        .device
        .create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("uni"),
            contents: uni_bytes,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        });
    // rho_p / acc_mu 输出。
    let mk = |label: &str| -> wgpu::Buffer {
        ctx.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some(label),
            size: (n * 4 * std::mem::size_of::<f32>()) as u64,
            usage: wgpu::BufferUsages::STORAGE
                | wgpu::BufferUsages::COPY_SRC
                | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        })
    };
    let rho_p_buf = mk("rho_p");
    let acc_mu_buf = mk("acc_mu");
    let rho_p_read = ctx.device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("rho_p_r"),
        size: rho_p_buf.size(),
        usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    });
    let acc_mu_read = ctx.device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("acc_mu_r"),
        size: acc_mu_buf.size(),
        usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    });

    let sph_bgl = &ctx.pipelines.sph_bgl;
    let _sph_pl = &ctx.pipelines.sph_pl;
    let p_dens = &ctx.pipelines.sph_dens;
    let p_force = &ctx.pipelines.sph_force;
    let bind = ctx.device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("sph_bg"),
        layout: sph_bgl,
        entries: &[
            wgpu::BindGroupEntry { binding: 0, resource: uni_buf.as_entire_binding() },
            wgpu::BindGroupEntry { binding: 1, resource: pos_buf.as_entire_binding() },
            wgpu::BindGroupEntry { binding: 2, resource: vel_buf.as_entire_binding() },
            wgpu::BindGroupEntry { binding: 3, resource: scl_buf.as_entire_binding() },
            wgpu::BindGroupEntry { binding: 4, resource: cs_buf.as_entire_binding() },
            wgpu::BindGroupEntry { binding: 5, resource: sd_buf.as_entire_binding() },
            wgpu::BindGroupEntry { binding: 6, resource: rho_p_buf.as_entire_binding() },
            wgpu::BindGroupEntry { binding: 7, resource: acc_mu_buf.as_entire_binding() },
        ],
    });
    let bg = &bind;

    let mut encoder =
        ctx.device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("sph_enc"),
            });
    // density pass
    {
        let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
            label: Some("dens"),
            ..Default::default()
        });
        pass.set_pipeline(p_dens);
        pass.set_bind_group(0, bg, &[]);
        pass.dispatch_workgroups(((n + 63) / 64) as u32, 1, 1);
    }
    // force pass(读 density 写的 rho_p)
    {
        let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
            label: Some("force"),
            ..Default::default()
        });
        pass.set_pipeline(p_force);
        pass.set_bind_group(0, bg, &[]);
        pass.dispatch_workgroups(((n + 63) / 64) as u32, 1, 1);
    }
    encoder.copy_buffer_to_buffer(&rho_p_buf, 0, &rho_p_read, 0, rho_p_buf.size());
    encoder.copy_buffer_to_buffer(&acc_mu_buf, 0, &acc_mu_read, 0, acc_mu_buf.size());
    ctx.queue.submit(std::iter::once(encoder.finish()));

    // 回读(两个 buffer 串行 map)。
    let rho_p = read_gpu_vec(ctx, &rho_p_read, n).await?;
    let acc_mu = read_gpu_vec(ctx, &acc_mu_read, n).await?;
    Ok((rho_p, acc_mu))
}

/// 辅助:map 一个 storage read buffer 为 Vec<[f32;4]>。
async fn read_gpu_vec(
    ctx: &GpuContext,
    buf: &wgpu::Buffer,
    _n: usize,
) -> Result<Vec<[f32; 4]>, String> {
    let slice = buf.slice(..);
    let (tx, rx) = futures::channel::oneshot::channel();
    slice.map_async(wgpu::MapMode::Read, move |r| {
        let _ = tx.send(r);
    });
    // 原生(桌面)wgpu:map_async 回调须在 device.poll 时触发;wasm 下浏览器事件循环自行推进。
    // poll(PollType::Wait) 阻塞直到全部提交工作完成并回调,保证 native 下 rx 一定收到结果。
    let _ = ctx.device.poll(wgpu::PollType::Wait { submission_index: None, timeout: None });
    rx.await
        .map_err(|e| format!("map 通道失败: {:?}", e))?
        .map_err(|e| format!("map_async 失败: {:?}", e))?;
    let view = slice.get_mapped_range()
        .map_err(|e| format!("get_mapped_range 失败: {:?}", e))?;
    let out: Vec<[f32; 4]> = bytemuck::cast_slice(&view).to_vec();
    drop(view);
    buf.unmap();
    Ok(out)
}

/// W4 自测:构造 dam-break 规则晶格,跑 GPU 密度/受力,验证内核数学正确。
pub async fn sph_self_test() -> Result<String, String> {
    use phy_fluid::SphParams;
    let params = SphParams::<f32>::defaults();
    let mut world = phy_fluid::FluidWorld::<f32>::new(params);
    // 规则晶格(5x5x5, spacing 0.1, in box)生成 dam-break 初始块。
    let spacing = 0.1f32;
    let y = 0.5f32;
    for xi in 0..5u32 {
        for yi in 0..5u32 {
            for zi in 0..5u32 {
                let x = 0.0f32 + xi as f32 * spacing;
                let z = 0.0f32 + zi as f32 * spacing;
                world.add_particle(phy_fluid::Particle::new(
                    phy_math::Vec3::new(x, y + yi as f32 * spacing, z),
                    world.params.mass,
                ));
            }
        }
    }
    // 构建网格(需在 to_gpu_flat 前)。
    world.build_grid();
    let flat = world.to_gpu_flat();
    let ctx = GpuContext::init().await?;
    let (rho_p, acc_mu) = render_sph_gpu(&ctx, &flat).await?;
    let mut mean_rho = 0.0f32;
    let mut finite = true;
    for v in &rho_p {
        mean_rho += v[0];
        if !v[0].is_finite() {
            finite = false;
        }
    }
    mean_rho /= rho_p.len().max(1) as f32;
    // 参考:规则晶格初始密度应接近 rest_density(CPU init_rho 设计)。
    let rest = flat.rest_density;
    let ratio = mean_rho / rest;
    Ok(format!(
        "W4 sph ok: n={} mean_rho={:.3} rest={:.3} ratio={:.3} finite={} acc0=({:.3},{:.3},{:.3})",
        rho_p.len(),
        mean_rho,
        rest,
        ratio,
        finite,
        acc_mu[0][0],
        acc_mu[0][1],
        acc_mu[0][2]
    ))
}

/// W4 SPH wgsl:两个 entry point(density_main / force_main),复刻 CPU SPH 内核。
const SPH_WGSL: &str = r#"
struct SphUni {
    hp_shear : vec4<f32>,
    grav     : vec4<f32>,
    nc       : vec4<f32>,
    gmin     : vec4<f32>,
    visc_k   : vec4<f32>,
    visc_n   : vec4<f32>,
    p6       : vec4<f32>,
    p7       : vec4<f32>,
    p8       : vec4<f32>,
    p9       : vec4<f32>,
    p10      : vec4<f32>,
    p11      : vec4<f32>,
};

@group(0) @binding(0) var<uniform> u : SphUni;
@group(0) @binding(1) var<storage, read> pos : array<vec4<f32>>;
@group(0) @binding(2) var<storage, read> vel : array<vec4<f32>>;
@group(0) @binding(3) var<storage, read> scl : array<vec4<f32>>; // (rho0,p0,mass,mat)
@group(0) @binding(4) var<storage, read> cell_start : array<i32>;
@group(0) @binding(5) var<storage, read> sorted : array<i32>;
@group(0) @binding(6) var<storage, read_write> rho_p : array<vec4<f32>>; // (rho,p,mass,mat)
@group(0) @binding(7) var<storage, read_write> acc_mu : array<vec4<f32>>; // (ax,ay,az,mu)

const PI = 3.14159265358979;

fn poly6(r2: f32, h: f32) -> f32 {
    if (r2 >= h * h) { return 0.0; }
    let diff = h * h - r2;
    let coeff = 315.0 / (64.0 * PI * pow(h, 9.0));
    return coeff * diff * diff * diff;
}
fn spiky_grad(r: f32, h: f32) -> f32 {
    if (r >= h || r <= 0.0) { return 0.0; }
    let diff = h - r;
    let coeff = 45.0 / (PI * pow(h, 6.0));
    return coeff * diff * diff; // 正系数(方向由 dir 决定)
}
fn visc_lap(r: f32, h: f32) -> f32 {
    if (r >= h) { return 0.0; }
    let coeff = 45.0 / (PI * pow(h, 6.0));
    return coeff * (h - r);
}

fn cell_idx(ci: i32, cj: i32, ck: i32) -> i32 {
    let ncx = i32(u.nc.x);
    let ncy = i32(u.nc.y);
    let ncz = i32(u.nc.z);
    let mi = i32(u.gmin.x);
    let mj = i32(u.gmin.y);
    let mk = i32(u.gmin.z);
    let ci2 = clamp(ci, mi, mi + ncx - 1);
    let cj2 = clamp(cj, mj, mj + ncy - 1);
    let ck2 = clamp(ck, mk, mk + ncz - 1);
    return (ci2 - mi) + ncx * ((cj2 - mj) + ncy * (ck2 - mk));
}

@compute @workgroup_size(64)
fn density_main(@builtin(global_invocation_id) gid: vec3<u32>) {
    let i = gid.x;
    if (i >= arrayLength(&pos)) { return; }
    let h = u.hp_shear.x;
    let pi = pos[i].xyz;
    // 格子 key = floor(pi/h)(cell==h);cell_idx 内部用 u.gmin 作为偏移基准。
    let ci = i32(floor(pi.x / h));
    let cj = i32(floor(pi.y / h));
    let ck = i32(floor(pi.z / h));
    var rho = 0.0;
    for (var di = -1; di <= 1; di = di + 1) {
        for (var dj = -1; dj <= 1; dj = dj + 1) {
            for (var dk = -1; dk <= 1; dk = dk + 1) {
                let c = cell_idx(ci + di, cj + dj, ck + dk);
                let base = cell_start[c];
                let end = cell_start[c + 1];
                for (var s = base; s < end; s = s + 1) {
                    let j = sorted[s];
                    let pj = pos[j].xyz;
                    let d = pi - pj;
                    let r2 = dot(d, d);
                    rho = rho + scl[j].z * poly6(r2, h); // mass * W
                }
            }
        }
    }
    let rest = u.hp_shear.y;
    let k = u.hp_shear.z;
    let p = k * (rho - rest);
    rho_p[i] = vec4<f32>(rho, p, scl[i].z, scl[i].w);
}

@compute @workgroup_size(64)
fn force_main(@builtin(global_invocation_id) gid: vec3<u32>) {
    let i = gid.x;
    if (i >= arrayLength(&pos)) { return; }
    let h = u.hp_shear.x;
    let pi = pos[i].xyz;
    let vi = vel[i].xyz;
    let rho_i = rho_p[i].x;
    let p_i = rho_p[i].y;
    let mat = scl[i].w;
    let mi = scl[i].z;
    let ki = u.visc_k[u32(mat)];
    let ni = u.visc_n[u32(mat)];
    let shear_min = u.hp_shear.w;
    // 格子 key = floor(pi/h)(cell==h);cell_idx 内部用 u.gmin 作为偏移基准。
    let ci = i32(floor(pi.x / h));
    let cj = i32(floor(pi.y / h));
    let ck = i32(floor(pi.z / h));
    var press = vec3<f32>(0.0);
    var visc = vec3<f32>(0.0);
    var shear = 0.0; // 局部应变率代理(非牛顿幂律用)
    for (var di = -1; di <= 1; di = di + 1) {
        for (var dj = -1; dj <= 1; dj = dj + 1) {
            for (var dk = -1; dk <= 1; dk = dk + 1) {
                let c = cell_idx(ci + di, cj + dj, ck + dk);
                let base = cell_start[c];
                let end = cell_start[c + 1];
                for (var s = base; s < end; s = s + 1) {
                    let j = sorted[s];
                    if (j == i32(i)) { continue; }
                    let pj = pos[j].xyz;
                    let vj = vel[j].xyz;
                    let d = pi - pj;
                    let r2 = dot(d, d);
                    if (r2 <= 0.0 || r2 >= h * h) { continue; }
                    let r = sqrt(r2);
                    let dir = (pi - pj) / r; // 由 j 指向 i,乘以正 fpress → 把 i 推离 j(与 CPU 生产一致)
                    let rho_j = rho_p[j].x;
                    let p_j = rho_p[j].y;
                    let mj = scl[j].z;
                    // 对称压力式(Müller 生产同款):含 m_i, 动量守恒
                    let fpress = spiky_grad(r, h);
                    let coef = mi * mj * (p_i / (rho_i * rho_i) + p_j / (rho_j * rho_j));
                    press = press + dir * (coef * fpress);
                    // 粘性(原始项, μ 在外层乘;含 m_i, 与 CPU 生产 compute_forces 一致)
                    let fvisc = visc_lap(r, h);
                    visc = visc + (vj - vi) * (mi * mj / rho_j * fvisc);
                    // 局部应变率代理(CPU 同款): Σ |v_j-v_i|/(r+ε)·(m_j/ρ_j)
                    let dv = length(vj - vi);
                    shear = shear + dv / (r + 1.0e-4) * (mj / rho_j);
                }
            }
        }
    }
    // 非牛顿幂律有效粘度(CPU 同款): μ_eff = ki·max(shear, shear_min)^(ni-1)
    let sreg = select(shear_min, shear, shear > shear_min);
    let mu_eff = ki * pow(sreg, ni - 1.0);
    var acc = vec3<f32>(0.0);
    if (rho_i > 1e-8) {
        acc = acc + (press + visc * mu_eff) / rho_i;
    }
    // 重力
    acc = acc + u.grav.xyz;
    acc_mu[i] = vec4<f32>(acc, mu_eff);
}
"#;

// ===========================================================================
// W5: 颗粒 PBD 接触投影上 GPU(`par_pairs_reduce`)。
//
// Jacobi 式并行:每对只读预测位置快照 → 计算位移修正 → 累加进 per-body
// delta 缓冲(确定性固定序 reduce)。由于多个对会写到同一 body,delta 用
// `atomic<i32>` 定点累加(×SCALE 后 bitcast),apply 阶段还原回 f32。
// 边界夹紧也在 apply 阶段逐体完成。速度回写/摩擦留主机端(轻量)。
// ===========================================================================

/// W5 颗粒 PBD wgsl:三个 entry point。
/// - `clear_main`:清 delta 缓冲(每体 3 分量)。
/// - `contact_main`:每对累加位移修正(atomic 定点)。
/// - `apply_main`:逐体施加修正 + 盒边界夹紧,写回预测位置。
const GRANULAR_WGSL: &str = r#"
struct GranUni {
    gravity   : vec4<f32>,  // xyz = 重力, w = dt
    bounds_lo : vec4<f32>,  // xyz = 下界
    bounds_hi : vec4<f32>,  // xyz = 上界
    damp_fric: vec4<f32>,   // x = vel_damp, y = friction
};

@group(0) @binding(0) var<uniform> u : GranUni;
@group(0) @binding(1) var<storage, read_write> pos : array<vec4<f32>>; // xyz=pos, w=radius
@group(0) @binding(2) var<storage, read> inv_mass : array<f32>;
@group(0) @binding(3) var<storage, read> pairs : array<u32>;           // [i0,j0,i1,j1,...]
@group(0) @binding(4) var<storage, read_write> deltas : array<atomic<i32>>; // 每体 3 分量,定点

@compute @workgroup_size(64)
fn clear_main(@builtin(global_invocation_id) gid : vec3<u32>) {
    let k = gid.x;
    // 每体 3 分量;调用方按 n*3 dispatch。
    if (k >= arrayLength(&deltas)) { return; }
    atomicStore(&deltas[k], 0);
}

@compute @workgroup_size(64)
fn contact_main(@builtin(global_invocation_id) gid : vec3<u32>) {
    let p = gid.x;
    let np = arrayLength(&pairs) / 2u;
    if (p >= np) { return; }
    let i = pairs[p * 2u];
    let j = pairs[p * 2u + 1u];
    let pi = pos[i].xyz;
    let pj = pos[j].xyz;
    let ri = pos[i].w;
    let rj = pos[j].w;
    let wi = inv_mass[i];
    let wj = inv_mass[j];
    let d = pj - pi;
    let dist = max(length(d), 1.0e-9);
    let min_d = ri + rj;
    let wsum = wi + wj;
    if (dist < min_d && wsum > 0.0) {
        let corr = (min_d - dist) / dist;
        let dir = d * corr;
        let di = dir * (wi / wsum); // i 位移(指向 -dir)
        let dj = dir * (wj / wsum); // j 位移(指向 +dir)
        // 定点累加(×SCALE, bitcast f32->i32)。
        let s = 1.0e6;
        atomicAdd(&deltas[i * 3u + 0u], i32(di.x * s));
        atomicAdd(&deltas[i * 3u + 1u], i32(di.y * s));
        atomicAdd(&deltas[i * 3u + 2u], i32(di.z * s));
        atomicAdd(&deltas[j * 3u + 0u], i32(dj.x * s));
        atomicAdd(&deltas[j * 3u + 1u], i32(dj.y * s));
        atomicAdd(&deltas[j * 3u + 2u], i32(dj.z * s));
    }
}

@compute @workgroup_size(64)
fn apply_main(@builtin(global_invocation_id) gid : vec3<u32>) {
    let i = gid.x;
    let n = arrayLength(&pos);
    if (i >= n) { return; }
    if (inv_mass[i] <= 0.0) { return; } // 固定体不参与投影
    let s = 1.0e6;
    let dx = f32(atomicLoad(&deltas[i * 3u + 0u])) / s;
    let dy = f32(atomicLoad(&deltas[i * 3u + 1u])) / s;
    let dz = f32(atomicLoad(&deltas[i * 3u + 2u])) / s;
    var p = pos[i].xyz;
    p = p + vec3<f32>(dx, dy, dz);
    // 盒边界夹紧(留半径余量)。
    let r = pos[i].w;
    let lo = u.bounds_lo.xyz;
    let hi = u.bounds_hi.xyz;
    if (p.x < lo.x + r) { p.x = lo.x + r; }
    if (p.x > hi.x - r) { p.x = hi.x - r; }
    if (p.y < lo.y + r) { p.y = lo.y + r; }
    if (p.y > hi.y - r) { p.y = hi.y - r; }
    if (p.z < lo.z + r) { p.z = lo.z + r; }
    if (p.z > hi.z - r) { p.z = hi.z - r; }
    pos[i] = vec4<f32>(p, r);
}
"#;

/// W5 主入口:在 GPU 上跑 `iterations` 轮 Jacobi 接触投影 + 边界夹紧。
/// 返回最终预测位置(扁平 `[x,y,z,radius]` 每体)。
pub async fn render_granular_gpu(
    ctx: &GpuContext,
    flat: &GranularFlatData,
) -> Result<Vec<[f32; 4]>, String> {
    let n = flat.n;
    let npairs = flat.npairs;
    if n == 0 {
        return Ok(Vec::new());
    }
    // uniform:gravity(dt)+bounds+damp/fric。
    #[repr(C, align(16))]
    #[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
    struct GranUni {
        gravity: [f32; 4],
        bounds_lo: [f32; 4],
        bounds_hi: [f32; 4],
        damp_fric: [f32; 4],
    }
    let uni = GranUni {
        gravity: [flat.gravity[0], flat.gravity[1], flat.gravity[2], flat.dt],
        bounds_lo: [flat.bounds_lo[0], flat.bounds_lo[1], flat.bounds_lo[2], 0.0],
        bounds_hi: [flat.bounds_hi[0], flat.bounds_hi[1], flat.bounds_hi[2], 0.0],
        damp_fric: [flat.vel_damp, flat.friction, 0.0, 0.0],
    };
    let uni_buf = ctx
        .device
        .create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("gran_uni"),
            contents: bytemuck::bytes_of(&uni),
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        });
    let pos_buf = ctx
        .device
        .create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("gran_pos"),
            contents: bytemuck::cast_slice(&flat.pos),
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::COPY_SRC,
        });
    let inv_buf = ctx
        .device
        .create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("gran_inv_mass"),
            contents: bytemuck::cast_slice(&flat.inv_mass),
            usage: wgpu::BufferUsages::STORAGE,
        });
    let pairs_bytes: Vec<u8> = if flat.pairs.is_empty() {
        vec![0u8; 16]
    } else {
        bytemuck::cast_slice(&flat.pairs).to_vec()
    };
    let pairs_buf = ctx
        .device
        .create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("gran_pairs"),
            contents: &pairs_bytes,
            usage: wgpu::BufferUsages::STORAGE,
        });
    let deltas_buf = ctx.device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("gran_deltas"),
        size: (n * 3 * std::mem::size_of::<i32>()) as u64,
        usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    });

    // 复用常驻管线(见 `GpuContext::init` 中的 `build_granular_pipelines`)。
    let bgl = &ctx.pipelines.granular_bgl;
    let _pl = &ctx.pipelines.granular_pl;
    let p_clear = &ctx.pipelines.granular_dens;
    let p_contact = &ctx.pipelines.granular_pred;
    let p_apply = &ctx.pipelines.granular_solve;
    let bind = |_: &wgpu::ComputePipeline| {
        ctx.device
            .create_bind_group(&wgpu::BindGroupDescriptor {
                label: Some("gran_bg"),
                layout: bgl,
                entries: &[
                    wgpu::BindGroupEntry { binding: 0, resource: uni_buf.as_entire_binding() },
                    wgpu::BindGroupEntry { binding: 1, resource: pos_buf.as_entire_binding() },
                    wgpu::BindGroupEntry { binding: 2, resource: inv_buf.as_entire_binding() },
                    wgpu::BindGroupEntry { binding: 3, resource: pairs_buf.as_entire_binding() },
                    wgpu::BindGroupEntry { binding: 4, resource: deltas_buf.as_entire_binding() },
                ],
            })
    };
    let bg = bind(p_clear);

    let wg = 64u32;
    let iters = flat.iterations.max(1) as usize;
    for _ in 0..iters {
        let mut encoder = ctx
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor { label: Some("gran_enc") });
        {
            let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("gran_pass"),
                ..Default::default()
            });
            pass.set_bind_group(0, &bg, &[]);
            pass.set_pipeline(&p_clear);
            pass.dispatch_workgroups(((n * 3) as u32 + wg - 1) / wg, 1, 1);
            pass.set_pipeline(&p_contact);
            pass.dispatch_workgroups((npairs as u32 + wg - 1) / wg, 1, 1);
            pass.set_pipeline(&p_apply);
            pass.dispatch_workgroups((n as u32 + wg - 1) / wg, 1, 1);
        }
        ctx.queue.submit(std::iter::once(encoder.finish()));
    }

    // 回读 pos。
    let read_buf = ctx.device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("gran_read"),
        size: (n * std::mem::size_of::<[f32; 4]>()) as u64,
        usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    });
    let mut encoder = ctx
        .device
        .create_command_encoder(&wgpu::CommandEncoderDescriptor { label: Some("gran_enc2") });
    encoder.copy_buffer_to_buffer(&pos_buf, 0, &read_buf, 0, read_buf.size());
    ctx.queue.submit(std::iter::once(encoder.finish()));
    let slice = read_buf.slice(..);
    let (tx, rx) = futures::channel::oneshot::channel();
    slice.map_async(wgpu::MapMode::Read, move |r| {
        let _ = tx.send(r);
    });
    // 原生(桌面)wgpu 需 poll 才触发 map_async 回调。
    let _ = ctx.device.poll(wgpu::PollType::Wait { submission_index: None, timeout: None });
    rx.await
        .map_err(|e| format!("gran map 通道失败: {:?}", e))?
        .map_err(|e| format!("gran map_async 失败: {:?}", e))?;
    let view = slice.get_mapped_range()
        .map_err(|e| format!("get_mapped_range 失败: {:?}", e))?;
    let out: Vec<[f32; 4]> = bytemuck::cast_slice(&view).to_vec();
    drop(view);
    read_buf.unmap();
    Ok(out)
}

/// W5 自测:堆一小盒颗粒,跑 GPU 接触投影,验证不重叠 + 有限。
pub async fn granular_self_test() -> Result<String, String> {
    use phy_granular::world::GranularWorld;
    let mut world = GranularWorld::<f32>::new();
    world.set_bounds(
        phy_math::Vec3::new(-1.0, -1.0, -1.0),
        phy_math::Vec3::new(1.0, 1.0, 1.0),
    );
    // 紧密网格撒布(留间隙,初始不重叠),半径 0.1。
    world.fill_grid(27, 0.1f32, 1.0f32, 1.05f32);
    let flat = world.to_gpu_flat();
    let ctx = GpuContext::init().await?;
    let pos = render_granular_gpu(&ctx, &flat).await?;
    // 统计最小间隙(任意两体)。
    let mut min_gap = f32::INFINITY;
    let mut overlaps = 0usize;
    let mut finite = true;
    let n = pos.len();
    for a in 0..n {
        let pa = [pos[a][0], pos[a][1], pos[a][2]];
        let ra = pos[a][3];
        if !pa[0].is_finite() || !pa[1].is_finite() || !pa[2].is_finite() {
            finite = false;
        }
        for b in (a + 1)..n {
            let pb = [pos[b][0], pos[b][1], pos[b][2]];
            let rb = pos[b][3];
            let dx = pb[0] - pa[0];
            let dy = pb[1] - pa[1];
            let dz = pb[2] - pa[2];
            let d = (dx * dx + dy * dy + dz * dz).sqrt();
            let gap = d - (ra + rb);
            if gap < min_gap {
                min_gap = gap;
            }
            if gap < -1e-4 {
                overlaps += 1;
            }
        }
    }
    Ok(format!(
        "W5 granular ok: n={} npairs={} min_gap={:.4} overlaps={} finite={}",
        n,
        flat.npairs,
        min_gap,
        overlaps,
        finite
    ))
}

// ───────────────────────────────────────────────────────────────────────────
// 运行时切换:完整 GPU step(替代 CPU `world.step`)
//
// 这些函数接收 f64 世界(Web Demo 的 `Scene::world: World<f64>` 内部子系统),
// 把粒子副本缩成 f32 送 GPU compute,再把结果写回 f64 世界。与 CPU `step` 等价:
// 流体 = 密度/压力/力(GPU) + 主机端半隐式积分 + 边界夹紧;
// 颗粒 = 先预测(重力)再接触投影(GPU) + 速度回写 + 摩擦衰减。
// `couple` 阶段仍由 `World::step_skipping` 走 CPU,跨子系统耦合矩阵保持完整。
// ───────────────────────────────────────────────────────────────────────────

/// f64 流体世界的完整一帧 GPU 步进,写回 `world.particles` 的 `pos/vel/rho/p`。
pub async fn step_sph_gpu(
    ctx: &GpuContext,
    world: &mut FluidWorld<f64>,
    dt: f32,
) -> Result<(), String> {
    if world.particles.is_empty() {
        return Ok(());
    }
    // 喂邻居网格(GPU 密度/力内核依赖它)。
    world.build_grid();
    let flat = world.to_gpu_flat();
    let (rho_p, acc) = render_sph_gpu(ctx, &flat).await?;
    let n = world.particles.len();
    let bm = world.params.bounds_min;
    let bx = world.params.bounds_max;
    for i in 0..n {
        let a = acc[i];
        let p = &mut world.particles[i];
        // 半隐式欧拉(力内核已含重力 f_ext,不再叠加重力)。
        let mut vx = p.vel.x + (a[0] as f64) * dt as f64;
        let mut vy = p.vel.y + (a[1] as f64) * dt as f64;
        let mut vz = p.vel.z + (a[2] as f64) * dt as f64;
        let mut x = p.pos.x + vx * dt as f64;
        let mut y = p.pos.y + vy * dt as f64;
        let mut z = p.pos.z + vz * dt as f64;
        // 边界夹紧(复刻 CPU enforce_bounds)。
        if x < bm.x {
            x = bm.x;
            vx = 0.0;
        } else if x > bx.x {
            x = bx.x;
            vx = 0.0;
        }
        if y < bm.y {
            y = bm.y;
            vy = 0.0;
        } else if y > bx.y {
            y = bx.y;
            vy = 0.0;
        }
        if z < bm.z {
            z = bm.z;
            vz = 0.0;
        } else if z > bx.z {
            z = bx.z;
            vz = 0.0;
        }
        p.vel.x = vx;
        p.vel.y = vy;
        p.vel.z = vz;
        p.pos.x = x;
        p.pos.y = y;
        p.pos.z = z;
        p.rho = rho_p[i][0] as f64;
        p.p = rho_p[i][1] as f64;
    }
    Ok(())
}

/// f64 颗粒世界的完整一帧 GPU 步进,写回 `world.grains` 的 `pos/vel`。
pub async fn step_granular_gpu(
    ctx: &GpuContext,
    world: &mut GranularWorld<f64>,
    dt: f32,
) -> Result<(), String> {
    if world.grains.is_empty() {
        return Ok(());
    }
    // 保存帧起点位置(用于投影后速度回写)。
    let old: Vec<V3<f64>> = world.grains.iter().map(|g| g.pos).collect();
    // 预测(复刻 CPU step 的前三步:old=pos, vel+=g*dt, pos+=vel*dt)。
    for g in world.grains.iter_mut() {
        g.vel = g.vel + world.gravity * dt as f64;
        g.pos = g.pos + g.vel * dt as f64;
    }
    let flat = world.to_gpu_flat();
    let pos = render_granular_gpu(ctx, &flat).await?;
    let n = world.grains.len();
    let friction = world.friction;
    for i in 0..n {
        let np = V3::new(pos[i][0] as f64, pos[i][1] as f64, pos[i][2] as f64);
        // 速度 = (投影后位置 - 帧起点位置) / dt。
        let mut v = (np - old[i]) * (1.0 / dt as f64);
        // 摩擦衰减(复刻 CPU step 末行)。
        v = v * (1.0 - friction * dt as f64);
        world.grains[i].pos = np;
        world.grains[i].vel = v;
    }
    Ok(())
}

/// 运行时切换入口:把 `World<f64>` 中流体 / 颗粒子系统的力学 step 路由到 GPU compute,
/// 其余子系统(刚体 / 热 / ...)与全部 `couple` 阶段仍走 CPU,耦合矩阵保持完整。
///
/// 实现:先遍历子系统,对 `FluidSubsystem<f64>` / `GranularSubsystem<f64>` 调各自 GPU step
/// (写回其 `world`),并把这些索引记入 `skip`;然后 `world.step_skipping(dt, &skip)` 跳过
/// 它们的 CPU `step`,但照常执行 `couple`。
pub async fn step_world_gpu(
    ctx: &GpuContext,
    world: &mut World<f64>,
    dt: f32,
) -> Result<(), String> {
    let mut i = 0usize;
    let mut skip: Vec<bool> = Vec::new();
    loop {
        let hit = match world.get_mut(i) {
            Some(sub) => {
                if let Some(fs) = sub.as_any_mut().downcast_mut::<FluidSubsystem<f64>>() {
                    step_sph_gpu(ctx, &mut fs.world, dt).await?;
                    true
                } else if let Some(gs) =
                    sub.as_any_mut().downcast_mut::<GranularSubsystem<f64>>()
                {
                    step_granular_gpu(ctx, &mut gs.world, dt).await?;
                    true
                } else {
                    false
                }
            }
            None => break,
        };
        skip.push(hit);
        i += 1;
    }
    world.step_skipping(dt as f64, &skip);
    Ok(())
}
