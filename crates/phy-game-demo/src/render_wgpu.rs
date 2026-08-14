//! GPU 渲染后端(wgpu 0.20 / WGSL)。
//!
//! 仅在 MSVC 工具链下启用(`target_env = "msvc"`,因为 wgpu 的 D3D12 后端
//! 需要 MSVC 链接器,MinGW/GNU 工具链无法链接)。GNU 工具链走 `main.rs`
//! 的 softbuffer 软件光栅化路径。
//!
//! 后端职责:把每个 body 的网格顶点一次性上传 GPU,每帧只更新
//! 模型矩阵 + 颜色 + 相机 VP 矩阵,绘制调用交给 GPU。

use bytemuck::{Pod, Zeroable};
use phy_math::na::{Matrix4, Vector3};
use std::sync::Arc;
use wgpu::util::DeviceExt;
use winit::window::Window;

/// 单个顶点的 GPU 布局:位置(3) + 法线(3) = 6 个 f32。
#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
pub struct Vertex {
    pos: [f32; 3],
    nrm: [f32; 3],
}

/// 每实例数据:模型矩阵(列主序 4x4) + 颜色(rgb)。
#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
pub struct Instance {
    model: [[f32; 4]; 4],
    color: [f32; 4],
}

/// 每帧 uniform:视图投影矩阵 + 光照方向 + 背景色。
#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
pub struct FrameUniform {
    view_proj: [[f32; 4]; 4],
    light: [f32; 4],
    bg: [f32; 4],
}

/// 单个 body 的 GPU 资源:独立顶点缓冲(几何固定,init 上传) + 独立实例缓冲
/// (每帧写一次模型矩阵 + 颜色)。
///
/// 必须每 body 独立缓冲:wgpu 的 `queue.write_buffer` 与 `queue.submit` 是两组
/// 独立队列提交,所有 write_buffer 在 submit 前执行完毕 —— 若多个 body 共用
/// 同一实例缓冲并反复写同一偏移,只有最后一次生效,其余 body 会拿错模型矩阵。
struct BodyGpu {
    vertex_buf: wgpu::Buffer,
    vertex_count: u32,
    instance_buf: wgpu::Buffer,
}

pub struct GpuRenderer {
    device: wgpu::Device,
    queue: wgpu::Queue,
    surface: wgpu::Surface<'static>,
    surface_config: wgpu::SurfaceConfiguration,
    pipeline: wgpu::RenderPipeline,
    depth_texture: wgpu::Texture,
    depth_view: wgpu::TextureView,
    /// 每个 body 的 GPU 资源(顶点 + 实例缓冲)。
    bodies: Vec<BodyGpu>,
    frame_buf: wgpu::Buffer,
    bind_group: wgpu::BindGroup,
}

impl GpuRenderer {
    /// 把一个 body 的三角网格拍平成交错顶点数组。
    pub fn collect_vertices(mesh: &[phy_demo::raster::Tri]) -> Vec<Vertex> {
        let mut verts: Vec<Vertex> = Vec::with_capacity(mesh.len() * 3);
        for tri in mesh {
            for i in 0..3 {
                verts.push(Vertex {
                    pos: tri.p[i],
                    nrm: tri.n[i],
                });
            }
        }
        verts
    }

    pub async fn new(window: Arc<Window>, meshes: &[Vec<phy_demo::raster::Tri>]) -> Self {
        let size = window.inner_size();
        let instance = wgpu::Instance::new(wgpu::InstanceDescriptor {
            backends: wgpu::Backends::PRIMARY,
            ..Default::default()
        });
        let surface = instance
            .create_surface(window)
            .expect("create wgpu surface");
        let adapter = instance
            .request_adapter(&wgpu::RequestAdapterOptions {
                power_preference: wgpu::PowerPreference::HighPerformance,
                compatible_surface: Some(&surface),
                force_fallback_adapter: false,
            })
            .await
            .expect("request adapter");
        let (device, queue) = adapter
            .request_device(
                &wgpu::DeviceDescriptor {
                    label: None,
                    required_features: wgpu::Features::empty(),
                    required_limits: wgpu::Limits::default(),
                },
                None,
            )
            .await
            .expect("request device");

        let surface_config = surface
            .get_default_config(&adapter, size.width.max(1), size.height.max(1))
            .expect("surface config");
        // 允许抓帧诊断(copy_texture_to_buffer 需要 COPY_SRC)。
        let surface_config = wgpu::SurfaceConfiguration {
            usage: surface_config.usage | wgpu::TextureUsages::COPY_SRC,
            ..surface_config
        };
        surface.configure(&device, &surface_config);

        // 每个 body 独立顶点 + 实例缓冲。
        let mut bodies: Vec<BodyGpu> = Vec::with_capacity(meshes.len());
        for mesh in meshes.iter() {
            let verts = Self::collect_vertices(mesh);
            let vertex_count = verts.len() as u32;
            let vertex_buf = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("vertices"),
                contents: bytemuck::cast_slice(&verts),
                usage: wgpu::BufferUsages::VERTEX,
            });
            let instance_buf = device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("instances"),
                size: std::mem::size_of::<Instance>() as u64,
                usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            });
            bodies.push(BodyGpu {
                vertex_buf,
                vertex_count,
                instance_buf,
            });
        }

        let frame_buf = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("frame uniform"),
            size: std::mem::size_of::<FrameUniform>() as u64,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("mesh.wgsl"),
            source: wgpu::ShaderSource::Wgsl(SHADER.into()),
        });

        let bind_group_layout =
            device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                label: Some("frame bgl"),
                entries: &[wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStages::VERTEX_FRAGMENT,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Uniform,
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                }],
            });

        let bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("frame bg"),
            layout: &bind_group_layout,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: frame_buf.as_entire_binding(),
            }],
        });

        let depth_texture = device.create_texture(&wgpu::TextureDescriptor {
            label: Some("depth"),
            size: wgpu::Extent3d {
                width: surface_config.width,
                height: surface_config.height,
                depth_or_array_layers: 1,
            },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: wgpu::TextureFormat::Depth24Plus,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
            view_formats: &[],
        });
        let depth_view = depth_texture.create_view(&wgpu::TextureViewDescriptor::default());

        let pipeline_layout =
            device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                label: None,
                bind_group_layouts: &[&bind_group_layout],
                push_constant_ranges: &[],
            });

        let pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: None,
            layout: Some(&pipeline_layout),
            vertex: wgpu::VertexState {
                module: &shader,
                entry_point: "vs_main",
                buffers: &[
                    wgpu::VertexBufferLayout {
                        array_stride: std::mem::size_of::<Vertex>() as u64,
                        step_mode: wgpu::VertexStepMode::Vertex,
                        attributes: &wgpu::vertex_attr_array![0 => Float32x3, 1 => Float32x3],
                    },
                    wgpu::VertexBufferLayout {
                        array_stride: std::mem::size_of::<Instance>() as u64,
                        step_mode: wgpu::VertexStepMode::Instance,
                        attributes: &wgpu::vertex_attr_array![
                            2 => Float32x4, 3 => Float32x4, 4 => Float32x4, 5 => Float32x4,
                            6 => Float32x4
                        ],
                    },
                ],
                compilation_options: wgpu::PipelineCompilationOptions::default(),
            },
            fragment: Some(wgpu::FragmentState {
                module: &shader,
                entry_point: "fs_main",
                targets: &[Some(wgpu::ColorTargetState {
                    format: surface_config.format,
                    blend: None,
                    write_mask: wgpu::ColorWrites::ALL,
                })],
                compilation_options: wgpu::PipelineCompilationOptions::default(),
            }),
            primitive: wgpu::PrimitiveState {
                topology: wgpu::PrimitiveTopology::TriangleList,
                // 关闭背面剔除:meshgen 中立方体 6 面绕序并非全部严格外 CCW
                // (部分面实际为顺时针),球面/胶囊经纬网格的上下半球绕序也可能
                // 不一致。开启 Back 剔除会在相机转到特定角度(如 180°)时把本应
                // 保留的正面错误剔除,造成"穿模"假象。这些几何体均为封闭凸体,
                // 配合深度写入关闭剔除即可获得正确遮挡结果,代价可忽略。
                cull_mode: None,
                front_face: wgpu::FrontFace::Ccw,
                ..Default::default()
            },
            depth_stencil: Some(wgpu::DepthStencilState {
                format: wgpu::TextureFormat::Depth24Plus,
                depth_write_enabled: true,
                depth_compare: wgpu::CompareFunction::Less,
                stencil: wgpu::StencilState::default(),
                bias: wgpu::DepthBiasState::default(),
            }),
            multisample: wgpu::MultisampleState::default(),
            multiview: None,
        });

        Self {
            device,
            queue,
            surface,
            surface_config,
            pipeline,
            depth_texture,
            depth_view,
            bodies,
            frame_buf,
            bind_group,
        }
    }

    /// 当前配置的表面尺寸(width, height)。
    pub fn size(&self) -> (u32, u32) {
        (self.surface_config.width, self.surface_config.height)
    }

    /// 窗口尺寸变化时(重新)配置 surface + 深度缓冲。
    pub fn resize(&mut self, width: u32, height: u32) {
        let w = width.max(1);
        let h = height.max(1);
        self.surface_config.width = w;
        self.surface_config.height = h;
        self.surface.configure(&self.device, &self.surface_config);
        self.depth_texture = self.device.create_texture(&wgpu::TextureDescriptor {
            label: Some("depth"),
            size: wgpu::Extent3d {
                width: w,
                height: h,
                depth_or_array_layers: 1,
            },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: wgpu::TextureFormat::Depth24Plus,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
            view_formats: &[],
        });
        self.depth_view = self.depth_texture.create_view(&wgpu::TextureViewDescriptor::default());
    }

    /// 用当前世界状态绘制一帧。
    ///
    /// 每个 body 已在 `new` 时拥有独立的顶点 + 实例缓冲;这里逐 body:
    /// 1) 把该 body 的模型矩阵 + 颜色写入它**自己**的实例缓冲(每缓冲每帧
    ///    只写一次,避免共用缓冲被 `write_buffer` 覆盖);
    /// 2) 绑定它自己的顶点/实例缓冲,单实例 draw。
    /// 这样每个 body 只绘制自己的三角形,且用各自的模型矩阵。
    pub fn render_frame(
        &mut self,
        vp: &Matrix4<f32>,
        body_models: &[Matrix4<f64>],
        body_colors: &[[f32; 3]],
        light: &Vector3<f32>,
        bg: [f32; 3],
    ) {
        let n = body_models.len().min(body_colors.len()).min(self.bodies.len());

        // WGSL 用列主序 mat4x4<f32>:nalgebra Matrix4 本身就是列主序,
        // 直接逐列铺开即可。
        let fu = FrameUniform {
            view_proj: [
                [vp[(0, 0)], vp[(1, 0)], vp[(2, 0)], vp[(3, 0)]],
                [vp[(0, 1)], vp[(1, 1)], vp[(2, 1)], vp[(3, 1)]],
                [vp[(0, 2)], vp[(1, 2)], vp[(2, 2)], vp[(3, 2)]],
                [vp[(0, 3)], vp[(1, 3)], vp[(2, 3)], vp[(3, 3)]],
            ],
            light: [light.x, light.y, light.z, 0.0],
            bg: [bg[0], bg[1], bg[2], 1.0],
        };
        self.queue.write_buffer(&self.frame_buf, 0, bytemuck::bytes_of(&fu));

        let frame = self
            .surface
            .get_current_texture()
            .expect("get current texture");
        let view = frame
            .texture
            .create_view(&wgpu::TextureViewDescriptor::default());

        let mut encoder = self
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor { label: None });
        {
            let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: None,
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: &view,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Clear(wgpu::Color {
                            r: bg[0] as f64,
                            g: bg[1] as f64,
                            b: bg[2] as f64,
                            a: 1.0,
                        }),
                        store: wgpu::StoreOp::Store,
                    },
                })],
                depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                    view: &self.depth_view,
                    depth_ops: Some(wgpu::Operations {
                        load: wgpu::LoadOp::Clear(1.0),
                        store: wgpu::StoreOp::Store,
                    }),
                    stencil_ops: None,
                }),
                occlusion_query_set: None,
                timestamp_writes: None,
            });
            pass.set_pipeline(&self.pipeline);
            pass.set_bind_group(0, &self.bind_group, &[]);

            for i in 0..n {
                let m = body_models[i].cast::<f32>();
                let c = &body_colors[i];
                let inst = Instance {
                    // nalgebra Matrix4 是列主序存储,WGSL mat4x4 也是列主序。
                    // 必须按**列**(col)逐元素铺开,不能按行(row)——否则矩阵被
                    // 隐式转置,平移变成旋转/缩放混合,物体塌缩到原点呈放射状。
                    model: [
                        [m[(0, 0)], m[(1, 0)], m[(2, 0)], m[(3, 0)]], // 列0
                        [m[(0, 1)], m[(1, 1)], m[(2, 1)], m[(3, 1)]], // 列1
                        [m[(0, 2)], m[(1, 2)], m[(2, 2)], m[(3, 2)]], // 列2
                        [m[(0, 3)], m[(1, 3)], m[(2, 3)], m[(3, 3)]], // 列3(平移)
                    ],
                    color: [c[0], c[1], c[2], 1.0],
                };
                self.queue
                    .write_buffer(&self.bodies[i].instance_buf, 0, bytemuck::bytes_of(&inst));

                pass.set_vertex_buffer(0, self.bodies[i].vertex_buf.slice(..));
                pass.set_vertex_buffer(1, self.bodies[i].instance_buf.slice(..));
                pass.draw(0..self.bodies[i].vertex_count, 0..1);
            }
        }
        self.queue.submit(std::iter::once(encoder.finish()));
        if std::env::var_os("GPU_DUMP").is_some() {
            self.dump(&frame.texture);
        }
        frame.present();
    }

    /// 把当前 surface 纹理拷回 CPU 并写成 `gpu_dump.png`(诊断用,需 surface
    /// 配置含 COPY_SRC)。
    fn dump(&self, tex: &wgpu::Texture) {
        let (w, h) = self.size();
        let buf = self.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("dump"),
            size: (w as u64 * h as u64 * 4).max(4),
            usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
            mapped_at_creation: false,
        });
        let mut enc = self.device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: None,
        });
        enc.copy_texture_to_buffer(
            wgpu::ImageCopyTexture {
                texture: tex,
                mip_level: 0,
                origin: wgpu::Origin3d::ZERO,
                aspect: wgpu::TextureAspect::All,
            },
            wgpu::ImageCopyBuffer {
                buffer: &buf,
                layout: wgpu::ImageDataLayout {
                    offset: 0,
                    bytes_per_row: Some(w * 4),
                    rows_per_image: Some(h),
                },
            },
            wgpu::Extent3d {
                width: w,
                height: h,
                depth_or_array_layers: 1,
            },
        );
        self.queue.submit(Some(enc.finish()));
        let slice = buf.slice(..);
        slice.map_async(wgpu::MapMode::Read, |_| {});
        self.device.poll(wgpu::Maintain::Wait);
        let data = slice.get_mapped_range();
        // 用 png crate 写出(RGBA8),与 main.rs 的 headless 导出保持一致。
        let mut png_enc = png::Encoder::new(
            std::io::BufWriter::new(std::fs::File::create("gpu_dump.png").expect("create png")),
            w,
            h,
        );
        png_enc.set_color(png::ColorType::Rgba);
        png_enc.set_depth(png::BitDepth::Eight);
        let mut out = png_enc.write_header().expect("png header");
        out.write_image_data(&data).expect("png data");
    }
}

/// WGSL 着色器:顶点做 VP*model 变换 + 朗伯光照;片段输出光照后的颜色。
const SHADER: &str = r#"
struct Instance {
    model : mat4x4<f32>,
    color : vec4<f32>,
};
struct Frame {
    view_proj : mat4x4<f32>,
    light : vec4<f32>,
    bg : vec4<f32>,
};
@group(0) @binding(0) var<uniform> frame : Frame;

struct VSOut {
    @builtin(position) pos : vec4<f32>,
    @location(0) normal : vec3<f32>,
    @location(1) color : vec3<f32>,
};

@vertex
fn vs_main(
    @location(0) in_pos : vec3<f32>,
    @location(1) in_nrm : vec3<f32>,
    @location(2) inst_m0 : vec4<f32>,
    @location(3) inst_m1 : vec4<f32>,
    @location(4) inst_m2 : vec4<f32>,
    @location(5) inst_m3 : vec4<f32>,
    @location(6) inst_color : vec4<f32>,
) -> VSOut {
    var m : mat4x4<f32>;
    m[0] = inst_m0; m[1] = inst_m1; m[2] = inst_m2; m[3] = inst_m3;
    let world = m * vec4<f32>(in_pos, 1.0);
    var out : VSOut;
    out.pos = frame.view_proj * world;
    // 法线用模型左上 3x3(视觉用途,忽略非均匀缩放)。
    let n = normalize((m * vec4<f32>(in_nrm, 0.0)).xyz);
    out.normal = n;
    out.color = inst_color.rgb;
    return out;
}

@fragment
fn fs_main(in : VSOut) -> @location(0) vec4<f32> {
    let diff = max(dot(normalize(in.normal), normalize(frame.light.xyz)), 0.0);
    let ambient = 0.45;
    let c = in.color * (ambient + 0.55 * diff);
    return vec4<f32>(c, 1.0);
}
"#;
