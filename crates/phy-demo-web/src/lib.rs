//! phy-demo Web 版:把多物理场 demo 渲染到 <canvas>(wasm + wasm-bindgen)。
//!
//! 与桌面版共用 phy-demo 的 Scene/Framebuffer/Camera 渲染逻辑,
//! 仅把"窗口/事件/呈现"替换为 Web API(canvas + 键盘监听 + requestAnimationFrame)。
//!
//! GPU 加速(W1, §5.7.4):仅当 `target_arch="wasm32" + feature="gpu"` 时,
//! `gpu` 模块提供 WebGPU compute 后端。桌面端 / 默认 wasm 构建不进入该模块,
//! 因此不拉入 wgpu,避开 MinGW 链接崩溃(M3 决策)。

use phy_demo::{Camera, DemoMode, Framebuffer, Scene};
use phy_math::Vec3 as V3;
use wasm_bindgen::prelude::*;

// P5 验收:W4/W5 wgsl 内核的纯 Rust 确定性串行参考(非门控,host `cargo test` 可跑),
// 用于在无 WebGPU adapter 的本机构建下验证 wgsl 数学自洽性。
pub mod gpu_ref;

// M1/G1 交付:CPU 生产实现 vs wgsl 参考(已逐公式对齐)的量化对比 harness(非门控,host `cargo test` 可跑),
// 为真实 WebGPU adapter 的 CPU↔GPU 误差对比建立浮点精度基线 + 数据导出接口。
pub mod gpu_accuracy;

// M3 内核全量审批:GPU 计算着色器输出 vs CPU 生产实现逐像素/逐单元数值对比(OPTIC/CAUSTIC;
// SPH/GRANULAR 已在 gpu_accuracy 覆盖),在真实 adapter 上确认 wgsl 正确复刻 GLSL/CPU 算法。
#[cfg(feature = "gpu")]
pub mod gpu_audit;

// W1: WebGPU compute 后端。wasm(浏览器)+gpu feature 是正式路径;native(桌面 wgpu 原生后端,
// 需 MSVC 工具链而非 MinGW,规避 M3 的 corrupt .drectve 崩溃)在 gpu feature 下也编译,
// 用于真机 adapter 的 CPU↔GPU 逐粒子误差对比(G1 验收)。其余构建忽略。
#[cfg(feature = "gpu")]
pub mod gpu;

// 用 console 输出 + panic hook,便于浏览器调试。
#[wasm_bindgen]
extern "C" {
    #[wasm_bindgen(js_namespace = console)]
    fn log(s: &str);
}

struct State {
    scene: Scene,
    cam: Camera,
    fb: Framebuffer,
    paused: bool,
}

#[wasm_bindgen]
pub struct DemoApp {
    // 用 Rc<RefCell> 持有可变状态,使 `frame_gpu` 的异步 future 可持有 'static 副本
    // (WebGPU step 必须在 async 内借可变 world,而 future_to_promise 要求 'static)。
    state: std::rc::Rc<std::cell::RefCell<State>>,
    canvas: web_sys::HtmlCanvasElement,
    ctx: web_sys::CanvasRenderingContext2d,
    #[cfg(all(target_arch = "wasm32", feature = "gpu"))]
    gpu: std::rc::Rc<std::cell::RefCell<Option<crate::gpu::GpuContext>>>,
    #[cfg(all(target_arch = "wasm32", feature = "gpu"))]
    gpu_mode: bool,
    #[cfg(all(target_arch = "wasm32", feature = "gpu"))]
    gpu_strategy: crate::gpu::GpuStrategy,
}

#[wasm_bindgen]
impl DemoApp {
    /// 在给定 canvas 上创建 demo 应用。
    #[wasm_bindgen(constructor)]
    pub fn new(canvas: web_sys::HtmlCanvasElement) -> Result<DemoApp, JsValue> {
        console_error_panic_hook::set_once();
        let ctx = canvas
            .get_context("2d")
            .map_err(|_| JsValue::from_str("get_context 失败"))?
            .unwrap()
            .dyn_into::<web_sys::CanvasRenderingContext2d>()
            .map_err(|_| JsValue::from_str("2d context 转换失败"))?;
        let w = canvas.width();
        let h = canvas.height();
        let state = State {
            scene: Scene::new(),
            cam: Camera::default(),
            fb: Framebuffer::new(w.max(1), h.max(1)),
            paused: false,
        };
        Ok(DemoApp {
            state: std::rc::Rc::new(std::cell::RefCell::new(state)),
            canvas,
            ctx,
            #[cfg(all(target_arch = "wasm32", feature = "gpu"))]
            gpu: std::rc::Rc::new(std::cell::RefCell::new(None)),
            #[cfg(all(target_arch = "wasm32", feature = "gpu"))]
            gpu_mode: false,
            #[cfg(all(target_arch = "wasm32", feature = "gpu"))]
            gpu_strategy: crate::gpu::GpuStrategy::Auto,
        })
    }

    /// 处理键盘事件(逻辑键名)。支持桌面版全部键位。
    pub fn key(&mut self, key: &str) {
        let mut st = (*self.state).borrow_mut();
        let st = &mut *st;
        match key {
            "1" => st.scene.set_mode(DemoMode::Rigid),
            "2" => st.scene.set_mode(DemoMode::Fluid),
            "3" => st.scene.set_mode(DemoMode::Heat),
            "4" => st.scene.set_mode(DemoMode::Soft),
            "5" => st.scene.set_mode(DemoMode::Optics),
            "6" => st.scene.set_mode(DemoMode::FluidHeat),
            "7" => st.scene.set_mode(DemoMode::Em),
            "8" => st.scene.set_mode(DemoMode::Grav),
            "9" => st.scene.set_mode(DemoMode::Wave),
            "0" => st.scene.set_mode(DemoMode::Acoustic),
            "a" | "A" => st.scene.set_mode(DemoMode::All),
            "p" | "P" => st.paused = !st.paused,
            "r" | "R" => st.scene.reset(),
            "o" | "O" => {
                // 循环模式。
                let next = match st.scene.mode {
                    DemoMode::Rigid => DemoMode::Fluid,
                    DemoMode::Fluid => DemoMode::Heat,
                    DemoMode::Heat => DemoMode::Soft,
                    DemoMode::Soft => DemoMode::Optics,
                    DemoMode::Optics => DemoMode::FluidHeat,
                    DemoMode::FluidHeat => DemoMode::Em,
                    DemoMode::Em => DemoMode::Grav,
                    DemoMode::Grav => DemoMode::Wave,
                    DemoMode::Wave => DemoMode::Acoustic,
                    DemoMode::Acoustic => DemoMode::All,
                    DemoMode::All => DemoMode::Rigid,
                };
                st.scene.set_mode(next);
            }
            "i" | "I" => {
                log(&format!(
                    "[demo] mode={} bodies={} steps={} paused={}",
                    st.scene.mode.name(),
                    st.scene.body_count(),
                    st.scene.steps,
                    st.paused
                ));
            }
            "F5" => {
                // 存档到 localStorage(字符串 JSON)。
                match st.scene.save_string() {
                    Ok(json) => match web_sys::window() {
                        Some(win) => match win.local_storage() {
                            Ok(store_opt) => match store_opt {
                                Some(store) => match store.set_item("world_save", &json) {
                                    Ok(_) => log("[demo] 已存档到 localStorage (world_save)"),
                                    Err(e) => log(&format!("[demo] 存档写入失败: {:?}", e)),
                                },
                                None => log("[demo] 无 Storage"),
                            },
                            Err(e) => log(&format!("[demo] local_storage 失败: {:?}", e)),
                        },
                        None => log("[demo] 无 window"),
                    },
                    Err(e) => log(&format!("[demo] 存档序列化失败: {}", e)),
                }
            }
            "F9" => {
                // 从 localStorage 读档。
                let json_opt: Option<String> = web_sys::window()
                    .and_then(|w| w.local_storage().ok())
                    .flatten()
                    .and_then(|s| s.get_item("world_save").ok())
                    .flatten();
                match json_opt {
                    Some(json) => match st.scene.load_string(&json) {
                        Ok(_) => log("[demo] 已读档 (world_save)"),
                        Err(e) => log(&format!("[demo] 读档失败: {}", e)),
                    },
                    None => log("[demo] 无存档"),
                }
            }
            _ => {}
        }
    }

    /// 鼠标拖拽旋转。
    pub fn drag(&mut self, dx: f32, dy: f32) {
        let mut st = (*self.state).borrow_mut();
        st.cam.yaw -= dx * 0.005;
        st.cam.pitch += dy * 0.005;
        let lim = 1.5_f32;
        st.cam.pitch = st.cam.pitch.clamp(-lim, lim);
    }

    /// 滚轮缩放。
    pub fn zoom(&mut self, delta: f32) {
        let mut st = (*self.state).borrow_mut();
        st.cam.distance *= (1.0 + delta * 0.001).clamp(0.5, 2.0);
        st.cam.distance = st.cam.distance.clamp(3.0, 200.0);
    }

    /// 推进一帧物理并渲染到 canvas。由 requestAnimationFrame 循环调用。
    pub fn frame(&mut self) {
        let mut st = (*self.state).borrow_mut();
        let st = &mut *st;
        if !st.paused {
            st.scene.step();
        }
        st.fb.clear();
        st.scene.render(&mut st.fb, &st.cam);
        let bg = st.fb.pixels.first().copied().unwrap_or(0);
        let non_bg = st
            .fb
            .pixels
            .iter()
            .filter(|&&p| p != bg)
            .count();
        // 诊断:统计 fb.pixels 中纯红(0xFFFF0000)和背景色的数量
        let red_count = st.fb.pixels.iter().filter(|&&p| p == 0xFFFF0000u32).count();
        let bg_count = st.fb.pixels.iter().filter(|&&p| p == bg).count();
        log(&format!(
            "[frame] mode={} non_bg={} red_px={} bg_px={} first={:08x}",
            st.scene.mode.name(),
            non_bg,
            red_count,
            bg_count,
            bg
        ));
        present(&self.canvas, &self.ctx, &st.fb);
    }

    /// 设置运行时 GPU 切换开关(仅 wasm + gpu feature 构建有效)。
    ///
    /// 打开后 JS 端应使用 `await app.frame_gpu()` 驱动渲染循环(因为 GPU 步是异步的),
    /// 关闭时继续用 `app.frame()`(纯 CPU 步进)。`Scene` 的其余子系统与全部 `couple`
    /// 阶段始终走 CPU,跨子系统耦合矩阵保持完整。
    #[cfg(all(target_arch = "wasm32", feature = "gpu"))]
    pub fn set_gpu_mode(&mut self, on: bool) {
        self.gpu_mode = on;
    }

    /// 查询当前 GPU 切换开关状态(仅 wasm + gpu feature 构建有效)。
    #[cfg(all(target_arch = "wasm32", feature = "gpu"))]
    pub fn get_gpu_mode(&self) -> bool {
        self.gpu_mode
    }

    /// 设置 GPU 加速策略(§2.3:默认 Auto)。`Auto`=有 adapter 用 GPU 否则回退 CPU;
    /// `ForceGpu`=强制 GPU(无 adapter 报错);`ForceCpu`=强制 CPU(不初始化 GPU)。
    #[cfg(all(target_arch = "wasm32", feature = "gpu"))]
    pub fn set_gpu_strategy(&mut self, strategy: crate::gpu::GpuStrategy) {
        self.gpu_strategy = strategy;
    }

    /// 查询当前 GPU 加速策略(仅 wasm + gpu feature 构建有效)。
    #[cfg(all(target_arch = "wasm32", feature = "gpu"))]
    pub fn get_gpu_strategy(&self) -> crate::gpu::GpuStrategy {
        self.gpu_strategy
    }

    /// 异步帧:当 GPU 模式开启时,流体 / 颗粒子系统的力学 step 走 GPU compute 写回
    /// `World<f64>`,其余子系统与 `couple` 走 CPU。由 JS 端 `await app.frame_gpu()` 驱动。
    #[cfg(all(target_arch = "wasm32", feature = "gpu"))]
    pub fn frame_gpu(&mut self) -> js_sys::Promise {
        use wasm_bindgen::JsCast;
        // 把可变状态克隆进 Rc,使 async future 拥有 'static 副本(future_to_promise 要求)。
        let st_rc = std::rc::Rc::clone(&self.state);
        let gpu_rc = std::rc::Rc::clone(&self.gpu);
        let canvas = self.canvas.clone();
        let ctx2d = self.ctx.clone();
        let fut = async move {
            // 惰性初始化 GPU 上下文(持久化在 Rc<RefCell> 中)。按策略决定:
            // ForceCpu 不初始化 GPU,直接走 CPU 帧(等价于 frame())。
            if gpu_rc.borrow().is_none() {
                match crate::gpu::GpuContext::init_with(self.gpu_strategy).await {
                    Ok(ctx) => {
                        *gpu_rc.borrow_mut() = Some(ctx);
                    }
                    Err(e) => {
                        // 策略要求回退 CPU(ForceCpu 或无 adapter 的 Auto):走纯 CPU 帧。
                        let mut st = (*st_rc).borrow_mut();
                        let st = &mut *st;
                        if !st.paused {
                            st.scene.step(1.0f32 / 60.0);
                        }
                        st.fb.clear();
                        st.scene.render(&mut st.fb, &st.cam);
                        present(&canvas, &ctx2d, &st.fb);
                        return Ok::<(), String>(());
                    }
                }
            }
            let ctx_guard = gpu_rc.borrow();
            let ctx = ctx_guard.as_ref().unwrap();
            {
                let mut st = (*st_rc).borrow_mut();
                let st = &mut *st;
                if !st.paused {
                    crate::gpu::step_world_gpu(ctx, &mut st.scene.world, 1.0f32 / 60.0).await?;
                }
                st.fb.clear();
                st.scene.render(&mut st.fb, &st.cam);
                present(&canvas, &ctx2d, &st.fb);
            }
            Ok::<(), String>(())
        };
        wasm_bindgen_futures::future_to_promise(async move {
            match fut.await {
                Ok(_) => Ok(JsValue::NULL),
                Err(e) => Err(js_sys::Error::new(&e).into()),
            }
        })
        .unchecked_into()
    }

    /// 调试用:把整个 canvas 填成红色,验证 present 管线通。
    pub fn debug_fill(&mut self) {
        let mut st = (*self.state).borrow_mut();
        for p in st.fb.pixels.iter_mut() {
            *p = 0xFFFF0000u32; // ARGB red
        }
        present(&self.canvas, &self.ctx, &st.fb);
    }

    /// M1 验收入口(仅 wasm + gpu feature):申请真实 WebGPU adapter 并返回其身份。
    /// 浏览器里 `app.adapter_info().then(s => console.log(s))` 应看到
    /// `{"ok":true,"vendor":"...","architecture":"...","device":"...","description":"..."}`。
    #[cfg(all(target_arch = "wasm32", feature = "gpu"))]
    pub fn adapter_info(&self) -> js_sys::Promise {
        use wasm_bindgen::JsCast;
        let fut = async move { crate::gpu::adapter_info().await };
        wasm_bindgen_futures::future_to_promise(async move {
            match fut.await {
                Ok(s) => Ok(js_sys::JsString::from(s.as_str()).into()),
                Err(e) => Err(js_sys::Error::new(&e).into()),
            }
        })
        .unchecked_into()
    }

    /// W1 验证入口(仅 wasm + gpu feature):跑 GPU compute 自测,返回 Promise<string>。
    /// 浏览器里 `app.gpu_self_test().then(s => console.log(s))` 应看到
    /// "W1 gpu self-test ok=true out=[1.0,4.0,9.0,16.0,25.0]"。
    #[cfg(all(target_arch = "wasm32", feature = "gpu"))]
    pub fn gpu_self_test(&self) -> js_sys::Promise {
        use wasm_bindgen::JsCast;
        let fut = async move { crate::gpu::gpu_self_test().await };
        wasm_bindgen_futures::future_to_promise(async move {
            match fut.await {
                Ok(s) => Ok(js_sys::JsString::from(s.as_str()).into()),
                Err(e) => Err(js_sys::Error::new(&e).into()),
            }
        })
        .unchecked_into()
    }

    /// W2 验证入口(仅 wasm + gpu feature):用 GPU 跑光学实时近似渲染自测。
    /// 浏览器里 `app.optic_self_test().then(s => console.log(s))` 应看到
    /// "W2 optic ok: center=(...) bg=(...)".
    #[cfg(all(target_arch = "wasm32", feature = "gpu"))]
    pub fn optic_self_test(&self) -> js_sys::Promise {
        use wasm_bindgen::JsCast;
        let fut = async move { crate::gpu::optic_self_test().await };
        wasm_bindgen_futures::future_to_promise(async move {
            match fut.await {
                Ok(s) => Ok(js_sys::JsString::from(s.as_str()).into()),
                Err(e) => Err(js_sys::Error::new(&e).into()),
            }
        })
        .unchecked_into()
    }

    /// W3 验证入口(仅 wasm + gpu feature):用 GPU 跑焦散 march 自测。
    /// 浏览器里 `app.caustics_self_test().then(s => console.log(s))` 应看到
    /// "W3 caustics ok: grid_n=16 max=... sum=...".
    #[cfg(all(target_arch = "wasm32", feature = "gpu"))]
    pub fn caustics_self_test(&self) -> js_sys::Promise {
        use wasm_bindgen::JsCast;
        let fut = async move { crate::gpu::caustics_self_test().await };
        wasm_bindgen_futures::future_to_promise(async move {
            match fut.await {
                Ok(s) => Ok(js_sys::JsString::from(s.as_str()).into()),
                Err(e) => Err(js_sys::Error::new(&e).into()),
            }
        })
        .unchecked_into()
    }

    /// W4 验证入口(仅 wasm + gpu feature):用 GPU 跑 SPH 密度/受力自测。
    /// 浏览器里 `app.sph_self_test().then(s => console.log(s))` 应看到
    /// "W4 sph ok: n=... mean_rho=... rest=... ratio=... finite=... acc0=(...)".
    #[cfg(all(target_arch = "wasm32", feature = "gpu"))]
    pub fn sph_self_test(&self) -> js_sys::Promise {
        use wasm_bindgen::JsCast;
        let fut = async move { crate::gpu::sph_self_test().await };
        wasm_bindgen_futures::future_to_promise(async move {
            match fut.await {
                Ok(s) => Ok(js_sys::JsString::from(s.as_str()).into()),
                Err(e) => Err(js_sys::Error::new(&e).into()),
            }
        })
        .unchecked_into()
    }

    /// W5 验证入口(仅 wasm + gpu feature):用 GPU 跑颗粒 PBD 接触投影自测。
    /// 浏览器里 `app.granular_self_test().then(s => console.log(s))` 应看到
    /// "W5 granular ok: n=... npairs=... min_gap=... overlaps=... finite=...".
    #[cfg(all(target_arch = "wasm32", feature = "gpu"))]
    pub fn granular_self_test(&self) -> js_sys::Promise {
        use wasm_bindgen::JsCast;
        let fut = async move { crate::gpu::granular_self_test().await };
        wasm_bindgen_futures::future_to_promise(async move {
            match fut.await {
                Ok(s) => Ok(js_sys::JsString::from(s.as_str()).into()),
                Err(e) => Err(js_sys::Error::new(&e).into()),
            }
        })
        .unchecked_into()
    }
}

/// 把 Framebuffer 的像素写到 canvas ImageData。
fn present(
    canvas: &web_sys::HtmlCanvasElement,
    ctx: &web_sys::CanvasRenderingContext2d,
    fb: &Framebuffer,
) {
    let w = fb.width as u32;
    let h = fb.height as u32;
    if canvas.width() != w || canvas.height() != h {
        canvas.set_width(w);
        canvas.set_height(h);
    }
    // 构造 RGBA buffer(Framebuffer 是 ARGB packed,需转 RGBA)。
    let mut rgba = Vec::with_capacity((w * h * 4) as usize);
    for &px in &fb.pixels {
        let a = ((px >> 24) & 0xff) as u8;
        let r = ((px >> 16) & 0xff) as u8;
        let g = ((px >> 8) & 0xff) as u8;
        let b = (px & 0xff) as u8;
        // 若 alpha 为 0,用不透明(背景已填充)。
        let a = if a == 0 { 255 } else { a };
        rgba.push(r);
        rgba.push(g);
        rgba.push(b);
        rgba.push(a);
    }
    let clamped = wasm_bindgen::Clamped(&rgba[..]);
    let image_data = match web_sys::ImageData::new_with_u8_clamped_array_and_sh(clamped, w, h) {
        Ok(d) => d,
        Err(e) => {
            log(&format!("[present] ImageData 创建失败: {:?}", e));
            return;
        }
    };
    if let Err(e) = ctx.put_image_data(&image_data, 0.0, 0.0) {
        log(&format!("[present] put_image_data 失败: {:?}", e));
    }
}

// 让 V3 在本模块可用(供将来扩展,避免未使用警告)。
#[allow(dead_code)]
fn _use_v3() -> V3<f64> {
    V3::new(0.0, 0.0, 0.0)
}

/// 离屏渲染(无 canvas):构造指定模式的场景,step 若干帧后返回 RGBA 像素缓冲。
/// 供 wasm-bindgen-test 在 Node 下做无 GUI 逻辑自测。
pub fn render_offscreen(mode: DemoMode, steps: usize, w: u32, h: u32) -> Vec<u8> {
    let mut scene = Scene::new();
    scene.set_mode(mode);
    let cam = Camera::default();
    let mut fb = Framebuffer::new(w.max(1), h.max(1));
    for _ in 0..steps {
        scene.step();
    }
    fb.clear();
    scene.render(&mut fb, &cam);
    // ARGB -> RGBA
    let mut rgba = Vec::with_capacity((fb.pixels.len() * 4) as usize);
    for &px in &fb.pixels {
        let r = (px >> 16) & 0xff;
        let g = (px >> 8) & 0xff;
        let b = px & 0xff;
        let a = (px >> 24) & 0xff;
        rgba.push(r as u8);
        rgba.push(g as u8);
        rgba.push(b as u8);
        rgba.push(if a == 0 { 255 } else { a as u8 });
    }
    rgba
}

#[cfg(test)]
mod tests {
    use super::render_offscreen;
    use phy_demo::DemoMode;

    #[test]
    fn all_web_modes_render_nonempty() {
        let modes = [
            DemoMode::Rigid,
            DemoMode::Fluid,
            DemoMode::Heat,
            DemoMode::Soft,
            DemoMode::Optics,
            DemoMode::FluidHeat,
            DemoMode::Em,
            DemoMode::Grav,
            DemoMode::Wave,
            DemoMode::Acoustic,
            DemoMode::All,
        ];
        for m in modes {
            let px = render_offscreen(m, 4, 200, 150);
            let mut non_bg = 0usize;
            for i in 0..px.len() / 4 {
                let r = px[i * 4];
                let g = px[i * 4 + 1];
                let b = px[i * 4 + 2];
                // 背景色 = pack(0.05,0.07,0.10) = (13,18,26)。
                if !(r == 13 && g == 18 && b == 26) {
                    non_bg += 1;
                }
            }
            assert!(non_bg > 0, "模式 {:?} 渲染为空", m);
        }
    }
}

