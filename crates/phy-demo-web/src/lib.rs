//! phy-demo Web 版:把多物理场 demo 渲染到 <canvas>(wasm + wasm-bindgen)。
//!
//! 与桌面版共用 phy-demo 的 Scene/Framebuffer/Camera 渲染逻辑,
//! 仅把"窗口/事件/呈现"替换为 Web API(canvas + 键盘监听 + requestAnimationFrame)。

use phy_demo::{Camera, DemoMode, Framebuffer, Scene};
use phy_math::Vec3 as V3;
use wasm_bindgen::prelude::*;

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
    state: State,
    canvas: web_sys::HtmlCanvasElement,
    ctx: web_sys::CanvasRenderingContext2d,
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
        Ok(DemoApp { state, canvas, ctx })
    }

    /// 处理键盘事件(逻辑键名)。支持桌面版全部键位。
    pub fn key(&mut self, key: &str) {
        match key {
            "1" => self.state.scene.set_mode(DemoMode::Rigid),
            "2" => self.state.scene.set_mode(DemoMode::Fluid),
            "3" => self.state.scene.set_mode(DemoMode::Heat),
            "4" => self.state.scene.set_mode(DemoMode::Soft),
            "5" => self.state.scene.set_mode(DemoMode::Optics),
            "6" => self.state.scene.set_mode(DemoMode::FluidHeat),
            "7" => self.state.scene.set_mode(DemoMode::Em),
            "8" => self.state.scene.set_mode(DemoMode::Grav),
            "9" => self.state.scene.set_mode(DemoMode::Wave),
            "0" => self.state.scene.set_mode(DemoMode::Acoustic),
            "a" | "A" => self.state.scene.set_mode(DemoMode::All),
            "p" | "P" => self.state.paused = !self.state.paused,
            "r" | "R" => self.state.scene.reset(),
            "o" | "O" => {
                // 循环模式。
                let next = match self.state.scene.mode {
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
                self.state.scene.set_mode(next);
            }
            "i" | "I" => {
                log(&format!(
                    "[demo] mode={} bodies={} steps={} paused={}",
                    self.state.scene.mode.name(),
                    self.state.scene.body_count(),
                    self.state.scene.steps,
                    self.state.paused
                ));
            }
            "F5" => {
                // 存档到 localStorage(字符串 JSON)。
                match self.state.scene.save_string() {
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
                    Some(json) => match self.state.scene.load_string(&json) {
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
        self.state.cam.yaw -= dx * 0.005;
        self.state.cam.pitch += dy * 0.005;
        let lim = 1.5_f32;
        self.state.cam.pitch = self.state.cam.pitch.clamp(-lim, lim);
    }

    /// 滚轮缩放。
    pub fn zoom(&mut self, delta: f32) {
        self.state.cam.distance *= (1.0 + delta * 0.001).clamp(0.5, 2.0);
        self.state.cam.distance = self.state.cam.distance.clamp(3.0, 200.0);
    }

    /// 推进一帧物理并渲染到 canvas。由 requestAnimationFrame 循环调用。
    pub fn frame(&mut self) {
        if !self.state.paused {
            self.state.scene.step();
        }
        self.state.fb.clear();
        self.state
            .scene
            .render(&mut self.state.fb, &self.state.cam);
        let bg = self.state.fb.pixels.first().copied().unwrap_or(0);
        let non_bg = self
            .state
            .fb
            .pixels
            .iter()
            .filter(|&&p| p != bg)
            .count();
        // 诊断:统计 fb.pixels 中纯红(0xFFFF0000)和背景色的数量
        let red_count = self.state.fb.pixels.iter().filter(|&&p| p == 0xFFFF0000u32).count();
        let bg_count = self.state.fb.pixels.iter().filter(|&&p| p == bg).count();
        log(&format!(
            "[frame] mode={} non_bg={} red_px={} bg_px={} first={:08x}",
            self.state.scene.mode.name(),
            non_bg,
            red_count,
            bg_count,
            bg
        ));
        present(&self.canvas, &self.ctx, &self.state.fb);
    }

    /// 调试用:把整个 canvas 填成红色,验证 present 管线通。
    pub fn debug_fill(&mut self) {
        for p in self.state.fb.pixels.iter_mut() {
            *p = 0xFFFF0000u32; // ARGB red
        }
        present(&self.canvas, &self.ctx, &self.state.fb);
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

