//! # Physics Sandbox Parkour —— 基于物理引擎 SDK 的 3D 小游戏 demo(窗口/渲染层)
//!
//! 游戏核心逻辑见 `lib.rs` 的 [`phy_game_demo::Game`];本文件负责 winit 窗口循环
//! 与渲染。
//!
//! - MSVC 工具链(`target_env = "msvc"`):走 `render_wgpu` 的 GPU 光栅化(D3D12),
//!   每帧仅上传实例矩阵 + 相机矩阵,绘制交 GPU,流畅度高。
//! - GNU/MinGW 工具链:走 softbuffer 软件光栅化(无需 GPU,跨工具链可链)。
//! - `--headless` 同样走软件光栅离屏导出 PNG,用于 CI / 无显示服务器。
//!
//! 玩法:
//! - `W/A/S/D`:相对相机方向的水平移动
//! - `Space`:跳跃
//! - `鼠标拖拽`:旋转相机视角
//! - `Q/E`:拉近/拉远
//! - `R`:重置角色到出生点
//! - `Esc`:退出
//!
//! 切换 MSVC 工具链即可启用 GPU 渲染:
//! `rustup default stable-x86_64-pc-windows-msvc`

#[cfg(not(target_env = "msvc"))]
use std::num::NonZeroU32;
use std::sync::Arc;
use std::time::{Duration, Instant};

use phy_demo::raster::Framebuffer;
use phy_game_demo::winit_key_codes::KeyCode;
use phy_math::na::Vector3;
use phy_game_demo::Game;
use winit::application::ApplicationHandler;
use winit::event::{ElementState, KeyEvent, MouseButton, WindowEvent};
use winit::event_loop::{ActiveEventLoop, ControlFlow, EventLoop};
use winit::keyboard::PhysicalKey::Code;
use winit::window::{CursorGrabMode, Window, WindowId};

/// 目标帧率。软件光栅化较慢,限制到 ~30fps 既流畅又留 CPU 给输入事件;
/// GPU 路径不受限,但这个节流同样避免无谓重绘。
const FRAME_DUR: Duration = Duration::from_millis(33);

#[cfg(target_env = "msvc")]
type GpuBackend = phy_game_demo::render_wgpu::GpuRenderer;

struct App {
    game: Option<Game>,
    window: Option<Arc<Window>>,
    /// softbuffer 上下文(仅 GNU 路径使用)。
    #[cfg(not(target_env = "msvc"))]
    context: Option<softbuffer::Context<Arc<Window>>>,
    /// softbuffer 表面(仅 GNU 路径使用)。
    #[cfg(not(target_env = "msvc"))]
    surface: Option<softbuffer::Surface<Arc<Window>, Arc<Window>>>,
    /// 复用同一块帧缓冲(GNU 路径),避免每帧重新分配 57 万像素 + 深度缓冲。
    #[cfg(not(target_env = "msvc"))]
    fb: Option<Framebuffer>,
    /// GPU 渲染后端(仅 MSVC 路径使用)。
    #[cfg(target_env = "msvc")]
    gpu: Option<GpuBackend>,
    last_frame: Instant,
    /// 已渲染帧数(用于 GPU_DUMP 诊断模式自动退出)。
    frames: u32,
}

impl Default for App {
    fn default() -> Self {
        Self {
            game: None,
            window: None,
            #[cfg(not(target_env = "msvc"))]
            context: None,
            #[cfg(not(target_env = "msvc"))]
            surface: None,
            #[cfg(not(target_env = "msvc"))]
            fb: None,
            #[cfg(target_env = "msvc")]
            gpu: None,
            last_frame: Instant::now(),
            frames: 0,
        }
    }
}

impl App {
    fn ensure_surface(&mut self, event_loop: &ActiveEventLoop) {
        if self.window.is_some() {
            return;
        }
        let attributes = Window::default_attributes()
            .with_title("Physics Sandbox Parkour (GPU)")
            .with_inner_size(winit::dpi::LogicalSize::new(960, 600));
        let window = Arc::new(event_loop.create_window(attributes).unwrap());
        let _ = window.set_cursor_grab(CursorGrabMode::None);

        if self.game.is_none() {
            self.game = Some(Game::new());
        }

        #[cfg(target_env = "msvc")]
        {
            // GPU 路径:用 pollster 把 async 初始化跑起来。
            let meshes = &self.game.as_ref().unwrap().body_meshes;
            let renderer = pollster::block_on(GpuBackend::new(window.clone(), meshes));
            self.gpu = Some(renderer);
        }

        #[cfg(not(target_env = "msvc"))]
        {
            let context = softbuffer::Context::new(window.clone()).expect("softbuffer context");
            let surface =
                softbuffer::Surface::new(&context, window.clone()).expect("softbuffer surface");
            self.context = Some(context);
            self.surface = Some(surface);
            self.fb = Some(Framebuffer::new(960, 600));
        }

        self.window = Some(window.clone());
    }
}

impl ApplicationHandler for App {
    fn resumed(&mut self, event_loop: &ActiveEventLoop) {
        self.ensure_surface(event_loop);
    }

    fn window_event(
        &mut self,
        event_loop: &ActiveEventLoop,
        _id: WindowId,
        event: WindowEvent,
    ) {
        self.ensure_surface(event_loop);
        let (game, window) = match (self.game.as_mut(), self.window.as_ref()) {
            (Some(g), Some(w)) => (g, w),
            _ => return,
        };

        match event {
            WindowEvent::CloseRequested
            | WindowEvent::KeyboardInput {
                event:
                    KeyEvent {
                        physical_key: Code(KeyCode::Escape),
                        ..
                    },
                ..
            } => {
                event_loop.exit();
            }
            WindowEvent::KeyboardInput {
                event:
                    KeyEvent {
                        physical_key: Code(code),
                        state,
                        ..
                    },
                ..
            } => {
                match code {
                    KeyCode::KeyR if state == ElementState::Pressed => game.reset_player(),
                    KeyCode::KeyQ if state == ElementState::Pressed => game.cam.distance -= 1.0,
                    KeyCode::KeyE if state == ElementState::Pressed => game.cam.distance += 1.0,
                    _ => {}
                }
                match state {
                    ElementState::Pressed => {
                        game.keys.insert(code);
                    }
                    ElementState::Released => {
                        game.keys.remove(&code);
                    }
                }
            }
            WindowEvent::MouseInput {
                state,
                button: MouseButton::Left,
                ..
            } => {
                game.mouse_drag = state == ElementState::Pressed;
                if !game.mouse_drag {
                    game.last_mouse = None;
                }
            }
            WindowEvent::CursorMoved { position, .. } => {
                if game.mouse_drag {
                    let (x, y) = (position.x, position.y);
                    if let Some((lx, ly)) = game.last_mouse {
                        let dx = (x - lx) as f32 * 0.005;
                        let dy = (y - ly) as f32 * 0.005;
                        game.cam.yaw -= dx;
                        game.cam.pitch = (game.cam.pitch + dy).clamp(-1.4, 1.4);
                    }
                    game.last_mouse = Some((x, y));
                }
            }
            WindowEvent::RedrawRequested => {
                game.step();
                let size = window.inner_size();
                let w = size.width.max(1);
                let h = size.height.max(1);
                let aspect = w as f32 / h as f32;

                #[cfg(target_env = "msvc")]
                {
                    // GPU 渲染路径。
                    if let Some(gpu) = self.gpu.as_mut() {
                        let (gw, gh) = gpu.size();
                        if w != gw || h != gh {
                            gpu.resize(w, h);
                        }
                        let vp = game.cam.view_proj(aspect);
                        let models = game.body_model_matrices();
                        let light = Vector3::new(0.4f32, 0.85, 0.3).normalize();
                        let bg = [0.40f32, 0.52, 0.62];
                        gpu.render_frame(&vp, &models, &game.body_colors, &light, bg);
                    }

                    // 诊断模式:渲染 2 帧后自动退出并留下 gpu_dump.png。
                    if std::env::var_os("GPU_DUMP").is_some() {
                        self.frames += 1;
                        if self.frames >= 2 {
                            eprintln!("[GPU_DUMP] wrote gpu_dump.png and exit");
                            std::process::exit(0);
                        }
                    }
                }

                #[cfg(not(target_env = "msvc"))]
                {
                    // 软件光栅化路径(复用帧缓冲,尺寸变化时才重建)。
                    let need_new = match &self.fb {
                        Some(fb) => fb.width != w || fb.height != h,
                        None => true,
                    };
                    if need_new {
                        self.fb = Some(Framebuffer::new(w, h));
                    }
                    let fb = self.fb.as_mut().unwrap();
                    game.render(fb, aspect);

                    if let (Some(surface), Some(fb_w), Some(fb_h)) = (
                        self.surface.as_mut(),
                        NonZeroU32::new(w),
                        NonZeroU32::new(h),
                    ) {
                        if surface.resize(fb_w, fb_h).is_ok() {
                            if let Ok(mut buffer) = surface.buffer_mut() {
                                let conv = phy_demo::raster::to_softbuffer(&fb.pixels);
                                buffer.copy_from_slice(&conv);
                                let _ = buffer.present();
                            }
                        }
                    }
                }

                let p = game.player_pos();
                let title = format!(
                    "{} | pos=({:.1},{:.1},{:.1}) grounded={}",
                    game.title,
                    p.x,
                    p.y,
                    p.z,
                    game.cc.grounded
                );
                window.set_title(&title);
            }
            _ => {}
        }
    }

    fn about_to_wait(&mut self, event_loop: &ActiveEventLoop) {
        // 节流到 ~30fps:用 WaitUntil 让事件循环在帧间隔里处理输入事件,
        // 避免无限制 request_redraw 占满 CPU 导致输入饿死 + 卡顿。
        let next = self.last_frame + FRAME_DUR;
        event_loop.set_control_flow(ControlFlow::WaitUntil(next));
        self.last_frame = next;
        if let Some(w) = &self.window {
            w.request_redraw();
        }
    }
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    if args.iter().any(|a| a == "--headless" || a == "-h") {
        run_headless();
        return;
    }

    let event_loop = EventLoop::new().unwrap();
    event_loop.set_control_flow(ControlFlow::WaitUntil(Instant::now() + FRAME_DUR));
    let mut app = App::default();
    event_loop.run_app(&mut app).unwrap();
}

/// 无头渲染模式:不创建任何窗口,离屏光栅化若干帧并把每一帧写成 PNG,
/// 用于 CI / 无显示服务器上验证渲染管线(等价于 `cargo run -- --headless`)。
fn run_headless() {
    use std::io::BufWriter;

    const W: u32 = 960;
    const H: u32 = 600;
    const FRAMES: usize = 120; // 模拟约 1 秒(DT=1/120)

    let mut game = Game::new();
    let aspect = W as f32 / H as f32;

    let fwd = Vector3::new(0.0, 0.0, -1.0);

    for frame in 0..FRAMES {
        let want_jump = frame == 10; // 第 10 帧起跳
        game.step_with_input(fwd, want_jump);

        let mut fb = Framebuffer::new(W, H);
        game.render(&mut fb, aspect);

        let path = format!("headless_frame_{:04}.png", frame);
        let file = std::fs::File::create(&path).expect("create png");
        let writer = BufWriter::new(file);
        let mut enc = png::Encoder::new(writer, W, H);
        enc.set_color(png::ColorType::Rgba);
        enc.set_depth(png::BitDepth::Eight);
        let mut out = enc.write_header().expect("png header");
        // Framebuffer.pixels 是 0xAARRGGBB,转成 RGBA 字节序。
        let mut buf = Vec::with_capacity((W * H * 4) as usize);
        for px in &fb.pixels {
            let a = (px >> 24) & 0xff;
            let r = (px >> 16) & 0xff;
            let g = (px >> 8) & 0xff;
            let b = px & 0xff;
            buf.extend_from_slice(&[r as u8, g as u8, b as u8, a as u8]);
        }
        out.write_image_data(&buf).expect("png data");
        if frame % 20 == 0 || frame == FRAMES - 1 {
            let p = game.player_pos();
            eprintln!(
                "[headless] frame {}/{} pos=({:.2},{:.2},{:.2}) grounded={} finite={}",
                frame,
                FRAMES,
                p.x,
                p.y,
                p.z,
                game.cc.grounded,
                game.all_finite()
            );
        }
    }
    eprintln!(
        "[headless] done: {} frames written (headless_frame_0000.png .. headless_frame_{:04}.png)",
        FRAMES,
        FRAMES - 1
    );
}
