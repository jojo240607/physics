//! # Physics Sandbox Parkour —— 基于物理引擎 SDK 的 3D 小游戏 demo(窗口/渲染层)
//!
//! 游戏核心逻辑见 `lib.rs` 的 [`phy_game_demo::Game`];本文件仅负责 winit 窗口循环
//! 与 softbuffer 软件光栅化渲染。
//!
//! 玩法:
//! - `W/A/S/D`:相对相机方向的水平移动
//! - `Space`:跳跃
//! - `鼠标拖拽`:旋转相机视角
//! - `Q/E`:拉近/拉远
//! - `R`:重置角色到出生点
//! - `Esc`:退出

use std::num::NonZeroU32;
use std::rc::Rc;

use phy_demo::raster::Framebuffer;
use phy_game_demo::winit_key_codes::KeyCode;
use phy_game_demo::Game;
use softbuffer::{Context, Surface};
use winit::application::ApplicationHandler;
use winit::event::{ElementState, KeyEvent, MouseButton, WindowEvent};
use winit::event_loop::{ActiveEventLoop, EventLoop};
use winit::keyboard::PhysicalKey::Code;
use winit::window::{CursorGrabMode, Window, WindowId};

struct App {
    game: Option<Game>,
    window: Option<Rc<Window>>,
    context: Option<Context<Rc<Window>>>,
    surface: Option<Surface<Rc<Window>, Rc<Window>>>,
}

impl Default for App {
    fn default() -> Self {
        Self {
            game: None,
            window: None,
            context: None,
            surface: None,
        }
    }
}

impl App {
    fn ensure_surface(&mut self, event_loop: &ActiveEventLoop) {
        if self.window.is_some() {
            return;
        }
        let attributes = Window::default_attributes()
            .with_title("Physics Sandbox Parkour")
            .with_inner_size(winit::dpi::LogicalSize::new(960, 600));
        let window = Rc::new(event_loop.create_window(attributes).unwrap());
        let _ = window.set_cursor_grab(CursorGrabMode::None);
        let context = Context::new(window.clone()).expect("softbuffer context");
        let surface = Surface::new(&context, window.clone()).expect("softbuffer surface");
        self.window = Some(window);
        self.context = Some(context);
        self.surface = Some(surface);
        if self.game.is_none() {
            self.game = Some(Game::new());
        }
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

                let mut fb = Framebuffer::new(w, h);
                game.render(&mut fb, aspect);

                if let (Some(surface), Some(fb_w), Some(fb_h)) = (
                    self.surface.as_mut(),
                    NonZeroU32::new(w),
                    NonZeroU32::new(h),
                ) {
                    if surface.resize(fb_w, fb_h).is_ok() {
                        if let Ok(mut buffer) = surface.buffer_mut() {
                            buffer.copy_from_slice(&fb.pixels);
                            let _ = buffer.present();
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

    fn about_to_wait(&mut self, _event_loop: &ActiveEventLoop) {
        if let Some(w) = &self.window {
            w.request_redraw();
        }
    }
}

fn main() {
    let event_loop = EventLoop::new().unwrap();
    let mut app = App::default();
    event_loop.run_app(&mut app).unwrap();
}
