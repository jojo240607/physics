//! phy-demo: 多物理场软件光栅化 Demo(M3/M5/M7 集成)。
//!
//! 用纯 Rust 的软件光栅化器实时渲染统一 `World` 中的多物理场场景:
//! - 刚体(M3)下落/堆叠
//! - 流体 SPH(M5)粒子云
//! - 连续标量场(M7)热扩散切片
//! - 光学(M6)玻璃球折射(离线/实时)
//!
//! 操作:
//! - 鼠标拖拽:旋转视角(轨道)
//! - 滚轮:缩放
//! - P:暂停/继续  R:重置场景  O:循环模式(F/H 直接定位)
//! - I:打印统计  关闭窗口:退出

mod camera;
mod raster;
mod scene;

use std::num::NonZeroU32;
use std::rc::Rc;

use camera::Camera;
use raster::Framebuffer;
use scene::{DemoMode, Scene};
use softbuffer::{Context, Surface};
use winit::application::ApplicationHandler;
use winit::event::{ElementState, MouseScrollDelta, WindowEvent};
use winit::event_loop::{ActiveEventLoop, ControlFlow, EventLoop, EventLoopProxy};
use winit::window::{Window, WindowId};

use phy_math::Vec3 as V3;
use phy_optics::{OpticScene, OpticBody, OpticSubsystem, Precision, Surface as OSurface, to_rgba8};
use phy_rigid::Body;

/// 后台初始化完成事件。
enum DemoEvent {
    Initialized,
}

/// 应用状态。
struct App {
    proxy: EventLoopProxy<DemoEvent>,
    window: Option<Rc<Window>>,
    context: Option<Context<Rc<Window>>>,
    surface: Option<Surface<Rc<Window>, Rc<Window>>>,
    scene: Scene,
    cam: Camera,
    fb: Framebuffer,
    paused: bool,
    dragging: bool,
    last_x: f64,
    last_y: f64,
    frames: u32,
    fps_timer: f64,
    started: bool,
}

impl App {
    fn new(event_loop: &EventLoop<DemoEvent>) -> Self {
        Self {
            proxy: event_loop.create_proxy(),
            window: None,
            context: None,
            surface: None,
            scene: Scene::new(),
            cam: Camera::default(),
            fb: Framebuffer::new(1, 1),
            paused: false,
            dragging: false,
            last_x: 0.0,
            last_y: 0.0,
            frames: 0,
            fps_timer: 0.0,
            started: false,
        }
    }

    /// 物理步进 + 渲染一帧到帧缓冲,并呈现到窗口。
    fn render_frame(&mut self) {
        if !self.paused {
            self.scene.step();
        }
        self.fb.clear();

        if self.scene.mode == DemoMode::Optics {
            self.render_optics();
        } else {
            self.scene.render(&mut self.fb, &self.cam);
        }

        // 呈现
        if let (Some(surface), Some(fb_w), Some(fb_h)) = (
            self.surface.as_mut(),
            NonZeroU32::new(self.fb.width),
            NonZeroU32::new(self.fb.height),
        ) {
            if surface.resize(fb_w, fb_h).is_ok() {
                if let Ok(mut buffer) = surface.buffer_mut() {
                    buffer.copy_from_slice(&self.fb.pixels);
                    let _ = buffer.present();
                }
            }
        }

        // 粗略 FPS
        self.frames += 1;
        self.fps_timer += 1.0 / 60.0;
        if self.fps_timer >= 1.0 {
            println!(
                "[demo] fps≈{} mode={} bodies={} {}",
                self.frames,
                self.scene.mode.name(),
                self.scene.body_count(),
                if self.paused { "(paused)" } else { "" }
            );
            self.frames = 0;
            self.fps_timer = 0.0;
        }
    }

    /// 光学演示:用 phy-optics 的实时近似后端渲染一个玻璃球 + 地面,
    /// 复用当前相机位姿直接写入帧缓冲像素。
    fn render_optics(&mut self) {
        let (w, h) = (self.fb.width as usize, self.fb.height as usize);
        let yaw = self.cam.yaw as f64;
        let pitch = self.cam.pitch as f64;
        let dist = self.cam.distance as f64;
        let target = V3::new(0.0, 0.0, 0.0);
        let eye = V3::new(
            dist * (pitch.cos()) * (yaw.sin()),
            dist * pitch.sin(),
            dist * (pitch.cos()) * (yaw.cos()),
        ) + target;
        let up = V3::new(0.0, 1.0, 0.0);
        let fov = std::f64::consts::FRAC_PI_4;

        let mut scene = OpticScene::<f64>::new();
        scene.add(OpticBody::new(
            Body {
                shape: phy_rigid::Shape::Box {
                    half: V3::new(4.0, 0.1, 4.0),
                },
                pos: V3::new(0.0, -1.5, 0.0),
                rot: phy_math::na::UnitQuaternion::identity(),
                vel: V3::zeros(),
                inv_mass: 0.0,
            },
            OSurface::diffuse(V3::new(0.5, 0.5, 0.5)),
        ));
        scene.add(OpticBody::new(
            Body {
                shape: phy_rigid::Shape::Sphere { r: 1.0 },
                pos: V3::new(0.0, 0.0, 0.0),
                rot: phy_math::na::UnitQuaternion::identity(),
                vel: V3::zeros(),
                inv_mass: 0.0,
            },
            OSurface::glass(1.5, V3::new(0.9, 0.95, 1.0)),
        ));
        scene.add(OpticBody::new(
            Body {
                shape: phy_rigid::Shape::Sphere { r: 0.5 },
                pos: V3::new(1.8, -0.5, 0.5),
                rot: phy_math::na::UnitQuaternion::identity(),
                vel: V3::zeros(),
                inv_mass: 0.0,
            },
            OSurface::glass(1.33, V3::new(0.4, 0.6, 1.0)),
        ));

        let sub = OpticSubsystem::new(scene, Precision::Realtime);
        let mut buf = vec![V3::new(0.0, 0.0, 0.0); w * h];
        sub.render_camera(&mut buf, w, h, &eye, &target, &up, fov);
        for i in 0..buf.len() {
            self.fb.pixels[i] = to_rgba8(&buf[i]);
        }
    }
}

impl ApplicationHandler<DemoEvent> for App {
    fn resumed(&mut self, event_loop: &ActiveEventLoop) {
        if self.started {
            return;
        }
        self.started = true;

        let attributes = Window::default_attributes()
            .with_title("phy-demo · multi-physics (software)")
            .with_inner_size(winit::dpi::PhysicalSize::new(1280, 720));
        let window = Rc::new(
            event_loop
                .create_window(attributes)
                .expect("创建窗口失败"),
        );
        let context = Context::new(window.clone()).expect("创建软缓冲上下文失败");
        let surface =
            Surface::new(&context, window.clone()).expect("创建软缓冲表面失败");
        let size = window.inner_size();
        self.fb = Framebuffer::new(size.width.max(1), size.height.max(1));
        self.window = Some(window);
        self.context = Some(context);
        self.surface = Some(surface);

        let _ = self.proxy.send_event(DemoEvent::Initialized);
        if let Some(w) = &self.window {
            w.request_redraw();
        }
    }

    fn window_event(
        &mut self,
        event_loop: &ActiveEventLoop,
        _id: WindowId,
        event: WindowEvent,
    ) {
        match event {
            WindowEvent::CloseRequested => event_loop.exit(),
            WindowEvent::Resized(size) => {
                self.fb = Framebuffer::new(size.width.max(1), size.height.max(1));
                if let Some(w) = &self.window {
                    w.request_redraw();
                }
            }
            WindowEvent::RedrawRequested => self.render_frame(),
            WindowEvent::MouseInput { state, button, .. } => {
                if button == winit::event::MouseButton::Left {
                    self.dragging = state == ElementState::Pressed;
                }
            }
            WindowEvent::CursorMoved { position, .. } => {
                if self.dragging {
                    let dx = position.x - self.last_x;
                    let dy = position.y - self.last_y;
                    self.cam.yaw -= dx as f32 * 0.005;
                    self.cam.pitch += dy as f32 * 0.005;
                    let lim = 1.5_f32;
                    self.cam.pitch = self.cam.pitch.clamp(-lim, lim);
                }
                self.last_x = position.x;
                self.last_y = position.y;
            }
            WindowEvent::MouseWheel { delta, .. } => {
                let y = match delta {
                    MouseScrollDelta::LineDelta(_, y) => y as f32,
                    MouseScrollDelta::PixelDelta(p) => (p.y as f32) * 0.02,
                };
                self.cam.distance *= (y * 0.1).exp();
                self.cam.distance = self.cam.distance.clamp(5.0, 200.0);
            }
            WindowEvent::KeyboardInput { event, .. } => {
                if event.state == ElementState::Pressed {
                    if let winit::keyboard::Key::Character(c) = &event.logical_key {
                        match c.as_str() {
                            "p" => self.paused = !self.paused,
                            "o" => self.scene.toggle_mode(),
                            "f" => self.scene.set_mode(DemoMode::Fluid),
                            "h" => self.scene.set_mode(DemoMode::Heat),
                            "r" => self.scene.reset(),
                            "i" => {
                                println!(
                                    "[demo] mode={} bodies={} steps={} paused={}",
                                    self.scene.mode.name(),
                                    self.scene.body_count(),
                                    self.scene.steps,
                                    self.paused
                                )
                            }
                            _ => {}
                        }
                    }
                }
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
    println!("phy-demo · multi-physics (software rasterizer)");
    println!("拖拽旋转 · 滚轮缩放 · P 暂停 · O 循环模式 · F 流体 · H 热场 · R 重置 · I 统计 · 关闭窗口退出");

    let event_loop = EventLoop::<DemoEvent>::with_user_event().build().unwrap();
    let mut app = App::new(&event_loop);
    event_loop.set_control_flow(ControlFlow::Poll);
    event_loop.run_app(&mut app).expect("运行失败");
}
