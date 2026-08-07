//! phy-demo: M3 软件光栅化 3D 可玩 Demo。
//!
//! 用纯 Rust 的软件光栅化器实时渲染 `RigidWorld` 下落/堆叠的刚体场景
//! (不依赖 GPU 后端,保证在任意 MinGW 工具链下可编译运行)。
//! 操作:
//! - 鼠标拖拽:旋转视角(轨道)
//! - 滚轮:缩放
//! - P:暂停/继续  R:重置场景  G:再撒一批盒子  B:再撒一批球
//! - I:打印统计  关闭窗口:退出

mod camera;
mod mesh;
mod raster;
mod scene;

use std::num::NonZeroU32;
use std::rc::Rc;

use camera::Camera;
use raster::{draw_mesh, Framebuffer};
use scene::Scene;
use softbuffer::{Context, Surface};
use winit::application::ApplicationHandler;
use winit::event::{ElementState, MouseScrollDelta, WindowEvent};
use winit::event_loop::{ActiveEventLoop, ControlFlow, EventLoop, EventLoopProxy};
use winit::window::{Window, WindowId};

use phy_math::na::{Matrix4, Vector3};

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
    cube: Vec<raster::Tri>,
    sphere: Vec<raster::Tri>,
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
            cube: mesh::cube_tris(),
            sphere: mesh::sphere_tris(16, 12),
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
            self.scene.step(1.0 / 60.0);
        }
        let aspect = self.fb.width as f32 / self.fb.height as f32;
        let vp = self.cam.view_proj(aspect);
        let light = Vector3::new(0.5, 1.0, 0.3).normalize();

        self.fb.clear();
        let (boxes, spheres) = self.scene.gather();

        for inst in &boxes {
            let m = instance_model(inst);
            draw_mesh(&mut self.fb, &vp, &m, &self.cube, inst.color, &light);
        }
        for inst in &spheres {
            let m = instance_model(inst);
            draw_mesh(&mut self.fb, &vp, &m, &self.sphere, inst.color, &light);
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
                "[demo] fps≈{} bodies={} {}",
                self.frames,
                self.scene.body_count(),
                if self.paused { "(paused)" } else { "" }
            );
            self.frames = 0;
            self.fps_timer = 0.0;
        }
    }
}

/// 由 Instance 的 4 个列向量还原模型矩阵(mat4,列主序 f64)。
fn instance_model(inst: &scene::Instance) -> Matrix4<f64> {
    Matrix4::from_column_slice(&[
        inst.m0[0] as f64,
        inst.m0[1] as f64,
        inst.m0[2] as f64,
        inst.m0[3] as f64,
        inst.m1[0] as f64,
        inst.m1[1] as f64,
        inst.m1[2] as f64,
        inst.m1[3] as f64,
        inst.m2[0] as f64,
        inst.m2[1] as f64,
        inst.m2[2] as f64,
        inst.m2[3] as f64,
        inst.m3[0] as f64,
        inst.m3[1] as f64,
        inst.m3[2] as f64,
        inst.m3[3] as f64,
    ])
}

impl ApplicationHandler<DemoEvent> for App {
    fn resumed(&mut self, event_loop: &ActiveEventLoop) {
        if self.started {
            return;
        }
        self.started = true;

        let attributes = Window::default_attributes()
            .with_title("phy-rigid · 3D demo (software)")
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
                            "r" => self.scene.reset(),
                            "g" => self.scene.add_boxes(8),
                            "b" => self.scene.add_spheres(6),
                            "i" => {
                                println!(
                                    "[demo] bodies={} paused={}",
                                    self.scene.body_count(),
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
    println!("phy-rigid · 3D demo (software rasterizer)");
    println!("拖拽旋转 · 滚轮缩放 · P 暂停 · R 重置 · G 加盒 · B 加球 · I 统计 · 关闭窗口退出");

    let event_loop = EventLoop::<DemoEvent>::with_user_event().build().unwrap();
    let mut app = App::new(&event_loop);
    event_loop.set_control_flow(ControlFlow::Poll);
    event_loop.run_app(&mut app).expect("运行失败");
}
