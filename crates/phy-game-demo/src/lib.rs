//! # phy-game-demo
//!
//! 基于物理引擎 SDK 的 3D 小游戏 demo 核心逻辑(无窗口,可 headless 测试)。
//! 窗口/渲染循环在 `main.rs` 的二进制 target 中。

pub mod meshgen;

// GPU 渲染后端仅在 MSVC 工具链下可用(wgpu 的 D3D12 后端需 MSVC 链接器)。
#[cfg(target_env = "msvc")]
pub mod render_wgpu;

use std::collections::HashSet;

use phy_core::World;
use phy_demo::camera::Camera;
use phy_demo::raster::Tri;
use phy_math::na::Vector3;
use phy_rigid::{Body, CharacterController, RigidSubsystem, Shape};
use phy_sdk::{get_as, get_as_mut, PhysicsBuilder};

pub const DT: f64 = 1.0 / 120.0;
pub const GRAVITY: f32 = -9.81;

/// 游戏状态(与渲染/窗口无关,可独立单测)。
pub struct Game {
    pub world: World<f64>,
    pub cam: Camera,
    pub cc: CharacterController<f64>,
    pub body_colors: Vec<[f32; 3]>,
    pub body_meshes: Vec<Vec<Tri>>,
    pub keys: HashSet<winit_key_codes::KeyCode>,
    pub mouse_drag: bool,
    pub last_mouse: Option<(f64, f64)>,
    pub spawn: Vector3<f64>,
    pub title: String,
}

/// 把 winit 的 KeyCode 重导出,避免测试依赖 winit。
pub mod winit_key_codes {
    pub use winit::keyboard::KeyCode;
}

fn color_for(i: usize) -> [f32; 3] {
    let palette: [[f32; 3]; 8] = [
        [0.85, 0.55, 0.30],
        [0.40, 0.70, 0.95],
        [0.55, 0.85, 0.45],
        [0.95, 0.45, 0.55],
        [0.90, 0.80, 0.40],
        [0.70, 0.55, 0.95],
        [0.45, 0.85, 0.80],
        [0.80, 0.80, 0.80],
    ];
    palette[i % palette.len()]
}

impl Game {
    pub fn new() -> Self {
        let mut world: World<f64> = PhysicsBuilder::new().rigid().build();
        // 关闭默认重力,改用关卡自己的重力(这里沿用物理引擎默认 -9.81)。
        let rigid = get_as_mut::<RigidSubsystem<f64>>(&mut world, 0).expect("刚体子系统");

        let mut bodies: Vec<Body<f64>> = Vec::new();

        // 地面(静态)
        bodies.push(Body {
            shape: Shape::Box {
                half: Vector3::new(20.0, 0.5, 20.0),
            },
            pos: Vector3::new(0.0, -0.5, 0.0),
            inv_mass: 0.0,
            ..Default::default()
        });
        // 四面矮墙(静态)
        for (hx, hz, px, pz) in [
            (20.0, 0.5, 0.0, 20.0),
            (20.0, 0.5, 0.0, -20.0),
            (0.5, 20.0, 20.0, 0.0),
            (0.5, 20.0, -20.0, 0.0),
        ] {
            bodies.push(Body {
                shape: Shape::Box {
                    half: Vector3::new(hx, 1.0, hz),
                },
                pos: Vector3::new(px, 1.0, pz),
                inv_mass: 0.0,
                ..Default::default()
            });
        }
        // 中央台阶(静态)
        bodies.push(Body {
            shape: Shape::Box {
                half: Vector3::new(3.0, 0.5, 3.0),
            },
            pos: Vector3::new(0.0, 0.5, -8.0),
            inv_mass: 0.0,
            ..Default::default()
        });
        // 一排可推箱子(动态)
        for i in 0..5 {
            bodies.push(Body {
                shape: Shape::Box {
                    half: Vector3::new(0.5, 0.5, 0.5),
                },
                pos: Vector3::new(-4.0 + i as f64 * 1.2, 0.5, 4.0),
                inv_mass: 1.0 / 2.0,
                ..Default::default()
            });
        }
        // 几个可滚动的球(动态)
        for i in 0..4 {
            let a = i as f64 * 1.7;
            bodies.push(Body {
                shape: Shape::Sphere { r: 0.6 },
                pos: Vector3::new(6.0 * a.cos(), 0.6, 6.0 * a.sin()),
                inv_mass: 1.0 / 1.5,
                ..Default::default()
            });
        }

        for b in bodies {
            rigid.world.add_body(b);
        }

        // 玩家角色(capsule,kinematic)
        let spawn = Vector3::new(0.0, 2.0, 8.0);
        let mut cc = CharacterController::new(&mut rigid.world, spawn);
        cc.speed = 6.0;
        cc.jump_speed = 7.0;
        cc.half_height = 0.9;
        cc.radius = 0.4;

        // 每个 body 预生成网格
        let mut body_meshes = Vec::new();
        for b in &rigid.world.bodies {
            let m = match &b.shape {
                Shape::Box { half } => {
                    meshgen::box_mesh(half.x as f32, half.y as f32, half.z as f32)
                }
                Shape::Sphere { r } => meshgen::sphere_mesh(*r as f32, 12, 16),
                Shape::Capsule { half_height, r } => {
                    meshgen::capsule_mesh(*half_height as f32, *r as f32)
                }
                _ => meshgen::box_mesh(0.5, 0.5, 0.5),
            };
            body_meshes.push(m);
        }

        let body_colors: Vec<[f32; 3]> = (0..rigid.world.bodies.len())
            .map(|i| match rigid.world.bodies[i].inv_mass {
                0.0 => [0.35, 0.37, 0.42],
                _ => color_for(i),
            })
            .collect();

        let cam = Camera {
            target: spawn.cast::<f32>().into(),
            distance: 18.0,
            yaw: 0.0,
            pitch: 0.55,
            fov: std::f32::consts::FRAC_PI_3,
        };

        Self {
            world,
            cam,
            cc,
            body_colors,
            body_meshes,
            keys: HashSet::new(),
            mouse_drag: false,
            last_mouse: None,
            spawn,
            title: "Physics Sandbox Parkour".to_string(),
        }
    }

    /// 用给定输入推进一帧物理(供窗口循环与测试共用)。
    pub fn step_with_input(&mut self, dir: Vector3<f64>, want_jump: bool) {
        let rigid = get_as_mut::<RigidSubsystem<f64>>(&mut self.world, 0).unwrap();
        self.cc.update(&mut rigid.world, DT, dir, want_jump);
        rigid.world.step(DT);

        let p = self.cc.position(&rigid.world);
        self.cam.target.x += (p.x as f32 - self.cam.target.x) * 0.15;
        self.cam.target.y += (p.y as f32 - self.cam.target.y) * 0.15;
        self.cam.target.z += (p.z as f32 - self.cam.target.z) * 0.15;
    }

    /// 用当前按住键集合推进一帧(窗口循环用)。
    pub fn step(&mut self) {
        // 相机在 +Z 看向 -Z(yaw=0 时),故"前"为 -Z。
        let fwd = Vector3::new(-(self.cam.yaw.sin() as f64), 0.0, -(self.cam.yaw.cos() as f64));
        let right = Vector3::new(self.cam.yaw.cos() as f64, 0.0, -self.cam.yaw.sin() as f64);
        let mut dir = Vector3::zeros();
        use winit_key_codes::KeyCode;
        if self.keys.contains(&KeyCode::KeyW) {
            dir += fwd;
        }
        if self.keys.contains(&KeyCode::KeyS) {
            dir -= fwd;
        }
        if self.keys.contains(&KeyCode::KeyD) {
            dir += right;
        }
        if self.keys.contains(&KeyCode::KeyA) {
            dir -= right;
        }
        let want_jump = self.keys.contains(&KeyCode::Space);
        self.step_with_input(dir, want_jump);
    }

    pub fn reset_player(&mut self) {
        let rigid = get_as_mut::<RigidSubsystem<f64>>(&mut self.world, 0).unwrap();
        if let Some(id) = self.cc.body_id {
            rigid.world.bodies[id].pos = self.spawn;
            rigid.world.bodies[id].vel = Vector3::zeros();
        }
        self.cc.grounded = false;
        self.cc.vel_y = 0.0;
    }

    /// 玩家当前位置(供 HUD/相机)。
    pub fn player_pos(&self) -> Vector3<f64> {
        let rigid = get_as::<RigidSubsystem<f64>>(&self.world, 0).unwrap();
        self.cc.position(&rigid.world)
    }

    /// 全世界状态有限性检查(防 NaN 静默污染)。
    pub fn all_finite(&self) -> bool {
        let rigid = match get_as::<RigidSubsystem<f64>>(&self.world, 0) {
            Some(r) => r,
            None => return false,
        };
        for b in &rigid.world.bodies {
            if !b.pos.iter().all(|v| v.is_finite()) {
                return false;
            }
            if !b.vel.iter().all(|v| v.is_finite()) {
                return false;
            }
        }
        true
    }

    /// 渲染当前世界到帧缓冲(软件光栅化)。
    pub fn render(&self, fb: &mut phy_demo::raster::Framebuffer, aspect: f32) {
        fb.clear();
        let vp = self.cam.view_proj(aspect);
        let light = Vector3::new(0.4f32, 0.85, 0.3).normalize();
        let rigid = get_as::<RigidSubsystem<f64>>(&self.world, 0).unwrap();
        for (i, b) in rigid.world.bodies.iter().enumerate() {
            let model = meshgen::model_matrix(&b.pos, &b.rot);
            let color = self.body_colors[i];
            for tri in &self.body_meshes[i] {
                phy_demo::raster::draw_mesh(
                    fb,
                    &vp,
                    &model,
                    std::slice::from_ref(tri),
                    [color[0], color[1], color[2], 1.0],
                    &light,
                );
            }
        }
    }

    /// 收集每个 body 的模型矩阵(f64),供 GPU 后端 `render_wgpu` 每帧上传实例数据。
    #[cfg(target_env = "msvc")]
    pub fn body_model_matrices(&self) -> Vec<phy_math::na::Matrix4<f64>> {
        let rigid = get_as::<RigidSubsystem<f64>>(&self.world, 0).unwrap();
        rigid
            .world
            .bodies
            .iter()
            .map(|b| meshgen::model_matrix(&b.pos, &b.rot))
            .collect()
    }
}
