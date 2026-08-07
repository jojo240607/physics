//! 演示场景:把 `RigidWorld` 包装成可渲染场景。
//!
//! - 一个静态地面(大扁盒)。
//! - 一批动态盒与球,从空中落下并相互堆叠/碰撞。
//! - 每个 body 配一个颜色(与 `bodies` 索引对齐),用于实例化渲染。

use phy_math::na::{Matrix4, Quaternion, UnitQuaternion, Vector3};
use phy_math::na as nalgebra;
use phy_math::Vec3;
use phy_rigid::{Body, RigidWorld, Shape};

/// 每个实例的数据:4x4 模型矩阵(4 个列向量)+ 颜色。
/// 列主序 mat4 的四个列。
pub struct Instance {
    pub m0: [f32; 4],
    pub m1: [f32; 4],
    pub m2: [f32; 4],
    pub m3: [f32; 4],
    pub color: [f32; 4],
}

/// 可渲染场景 = 物理世界 + 每体颜色。
pub struct Scene {
    world: RigidWorld<f64>,
    colors: Vec<[f32; 4]>,
    rng_state: u64,
}

impl Scene {
    pub fn new() -> Self {
        let mut s = Self {
            world: RigidWorld::new(),
            colors: Vec::new(),
            rng_state: 0x9E37_79B9_7F4A_7C15,
        };
        s.reset();
        s
    }

    /// 重置场景:重新放置地面与初始物体。
    pub fn reset(&mut self) {
        self.world = RigidWorld::new();
        self.colors.clear();

        // 静态地面(大扁盒)
        let ground = Body {
            shape: Shape::Box {
                half: Vec3::new(50.0, 0.5, 50.0),
            },
            pos: Vec3::new(0.0, -0.5, 0.0),
            rot: UnitQuaternion::identity(),
            vel: Vec3::zeros(),
            inv_mass: 0.0,
        };
        self.world.add_body(ground);
        self.colors.push([0.20, 0.45, 0.30, 1.0]);

        // 初始一簇下落物体
        self.add_boxes(14);
        self.add_spheres(6);
    }

    /// 追加 n 个随机动态盒。
    pub fn add_boxes(&mut self, n: usize) {
        for _ in 0..n {
            let s = 0.4 + self.rng() * 0.8;
            let body = Body {
                shape: Shape::Box {
                    half: Vec3::new(s, s, s),
                },
                pos: Vec3::new(
                    (self.rng() - 0.5) * 12.0,
                    4.0 + self.rng() * 10.0,
                    (self.rng() - 0.5) * 12.0,
                ),
                rot: random_quat(&mut self.rng_state),
                vel: Vec3::zeros(),
                inv_mass: 1.0,
            };
            self.world.add_body(body);
            self.colors.push(random_color(&mut self.rng_state));
        }
    }

    /// 追加 n 个随机动态球。
    pub fn add_spheres(&mut self, n: usize) {
        for _ in 0..n {
            let r = 0.4 + self.rng() * 0.6;
            let body = Body {
                shape: Shape::Sphere { r },
                pos: Vec3::new(
                    (self.rng() - 0.5) * 12.0,
                    4.0 + self.rng() * 10.0,
                    (self.rng() - 0.5) * 12.0,
                ),
                rot: UnitQuaternion::identity(),
                vel: Vec3::zeros(),
                inv_mass: 1.0,
            };
            self.world.add_body(body);
            self.colors.push(random_color(&mut self.rng_state));
        }
    }

    /// 推进物理一个时间步。
    pub fn step(&mut self, dt: f64) {
        self.world.step(dt);
    }

    /// 收集渲染实例:返回 (盒实例, 球实例)。
    pub fn gather(&self) -> (Vec<Instance>, Vec<Instance>) {
        let mut boxes = Vec::new();
        let mut spheres = Vec::new();
        for (i, b) in self.world.bodies.iter().enumerate() {
            let color = self.colors.get(i).copied().unwrap_or([0.8, 0.8, 0.8, 1.0]);
            let inst = body_instance(b, color);
            match b.shape {
                Shape::Box { .. } => boxes.push(inst),
                Shape::Sphere { .. } => spheres.push(inst),
                Shape::Convex { .. } => boxes.push(inst),
            }
        }
        (boxes, spheres)
    }

    pub fn body_count(&self) -> usize {
        self.world.bodies.len()
    }
}

/// 由 body 的位姿/形状构造一个实例化数据(模型矩阵 + 颜色)。
fn body_instance(b: &Body<f64>, color: [f32; 4]) -> Instance {
    // 旋转:把 f64 四元数转成 f32。
    let q = &b.rot;
    let qf = Quaternion::new(
        q.w as f32,
        q.i as f32,
        q.j as f32,
        q.k as f32,
    );
    let rot: Matrix4<f32> = Matrix4::from(UnitQuaternion::from_quaternion(qf));

    let t = b.pos;
    let trans = Matrix4::new_translation(&Vector3::new(
        t.x as f32,
        t.y as f32,
        t.z as f32,
    ));

    let scale = match b.shape {
        Shape::Box { ref half } => Matrix4::new_nonuniform_scaling(&Vector3::new(
            half.x as f32 * 2.0,
            half.y as f32 * 2.0,
            half.z as f32 * 2.0,
        )),
        Shape::Sphere { r } => Matrix4::new_scaling(r as f32),
        Shape::Convex { .. } => Matrix4::identity(),
    };

    let model = trans * rot * scale;
    let s = model.as_slice();
    let col = |a: usize| [s[a], s[a + 1], s[a + 2], s[a + 3]];
    Instance {
        m0: col(0),
        m1: col(4),
        m2: col(8),
        m3: col(12),
        color,
    }
}

/// 简易确定性 RNG(xorshift64*)。
impl Scene {
    fn rng(&mut self) -> f64 {
        let mut x = self.rng_state;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        x = x.wrapping_mul(0x2545F4914F6CDD1D);
        self.rng_state = x;
        // 映射到 [0,1)
        ((x >> 11) as f64) / (1u64 << 53) as f64
    }
}

fn random_quat(state: &mut u64) -> UnitQuaternion<f64> {
    let r = |s: &mut u64| -> f64 {
        let mut x = *s;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        x = x.wrapping_mul(0x2545F4914F6CDD1D);
        *s = x;
        ((x >> 11) as f64) / (1u64 << 53) as f64
    };
    let u1 = r(state);
    let u2 = r(state);
    let u3 = r(state);
    let q = Quaternion::new(
        (1.0 - u1).sqrt(),
        (u1).sqrt() * (2.0 * std::f64::consts::PI * u2).sin(),
        (u1).sqrt() * (2.0 * std::f64::consts::PI * u2).cos(),
        (u3 * 2.0 * std::f64::consts::PI).sin(),
    );
    UnitQuaternion::from_quaternion(q)
}

fn random_color(state: &mut u64) -> [f32; 4] {
    let r = |s: &mut u64| -> f64 {
        let mut x = *s;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        x = x.wrapping_mul(0x2545F4914F6CDD1D);
        *s = x;
        ((x >> 11) as f64) / (1u64 << 53) as f64
    };
    // HSV -> RGB,固定高饱和、明亮
    let h = r(state);
    let (r, g, b) = hsv_to_rgb(h, 0.7, 0.95);
    [r, g, b, 1.0]
}

fn hsv_to_rgb(h: f64, s: f64, v: f64) -> (f32, f32, f32) {
    let i = (h * 6.0).floor();
    let f = h * 6.0 - i;
    let p = v * (1.0 - s);
    let q = v * (1.0 - f * s);
    let t = v * (1.0 - (1.0 - f) * s);
    let (r, g, b) = match (i as i32) % 6 {
        0 => (v, t, p),
        1 => (q, v, p),
        2 => (p, v, t),
        3 => (p, q, v),
        4 => (t, p, v),
        _ => (v, p, q),
    };
    (r as f32, g as f32, b as f32)
}

// 避免 nalgebra 重导出未使用告警(用于类型约束)。
#[allow(dead_code)]
type _Na = nalgebra::Matrix4<f64>;
