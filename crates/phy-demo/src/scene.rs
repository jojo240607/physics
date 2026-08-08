//! Demo 场景:多物理场世界。
//!
//! 用 `phy_core::World` 统一调度刚体(M3)、流体 SPH(M5)、连续标量场(M7)。
//! 各子系统独立演化(本里程碑不做跨场耦合),由 `mode` 决定 Demo 当前可视化哪一个。

use std::any::Any;

use phy_core::World;
use phy_field::{HeatField, ScalarField, Bc};
use phy_fluid::{FluidSubsystem, FluidWorld};
use phy_math::{na, RealField, Vec3};
use phy_rigid::{Body, RigidSubsystem, RigidWorld, Shape};

use crate::camera::Camera;
use crate::raster::Framebuffer;

/// 演示模式(决定当前可视化哪个子系统)。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DemoMode {
    /// 刚体(默认)。
    Rigid,
    /// 流体 SPH 粒子云。
    Fluid,
    /// 连续标量场(热扩散)切片。
    Heat,
    /// 光学(玻璃球 + 地面,Whitted/Approx 离线/实时)。
    Optics,
}

impl DemoMode {
    /// 循环到下一个模式。
    pub fn next(self) -> Self {
        match self {
            DemoMode::Rigid => DemoMode::Fluid,
            DemoMode::Fluid => DemoMode::Heat,
            DemoMode::Heat => DemoMode::Optics,
            DemoMode::Optics => DemoMode::Rigid,
        }
    }

    /// 模式名(用于 HUD)。
    pub fn name(self) -> &'static str {
        match self {
            DemoMode::Rigid => "Rigid",
            DemoMode::Fluid => "Fluid(SPH)",
            DemoMode::Heat => "Heat Field",
            DemoMode::Optics => "Optics",
        }
    }
}

/// 子系统在 World 中的固定索引(用于 downcast 渲染)。
const IDX_RIGID: usize = 0;
const IDX_FLUID: usize = 1;
const IDX_HEAT: usize = 2;

/// Demo 场景:持有统一世界。
pub struct Scene {
    /// 统一调度世界(刚体 + 流体 + 热场)。
    pub world: World<f64>,
    /// 当前可视化模式。
    pub mode: DemoMode,
    /// 累计步数。
    pub steps: u64,
}

impl Scene {
    /// 构造默认场景:地面 + 掉落刚体 + 流体块 + 热斑。
    pub fn new() -> Self {
        let mut world: World<f64> = World::default();

        // --- 刚体(M3) ---
        let mut rigid = RigidWorld::<f64>::new();
        // 地面(静止大质量盒)。
        rigid.add_body(Body {
            shape: Shape::Box {
                half: Vec3::new(20.0, 0.5, 20.0),
            },
            pos: Vec3::new(0.0, -0.5, 0.0),
            rot: na::UnitQuaternion::identity(),
            vel: Vec3::zeros(),
            inv_mass: 0.0,
        });
        // 掉落小球。
        for i in 0..3 {
            let r = 1.0 + 0.3 * (i as f64);
            rigid.add_body(Body {
                shape: Shape::Sphere { r },
                pos: Vec3::new(-6.0 + i as f64 * 6.0, 8.0 + i as f64 * 2.0, 0.0),
                rot: na::UnitQuaternion::identity(),
                vel: Vec3::zeros(),
                inv_mass: 1.0 / 2.0,
            });
        }

        // --- 流体(M5) ---
        let params = phy_fluid::SphParams::<f64>::defaults();
        let mut fluid = FluidWorld::<f64>::new(params);
        fluid.fill_box(
            Vec3::new(-2.0, 1.0, -2.0), // 盒最小角
            Vec3::new(2.0, 5.0, 2.0),   // 盒最大角
            0.3,                        // 粒子间距
            0.1,                        // 内壁内缩
        );

        // --- 热场(M7) ---
        // 32x1x32 薄切片(2D 热扩散),dx=0.5,中心热斑。
        let mut field = ScalarField::<f64>::new(32, 1, 32, 0.5, 0.0, Bc::Neumann);
        let cx = 16;
        let cz = 16;
        for dz in -2..=2 {
            for dx in -2..=2 {
                let x = (cx as isize + dx) as usize;
                let z = (cz as isize + dz) as usize;
                if x < field.nx && z < field.nz {
                    let i = field.idx(x, 0, z);
                    field.u[i] = 1.0;
                }
            }
        }
        let heat = HeatField::<f64>::new(field, 0.1);

        // 注册到统一世界(顺序即 step 顺序)。
        world.add_subsystem(Box::new(RigidSubsystem::new(rigid)));
        world.add_subsystem(Box::new(FluidSubsystem::new(fluid)));
        world.add_subsystem(Box::new(heat));

        Self {
            world,
            mode: DemoMode::Rigid,
            steps: 0,
        }
    }

    /// 推进一帧(固定子步)。
    pub fn step(&mut self) {
        let dt = 1.0 / 60.0;
        self.world.step(dt);
        self.steps += 1;
    }

    /// 切换模式。
    pub fn toggle_mode(&mut self) {
        self.mode = self.mode.next();
    }

    /// 直接设置模式。
    pub fn set_mode(&mut self, mode: DemoMode) {
        self.mode = mode;
    }

    /// 重置场景为新构造状态。
    pub fn reset(&mut self) {
        *self = Scene::new();
    }

    /// 当前刚体数量(用于 HUD)。
    pub fn body_count(&self) -> usize {
        self.world
            .get(IDX_RIGID)
            .unwrap()
            .as_any()
            .downcast_ref::<RigidSubsystem<f64>>()
            .unwrap()
            .world
            .bodies
            .len()
    }

    /// 渲染当前模式。
    pub fn render(&self, fb: &mut Framebuffer, cam: &Camera) {
        match self.mode {
            DemoMode::Rigid => self.render_rigid(fb, cam),
            DemoMode::Fluid => self.render_fluid(fb, cam),
            DemoMode::Heat => self.render_heat(fb, cam),
            DemoMode::Optics => { /* 光学由 App 独立渲染,这里不处理 */ }
        }
    }

    fn render_rigid(&self, fb: &mut Framebuffer, cam: &Camera) {
        let sub = self
            .world
            .get(IDX_RIGID)
            .unwrap()
            .as_any()
            .downcast_ref::<RigidSubsystem<f64>>()
            .unwrap();
        for body in &sub.world.bodies {
            render_body(fb, cam, body);
        }
    }

    fn render_fluid(&self, fb: &mut Framebuffer, cam: &Camera) {
        let sub = self
            .world
            .get(IDX_FLUID)
            .unwrap()
            .as_any()
            .downcast_ref::<FluidSubsystem<f64>>()
            .unwrap();
        for p in &sub.world.particles {
            if let Some((sx, sy, depth)) = project_point(cam, fb, p.pos) {
                let r = (2.0 * depth).clamp(1.0, 8.0) as i32;
                fb.fill_circle(sx, sy, r, depth as f32, [40u8, 120u8, 255u8]);
            }
        }
    }

    fn render_heat(&self, fb: &mut Framebuffer, cam: &Camera) {
        let sub = self
            .world
            .get(IDX_HEAT)
            .unwrap()
            .as_any()
            .downcast_ref::<HeatField<f64>>()
            .unwrap();
        let f = &sub.field;
        let nx = f.nx;
        let nz = f.nz;
        for ix in 0..nx {
            for iz in 0..nz {
                let v = f.u[f.idx(ix, 0, iz)];
                let wx = (ix as f64 - nx as f64 / 2.0) * f.dx;
                let wz = (iz as f64 - nz as f64 / 2.0) * f.dx;
                let wp = Vec3::new(wx, 0.6, wz);
                if let Some((sx, sy, depth)) = project_point(cam, fb, wp) {
                    let t = v.clamp(0.0, 1.0);
                    let col = [(t * 255.0) as u8, 40u8, ((1.0 - t) * 255.0) as u8];
                    fb.fill_circle(sx, sy, 3, depth as f32, col);
                }
            }
        }
    }
}

/// 把世界点投影到屏幕像素,返回 (sx, sy, depth) 其中 depth=1/w(越小越近)。
/// 复用 `Camera::view_proj`(f32 矩阵)。
fn project_point(cam: &Camera, fb: &Framebuffer, world: Vec3<f64>) -> Option<(i32, i32, f64)> {
    let aspect = fb.width as f32 / fb.height as f32;
    let vp = cam.view_proj(aspect);
    let clip = vp
        * na::Vector4::new(world.x as f32, world.y as f32, world.z as f32, 1.0);
    if clip.w <= 1e-5 {
        return None;
    }
    let inv = 1.0 / clip.w;
    let ndc_x = clip.x * inv;
    let ndc_y = clip.y * inv;
    let sx = ((ndc_x * 0.5 + 0.5) * fb.width as f32) as i32;
    let sy = ((1.0 - (ndc_y * 0.5 + 0.5)) * fb.height as f32) as i32;
    let depth = inv as f64; // 1/w
    Some((sx, sy, depth))
}

/// 渲染单个刚体(地面 + 球体/盒)。
fn render_body(fb: &mut Framebuffer, cam: &Camera, body: &Body<f64>) {
    match &body.shape {
        Shape::Sphere { r } => {
            if let Some((sx, sy, depth)) = project_point(cam, fb, body.pos) {
                let r_screen = (*r * depth) as i32;
                let col = if body.inv_mass < 1e-9 {
                    [90u8, 90u8, 90u8]
                } else {
                    [220u8, 60u8, 60u8]
                };
                fb.fill_circle(sx, sy, r_screen.max(1), depth as f32, col);
            }
        }
        Shape::Box { half } => {
            let corners = [
                Vec3::new(body.pos.x - half.x, body.pos.y - half.y, body.pos.z - half.z),
                Vec3::new(body.pos.x + half.x, body.pos.y - half.y, body.pos.z - half.z),
                Vec3::new(body.pos.x + half.x, body.pos.y + half.y, body.pos.z - half.z),
                Vec3::new(body.pos.x - half.x, body.pos.y + half.y, body.pos.z - half.z),
                Vec3::new(body.pos.x - half.x, body.pos.y - half.y, body.pos.z + half.z),
                Vec3::new(body.pos.x + half.x, body.pos.y - half.y, body.pos.z + half.z),
                Vec3::new(body.pos.x + half.x, body.pos.y + half.y, body.pos.z + half.z),
                Vec3::new(body.pos.x - half.x, body.pos.y + half.y, body.pos.z + half.z),
            ];
            let mut minx = i32::MAX;
            let mut maxx = i32::MIN;
            let mut miny = i32::MAX;
            let mut maxy = i32::MIN;
            let mut depth = f64::INFINITY;
            for v in &corners {
                if let Some((sx, sy, d)) = project_point(cam, fb, *v) {
                    depth = depth.min(d);
                    minx = minx.min(sx);
                    maxx = maxx.max(sx);
                    miny = miny.min(sy);
                    maxy = maxy.max(sy);
                }
            }
            if minx > maxx || miny > maxy {
                return;
            }
            let col = if body.inv_mass < 1e-9 {
                [90u8, 90u8, 90u8]
            } else {
                [220u8, 60u8, 60u8]
            };
            for y in miny..=maxy {
                for x in minx..=maxx {
                    fb.set_depth(x, y, depth as f32, col);
                }
            }
        }
        Shape::Convex { .. } => { /* 凸体简略不渲染 */ }
    }
}

// 确保 Subsystem trait 仍是 Any(供 downcast)。
fn _assert_any<T: RealField>(_: &dyn Any) {}
