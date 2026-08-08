//! Demo 场景:多物理场世界。
//!
//! 用 `phy_core::World` 统一调度刚体(M3)、流体 SPH(M5)、连续标量场(M7)。
//! 各子系统独立演化(本里程碑不做跨场耦合),由 `mode` 决定 Demo 当前可视化哪一个。

use std::any::Any;

use phy_core::World;
use phy_field::{HeatField, ScalarField, Bc};
use phy_fluid::{FluidSubsystem, FluidWorld, SphParams};
use phy_math::{na, RealField, Vec3};
use phy_rigid::{Body, RigidSubsystem, RigidWorld, Shape};
use phy_soft::{SoftBody, SoftSubsystem};

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
    /// 软体(质点-弹簧,M4)。
    Soft,
    /// 光学(玻璃球 + 地面,Whitted/Approx 离线/实时)。
    Optics,
}

impl DemoMode {
    /// 循环到下一个模式。
    pub fn next(self) -> Self {
        match self {
            DemoMode::Rigid => DemoMode::Fluid,
            DemoMode::Fluid => DemoMode::Heat,
            DemoMode::Heat => DemoMode::Soft,
            DemoMode::Soft => DemoMode::Optics,
            DemoMode::Optics => DemoMode::Rigid,
        }
    }

    /// 模式名(用于 HUD)。
    pub fn name(self) -> &'static str {
        match self {
            DemoMode::Rigid => "Rigid",
            DemoMode::Fluid => "Fluid(SPH)",
            DemoMode::Heat => "Heat Field",
            DemoMode::Soft => "Soft Body",
            DemoMode::Optics => "Optics",
        }
    }
}

/// 子系统在 World 中的固定索引(用于 downcast 渲染)。
const IDX_RIGID: usize = 0;
const IDX_FLUID: usize = 1;
const IDX_HEAT: usize = 2;
const IDX_SOFT: usize = 3;

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

        // --- 软体(M4) ---
        // 悬挂的 6x6x6 晶格软块(顶部层钉扎),落在地面上方自由晃动。
        let soft = SoftBody::<f64>::from_lattice(6, 6, 6, 0.6, Vec3::new(0.0, 2.0, 0.0));
        // 地面高度与刚体地面一致(y=-0.5 是地面盒顶)。
        let mut soft = soft;
        soft.ground_y = -0.5;

        // 注册到统一世界(顺序即 step 顺序)。
        world.add_subsystem(Box::new(RigidSubsystem::new(rigid)));
        world.add_subsystem(Box::new(FluidSubsystem::new(fluid)));
        world.add_subsystem(Box::new(heat));
        world.add_subsystem(Box::new(SoftSubsystem::new(soft)));

        Self {
            world,
            mode: DemoMode::Rigid,
            steps: 0,
        }
    }

    /// 推进一帧(固定子步)。
    ///
    /// `World::step` 内部已按 `step` → `couple` 顺序驱动所有子系统,
    /// 软体↔刚体的双向耦合在 `SoftSubsystem::couple` 中经 `World` 完成。
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
            DemoMode::Soft => self.render_soft(fb, cam),
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

    fn render_soft(&self, fb: &mut Framebuffer, cam: &Camera) {
        let sub = self
            .world
            .get(IDX_SOFT)
            .unwrap()
            .as_any()
            .downcast_ref::<SoftSubsystem<f64>>()
            .unwrap();
        let body = &sub.body;
        // 弹簧(线段,灰白)。
        for s in &body.springs {
            let pa = body.particles[s.a].pos;
            let pb = body.particles[s.b].pos;
            if let (Some((ax, ay, da)), Some((bx, by, db))) =
                (project_point(cam, fb, pa), project_point(cam, fb, pb))
            {
                let depth = ((da + db) * 0.5) as f32;
                fb.draw_line(ax, ay, bx, by, depth, [180u8, 180u8, 200u8]);
            }
        }
        // 质点(蓝绿,按速度上色)。
        for p in &body.particles {
            if let Some((sx, sy, depth)) = project_point(cam, fb, p.pos) {
                let speed = p.vel.norm();
                // 静止=青,快=洋红。
                let t = (speed / 6.0).clamp(0.0, 1.0);
                let col = [
                    (t * 255.0) as u8,
                    (220.0 - t * 160.0) as u8,
                    (200.0 - t * 40.0) as u8,
                ];
                fb.fill_circle(sx, sy, 3, depth as f32, col);
            }
        }
    }
}
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

#[cfg(test)]
mod tests {
    use super::*;

    fn build_fluid_heat_world(warm_top: bool) -> World<f64> {
        let mut w = World::<f64>::new();
        // 流体:小盒、重力开启。
        let mut fp = SphParams::defaults();
        fp.bounds_min = Vec3::new(-3.0, -3.0, -3.0);
        fp.bounds_max = Vec3::new(3.0, 3.0, 3.0);
        fp.gravity = Vec3::new(0.0, -9.81, 0.0);
        let mut fluid = FluidWorld::new(fp);
        fluid.fill_box(
            Vec3::new(-1.0, -1.0, -1.0),
            Vec3::new(1.0, 1.0, 1.0),
            0.3,
            0.05,
        );
        let mut fsub = FluidSubsystem::new(fluid);
        fsub.thermal_expansion = 0.5; // 开启热浮力
        fsub.heat_gain = 0.1; // 开启对流热源
        w.add_subsystem(Box::new(fsub));

        // 热场:下半冷、上半热(与流体盒对齐)。
        let nx = 16usize;
        let dx = 6.0 / (nx as f64 - 1.0); // 覆盖 [-3,3]
        let mut f = ScalarField::<f64>::new(nx, nx, nx, dx, 0.0, Bc::Neumann)
            .with_origin(Vec3::new(-3.0, -3.0, -3.0));
        for iy in 0..nx {
            let y = -3.0 + (iy as f64) * dx;
            let t = if warm_top && y > 0.0 { 50.0 } else { 0.0 };
            for ix in 0..nx {
                for iz in 0..nx {
                    let idx = f.idx(ix, iy, iz);
                    f.u[idx] = t;
                }
            }
        }
        let heat = HeatField::new(f, 0.1);
        w.add_subsystem(Box::new(heat));
        w
    }

    #[test]
    fn world_couples_fluid_heat_thermal_buoyancy() {
        // 上热下冷 → 上方流体粒子获得向上浮力修正(acc.y > -g)。
        let mut w = build_fluid_heat_world(true);
        w.step(1.0 / 60.0);
        // 动态查找流体子系统索引(不依赖注册顺序)。
        let mut fidx = None;
        for i in 0..w.subsystem_count() {
            if let Some(s) = w.get(i) {
                if s.as_any().downcast_ref::<FluidSubsystem<f64>>().is_some() {
                    fidx = Some(i);
                    break;
                }
            }
        }
        let fidx = fidx.expect("流体子系统应已注册");
        let fsub = w
            .get(fidx)
            .unwrap()
            .as_any()
            .downcast_ref::<FluidSubsystem<f64>>()
            .unwrap();
        // 取一个位于 y>0(热区)的粒子,验证其竖直加速度被上举修正。
        let mut found = false;
        for p in &fsub.world.particles {
            if p.pos.y > 0.0 {
                assert!(
                    p.acc.y > -9.81,
                    "热区粒子应获向上热浮力修正, acc.y={}",
                    p.acc.y
                );
                found = true;
                break;
            }
        }
        assert!(found, "应存在位于热区的流体粒子");
    }

    #[test]
    fn world_couples_fluid_heat_source_injection() {
        // 流体运动 → 热场运动区域升温(对流换热)。均匀场初始 0,注入后中心升温。
        let mut w = build_fluid_heat_world(false);
        // 动态查找热场索引。
        let mut hidx = None;
        for i in 0..w.subsystem_count() {
            if let Some(s) = w.get(i) {
                if s.as_any().downcast_ref::<HeatField<f64>>().is_some() {
                    hidx = Some(i);
                    break;
                }
            }
        }
        let hidx = hidx.expect("热场子系统应已注册");
        // 给流体粒子一个初始速度,确保有运动强度可注入热源。
        let mut fidx2 = None;
        for i in 0..w.subsystem_count() {
            if let Some(s) = w.get(i) {
                if s.as_any().downcast_ref::<FluidSubsystem<f64>>().is_some() {
                    fidx2 = Some(i);
                    break;
                }
            }
        }
        if let Some(fi) = fidx2 {
            if let Some(fs) = w.get_mut(fi) {
                if let Some(fs) = fs.as_any_mut().downcast_mut::<FluidSubsystem<f64>>() {
                    for p in fs.world.particles.iter_mut() {
                        p.vel = Vec3::new(1.5, 0.0, 0.0);
                    }
                }
            }
        }
        let center = {
            let heat = w
                .get(hidx)
                .unwrap()
                .as_any()
                .downcast_ref::<HeatField<f64>>()
                .unwrap();
            heat.field.sample(nx_center(), nx_center(), nx_center())
        };
        for _ in 0..20 {
            w.step(1.0 / 60.0);
        }
        let heat = w
            .get(hidx)
            .unwrap()
            .as_any()
            .downcast_ref::<HeatField<f64>>()
            .unwrap();
        let after = heat.field.sample(nx_center(), nx_center(), nx_center());
        assert!(
            after > center,
            "运动区热场中心应因注入升温, before={}, after={}",
            center,
            after
        );
        // 全部有限。
        assert!(heat.field.max_abs().is_finite());
    }

    fn nx_center() -> usize {
        8
    }
}
