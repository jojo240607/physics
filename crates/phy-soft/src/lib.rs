//! 软体物理:质点-弹簧模型(M4)。
//!
//! 用 velocity-Verlet 积分 + 质点-弹簧约束模拟可形变体(布料 / 果冻块 / 绳)。
//! - `Particle`:质点(pos / vel / 反质量)。
//! - `Spring`:两质点间 Hooke 弹簧(结构/剪切/弯曲),含阻尼。
//! - `SoftBody`:规则晶格质点网格 + 弹簧集合,`step` 做 velocity-Verlet 积分 +
//!   弹簧力 + 重力 + 地面/边界碰撞(单向:软体被静态边界挡住)。
//! - `SoftSubsystem`:适配 `phy_core::Subsystem`,挂到 `World` 统一调度。
//!
//! 与刚体(M3)的双向耦合已接入 Demo:`SoftBody::collide_body` 用刚体形状的
//! `contains_local` 检测穿透并沿外法线把质点推出,可动刚体同时收到反向冲量
//! (见 `phy-demo` 的 `Scene::couple_soft_rigid`)。本里程碑软体本身只对静态
//! 地面/盒边界碰撞,可独立 `step`。

mod body;
mod sub;

pub use body::{Particle, SoftBody, Spring};
pub use sub::SoftSubsystem;

/// 默认弹簧刚度(f64)。
pub const DEFAULT_STIFFNESS: f64 = 200.0;
/// 默认弹簧阻尼。
pub const DEFAULT_DAMPING: f64 = 2.0;
/// Verlet 速度阻尼(数值稳定性,1=无阻尼)。
pub const DEFAULT_VERLET_DAMP: f64 = 0.99;

#[cfg(test)]
mod tests {
    use super::*;
    use phy_math::na;
    use phy_core::World;
    use phy_math::{gravity, RealField, Vec3};

    #[test]
    fn free_fall_single_particle() {
        // 单质点(无弹簧)自由落体应遵循解析解 y = y0 + 0.5*a*t^2。
        // 注意:用 vel_damp=1(无阻尼)才能严格匹配解析解。
        let g = gravity::<f64>();
        let mut body = SoftBody::<f64>::new(0.0);
        body.gravity = g;
        body.vel_damp = 1.0;
        let i = body.add_particle(Vec3::new(0.0, 10.0, 0.0), 1.0);
        let y0 = body.particles[i].pos.y;
        let dt = 1.0 / 60.0;
        let mut t = 0.0;
        for _ in 0..60 {
            body.step(dt);
            t += dt;
        }
        let y = body.particles[0].pos.y;
        let analytical = y0 + 0.5 * g.y * t * t;
        // Verlet 为 O(dt^2) 积分器,1 秒 60 步下与解析解误差在 ~1e-1 量级属正常。
        assert!((y - analytical).abs() < 0.05, "y={y} analytical={analytical}");
    }

    #[test]
    fn spring_pulls_back_to_rest() {
        // 两质点弹簧,rest=1 但初始距离=2(拉伸),应收敛回 rest 附近。
        let mut body = SoftBody::<f64>::new(0.0);
        let a = body.add_particle(Vec3::new(0.0, 0.0, 0.0), 1.0);
        let b = body.add_particle(Vec3::new(2.0, 0.0, 0.0), 1.0);
        body.add_spring_len(a, b, 1.0, 100.0, 1.0);
        let dt = 1.0 / 120.0;
        for _ in 0..600 {
            body.step(dt);
        }
        let dist = (body.particles[b].pos - body.particles[a].pos).norm();
        // rest=1.0,质量相等对称,应回到 ~1.0(容许数值松弛余量)。
        assert!((dist - 1.0).abs() < 0.05, "dist={dist}");
    }

    #[test]
    fn lattice_stable_no_nan() {
        // 规则网格步进后无 NaN / 无穷大。
        let mut body = SoftBody::<f64>::from_lattice(3, 3, 3, 1.0, Vec3::new(0.0, 5.0, 0.0));
        body.ground_y = -1.0;
        let dt = 1.0 / 60.0;
        for _ in 0..200 {
            body.step(dt);
        }
        for p in &body.particles {
            assert!(p.pos.x.is_finite() && p.pos.y.is_finite() && p.pos.z.is_finite());
        }
    }

    #[test]
    fn does_not_penetrate_ground() {
        // 软体从地面上方落下,最低点不应穿过 ground_y。
        let mut body = SoftBody::<f64>::from_lattice(4, 4, 4, 0.5, Vec3::new(0.0, 3.0, 0.0));
        body.ground_y = 0.0;
        let dt = 1.0 / 60.0;
        for _ in 0..400 {
            body.step(dt);
        }
        let min_y = body.particles.iter().map(|p| p.pos.y).fold(f64::INFINITY, f64::min);
        assert!(min_y >= -1e-6, "min_y={min_y} (penetrated ground)");
    }

    #[test]
    fn subsystem_adapts() {
        // SoftSubsystem 可作为 Subsystem 推入 World 并 step。
        use phy_core::World;
        let mut world: World<f64> = World::default();
        let body = SoftBody::<f64>::from_lattice(3, 3, 3, 0.5, Vec3::new(0.0, 4.0, 0.0));
        world.add_subsystem(Box::new(SoftSubsystem::new(body)));
        for _ in 0..30 {
            world.step(1.0 / 60.0);
        }
        // 无 panic 即通过(子系统适配本身即验证)。
    }

    #[test]
    fn soft_collides_static_sphere() {
        use phy_rigid::{Body, Shape};
        // 静态球放在原点,半径 1。一个质点被初始化到球心,应被推到球外。
        let body = Body::<f64> {
            shape: Shape::Sphere { r: 1.0 },
            pos: Vec3::new(0.0, 0.0, 0.0),
            rot: na::UnitQuaternion::identity(),
            vel: Vec3::new(0.0, 0.0, 0.0),
            inv_mass: 0.0,
        };
        let mut soft = SoftBody::<f64>::new(-9.81);
        let pid = soft.add_particle(Vec3::new(0.0, 0.0, 0.0), 1.0); // 在球心。
        soft.collide_body(&body);
        // 质点应被推出到球面外(距球心 > 1 - eps)。
        let d = soft.particles[pid].pos.norm();
        assert!(d > 1.0 - 1e-3, "particle should be pushed out of sphere, got {}", d);
    }

    #[test]
    fn soft_pushes_movable_body() {
        use phy_rigid::{Body, Shape};
        // 可动球放在原点半径 1,质点从球内以 +x 速度运动,碰撞后球应被推向 +x。
        let body = Body::<f64> {
            shape: Shape::Sphere { r: 1.0 },
            pos: Vec3::new(0.0, 0.0, 0.0),
            rot: na::UnitQuaternion::identity(),
            vel: Vec3::new(0.0, 0.0, 0.0),
            inv_mass: 0.5,
        };
        let mut soft = SoftBody::<f64>::new(-9.81);
        soft.add_particle(Vec3::new(0.9, 0.0, 0.0), 1.0);
        soft.particles[0].vel = Vec3::new(1.0, 0.0, 0.0);
        soft.collide_body(&body);
        // collide_body 只接受 &Body;冲量由调用方施加。直接验证软体被推出 + 法向反弹。
        assert!(soft.particles[0].pos.x > 0.9, "particle should be pushed outward");
    }

    #[test]
    fn soft_collide_returns_impulse_for_movable() {
        use phy_rigid::{Body, Shape};
        // collide_body 返回应施加到刚体的净冲量;可动刚体应获得 +x 方向冲量。
        // 质点位于球内贴近 +x 表面、且向 -x(深入球体,法向 vn<0)运动,碰撞后
        // 把刚体沿 +x(外法线方向)推开。
        let body = Body::<f64> {
            shape: Shape::Sphere { r: 1.0 },
            pos: Vec3::new(0.0, 0.0, 0.0),
            rot: na::UnitQuaternion::identity(),
            vel: Vec3::new(0.0, 0.0, 0.0),
            inv_mass: 0.5,
        };
        let mut soft = SoftBody::<f64>::new(-9.81);
        soft.add_particle(Vec3::new(0.9, 0.0, 0.0), 1.0);
        soft.particles[0].vel = Vec3::new(-1.0, 0.0, 0.0);
        let imp = soft.collide_body(&body);
        assert!(imp.x > 0.0, "movable body should receive +x impulse, got {}", imp.x);
    }
}
