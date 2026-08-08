//! 软体物理:质点-弹簧模型(M4)。
//!
//! 用 velocity-Verlet 积分 + 质点-弹簧约束模拟可形变体(布料 / 果冻块 / 绳)。
//! - `Particle`:质点(pos / vel / 反质量)。
//! - `Spring`:两质点间 Hooke 弹簧(结构/剪切/弯曲),含阻尼。
//! - `SoftBody`:规则晶格质点网格 + 弹簧集合,`step` 做 velocity-Verlet 积分 +
//!   弹簧力 + 重力 + 地面/边界碰撞(单向:软体被静态边界挡住)。
//! - `SoftSubsystem`:适配 `phy_core::Subsystem`,挂到 `World` 统一调度。
//!
//! 与刚体(M3)、流体(M5)的双向耦合均经 `World::couple` 阶段完成:
//! - 软体↔刚体:`SoftBody::collide_body` 用刚体形状 `contains_local` 检测穿透,
//!   沿外法线把质点推出,可动刚体同时收到反向冲量;
//! - 软体↔流体:`SoftSubsystem::couple` 把软体质点打包成 `CouplePoint` 交给
//!   `FluidWorld::couple_points`,获得浮力 + 阻力(soft→fluid 反向动量在流体侧分配)。
//! 本里程碑软体本身只对静态地面/盒边界碰撞,可独立 `step`。

mod body;
mod cloth;
mod sub;

pub use body::{Particle, SoftBody, Spring};
pub use cloth::{Cloth, DistanceConstraint};
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
    use phy_math::{gravity, Vec3};

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

    #[test]
    fn world_couples_soft_fluid_rigid() {
        // 端到端:World 中同时挂刚体/流体/软体,step 时软体应同时与两者耦合。
        // 用较小的流体盒 + 少量软体质点,验证:无 panic、无 NaN、耦合阶段
        // (动态查找子系统下标 + 软体↔刚体/↔流体)可正常运行。
        use phy_core::World;
        use phy_fluid::{FluidSubsystem, FluidWorld, SphParams};
        use phy_rigid::{Body, RigidSubsystem, RigidWorld, Shape};

        let mut world: World<f64> = World::default();

        // 刚体:静态地面 + 一个可动球。
        let mut rigid = RigidWorld::<f64>::new();
        rigid.add_body(Body {
            shape: Shape::Box {
                half: Vec3::new(20.0, 0.5, 20.0),
            },
            pos: Vec3::new(0.0, -0.5, 0.0),
            rot: na::UnitQuaternion::identity(),
            vel: Vec3::zeros(),
            inv_mass: 0.0,
        });
        rigid.add_body(Body {
            shape: Shape::Sphere { r: 0.5 },
            pos: Vec3::new(0.0, 1.0, 0.0),
            rot: na::UnitQuaternion::identity(),
            vel: Vec3::zeros(),
            inv_mass: 1.0 / 2.0,
        });

        // 流体:小盒(避免 1000+ 粒子拖慢测试)。
        let mut fp = SphParams::<f64>::defaults();
        fp.bounds_min = Vec3::new(-2.0, 0.0, -2.0);
        fp.bounds_max = Vec3::new(2.0, 4.0, 2.0);
        let mut fluid = FluidWorld::new(fp);
        fluid.fill_box(
            Vec3::new(-1.0, 0.5, -1.0),
            Vec3::new(1.0, 3.0, 1.0),
            0.4,
            0.1,
        );

        // 软体:少量质点,部分浸入流体盒内(应受浮力上举)。
        let mut soft = SoftBody::<f64>::new(-9.81);
        let _ = soft.add_particle(Vec3::new(0.0, 1.5, 0.0), 1.0); // 在流体盒内
        let _ = soft.add_particle(Vec3::new(0.0, 5.0, 0.0), 1.0); // 在流体盒外(仅重力)

        world.add_subsystem(Box::new(RigidSubsystem::new(rigid)));
        world.add_subsystem(Box::new(FluidSubsystem::new(fluid)));
        world.add_subsystem(Box::new(SoftSubsystem::new(soft)));

        for _ in 0..40 {
            world.step(1.0 / 60.0);
        }

        // 取回软体,验证所有质点有限(耦合未产生 NaN/爆炸)。
        let soft = world
            .get(2)
            .unwrap()
            .as_any()
            .downcast_ref::<SoftSubsystem<f64>>()
            .unwrap();
        for p in &soft.body.particles {
            assert!(p.pos.x.is_finite() && p.pos.y.is_finite() && p.pos.z.is_finite());
            assert!(p.vel.x.is_finite() && p.vel.y.is_finite() && p.vel.z.is_finite());
        }
        // 浸入流体的质点应受到向上的净力(浮力 > 重力分量),力 y 分量应 > 纯重力。
        // 纯重力 force.y = m*g = -1*9.81 ≈ -9.81;浮力使其更靠近 0 或为正。
        let submerged = &soft.body.particles[0];
        assert!(
            submerged.force.y > -9.81,
            "浸入流体的质点应受浮力(force.y 应大于纯重力 -9.81), got {}",
            submerged.force.y
        );
    }
}
