//! # phy-fluid
//!
//! 弱可压缩 SPH(光滑粒子流体动力学)求解器(Müller et al. 2003),
//! 支持与 `phy-rigid` 刚体的双向耦合(浮力 / 阻力 / 动量交换)。
//!
//! 设计对齐姊妹 crate `phy-rigid`:提供自持的 `FluidWorld<T>` 世界,
//! 亦可经 `FluidSubsystem` 挂入 `phy_core::World<T>` 多物理场统一驱动。

mod kernels;
mod particle;
mod sph;
mod subsystem;
mod cfd;

pub use kernels::{dist, Kernels};
pub use particle::Particle;
pub use sph::{CouplePoint, FluidWorld, SphFlatData, SphParams};
pub use subsystem::FluidSubsystem;
pub use cfd::{CfdParams, CfdWorld};

#[cfg(test)]
mod tests {
    use super::*;
    use phy_core::world::World;
    use phy_math::{na, Vec3};
    use phy_rigid::{Body, RigidSubsystem, Shape};

    /// #10 增强:在统一 `World` 中把流体与刚体挂在一起,`World::step` 应自动
    /// 通过 `couple` 把浮力/阻力注入刚体。验证:浸没在静止流体中的轻刚体整体
    /// 受到上举(净竖直速度向上),且耦合过程不 panic、粒子数守恒。
    ///
    /// 场景沿用 sph::dynamic_body_gets_buoyancy_upward 的受控隔离法:流体重力
    /// 关闭(避免自由下落流体下拽掩盖浮力),粒子被压缩进刚体形状内制造淹没,
    /// 从而纯粹验证 `World::couple` 是否真正自动触发了流体↔刚体耦合链路。
    #[test]
    fn rigid_body_gets_buoyancy_via_world_couple() {
        // --- 流体:重力场开启的水池(浮力本质是流体静压梯度,依赖重力存在) ---
        let mut p = SphParams::defaults();
        p.gravity = Vec3::new(0.0, -9.81, 0.0);
        let mut fworld: FluidWorld<f64> = FluidWorld::new(p);
        // 盒子略大于球(r=0.6),使球完全浸没且被流体从四面八方包围。
        fworld.fill_box(
            Vec3::new(-0.7, -0.7, -0.7),
            Vec3::new(0.7, 0.7, 0.7),
            0.1,
            0.05,
        );
        // 沉降稳定:流体在盒底边界阻尼后静止,从四面八方包围轻球。
        for _ in 0..40 {
            fworld.step(0.0025);
        }
        let n_fluid0 = fworld.particles.len();

        // --- 刚体:密度约为流体 1/4 的轻球,完全浸没于流体中央 ---
        let mut rworld = phy_rigid::RigidWorld::new();
        rworld.gravity = Vec3::new(0.0, -9.81, 0.0);
        let r: f64 = 0.6;
        let vol = 4.0 / 3.0 * std::f64::consts::PI * r.powi(3);
        let mass_b = fworld.params.rest_density * vol * 0.25;
        let body = Body {
            shape: Shape::Sphere { r },
            pos: Vec3::new(0.0, 0.0, 0.0),
            rot: na::UnitQuaternion::identity(),
            vel: Vec3::zeros(),
            inv_mass: 1.0 / mass_b,
            ..Default::default()
        };
        rworld.add_body(body);

        let mut world: World<f64> = World::default();
        world.add_subsystem(Box::new(FluidSubsystem::new(fworld)));
        world.add_subsystem(Box::new(RigidSubsystem::new(rworld)));

        // 轻球(ρ_b=0.25ρ_f)净力 (ρ_b−ρ_f)Vg 向上 => 应被浮力顶起,浮出水面。
        // 跑足够帧让上浮趋势显现(World 框架自动对刚体做重力积分 + 流体耦合浮力)。
        let n_frames = 100;
        for _ in 0..n_frames {
            world.step(0.0025);
        }

        // 取回刚体检查位移/速度。
        let rb = world
            .get(1)
            .unwrap()
            .as_any()
            .downcast_ref::<RigidSubsystem<f64>>()
            .unwrap();
        let b = &rb.world.bodies[0];
        // 浮力应为正(上举):球净上浮,竖直位置高于初始。
        assert!(
            b.pos.y > 0.0,
            "轻刚体应被浮力上举: y0=0.0 y_end={}, vy={}",
            b.pos.y,
            b.vel.y
        );

        // 流体粒子数守恒。
        let fb = world
            .get(0)
            .unwrap()
            .as_any()
            .downcast_ref::<FluidSubsystem<f64>>()
            .unwrap();
        assert_eq!(fb.world.particles.len(), n_fluid0);
    }
}
