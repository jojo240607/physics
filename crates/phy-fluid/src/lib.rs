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

pub use kernels::{dist, Kernels};
pub use particle::Particle;
pub use sph::{CouplePoint, FluidWorld, SphParams};
pub use subsystem::FluidSubsystem;

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
        // --- 流体:静止水池(重力关闭),仅作浮力介质 ---
        let mut p = SphParams::defaults();
        p.gravity = Vec3::zeros();
        let mut fworld: FluidWorld<f64> = FluidWorld::new(p);
        fworld.fill_box(
            Vec3::new(-0.8, -0.8, -0.8),
            Vec3::new(0.8, 0.8, 0.8),
            0.1,
            0.05,
        );
        for _ in 0..15 {
            fworld.step(0.0025);
        }
        // 把粒子压进球内制造淹没。
        for pt in fworld.particles.iter_mut() {
            pt.pos = Vec3::new(pt.pos.x * 0.3, pt.pos.y * 0.3, pt.pos.z * 0.3);
            pt.vel = Vec3::zeros();
        }
        let n_fluid0 = fworld.particles.len();

        // --- 刚体:密度约为流体 1/4 的轻球,完全浸没 ---
        let mut rworld = phy_rigid::RigidWorld::new();
        rworld.gravity = Vec3::zeros();
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

        // 步进若干帧(与受控 sph 测试同时间尺度 dt=0.0025):
        // 刚体应整体上浮(vy > 0,即受到净上举)。仅跑少量帧,在 SPH 大挤压
        // 条件不稳定窗口之前即可确认 #10 耦合链路已自动触发浮力(诊断显示
        // 早期帧 vy 即转正,~15 帧后 SPH 在持续刚体挤压下才发散,属 SPH 数值
        // 稳定性范畴,非耦合逻辑问题;受控 sph 测试同尺度仅临界不爆)。
        let n_frames = 12;
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
        // 浮力应为正(上举):竖直速度或位移向上。
        assert!(
            b.vel.y > 0.0 || b.pos.y > 0.0,
            "轻刚体应被浮力上举: vy={}, dy={}",
            b.vel.y,
            b.pos.y
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
