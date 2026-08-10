//! L4-3 (CPU 端可验证部分):W7 GPU 运行时切换的**路由逻辑**一致性检查。
//!
//! 浏览器内的 WebGPU compute 内核本身需要真实 adapter,无法在 CI/主机上跑;
//! 但其 **CPU 侧的路由结构**(`step_world_gpu` 的逻辑:把 fluid/granular 的力学
//! step 从 `world.step` 里摘出、其余子系统 + 全部 `couple` 仍走 `step_skipping`)
//! 是可在主机上验证的。
//!
//! 本测试构造耦合世界,分别走:
//!   A. `world.step(dt)` —— 全 CPU(基准);
//!   B. W7 同款路由:手动 step fluid/granular 子系统的 `world.step(dt)`(对应
//!      `step_sph_gpu` / `step_granular_gpu` 的“主机回写集成”侧),再
//!      `world.step_skipping(dt, skip)` 跳过它们的 CPU step 但保留 couple。
//! 断言二者在 1e-12 内逐位一致 —— 证明 W7 摘出/跳过 fluid/granular 不会破坏
//! 耦合矩阵,且“手动 step 子世界 + step_skipping”与“整步”等价。
//! (GPU compute 内核的数值正确性由浏览器端 `sph_self_test` / `granular_self_test`
//!  的 console 输出人工核对,见 PLAN.md §5.8 L4-3。)

use nalgebra::UnitQuaternion;
use phy_core::World;
use phy_fluid::{FluidSubsystem, FluidWorld, SphParams};
use phy_math::Vec3 as V3;
use phy_rigid::{Body, RigidSubsystem, RigidWorld, Shape};
use std::any::Any;

fn coupled_world() -> World<f64> {
    let params = SphParams::<f64>::defaults();
    let mut fluid = FluidWorld::<f64>::new(params);
    fluid.fill_box(
        V3::new(-1.0, 1.0, -1.0),
        V3::new(1.0, 3.0, 1.0),
        0.3,
        0.1,
    );
    let mut rworld = RigidWorld::<f64>::new();
    rworld.add_body(Body {
        shape: Shape::Box {
            half: V3::new(10.0, 0.5, 10.0),
        },
        pos: V3::new(0.0, -0.5, 0.0),
        rot: UnitQuaternion::identity(),
        vel: V3::zeros(),
        inv_mass: 0.0,
        ..Default::default()
    });
    let mut w = World::new();
    w.add_subsystem(Box::new(RigidSubsystem::new(rworld)));
    w.add_subsystem(Box::new(FluidSubsystem::new(fluid)));
    w
}

#[test]
fn w7_skip_routing_matches_full_step() {
    let dt = 0.01;

    // A: 全 CPU 基准。
    let mut baseline = coupled_world();
    baseline.step(dt);
    let base_fluid = baseline
        .get(1)
        .unwrap()
        .as_any()
        .downcast_ref::<FluidSubsystem<f64>>()
        .unwrap();
    let base_pos: Vec<V3<f64>> = base_fluid.world.particles.iter().map(|p| p.pos).collect();

    // B: W7 同款路由(主机侧,无 GPU dispatch)。
    let mut w = coupled_world();
    let n = w.subsystem_count();
    let mut skip = vec![false; n];
    for i in 0..n {
        let s = w.get(i).unwrap();
        if s.as_any().is::<FluidSubsystem<f64>>() {
            skip[i] = true;
        }
    }
    // 手动 step 被摘出的流体子世界(对应 step_sph_gpu 主机回写侧)。
    for i in 0..n {
        if skip[i] {
            let s = w.get_mut(i).unwrap();
            let fs = s
                .as_any_mut()
                .downcast_mut::<FluidSubsystem<f64>>()
                .unwrap();
            fs.world.step(dt);
        }
    }
    // 其余子系统 + 全部 couple 走 step_skipping(skip 掉流体的 CPU step)。
    w.step_skipping(dt, &skip);

    let routed_fluid = w
        .get(1)
        .unwrap()
        .as_any()
        .downcast_ref::<FluidSubsystem<f64>>()
        .unwrap();
    let routed_pos: Vec<V3<f64>> = routed_fluid.world.particles.iter().map(|p| p.pos).collect();

    assert_eq!(base_pos.len(), routed_pos.len());
    let mut max_err = 0.0f64;
    for (a, b) in base_pos.iter().zip(routed_pos.iter()) {
        max_err = max_err.max((a.x - b.x).abs());
        max_err = max_err.max((a.y - b.y).abs());
        max_err = max_err.max((a.z - b.z).abs());
    }
    assert!(
        max_err < 1e-12,
        "W7 路由结果与全 CPU 步进不一致,max_err={}",
        max_err
    );
}
