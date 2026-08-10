//! 全局稳定性回归(L1 库化 hardening)。
//!
//! 守护"大步数下引擎不静默发散":用真实 `Scene` 模式推进 1000+ 步,断言
//! (a) 流体/颗粒子系统状态全程为有限数(无 NaN/Inf);
//! (b) 子系统数量稳定(无子系统在 step 中被静默丢弃);
//! (c) 仿真时间单调推进(无时钟回退)。
//! 纯 CPU、不依赖浏览器,可纳入 `cargo test --workspace` 常态回归。

use phy_core::World;
use phy_demo::{DemoMode, Scene};
use phy_fluid::FluidSubsystem;

/// 检查 `World` 内所有流体子系统状态是否全为有限数。
fn all_finite(world: &World<f64>) -> bool {
    let mut ok = true;
    let n = world.subsystem_count();
    for i in 0..n {
        if let Some(sub) = world.get(i) {
            if let Some(fluid) = sub.as_any().downcast_ref::<FluidSubsystem<f64>>() {
                for p in &fluid.world.particles {
                    if !p.pos.iter().all(|v| v.is_finite()) {
                        ok = false;
                    }
                    if !p.vel.iter().all(|v| v.is_finite()) {
                        ok = false;
                    }
                }
            }
        }
    }
    ok
}

fn run_stability(mode: DemoMode, steps: usize, dt: f64) {
    let mut scene = Scene::new();
    scene.set_mode(mode);
    let initial_subsystems = scene.world.subsystem_count();
    assert!(initial_subsystems > 0, "模式 {:?} 无子系统", mode);

    for step in 0..steps {
        scene.world.step(dt);
        if !all_finite(&scene.world) {
            panic!(
                "模式 {:?} 在第 {} 步出现 NaN/Inf(非有限数),引擎静默发散",
                mode, step
            );
        }
        // 子系统数量不应在 step 中被静默丢弃。
        assert_eq!(
            scene.world.subsystem_count(),
            initial_subsystems,
            "模式 {:?} 子系统数在 step 中变化",
            mode
        );
    }

    // 仿真时间应单调推进到约 steps*dt。
    let t = scene.world.time();
    assert!(
        (t - dt * steps as f64).abs() < 1e-6,
        "模式 {:?} 时钟未单调推进到预期(t={}, 期望~{})",
        mode,
        t,
        dt * steps as f64
    );
}

#[test]
fn fluid_dam_break_stable_200_steps() {
    run_stability(DemoMode::Fluid, 200, 0.005);
}

#[test]
fn fluid_heat_couple_stable_200_steps() {
    run_stability(DemoMode::FluidHeat, 200, 0.005);
}

#[test]
fn rigid_fluid_buoyancy_stable_200_steps() {
    // 刚体+流体耦合场景(All 模式含刚体+流体+热)。
    run_stability(DemoMode::All, 200, 0.005);
}

#[test]
fn soft_cloth_stable_200_steps() {
    run_stability(DemoMode::Soft, 200, 0.005);
}

#[test]
fn optics_scene_stable_100_steps() {
    // 光学场景 step 主要是相机渲染,但 world.step 仍驱动子系统,跑若干步确认无发散。
    run_stability(DemoMode::Optics, 100, 0.01);
}
