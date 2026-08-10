//! 数值确定性回归(L2 库化 hardening / §5.6 S8)。
//!
//! 守护"同输入同输出":在固定步长下,相同初始 `World` 重复运行应得逐位/近似一致结果,
//! 且经存档读档重放也应一致。这是防战建模可复现性的硬门槛。
//!
//! 注:引擎 `World::step(dt)` 由调用方给定步长,内部不自行变步长;SPH/颗粒已在 S7 改为
//! Jacobi 式独立缓冲写回 + 固定序 reduce,保证并行路径确定性。本测试实证该性质。

use phy_core::World;
use phy_demo::{DemoMode, Scene};
use phy_fluid::FluidSubsystem;

/// 把 `World` 内所有流体粒子的位置/速度拍平为一维向量,用于逐元素比较。
fn fluid_state(world: &World<f64>) -> Vec<f64> {
    let mut out = Vec::new();
    let n = world.subsystem_count();
    for i in 0..n {
        if let Some(sub) = world.get(i) {
            if let Some(fluid) = sub.as_any().downcast_ref::<FluidSubsystem<f64>>() {
                for p in &fluid.world.particles {
                    out.push(p.pos.x);
                    out.push(p.pos.y);
                    out.push(p.pos.z);
                    out.push(p.vel.x);
                    out.push(p.vel.y);
                    out.push(p.vel.z);
                }
            }
        }
    }
    out
}

fn run_fluid(steps: usize, dt: f64) -> Vec<f64> {
    let mut scene = Scene::new();
    scene.set_mode(DemoMode::Fluid);
    for _ in 0..steps {
        scene.world.step(dt);
    }
    fluid_state(&scene.world)
}

#[ignore = "heavy numerical regression; run: cargo test --release -p phy-demo -- --ignored"]
#[test]
fn repeat_run_is_deterministic() {
    // 两次独立构造 + 独立运行,相同步数,断言逐元素一致(容差 1e-9)。
    let a = run_fluid(200, 0.005);
    let b = run_fluid(200, 0.005);
    assert_eq!(a.len(), b.len(), "粒子数不一致");
    for (x, y) in a.iter().zip(b.iter()) {
        assert!(
            (x - y).abs() < 1e-9,
            "重复运行状态分歧: {} vs {} (差 {})",
            x,
            y,
            (x - y).abs()
        );
    }
}

#[ignore = "heavy numerical regression; run: cargo test --release -p phy-demo -- --ignored"]
#[test]
fn save_reload_replay_is_deterministic() {
    // 直跑路径:step 50 -> 存档(50步状态) -> step 50 -> 共 100 步。
    let mut s1 = Scene::new();
    s1.set_mode(DemoMode::Fluid);
    for _ in 0..50 {
        s1.world.step(0.005);
    }
    let json = s1.save_string().expect("save to string");
    for _ in 0..50 {
        s1.world.step(0.005);
    }
    let direct = fluid_state(&s1.world);

    // 重放路径:读档(50步) -> step 50 -> 共 100 步。
    let mut s2 = Scene::new();
    s2.load_string(&json).expect("load from string");
    for _ in 0..50 {
        s2.world.step(0.005);
    }
    let replay = fluid_state(&s2.world);

    assert_eq!(direct.len(), replay.len(), "重放粒子数不一致");
    for (x, y) in direct.iter().zip(replay.iter()) {
        // 存档经 JSON f64 roundtrip 有 ~1e-9 级误差,容差放宽到 1e-6。
        assert!(
            (x - y).abs() < 1e-6,
            "存档重放状态分歧: {} vs {} (差 {})",
            x,
            y,
            (x - y).abs()
        );
    }
}

#[ignore = "heavy numerical regression; run: cargo test --release -p phy-demo -- --ignored"]
#[test]
fn rigid_fluid_replay_is_deterministic() {
    // 含刚体+流体耦合的 All 模式同样应可复现。
    let run = || {
        let mut scene = Scene::new();
        scene.set_mode(DemoMode::All);
        for _ in 0..150 {
            scene.world.step(0.005);
        }
        fluid_state(&scene.world)
    };
    let a = run();
    let b = run();
    assert_eq!(a.len(), b.len());
    for (x, y) in a.iter().zip(b.iter()) {
        assert!((x - y).abs() < 1e-9, "All 模式重复运行分歧: {} vs {}", x, y);
    }
}
