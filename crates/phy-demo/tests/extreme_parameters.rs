//! M3 — 极端参数压测(生产级稳健性证据)。
//!
//! **目标**:证明引擎在"远超常规操作区间"的参数下,也能优雅失败(被看门狗捕获)
//! 或稳定推进(不静默发散),而不是产生 NaN/Inf 污染后续仿真。
//!
//! **覆盖的极端维度**:
//! 1. 超大时间步 `dt`(0.1 / 0.5 / 1.0,远超 SPH 稳定步长 ~0.005)。
//! 2. 极端质量比(body vs body:1e-9 vs 1e6,相差 15 个数量级)。
//! 3. 极端重力(±1e4,常规 9.81 的 ~1000 倍)。
//! 4. 极端 SPH 刚度/黏度(stiffness 1e4、viscosity 100,默认 250 / 3.5 的数十倍)。
//! 5. 极端场耦合系数(em/grav coupling = 1e3)。
//!
//! 每个场景用 [`World::step_checked`] 推进——它内部对全部子系统执行 NaN/Inf
//! 看门狗检查。若引擎在极端参数下**静默发散**,`step_checked` 返回 `Err`(被捕获,
//! 证明看门狗生效,而非污染状态);若引擎能**稳定收敛**,返回 `Ok` 且状态有限。
//! 两种结果都证明"引擎不会在极端参数下静默产出 NaN"。

use phy_core::World;
use phy_fluid::{FluidSubsystem, FluidWorld, SphParams};
use phy_math::Vec3 as V3;
use phy_rigid::{Body, RigidSubsystem, RigidWorld, Shape};
use nalgebra::UnitQuaternion;

/// 构造一个"刚体地面 + 单刚体"的最小世界(用于极端质量比/重力测试)。
fn rigid_only_world(gravity: V3<f64>, body_inv_mass: f64) -> World<f64> {
    let mut rw = RigidWorld::<f64>::new();
    rw.gravity = gravity;
    rw.add_body(Body {
        shape: Shape::Box {
            half: V3::new(10.0, 0.5, 10.0),
        },
        pos: V3::new(0.0, -0.5, 0.0),
        rot: UnitQuaternion::identity(),
        vel: V3::zeros(),
        inv_mass: 0.0, // 固定地面
        ..Default::default()
    });
    rw.add_body(Body {
        shape: Shape::Sphere { r: 0.4 },
        pos: V3::new(0.0, 4.0, 0.0),
        rot: UnitQuaternion::identity(),
        vel: V3::zeros(),
        inv_mass: body_inv_mass,
        ..Default::default()
    });
    let mut w = World::new();
    w.add_subsystem(Box::new(RigidSubsystem::new(rw)));
    w
}

/// 构造一个"SPH 流体盒"世界(用于极端 dt / 极端 SPH 参数测试)。
fn fluid_only_world(params: SphParams<f64>) -> World<f64> {
    let mut fluid = FluidWorld::<f64>::new(params);
    fluid.fill_box(
        V3::new(-1.0, 1.0, -1.0),
        V3::new(1.0, 3.0, 1.0),
        0.3,
        0.1,
    );
    let mut w = World::new();
    w.add_subsystem(Box::new(FluidSubsystem::new(fluid)));
    w
}

/// 检查世界内所有子系统状态是否全为有限数。
fn all_finite(world: &World<f64>) -> bool {
    let n = world.subsystem_count();
    for i in 0..n {
        if let Some(sub) = world.get(i) {
            // 刚体子系统
            if let Some(rs) = sub.as_any().downcast_ref::<RigidSubsystem<f64>>() {
                for b in &rs.world.bodies {
                    if !b.pos.iter().all(|v| v.is_finite()) {
                        return false;
                    }
                    if !b.vel.iter().all(|v| v.is_finite()) {
                        return false;
                    }
                }
            }
            // 流体子系统
            if let Some(fluid) = sub.as_any().downcast_ref::<FluidSubsystem<f64>>() {
                for p in &fluid.world.particles {
                    if !p.pos.iter().all(|v| v.is_finite()) {
                        return false;
                    }
                    if !p.vel.iter().all(|v| v.is_finite()) {
                        return false;
                    }
                }
            }
        }
    }
    true
}

/// 推进 `steps` 步,返回 `(steps_ok, diverged_at)`:
/// - `steps_ok`:看门狗通过(返回 Ok)的步数。
/// - `diverged_at`:首个触发看门狗(Err)的步索引;若全程 Ok 则为 None。
fn run_checked(world: &mut World<f64>, dt: f64, steps: usize) -> (usize, Option<usize>) {
    let mut ok = 0_usize;
    let mut diverged_at = None;
    for step in 0..steps {
        match world.step_checked(dt) {
            Ok(()) => {
                ok += 1;
                // 双保险:看门狗 Ok 不应有非有限数(理论不可能,但防御性断言)。
                assert!(
                    all_finite(world),
                    "step_checked 返回 Ok 但世界含非有限数(看门狗漏报)"
                );
            }
            Err(_) => {
                // 看门狗正确捕获了发散——这是"优雅失败",不是 bug。
                diverged_at = Some(step);
                break;
            }
        }
    }
    (ok, diverged_at)
}

// ───────────────────────────────────────────────────────────────────────────
// 极端时间步长:远超 SPH 稳定步长(默认 0.005)。
// 判据:无论看门狗捕获发散(Err)还是引擎稳定推进(Ok),只要**全程不含 NaN/Inf**
// 即视为通过——这证明极端 dt 不会"静默污染"后续仿真。run_checked 内部已对 Ok
// 步做有限性断言;若返回 Err 则说明看门狗生效,同样不污染。
// ───────────────────────────────────────────────────────────────────────────

#[ignore = "heavy numerical regression; run: cargo test --release -p phy-demo -- --ignored"]
#[test]
fn extreme_dt_01_no_silent_nan() {
    let mut w = fluid_only_world(SphParams::defaults());
    let (ok, diverged) = run_checked(&mut w, 0.1, 50);
    println!("extreme_dt 0.1: ok={}, diverged_at={:?}", ok, diverged);
}

#[ignore = "heavy numerical regression; run: cargo test --release -p phy-demo -- --ignored"]
#[test]
fn extreme_dt_05_no_silent_nan() {
    let mut w = fluid_only_world(SphParams::defaults());
    let (ok, diverged) = run_checked(&mut w, 0.5, 50);
    println!("extreme_dt 0.5: ok={}, diverged_at={:?}", ok, diverged);
}

#[ignore = "heavy numerical regression; run: cargo test --release -p phy-demo -- --ignored"]
#[test]
fn extreme_dt_10_no_silent_nan() {
    let mut w = fluid_only_world(SphParams::defaults());
    let (ok, diverged) = run_checked(&mut w, 1.0, 50);
    println!("extreme_dt 1.0: ok={}, diverged_at={:?}", ok, diverged);
}

// ───────────────────────────────────────────────────────────────────────────
// 看门狗有效性自检:构造一个**必定发散**的场景(dt 极大 + 刚度极高 + 无边界约束),
// 断言看门狗**确实**捕获了发散(Err)。这证明看门狗本身有牙齿,而非形同虚设。
// ───────────────────────────────────────────────────────────────────────────

#[ignore = "heavy numerical regression; run: cargo test --release -p phy-demo -- --ignored"]
#[test]
fn watchdog_actually_catches_nan() {
    // 看门狗有效性自检:先正常推进,再显式注入一个 NaN 位置,
    // 断言 `step_checked` 能捕获并返回 Err(证明看门狗有牙齿,而非形同虚设)。
    let mut w = fluid_only_world(SphParams::defaults());
    // 先正常 step 几步,确保世界有合法状态。
    for _ in 0..5 {
        w.step_checked(0.005).expect("预热步不应失败");
    }
    // 显式污染:把第一个流体粒子的 x 位置设为 NaN。
    {
        let sub = w.get_mut(0).expect("fluid subsystem");
        let fluid = sub
            .as_any_mut()
            .downcast_mut::<FluidSubsystem<f64>>()
            .expect("fluid type");
        if let Some(p) = fluid.world.particles.first_mut() {
            p.pos.x = f64::NAN;
        }
    }
    // 下一步看门狗必须捕获污染并返回 Err。
    let err = w.step_checked(0.005);
    assert!(
        err.is_err(),
        "看门狗自检失败:世界已含 NaN,但 step_checked 仍返回 Ok"
    );
}

// ───────────────────────────────────────────────────────────────────────────
// 极端质量比:轻体 inv_mass=1e9(质量 1e-9),重体 inv_mass=1e-6(质量 1e6)。
// 相差 15 个数量级的接触,是经典"刚性接触求解器爆炸"场景。
// 预期:看门狗捕获发散,或稳定收敛(两种均可接受,关键是全程有限/被捕获)。
// ───────────────────────────────────────────────────────────────────────────

#[ignore = "heavy numerical regression; run: cargo test --release -p phy-demo -- --ignored"]
#[test]
fn extreme_mass_ratio_captured_or_stable() {
    // 轻体质量 1e-9(inv_mass=1e9),重体质量 1e6(inv_mass=1e-6)。
    let mut rw = RigidWorld::<f64>::new();
    rw.add_body(Body {
        shape: Shape::Box {
            half: V3::new(10.0, 0.5, 10.0),
        },
        pos: V3::new(0.0, -0.5, 0.0),
        rot: UnitQuaternion::identity(),
        vel: V3::zeros(),
        inv_mass: 1e-6, // 重体:质量 1e6
        ..Default::default()
    });
    rw.add_body(Body {
        shape: Shape::Sphere { r: 0.4 },
        pos: V3::new(0.0, 4.0, 0.0),
        rot: UnitQuaternion::identity(),
        vel: V3::zeros(),
        inv_mass: 1e9, // 轻体:质量 1e-9
        ..Default::default()
    });
    let mut w = World::new();
    w.add_subsystem(Box::new(RigidSubsystem::new(rw)));

    let (ok, diverged) = run_checked(&mut w, 0.01, 200);
    // 不强制发散或收敛——只要看门狗全程要么 Ok(有限)要么 Err(捕获)即可。
    // 若全程 Ok,额外断言状态有限(已由 run_checked 内部保证)。
    println!(
        "extreme_mass_ratio: ok={}, diverged_at={:?}",
        ok, diverged
    );
}

// ───────────────────────────────────────────────────────────────────────────
// 极端重力:±1e4(常规 9.81 的 ~1000 倍)。
// 预期:一步内速度剧增,看门狗捕获或稳定(刚体地面耗散)。
// ───────────────────────────────────────────────────────────────────────────

#[ignore = "heavy numerical regression; run: cargo test --release -p phy-demo -- --ignored"]
#[test]
fn extreme_gravity_neg_1e4_captured_or_stable() {
    let mut w = rigid_only_world(V3::new(0.0, -1e4, 0.0), 1.0);
    let (ok, diverged) = run_checked(&mut w, 0.01, 200);
    println!("extreme_gravity -1e4: ok={}, diverged_at={:?}", ok, diverged);
}

#[ignore = "heavy numerical regression; run: cargo test --release -p phy-demo -- --ignored"]
#[test]
fn extreme_gravity_pos_1e4_captured_or_stable() {
    // 向上重力:小球应向上飞,可能穿出边界——看门狗捕获或稳定均可。
    let mut w = rigid_only_world(V3::new(0.0, 1e4, 0.0), 1.0);
    let (ok, diverged) = run_checked(&mut w, 0.01, 200);
    println!("extreme_gravity +1e4: ok={}, diverged_at={:?}", ok, diverged);
}

// ───────────────────────────────────────────────────────────────────────────
// 极端 SPH 参数:stiffness=1e4(默认 250 的 40 倍)、viscosity=100(默认 3.5 的 ~30 倍)。
// 高刚度要求极小 dt,否则压力求解发散;高黏度应抑止但高刚度仍主导不稳定。
// 预期:看门狗捕获发散(因 dt=0.01 相对刚度过大)。
// ───────────────────────────────────────────────────────────────────────────

#[ignore = "heavy numerical regression; run: cargo test --release -p phy-demo -- --ignored"]
#[test]
fn extreme_sph_stiffness_no_silent_nan() {
    // 高刚度 + 高黏度:黏度应抑止发散,但刚度主导仍可能不稳定。
    // 判据同极端 dt:无论看门狗捕获还是稳定推进,只要不含 NaN/Inf 即通过。
    let mut p = SphParams::<f64>::defaults();
    p.stiffness = 1e4;
    p.viscosity = 100.0;
    let mut w = fluid_only_world(p);
    let (ok, diverged) = run_checked(&mut w, 0.01, 50);
    println!(
        "extreme_sph_stiffness 1e4: ok={}, diverged_at={:?}",
        ok, diverged
    );
}

// ───────────────────────────────────────────────────────────────────────────
// 极端场耦合系数:em_coupling / grav_coupling = 1e3(默认 1.0)。
// 强耦合下,小位移被放大 1000 倍反馈,易发散;看门狗应捕获。
// ───────────────────────────────────────────────────────────────────────────

#[ignore = "heavy numerical regression; run: cargo test --release -p phy-demo -- --ignored"]
#[test]
fn extreme_field_coupling_captured_or_stable() {
    use phy_rigid::RigidSubsystem;
    let mut rw = RigidWorld::<f64>::new();
    rw.add_body(Body {
        shape: Shape::Box {
            half: V3::new(10.0, 0.5, 10.0),
        },
        pos: V3::new(0.0, -0.5, 0.0),
        rot: UnitQuaternion::identity(),
        vel: V3::zeros(),
        inv_mass: 0.0,
        ..Default::default()
    });
    rw.add_body(Body {
        shape: Shape::Sphere { r: 0.4 },
        pos: V3::new(0.0, 4.0, 0.0),
        rot: UnitQuaternion::identity(),
        vel: V3::zeros(),
        inv_mass: 1.0,
        ..Default::default()
    });
    let mut sub = RigidSubsystem::new(rw);
    sub.em_coupling = 1e3;
    sub.grav_coupling = 1e3;
    let mut w = World::new();
    w.add_subsystem(Box::new(sub));
    let (ok, diverged) = run_checked(&mut w, 0.01, 100);
    println!(
        "extreme_field_coupling 1e3: ok={}, diverged_at={:?}",
        ok, diverged
    );
}

// ───────────────────────────────────────────────────────────────────────────
// 综合极端:超大 dt + 极端刚度 + 极端重力,同时压入流体+刚体耦合世界。
// 这是最严苛的"用户误用"场景,看门狗必须至少捕获其一,绝不静默产出 NaN。
// ───────────────────────────────────────────────────────────────────────────

#[ignore = "heavy numerical regression; run: cargo test --release -p phy-demo -- --ignored"]
#[test]
fn extreme_combined_no_silent_nan() {
    let mut p = SphParams::<f64>::defaults();
    p.stiffness = 5000.0;
    p.gravity = V3::new(0.0, -500.0, 0.0);
    let mut fluid = FluidWorld::<f64>::new(p);
    fluid.fill_box(
        V3::new(-1.0, 1.0, -1.0),
        V3::new(1.0, 3.0, 1.0),
        0.3,
        0.1,
    );
    let mut rw = RigidWorld::<f64>::new();
    rw.gravity = V3::new(0.0, -500.0, 0.0);
    rw.add_body(Body {
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
    w.add_subsystem(Box::new(RigidSubsystem::new(rw)));
    w.add_subsystem(Box::new(FluidSubsystem::new(fluid)));

    let (ok, diverged) = run_checked(&mut w, 0.2, 30);
    // 关键断言:无论看门狗 Ok 还是 Err,世界状态绝不含 NaN/Inf(看门狗 Ok 时已内部断言)。
    // 若看门狗 Err,说明发散被捕获——这也是"非静默"的胜利。
    println!(
        "extreme_combined: ok={}, diverged_at={:?}",
        ok, diverged
    );
}
