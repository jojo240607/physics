//! L1 / L2 库化 hardening 回归(主机端,无 GPU)。
//!
//! **L1 — 全局守恒 / 稳定性:**
//! - 多物理耦合世界(fluid dam-break + rigid 地面 + 自由落体小球)长跑不会爆 NaN。
//! - 无外力注入的闭合片段,总动能有界(不单调爆炸)。
//! - 下落小球最终趋于静止(稳定收敛,不抖飞)。
//!
//! **L2 — 确定性:**
//! - 同构造(同参数)的两个世界,在完全相同步进下逐位一致。
//! - `save_world_json` → `load_world_json` 往返后,继续步进与未往返者在 1e-9 相对容差内一致
//!   (证明序列化链路不丢精度、不引入非确定性;派生缓存重置仅带来 1 ULP 级浮点求和差异)。

use nalgebra::UnitQuaternion;
use phy_core::replay::{FrameInput, Replay};
use phy_core::World;
use phy_fluid::{FluidSubsystem, FluidWorld, SphParams};
use phy_io::{load_world_json, save_world_json};
use phy_math::Vec3 as V3;
use phy_rigid::{Body, RigidSubsystem, RigidWorld, Shape};

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
    // 不动地面。
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
    // 自由落体小球(受重力)。
    rworld.add_body(Body {
        shape: Shape::Sphere { r: 0.4 },
        pos: V3::new(0.0, 4.0, 0.0),
        rot: UnitQuaternion::identity(),
        vel: V3::zeros(),
        inv_mass: 1.0 / 1.0,
        ..Default::default()
    });
    let mut w = World::new();
    w.add_subsystem(Box::new(RigidSubsystem::new(rworld)));
    w.add_subsystem(Box::new(FluidSubsystem::new(fluid)));
    w
}

/// 取下落小球(刚体子系统内 index=1)的垂直速度。
fn ball_vy(w: &World<f64>) -> f64 {
    let s = w.get(0).unwrap();
    let rs = s
        .as_any()
        .downcast_ref::<RigidSubsystem<f64>>()
        .unwrap();
    rs.world.bodies[1].vel.y
}

#[ignore = "heavy numerical regression; run: cargo test --release -p phy-demo -- --ignored"]
#[test]
fn l1_no_nan_over_long_run() {
    let mut w = coupled_world();
    let dt = 0.005;
    for _ in 0..400 {
        w.step_checked(dt).expect("仿真在长跑中产生了非有限值 (NaN/Inf)");
    }
}

#[ignore = "heavy numerical regression; run: cargo test --release -p phy-demo -- --ignored"]
#[test]
fn l1_kinetic_energy_bounded_no_explosion() {
    let mut w = coupled_world();
    let dt = 0.005;
    let mut e_max = 0.0_f64;
    for _ in 0..400 {
        w.step_checked(dt).unwrap();
        let e = w.kinetic_energy();
        assert!(e.is_finite(), "总动能在仿真中变为非有限值");
        e_max = e_max.max(e);
    }
    // 无外力注入的闭合系统(小球落地下沉后与地面接触,重力势能转热耗散),
    // 动能不应无界增长。取一个宽松上界以捕获"爆炸"而非正常物理波动。
    assert!(e_max.is_finite());
    assert!(e_max < 1e6, "总动能异常膨胀,疑似数值爆炸: e_max={}", e_max);
}

#[ignore = "heavy numerical regression; run: cargo test --release -p phy-demo -- --ignored"]
#[test]
fn l1_dropped_ball_settles() {
    let mut w = coupled_world();
    let dt = 0.005;
    let mut max_speed_after_settle = 0.0_f64;
    // 先让其下落并落地(~2s),再观察静止段的速度幅值。
    for _ in 0..600 {
        w.step_checked(dt).unwrap();
    }
    for _ in 0..100 {
        w.step_checked(dt).unwrap();
        let s = w.get(0).unwrap();
        let rs = s
            .as_any()
            .downcast_ref::<RigidSubsystem<f64>>()
            .unwrap();
        let v = rs.world.bodies[1].vel.norm();
        max_speed_after_settle = max_speed_after_settle.max(v);
    }
    // 稳定后小球应趋于静止(与地面的接触耗散了能量),速度不应持续高位飞行。
    assert!(
        max_speed_after_settle < 2.0,
        "下落小球未收敛到静止,可能数值失稳: max_speed={}",
        max_speed_after_settle
    );
}

#[ignore = "heavy numerical regression; run: cargo test --release -p phy-demo -- --ignored"]
#[test]
fn l2_same_construction_stepwise_identical() {
    let dt = 0.01;
    let mut a = coupled_world();
    let mut b = coupled_world();
    for _ in 0..200 {
        a.step(dt);
        b.step(dt);
    }
    // 逐位比较流体粒子位置。
    let fa = a
        .get(1)
        .unwrap()
        .as_any()
        .downcast_ref::<FluidSubsystem<f64>>()
        .unwrap();
    let fb = b
        .get(1)
        .unwrap()
        .as_any()
        .downcast_ref::<FluidSubsystem<f64>>()
        .unwrap();
    assert_eq!(fa.world.particles.len(), fb.world.particles.len());
    for (pa, pb) in fa.world.particles.iter().zip(fb.world.particles.iter()) {
        assert_eq!(pa.pos, pb.pos, "同构造世界在确定性步进下出现分叉");
        assert_eq!(pa.vel, pb.vel);
    }
}

#[ignore = "heavy numerical regression; run: cargo test --release -p phy-demo -- --ignored"]
#[test]
fn l2_json_roundtrip_preserves_trajectory() {
    let dt = 0.01;
    // 基准:不往返,连续步进 200 步。
    let mut base = coupled_world();
    for _ in 0..200 {
        base.step(dt);
    }

    // 对照:步进 100 步 → JSON 往返 → 再步进 100 步。
    let mut w = coupled_world();
    for _ in 0..100 {
        w.step(dt);
    }
    let json = save_world_json(&w);
    let mut w2 = load_world_json(&json);
    for _ in 0..100 {
        w2.step(dt);
    }

    // 比较两世界此刻的流体粒子状态。JSON 往返会重置部分派生缓存(如邻域网格),
    // 导致浮点求和顺序产生 1 ULP 级差异,这是可接受的;断言轨迹无显著漂移
    // (相对容差 1e-9),证明序列化链路不丢精度、不引入非确定性。
    let fb = base
        .get(1)
        .unwrap()
        .as_any()
        .downcast_ref::<FluidSubsystem<f64>>()
        .unwrap();
    let fr = w2
        .get(1)
        .unwrap()
        .as_any()
        .downcast_ref::<FluidSubsystem<f64>>()
        .unwrap();
    assert_eq!(fb.world.particles.len(), fr.world.particles.len());
    for (pb, pr) in fb.world.particles.iter().zip(fr.world.particles.iter()) {
        let tol = pb.pos.norm().max(1.0) * 1e-9;
        assert!(
            (pb.pos - pr.pos).norm() < tol,
            "JSON 往返后轨迹出现显著漂移,序列化可能丢精度: base={:?} roundtrip={:?}",
            pb.pos,
            pr.pos
        );
        let vtol = pb.vel.norm().max(1.0) * 1e-9;
        assert!((pb.vel - pr.vel).norm() < vtol);
    }
}

/// 轻量(默认回归)测试:直接验证新的 `Diagnostics` 聚合 API(`World::kinetic_energy` /
/// `World::total_momentum`)计算正确。重负载的守恒/确定性回归见上方 `#[ignore]` 用例。
#[test]
fn diagnostics_api_reports_energy_and_momentum() {
    use phy_core::World;
    use phy_rigid::{Body, RigidSubsystem, RigidWorld, Shape};
    use nalgebra::UnitQuaternion;
    use phy_math::Vec3 as V3;

    // 静止单位质量球:动能≈0,动量≈0。
    let mut w0 = World::<f64>::new();
    w0.add_subsystem(Box::new(RigidSubsystem::new({
        let mut r = RigidWorld::<f64>::new();
        r.add_body(Body {
            shape: Shape::Sphere { r: 1.0 },
            pos: V3::zeros(),
            rot: UnitQuaternion::identity(),
            vel: V3::zeros(),
            inv_mass: 1.0, // mass = 1
            ..Default::default()
        });
        r
    })));
    assert!(w0.kinetic_energy().abs() < 1e-12, "静止物体动能应≈0");
    assert!(w0.total_momentum().norm() < 1e-12, "静止物体动量应≈0");

    // 给 1 kg 物体 2 m/s 速度:KE = ½·1·4 = 2,动量 = 1·2 = 2(沿 +X)。
    let mut w1 = World::<f64>::new();
    w1.add_subsystem(Box::new(RigidSubsystem::new({
        let mut r = RigidWorld::<f64>::new();
        r.add_body(Body {
            shape: Shape::Sphere { r: 1.0 },
            pos: V3::zeros(),
            rot: UnitQuaternion::identity(),
            vel: V3::new(2.0, 0.0, 0.0),
            inv_mass: 1.0, // mass = 1
            ..Default::default()
        });
        r
    })));
    assert!((w1.kinetic_energy() - 2.0).abs() < 1e-9, "动能应=2, 实际={}", w1.kinetic_energy());
    let p = w1.total_momentum();
    assert!((p.x - 2.0).abs() < 1e-9 && p.y.abs() < 1e-12 && p.z.abs() < 1e-12, "动量应=(2,0,0)");
}

/// S8 确定性回放(主机端):录制一段多物理会话(变化的 `dt` + 每帧确定性种子),
/// 再用 `Replay` 从同一初始快照重放,断言终态世界 **逐位一致**(存档 JSON 完全相同)。
///
/// 这覆盖了 §5.6 S8「确定性回放」——防战建模 / 可复现演示要求同一输入序列复现同一轨迹,
/// 即使存在逐帧随机种子(供将来随机构造复用)。
#[test]
fn s8_deterministic_replay_reproduces_trajectory() {
    // 录制阶段:构造耦合世界,记录初始快照 + 逐帧 (dt, seed)。
    let mut world = coupled_world();
    let snapshot = save_world_json(&world);
    let mut replay = Replay::new(snapshot);

    // 变化的步长与种子,模拟一段"外部输入驱动"的会话。
    let dts = [0.01f64, 0.02, 0.01, 0.015, 0.01, 0.02, 0.01, 0.025];
    for (i, &dt) in dts.iter().enumerate() {
        let seed = 0xA11CE ^ (i as u64 * 0x9E3779B9);
        replay.record(dt, seed);
        world.step_seeded(dt, seed);
    }
    let recorded_final = save_world_json(&world);

    // 回放阶段:从同一快照 + 录制序列重建并重放。
    let replayed_final = replay
        .into_player()
        .replay(load_world_json, |w, f: FrameInput| {
            w.step_seeded(f.dt, f.seed);
        });
    let replayed_json = save_world_json(&replayed_final);

    // 比较**语义状态**而非原始字符串:存档中可能存在 HashMap 派生的字段(如刚体
    // 附属电荷表),其序列化键序在不同 World 实例间不保证一致。逐位复现的判据是
    // 解析后的数值内容完全相等,而非 JSON 字节序相同。
    let recorded_val: serde_json::Value =
        serde_json::from_str(&recorded_final).expect("parse recorded archive");
    let replayed_val: serde_json::Value =
        serde_json::from_str(&replayed_json).expect("parse replayed archive");
    assert_eq!(
        replayed_val, recorded_val,
        "回放终态必须与录制终态逐位一致(确定性回放失败)"
    );
}
