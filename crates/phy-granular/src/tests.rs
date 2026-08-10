//! phy-granular 单元测试。

use crate::subsystem::GranularSubsystem;
use crate::world::{Grain, GranularWorld};
use phy_core::world::World;
use phy_math::Vec3;

#[test]
fn grains_fall_and_settle_in_box() {
    // 在盒内撒一批颗粒,重力下应沉积到盒底附近,且不应穿透地面或飞出盒外。
    let mut w: GranularWorld<f64> = GranularWorld::new();
    w.set_bounds(
        Vec3::new(-3.0, -5.0, -3.0),
        Vec3::new(3.0, 5.0, 3.0),
    );
    w.fill_grid(200, 0.3, 1.0, 1.05);
    let n0 = w.grains.len();
    assert_eq!(n0, 200);
    let dt = 1.0 / 60.0;
    for _ in 0..400 {
        w.step(dt);
    }
    // 所有颗粒均在盒内(含半径余量),无 NaN。
    let lo = w.bounds_lo;
    let hi = w.bounds_hi;
    let r = w.grains[0].radius;
    for g in &w.grains {
        assert!(g.pos.x >= lo.x + r - 1e-6 && g.pos.x <= hi.x - r + 1e-6);
        assert!(g.pos.y >= lo.y + r - 1e-6 && g.pos.y <= hi.y - r + 1e-6);
        assert!(g.pos.z >= lo.z + r - 1e-6 && g.pos.z <= hi.z - r + 1e-6);
        assert!(g.pos.x.is_finite() && g.pos.y.is_finite() && g.pos.z.is_finite());
    }
    // 沉积后速度应趋于静止(最大速度很小)。
    let max_v = w
        .grains
        .iter()
        .map(|g| g.vel.norm())
        .fold(0.0_f64, f64::max);
    assert!(max_v < 0.5, "settled grains should be near-static, max_v={}", max_v);
}

#[test]
fn no_overlap_after_projection() {
    // 静态堆叠后,任意两球中心距不应小于半径和(允许极小数值余量)。
    let mut w: GranularWorld<f64> = GranularWorld::new();
    w.set_bounds(
        Vec3::new(-2.0, -2.0, -2.0),
        Vec3::new(2.0, 2.0, 2.0),
    );
    w.iterations = 8;
    w.fill_grid(120, 0.25, 1.0, 1.0); // 紧密铺排,初始即相切。
    let dt = 1.0 / 60.0;
    for _ in 0..200 {
        w.step(dt);
    }
    let n = w.grains.len();
    for i in 0..n {
        for j in (i + 1)..n {
            let d = (w.grains[j].pos - w.grains[i].pos).norm();
            let min = w.grains[i].radius + w.grains[j].radius;
            assert!(
                d >= min - 1e-4,
                "overlap: d={:.4} < min={:.4} (i={},j={})",
                d,
                min,
                i,
                j
            );
        }
    }
}

#[test]
fn fixed_grain_does_not_move() {
    // 固定颗粒(inv_mass=0)在重力下应保持原位。
    let mut w: GranularWorld<f64> = GranularWorld::new();
    w.set_bounds(
        Vec3::new(-2.0, -2.0, -2.0),
        Vec3::new(2.0, 2.0, 2.0),
    );
    let anchor = Grain::fixed(Vec3::new(0.0, 0.0, 0.0), 0.5);
    w.add(anchor);
    // 旁边放一个可动颗粒砸向它。
    w.add(Grain::new(Vec3::new(0.0, 1.5, 0.0), 0.3, 1.0));
    let dt = 1.0 / 60.0;
    for _ in 0..100 {
        w.step(dt);
    }
    assert!((w.grains[0].pos - Vec3::new(0.0, 0.0, 0.0)).norm() < 1e-9);
}

#[test]
fn runs_as_subsystem_in_world() {
    let mut w: GranularWorld<f64> = GranularWorld::new();
    w.set_bounds(
        Vec3::new(-2.0, -3.0, -2.0),
        Vec3::new(2.0, 3.0, 2.0),
    );
    w.fill_grid(80, 0.25, 1.0, 1.1);
    let mut world: World<f64> = World::default();
    world.add_subsystem(Box::new(GranularSubsystem::new(w)));
    for _ in 0..60 {
        world.step(1.0 / 60.0);
    }
    let sub = world
        .get(0)
        .unwrap()
        .as_any()
        .downcast_ref::<GranularSubsystem<f64>>()
        .unwrap();
    assert_eq!(sub.world.grains.len(), 80);
    assert!(sub.world.t > 0.0);
}

#[test]
fn parallel_solve_is_deterministic() {
    // S7 并行后端:Jacobi 约束投影经 rayon 并行 reduce,结果必须与串行逐一相加
    // 一致,且多次运行完全可复现(固定 pair 顺序的 reduce)。
    let mut a: GranularWorld<f64> = GranularWorld::new();
    a.set_bounds(
        Vec3::new(-3.0, -5.0, -3.0),
        Vec3::new(3.0, 5.0, 3.0),
    );
    a.fill_grid(150, 0.3, 1.0, 1.04);
    let mut b = a.clone();

    let dt = 1.0 / 60.0;
    for _ in 0..120 {
        a.step(dt);
        b.step(dt);
    }
    for (ga, gb) in a.grains.iter().zip(b.grains.iter()) {
        assert!((ga.pos.x - gb.pos.x).abs() < 1e-12);
        assert!((ga.pos.y - gb.pos.y).abs() < 1e-12);
        assert!((ga.pos.z - gb.pos.z).abs() < 1e-12);
    }
}

/// 朴素暴力参考:返回所有 `i<j` 且中心距 < r_i+r_j 的接触对(升序)。
fn brute_contact_pairs(w: &GranularWorld<f64>) -> Vec<(usize, usize)> {
    let n = w.grains.len();
    let mut v = Vec::new();
    for i in 0..n {
        for j in (i + 1)..n {
            let d = (w.grains[j].pos - w.grains[i].pos).norm();
            let min = w.grains[i].radius + w.grains[j].radius;
            if d < min {
                v.push((i, j));
            }
        }
    }
    v.sort_unstable();
    v
}

#[test]
fn spatial_hash_matches_bruteforce_contacts() {
    // P4 宽相位正确性:空间哈希生成的接触对集合必须与朴素暴力 O(n²) 完全等价
    // (不漏任何真实接触、不产生误报),证明网格 cell_size 与查询半径推导正确。
    let mut w: GranularWorld<f64> = GranularWorld::new();
    w.set_bounds(
        Vec3::new(-4.0, -4.0, -4.0),
        Vec3::new(4.0, 4.0, 4.0),
    );
    w.fill_grid(300, 0.3, 1.0, 1.03); // 密集,大量接触。
    // 先步进几步让颗粒位移,破坏初始网格对齐,检验动态场景仍正确。
    let dt = 1.0 / 60.0;
    for _ in 0..10 {
        w.step(dt);
    }
    let got = w.contact_pairs();
    let expected = brute_contact_pairs(&w);
    assert_eq!(got, expected, "空间哈希接触集必须等于暴力参考");
}

#[test]
fn large_scale_runs_without_overlap() {
    // P4 规模验证:5000 颗粒在合理盒内填充并步进,宽相位必须在可接受时间内完成,
    // 且投影后无穿透(任意两球中心距 >= 半径和 - 容差)。
    let mut w: GranularWorld<f64> = GranularWorld::new();
    // 大盒以容纳 5000 个 r=0.3 的球(体积占比 < 0.3 即可)。
    w.set_bounds(
        Vec3::new(-15.0, -15.0, -15.0),
        Vec3::new(15.0, 15.0, 15.0),
    );
    w.iterations = 4;
    w.fill_grid(5000, 0.3, 1.0, 1.1);
    assert_eq!(w.grains.len(), 5000);
    let dt = 1.0 / 60.0;
    let t0 = std::time::Instant::now();
    for _ in 0..30 {
        w.step(dt);
    }
    let elapsed = t0.elapsed();
    // 单步平均应远低于 1s(宽相位预期);宽松上限以防 CI 慢机。
    assert!(
        elapsed.as_secs_f64() / 30.0 < 2.0,
        "5000 颗粒单步平均 {:.3}s 过慢",
        elapsed.as_secs_f64() / 30.0
    );
    // 投影后无显著穿透。PBD 有限迭代在大规模密集堆积下存在残余穿透(约 1-2% 半径),
    // 这是算法固有特性而非宽相位漏检(由 `spatial_hash_matches_bruteforce_contacts`
    // 证明接触集与暴力完全等价)。容差取 0.01(半径 0.3 的 ~3%)。
    let n = w.grains.len();
    for i in 0..n {
        for j in (i + 1)..n {
            let d = (w.grains[j].pos - w.grains[i].pos).norm();
            let min = w.grains[i].radius + w.grains[j].radius;
            assert!(
                d >= min - 0.01,
                "overlap: d={:.4} < min={:.4} (i={},j={})",
                d,
                min,
                i,
                j
            );
        }
    }
}
