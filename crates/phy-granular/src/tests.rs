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
