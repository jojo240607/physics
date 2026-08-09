//! 存档/读档往返测试。

use crate::archive::{load_world_json, save_world_json};
use phy_core::World;
use phy_field::{Bc, HeatField, ScalarField};
use phy_rigid::shape::Body;
use phy_rigid::shape::Shape;
use phy_rigid::RigidSubsystem;
use phy_rigid::RigidWorld;
use phy_math::Vec3;

fn make_world() -> World<f64> {
    let mut w = World::new();
    // 刚体子系统:一个落地球。
    let mut rw = RigidWorld::new();
    rw.bodies.push(Body {
        shape: Shape::Sphere { r: 1.0 },
        pos: Vec3::new(0.0, 10.0, 0.0),
        rot: phy_math::na::UnitQuaternion::identity(),
        vel: Vec3::new(0.0, 0.0, 0.0),
        inv_mass: 0.5,
    
            ..Default::default()
        });
    let rigid = RigidSubsystem {
        world: rw,
        thermal_expansion: 0.0,
        heat_gain: 0.0,
        t_ref: 0.0,
        em_coupling: 0.0,
        grav_coupling: 0.0,
    };
    w.add_subsystem(Box::new(rigid));
    // 热场子系统:8³ 网格。
    let field = ScalarField::<f64>::new(8, 8, 8, 1.0, 0.0, Bc::Neumann);
    let heat = HeatField::<f64>::new(field, 1.0);
    w.add_subsystem(Box::new(heat));
    w
}

#[test]
fn world_save_load_roundtrip() {
    let w = make_world();
    let json = save_world_json(&w);
    let w2 = load_world_json::<f64>(&json);

    // 子系统数量一致。
    assert_eq!(w2.subsystem_count(), 2);

    // 刚体子系统状态可恢复。
    let r = w2
        .get(0)
        .unwrap()
        .as_any()
        .downcast_ref::<RigidSubsystem<f64>>()
        .expect("rigid subsystem restored");
    assert_eq!(r.world.bodies.len(), 1);
    assert!((r.world.bodies[0].pos.y - 10.0).abs() < 1e-12);
    assert!((r.world.bodies[0].inv_mass - 0.5).abs() < 1e-12);

    // 热场子系统状态可恢复。
    let h = w2
        .get(1)
        .unwrap()
        .as_any()
        .downcast_ref::<HeatField<f64>>()
        .expect("heat field restored");
    assert_eq!((h.field.nx, h.field.ny, h.field.nz), (8, 8, 8));
    assert!((h.field.dx - 1.0).abs() < 1e-12);
}
