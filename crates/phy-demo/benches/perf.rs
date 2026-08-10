//! 性能基线基准(L4-1)。
//!
//! 用 criterion 建立两块“业务关心”场景的单帧步进耗时基线:
//!   - SPH 溃坝:N 体流体自由演化一帧(密度/压力/受力两遍 + 积分)。
//!   - 刚体接触:M 个刚体在重力下相互碰撞/堆叠一帧(接触约束求解)。
//!
//! 运行:`cargo bench -p phy-demo`(首次出基线,后续回归可对比数值)。
//! 这些数字给“库化后喂游戏/防战建模”提供吞吐参考点。

use criterion::{black_box, criterion_group, criterion_main, Criterion};
use nalgebra::Vector3 as Vec3;
use phy_core::World;
use phy_fluid::{FluidSubsystem, FluidWorld, SphParams};
use phy_rigid::{RigidSubsystem, RigidWorld, Body, Shape};

/// 构造 SPH 溃坝世界:在一个盒子里撒满流体粒子(不挂地面,纯自由演化)。
fn sph_dam_world(n: usize) -> World<f64> {
    let params = SphParams::<f64>::defaults();
    let mut fluid = FluidWorld::<f64>::new(params);
    // 让盒子尺寸随 n 粗略缩放,保证粒子密度近似恒定。
    let span = (n as f64).sqrt() * 0.3;
    let h = span / 2.0;
    fluid.fill_box(
        Vec3::new(-h, 1.0, -h),
        Vec3::new(h, 1.0 + span, h),
        0.3,
        0.1,
    );
    let mut w = World::new();
    w.add_subsystem(Box::new(FluidSubsystem::new(fluid)));
    w
}

/// 构造刚体接触世界:M 个盒体在重力下堆叠/碰撞(含一个静止地面)。
fn rigid_contact_world(m: usize) -> World<f64> {
    let mut rworld = RigidWorld::<f64>::new();
    // 静止地面。
    rworld.add_body(Body {
        shape: Shape::Box {
            half: Vec3::new(20.0, 0.5, 20.0),
        },
        pos: Vec3::new(0.0, -0.5, 0.0),
        rot: nalgebra::UnitQuaternion::identity(),
        vel: Vec3::zeros(),
        inv_mass: 0.0,
        ..Default::default()
    });
    // 在地面上方按网格堆 m 个小球,初始轻微错开以触发接触求解。
    let side = (m as f64).sqrt().ceil() as usize;
    let spacing = 2.2;
    for i in 0..m {
        let gx = (i % side) as f64;
        let gz = ((i / side) % side) as f64;
        let layer = (i / (side * side)) as f64;
        rworld.add_body(Body::new(
            Shape::Sphere { r: 1.0 },
            Vec3::new(
                (gx - side as f64 / 2.0) * spacing,
                1.0 + layer * 2.1,
                (gz - side as f64 / 2.0) * spacing,
            ),
            1.0 / 2.0,
        ));
    }
    let mut w = World::new();
    w.add_subsystem(Box::new(RigidSubsystem::new(rworld)));
    w
}

fn bench_sph(c: &mut Criterion) {
    let mut group = c.benchmark_group("sph_dam");
    for &n in &[500usize, 1000, 2000] {
        let mut w = sph_dam_world(n);
        group.bench_function(format!("step_n{}", n), |b| {
            b.iter(|| {
                w.step(black_box(0.01));
            })
        });
    }
    group.finish();
}

fn bench_rigid(c: &mut Criterion) {
    let mut group = c.benchmark_group("rigid_contact");
    for &m in &[64usize, 128, 256] {
        let mut w = rigid_contact_world(m);
        group.bench_function(format!("step_m{}", m), |b| {
            b.iter(|| {
                w.step(black_box(0.01));
            })
        });
    }
    group.finish();
}

criterion_group!(benches, bench_sph, bench_rigid);
criterion_main!(benches);
