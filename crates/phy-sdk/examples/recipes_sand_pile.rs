//! 用途: 颗粒(PBD)堆积 —— 一批颗粒下落堆成小丘。
//!
//! 运行: `cargo run -p phy-sdk --example recipes_sand_pile`
//!
//! 坐标契约: 右手系, +Y 向上, 单位米(m); 重力 -Y 9.81。

use phy_math::Vec3;
use phy_sdk::{PhysicsBuilder, World, WorldExt};

fn main() {
    // 1) 启用颗粒子系统。
    let mut world: World<f64> = PhysicsBuilder::new().granular().build();

    // 2) 静态地面(用刚体子系统承载颗粒落点) + 颗粒云。
    //    注意: 颗粒与刚体同世界内各自 step, 这里加一块静态地板让颗粒停住。
    world.spawn_floor(Vec3::new(5.0, 0.5, 5.0), -0.5);

    {
        let granular = world.granular_mut().expect("颗粒子系统");
        // fill_grid(count, radius, mass, pack): 在原点上方立方体网格灌 count 个颗粒。
        granular.world.fill_grid(800, 0.15, 0.1, 1.05);
    }

    // 3) 推进仿真(循环内只读取, 用不可变句柄打印)。
    let dt = 1.0 / 60.0;
    for step in 0..240 {
        world.step(dt);
        if step % 60 == 0 {
            let n = world
                .granular()
                .map(|g| g.world.grains.len())
                .unwrap_or(0);
            println!("[SIM] step={:>3} 颗粒数={}", step, n);
        }
    }
    let total = world.granular().map(|g| g.world.grains.len()).unwrap_or(0);
    println!("[SIM] 完成: 颗粒堆积 {} 粒", total);
}
