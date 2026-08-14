//! 用途: 刚体入门 —— 静态地面 + 一批下落球, 读回位姿验证下落。
//! 这是 AI 最常生成的"物理仿真 Hello World"。
//!
//! 运行: `cargo run -p phy-sdk --example recipes_drop_bouncing_balls`
//!
//! 坐标契约: 右手系, +Y 向上, 单位米(m); 四元数 (w,x,y,z) 机体->世界; 重力 -Y 9.81。

use phy_math::Vec3;
use phy_sdk::{PhysicsBuilder, World, WorldExt};

fn main() {
    // 1) 声明并构建只含刚体的世界。
    let mut world: World<f64> = PhysicsBuilder::new().rigid().build();

    // 2) 按意图造物体, 不用记索引/downcast。
    world.spawn_floor(Vec3::new(20.0, 0.5, 20.0), -0.5); // 地板中心 y=-0.5, 半尺寸 20
    world.spawn_sphere(Vec3::new(0.0, 3.0, 0.0), 0.5, 1.0); // 单个下落球
    let ids = world.spawn_sphere_grid(Vec3::new(-1.0, 1.0, -1.0), 5, 0.3, 0.1, 0.1); // 5³=125 球

    // 3) 推进仿真。
    let dt = 1.0 / 60.0;
    for step in 0..180 {
        world.step(dt);
        if step % 60 == 0 {
            let r = world.rigid().unwrap();
            let ball = &r.world.bodies[ids[0].0];
            println!("[SIM] step={:>3} ball0.y={:.3}", step, ball.pos.y);
        }
    }

    // 4) 读回位姿 (pos.xyz 米; rot.wxyz 四元数)。
    let r = world.rigid().unwrap();
    let ball = &r.world.bodies[ids[0].0];
    assert!(ball.pos.y < 3.0, "球应已下落");
    println!(
        "[SIM] 完成: {} 个球, 最低球 y={:.3} (应接近地板上方)",
        r.world.bodies.len(),
        ball.pos.y
    );
}
