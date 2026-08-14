//! 用途: 软体(质点-弹簧)旗帜 —— 顶部钉扎的一张悬挂软布在重力下下垂。
//! 用 `SoftBody::from_lattice` 生成规则晶格,nz=1 即薄布料,顶层(ny-1)自动钉死。
//!
//! 运行: `cargo run -p phy-sdk --example recipes_cloth_flag`
//!
//! 坐标契约: 右手系, +Y 向上, 单位米(m); 重力 -Y 9.81。

use phy_math::Vec3;
use phy_sdk::{PhysicsBuilder, World, WorldExt};

fn main() {
    // 1) 启用软体子系统。
    let mut world: World<f64> = PhysicsBuilder::new().soft().build();

    // 2) 造一张薄软布: nx 列 × ny 行 × nz=1 层, 顶层固定(钉在旗杆上)。
    {
        let soft = world.soft_mut().expect("软体子系统");
        let cloth = phy_soft::SoftBody::from_lattice(8, 6, 1, 0.2, Vec3::new(0.0, 3.0, 0.0));
        soft.body = cloth;
    }

    // 3) 推进仿真(循环内只读取质点, 用不可变句柄)。
    let dt = 1.0 / 60.0;
    for step in 0..240 {
        world.step(dt);
        if step % 60 == 0 {
            let soft = world.soft().unwrap();
            // 质点 0 在顶部(被钉死, y 应保持 ≈3.0); 底排质点会下垂。
            let top = &soft.body.particles[0];
            let bottom = &soft.body.particles[soft.body.particles.len() - 1];
            println!(
                "[SIM] step={:>3} top.y={:.3} bottom.y={:.3}",
                step, top.pos.y, bottom.pos.y
            );
        }
    }
    println!("[SIM] 完成: 软布 {} 质点", world.soft().unwrap().body.particles.len());
}
