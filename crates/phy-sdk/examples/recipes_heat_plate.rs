//! 用途: 标量热场扩散 —— 在金属板中心持续注入热源, 观察热量随扩散方程外溢。
//! 默认网格 nx=32, 间距 dx=0.2 (见 `PhysicsBuilder::field`)。
//!
//! 运行: `cargo run -p phy-sdk --example recipes_heat_plate`
//!
//! 坐标契约: 右手系, +Y 向上, 单位米(m)。

use phy_sdk::{PhysicsBuilder, World, WorldExt};

fn main() {
    // 1) 启用热场子系统。
    let mut world: World<f64> = PhysicsBuilder::new().field().build();

    // 2) 在网格正中持续注入热源(每步累加)。
    {
        let heat = world.field_mut().expect("热场子系统");
        let c = heat.field.nx / 2; // 中心格点索引
        heat.field.add_source(c, c, c, 10.0);
    }

    // 3) 推进仿真并抽样(距热源 3 格处的温度应随时间升高)。
    let dt = 1.0 / 60.0;
    for step in 0..240 {
        {
            // 每步补一点源, 模拟持续加热。
            let heat = world.field_mut().expect("热场子系统");
            let c = heat.field.nx / 2;
            heat.field.add_source(c, c, c, 10.0);
        }
        world.step(dt);

        if step % 60 == 0 {
            let heat = world.field().unwrap();
            let c = heat.field.nx / 2;
            let center = heat.field.sample(c, c, c);
            let edge = heat.field.sample((c + 3).min(heat.field.nx - 1), c, c);
            println!(
                "[SIM] step={:>3} 中心温={:.3} 边缘温={:.3}",
                step, center, edge
            );
        }
    }
    println!("[SIM] 完成: 热扩散 alpha={}", world.field().unwrap().alpha);
}
