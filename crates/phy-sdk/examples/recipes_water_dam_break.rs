//! 用途: SPH 流体溃坝 —— 一排静止水柱在重力下塌落铺开。
//!
//! 运行: `cargo run -p phy-sdk --example recipes_water_dam_break`
//!
//! 坐标契约: 右手系, +Y 向上, 单位米(m); 重力 -Y 9.81。

use phy_math::Vec3;
use phy_sdk::{PhysicsBuilder, World, WorldExt};

fn main() {
    // 1) 启用流体子系统。
    let mut world: World<f64> = PhysicsBuilder::new().fluid().build();

    // 2) 用 SPH 参数 + fill_box 灌粒子(由目标粒子数反推间距, 避免立方爆炸)。
    {
        let fluid = world.fluid_mut().expect("流体子系统");
        let half = 3.0f64;
        let n_fluid = 1500usize;
        let spacing = (8.0 * half.powi(3) / n_fluid as f64).powf(1.0 / 3.0);
        // 水柱放在左半边、靠上, 模拟"坝后蓄水"。
        fluid.world.fill_box(
            Vec3::new(-half, 1.0, -half),
            Vec3::new(0.0, 1.0 + 2.0 * half, half),
            spacing,
            0.1,
        );
    }

    // 3) 推进仿真(循环内只读取, 用不可变句柄打印)。
    let dt = 1.0 / 60.0;
    for step in 0..240 {
        world.step(dt);
        if step % 60 == 0 {
            let n = world
                .fluid()
                .map(|f| f.world.particles.len())
                .unwrap_or(0);
            println!("[SIM] step={:>3} 粒子数={}", step, n);
        }
    }
    let total = world.fluid().map(|f| f.world.particles.len()).unwrap_or(0);
    println!("[SIM] 完成: 溃坝 {} 粒子", total);
}
