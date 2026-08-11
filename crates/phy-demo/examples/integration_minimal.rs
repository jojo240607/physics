//! M4 交付 — 业务集成最小骨架(不依赖 `phy-demo` 的 `Scene` 封装)。
//!
//! 展示业务方如何只用底层公共 crate 组合物理世界:
//!   `phy-core`(World 抽象) + `phy-fluid`(SPH) + `phy-granular`(PBD)
//!
//! 运行:`cargo run -p phy-demo --example integration_minimal`
//!
//! 这是"可嵌入 SDK"的最小可用入口模板,业务方可据此封装自己的游戏/仿真循环。

use phy_core::World;
use phy_fluid::{FluidSubsystem, FluidWorld, SphParams};
use phy_granular::{subsystem::GranularSubsystem, world::GranularWorld};
use phy_math::Vec3;

fn main() {
    // 1) 构造一个物理世界(持有所有子系统与共享状态)
    let mut world: World<f64> = World::new();

    // 2) 添加 SPH 流体子系统(自由液面,带重力)
    let fluid_params = SphParams::<f64>::defaults();
    let mut fluid = FluidWorld::new(fluid_params);
    // 固定盒子 + 由目标粒子数反推间距,避免 fill_box 立方爆炸
    let half = 5.0f64;
    let n_fluid = 2000usize;
    let spacing = (8.0 * half.powi(3) / n_fluid as f64).powf(1.0 / 3.0);
    fluid.fill_box(
        Vec3::new(-half, 1.0, -half),
        Vec3::new(half, 1.0 + 2.0 * half, half),
        spacing,
        0.1,
    );
    world.add_subsystem(Box::new(FluidSubsystem::new(fluid)));

    // 3) 添加 PBD 颗粒子系统(落体堆积)
    let mut granular = GranularWorld::new();
    granular.fill_grid(500, 0.3, 1.0, 1.05);
    world.add_subsystem(Box::new(GranularSubsystem::new(granular)));

    // 4) 仿真循环(业务侧固定步长;真实游戏应接渲染帧率 + 子步)
    let dt = 1.0 / 60.0;
    let total_steps = 120; // 2 秒
    for step in 0..total_steps {
        // step_checked 提供看门狗:任一子系统出现 NaN/Inf 即返回 Err,
        // 便于业务侧安全降级(如回滚 / 暂停)而非污染后续帧。
        if let Err(e) = world.step_checked(dt) {
            eprintln!("[SIM] 第 {} 步数值异常,安全中断: {:?}", step, e);
            break;
        }
        if step % 30 == 0 {
            println!(
                "[SIM] step={:>3} 动能={:.3}",
                step,
                world.kinetic_energy()
            );
        }
    }

    println!("[SIM] 完成: 业务集成最小骨架跑通(2000 流体 + 500 颗粒)");
}
