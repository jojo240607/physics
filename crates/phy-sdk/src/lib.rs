//! # phy-sdk —— 物理引擎集成 SDK
//!
//! 这是面向**业务集成**的便利层(M4 里程碑),目标是把散落在多个 `phy-*` crate 的
//! 子系统拼装细节屏蔽掉,让接入方拿到一个**统一入口 + 开箱即用工厂**:
//!
//! - 通过 [`PhysicsBuilder`] 一行声明要启用哪些子系统,自动创建内部世界并挂载;
//! - 通过 [`World`] / [`step`] / [`step_checked`] 驱动仿真,或用 [`World::step`];
//! - 通过 [`save_world`] / [`load_world`] 做存档(兼容 `phy-io` 的 ron 格式);
//! - 运行时用 [`get_as`] / [`get_as_mut`] 把子系统取回强类型句柄做细粒度控制。
//!
//! 所有物理算法实现仍由各 `phy-*` crate 负责,本 crate **只做重导出与便捷封装**,
//! 不重新实现任何物理,保证与底层 crate 行为字节级一致。
//!
//! ## 快速集成
//!
//! ```
//! use phy_sdk::{PhysicsBuilder, World, get_as_mut, granular::{GranularSubsystem, Grain}};
//!
//! // 1) 声明要用的子系统,SDK 自动建好内部世界并挂载。
//! let mut world: World<f64> = PhysicsBuilder::new().granular().build();
//!
//! // 2) 往颗粒世界添加若干颗粒(经 subsystem 句柄访问内部世界)。
//! //    索引 0 = 第一个被声明的子系统(此处即颗粒)。Grain::new(pos, radius, mass)。
//! if let Some(g) = get_as_mut::<GranularSubsystem<f64>>(&mut world, 0) {
//!     for i in 0..100 {
//!         let x = (i % 10) as f64 * 0.2 - 1.0;
//!         let y = (i / 10) as f64 * 0.2;
//!         g.world.add(Grain::new(nalgebra::Vector3::new(x, y, 0.0), 0.1, 1.0));
//!     }
//! }
//!
//! // 3) 推进 30 步(带看门狗的内部 world.step(dt))。
//! for _ in 0..30 {
//!     world.step(1.0 / 60.0);
//! }
//!
//! // 4) 取回结果:颗粒世界里的体数量。
//! let n = get_as_mut::<GranularSubsystem<f64>>(&mut world, 0)
//!     .map(|g| g.world.grains.len())
//!     .unwrap_or(0);
//! assert_eq!(n, 100, "应已添加 100 个颗粒");
//! ```
//!
//! ## 多子系统组合
//!
//! ```
//! use phy_sdk::{
//!     PhysicsBuilder, World, get_as,
//!     fluid::FluidSubsystem, granular::GranularSubsystem,
//! };
//!
//! // 同时启用流体 + 颗粒(同一 World 内多子系统各自独立 step)。
//! let mut world: World<f64> =
//!     PhysicsBuilder::new().fluid().granular().build();
//!
//! for _ in 0..10 {
//!     world.step(1.0 / 60.0);
//! }
//!
//! // 流体子系统在索引 0,颗粒在索引 1(按 build 时声明顺序)。
//! assert!(get_as::<FluidSubsystem<f64>>(&world, 0).is_some());
//! assert!(get_as::<GranularSubsystem<f64>>(&world, 1).is_some());
//! ```
//!
//! ## 存档(ron/JSON)
//!
//! ```
//! use phy_sdk::{PhysicsBuilder, World, save_world_json, load_world_json};
//!
//! let world: World<f64> = PhysicsBuilder::new().granular().build();
//! let s = save_world_json(&world); // 序列化到字符串(也支持 save_world 写文件)
//! let restored: World<f64> = load_world_json(&s); // 从字符串反序列化
//! // 序列化往返一致即证明存档/读档闭环。
//! assert_eq!(save_world_json(&restored), s);
//! ```
//!
//! ## 看门狗与数值稳定性
//!
//! 生产环境建议用 [`step_checked`],它会在任一子系统产生 `NaN`/`Inf` 或卡死
//! (`delta_t` 推进为 0)时返回 [`WorldError`],而非静默产出坏数据:
//!
//! ```
//! use phy_sdk::{PhysicsBuilder, World};
//!
//! let mut world: World<f64> = PhysicsBuilder::new().fluid().build();
//! if let Err(e) = world.step_checked(1.0 / 60.0) {
//!     eprintln!("仿真异常: {:?}", e);
//! }
//! ```

use std::any::Any;

// ===== 重导出:核心类型与 trait =====
pub use phy_core::{Subsystem, World, WorldError};

// ===== 命名子模块:各子系统强类型句柄(供业务按域名引用) =====
pub mod fluid {
    //! 流体(SPH/WCSPH)。
    pub use phy_fluid::{FluidSubsystem, FluidWorld, SphParams};
}
pub mod rigid {
    //! 刚体。
    pub use phy_rigid::{RigidSubsystem, RigidWorld};
}
pub mod granular {
    //! 颗粒(粉末/PBD)。
    pub use phy_granular::subsystem::GranularSubsystem;
    pub use phy_granular::world::{GranularWorld, Grain};
}
pub mod soft {
    //! 软体(可变形)。
    pub use phy_soft::{SoftBody, SoftSubsystem};
}
pub mod field {
    //! 标量场/传热(注意:`HeatField` 本身即 `Subsystem`,无额外 wrapper)。
    pub use phy_field::{Bc, HeatField, ScalarField};
}
pub mod optics {
    //! 光学/波动。
    pub use phy_optics::{OpticScene, OpticSubsystem, Precision};
}
pub mod solid {
    //! 可破坏实体(FEM 连续介质)。
    pub use phy_solid::{SolidSubsystem, SolidWorld};
}

// 存档 API(文件版 + 字符串/JSON 版,后者更便于嵌入与测试)。
pub use phy_io::{load_world, load_world_json, save_world, save_world_json};

/// 把 `World` 中第 `idx` 个子系统取回为强类型 `&T`(G4 集成常用:拿到句柄做细粒度控制)。
///
/// 失败(索引越界或类型不符)返回 `None`,不会 panic —— 便于业务侧优雅降级。
///
/// # 示例
/// ```
/// use phy_sdk::{PhysicsBuilder, World, get_as, granular::GranularSubsystem};
/// let world: World<f64> = PhysicsBuilder::new().granular().build();
/// let g = get_as::<GranularSubsystem<f64>>(&world, 0).expect("颗粒子系统");
/// assert_eq!(g.world.grains.len(), 0); // 尚未灌粒子
/// ```
pub fn get_as<'a, T: Any>(world: &'a World<f64>, idx: usize) -> Option<&'a T> {
    world
        .get(idx)
        .and_then(|sub| sub.as_any().downcast_ref::<T>())
}

/// 把 `World` 中第 `idx` 个子系统取回为强类型 `&mut T`。
///
/// 失败返回 `None`,不会 panic。
pub fn get_as_mut<'a, T: Any>(world: &'a mut World<f64>, idx: usize) -> Option<&'a mut T> {
    world
        .get_mut(idx)
        .and_then(|sub| sub.as_any_mut().downcast_mut::<T>())
}

/// 集成 SDK 的统一构建入口(M4 核心 API)。
///
/// 调用方按"声明式"启用要用的子系统,`build()` 自动创建内部世界并**只挂载被启用**
/// 的子系统。各子系统在 `World.subsystems` 中的索引**等于声明命中顺序**,可用
/// [`get_as`]/[`get_as_mut`] 取回强类型句柄。
#[derive(Clone, Default)]
pub struct PhysicsBuilder {
    fluid: bool,
    rigid: bool,
    granular: bool,
    soft: bool,
    field: bool,
    optics: bool,
    solid: bool,
}

impl PhysicsBuilder {
    /// 新建空构建器(不含任何子系统)。
    pub fn new() -> Self {
        Self::default()
    }

    /// 启用流体子系统(初始为空世界,后续经句柄 `fill_box` 灌粒子)。
    pub fn fluid(mut self) -> Self {
        self.fluid = true;
        self
    }

    /// 启用刚体子系统。
    pub fn rigid(mut self) -> Self {
        self.rigid = true;
        self
    }

    /// 启用颗粒(粉末)子系统。
    pub fn granular(mut self) -> Self {
        self.granular = true;
        self
    }

    /// 启用软体(可变形)子系统。
    pub fn soft(mut self) -> Self {
        self.soft = true;
        self
    }

    /// 启用标量场/传热子系统(默认挂一个 16³ 热扩散场,稳态用)。
    pub fn field(mut self) -> Self {
        self.field = true;
        self
    }

    /// 启用光学/波动子系统。
    pub fn optics(mut self) -> Self {
        self.optics = true;
        self
    }

    /// 启用可破坏实体(FEM)子系统。
    pub fn solid(mut self) -> Self {
        self.solid = true;
        self
    }

    /// 构建统一 `World<f64>` 并挂载所有已声明子系统。
    ///
    /// 索引顺序固定为 `fluid → rigid → granular → soft → field → optics → solid`,
    /// 仅包含被启用的子系统。
    pub fn build(self) -> World<f64> {
        let mut world = World::default();
        if self.fluid {
            use crate::fluid::{FluidSubsystem, FluidWorld, SphParams};
            let fw = FluidWorld::new(SphParams::defaults());
            world.add_subsystem(Box::new(FluidSubsystem::new(fw)));
        }
        if self.rigid {
            use crate::rigid::{RigidSubsystem, RigidWorld};
            world.add_subsystem(Box::new(RigidSubsystem::new(RigidWorld::new())));
        }
        if self.granular {
            use crate::granular::{GranularSubsystem, GranularWorld};
            world.add_subsystem(Box::new(GranularSubsystem::new(GranularWorld::new())));
        }
        if self.soft {
            use crate::soft::{SoftBody, SoftSubsystem};
            world.add_subsystem(Box::new(SoftSubsystem::new(SoftBody::new(0.0))));
        }
        if self.field {
            use crate::field::{Bc, HeatField, ScalarField};
            let grid = ScalarField::new(16, 16, 16, 1.0, 0.0, Bc::Neumann);
            world.add_subsystem(Box::new(HeatField::new(grid, 0.1)));
        }
        if self.optics {
            use crate::optics::{OpticScene, OpticSubsystem, Precision};
            world.add_subsystem(Box::new(OpticSubsystem::new(
                OpticScene::new(),
                Precision::Offline,
            )));
        }
        if self.solid {
            use crate::solid::{SolidSubsystem, SolidWorld};
            world.add_subsystem(Box::new(SolidSubsystem::new(SolidWorld::new(
                1.0, 1.0, 1.0,
            ))));
        }
        world
    }
}
