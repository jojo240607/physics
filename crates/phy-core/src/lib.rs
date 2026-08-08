//! phy-core: 世界物理仿真内核的核心抽象。
//!
//! 提供 `World`(持有所有子系统与共享状态)与 `Subsystem` trait
//! (任意物理规则(刚体/流体/光学/场)的统一接口)。
//! `World::step` 在每个时间步依次驱动所有已注册子系统,实现多物理场可组合耦合。


pub mod world;
pub use world::{Subsystem, World};

pub mod events;
pub use events::{EventBus, EventKind, WorldEvent};

pub mod spatial;
pub use spatial::SpatialGrid;

pub mod timestep;
pub use timestep::{StepMode, TimeController};

pub mod stats;
pub use stats::{attach_stats_observer, StatsObserver};

/// 引擎错误类型。
#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error("subsystem step failed: {0}")]
    Step(String),
}
