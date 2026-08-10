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

pub mod replay;
pub use replay::{FrameInput, Replay, ReplayPlayer, Rng};

/// 引擎错误类型。
#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error("subsystem step failed: {0}")]
    Step(String),
}

/// 仿真健康度错误:`World` 级看门狗在步进后检测到数值异常时返回。
///
/// 由 `World::step_checked` / `step_skipping_checked` 在子系统 `validate` 失败、
/// 或某子系统在一帧内时间未推进(`dt` 被吞掉)时抛出。
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum WorldError {
    /// 某子系统在步进后包含非有限数值(NaN / Inf)。
    #[error("子系统 {subsystem} 字段 {field} 出现非有限值(NaN/Inf)")]
    NonFinite {
        /// 出问题的子系统名(对应 `Subsystem::name`)。
        subsystem: &'static str,
        /// 首个非有限字段的简短标签(如 `"pos.x"` / `"quat.w"`)。
        field: &'static str,
    },
    /// 子系统在一帧内未推进仿真时间(可能陷入死循环 / 被跳过)。
    #[error("子系统 {subsystem} 在一帧内未推进仿真时间")]
    Stalled {
        /// 未推进的子系统名。
        subsystem: &'static str,
    },
}
