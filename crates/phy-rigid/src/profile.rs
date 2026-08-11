//! C3 运行时性能分析(step 分阶段计时,商用游戏引擎补齐计划 §11)。
//!
//! 默认关闭、零开销:`profiler` feature 未启用时,`StepProfile` 为零大小类型、
//! `step` 内不调用任何 `Instant::now`,热路径与未插桩完全一致。
//!
//! 启用后对 `step` 的每个管线阶段做纳秒级计时:
//! - `broad_narrow`:粗筛(SAP) + 窄相碰撞检测(含碰撞层过滤 / 休眠唤醒)
//! - `velocity`:速度层顺序冲量求解(按岛屿并行)
//! - `advance`:子步位置 / 姿态积分
//! - `position`:位置投影(split-impulse 伪速度)求解(按岛屿并行)
//! - `sleep` :休眠管理(速度积分前的近静止判定 / 置眠)
//!
//! 各阶段互不重叠(顺序测量),`total` 为四者之和(≈ 整步真实耗时)。

/// 单步 `step` 的分阶段耗时(纳秒)。
///
/// 由 `RigidWorld::step` 返回;仅当编译时启用 `profiler` feature 时才有意义。
/// 未启用时所有字段恒为 0,且测量本身不消耗任何 CPU。
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct StepProfile {
    /// 粗筛 + 窄相(所有子步合计)。
    pub broad_narrow_ns: u64,
    /// 速度层求解(所有子步合计)。
    pub velocity_ns: u64,
    /// 子步位置/姿态积分(所有子步合计)。
    pub advance_ns: u64,
    /// 位置投影求解。
    pub position_ns: u64,
    /// 休眠管理(速度积分前的近静止判定)。
    pub sleep_ns: u64,
    /// 总耗时 ≈ 上述之和(含未被单独计时的极轻量收尾)。
    pub total_ns: u64,
}

#[cfg(feature = "profiler")]
pub mod measure {
    use std::time::Instant;

    /// 阶段计时器:构造即开始,`StageGuard::finish` 把耗时累加进目标字段。
    pub struct StageGuard {
        start: Instant,
        accum: *mut u64,
    }

    impl StageGuard {
        /// 开始测量,累加目标为 `slot` 指向的字段。
        ///
        /// # Safety
        /// `slot` 必须指向一个有效的、生命周期长于本 guard 的 `u64`。
        #[inline]
        pub(crate) unsafe fn start(slot: *mut u64) -> Self {
            StageGuard {
                start: Instant::now(),
                accum: slot,
            }
        }

        /// 结束本阶段并把耗时累加进 `slot`。
        #[inline]
        pub fn finish(self) {
            let elapsed = self.start.elapsed().as_nanos() as u64;
            unsafe {
                *self.accum += elapsed;
            }
        }
    }
}

/// 在 `profiler` feature 下展开为一次带计时的阶段块;否则零开销地直接执行块体。
///
/// 用法:`profile_stage!(prof, broad_narrow_ns, { /* 阶段代码 */ });`
#[cfg(feature = "profiler")]
#[macro_export]
macro_rules! profile_stage {
    ($prof:expr, $field:ident, $body:block) => {{
        let guard = unsafe {
            crate::profile::measure::StageGuard::start(
                std::ptr::addr_of_mut!($prof.$field),
            )
        };
        $body
        guard.finish();
    }};
}

/// 非 `profiler` 构建:直接执行块体,不产生任何计时代码。
#[cfg(not(feature = "profiler"))]
#[macro_export]
macro_rules! profile_stage {
    ($prof:expr, $field:ident, $body:block) => {
        // 默认(非 profiler)构建:直接执行块体,不产生任何计时代码。
        // `let _ = &mut $prof;` 仅用于"消费" mut 标记,避免 `unused_mut` 警告
        // (profiler 构建下宏内部已通过 addr_of_mut! 使用 mut)。
        let _ = &mut $prof;
        $body
    };
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn profile_default_is_zero() {
        let p = StepProfile::default();
        assert_eq!(p.broad_narrow_ns, 0);
        assert_eq!(p.total_ns, 0);
    }

    #[test]
    fn profile_macro_runs_body() {
        let mut p = StepProfile::default();
        let mut v = 0i32;
        profile_stage!(p, sleep_ns, {
            v += 1;
        });
        assert_eq!(v, 1);
    }
}
