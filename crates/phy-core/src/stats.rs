//! 仿真统计观测器(M19 / 路线图 #12)。
//!
//! 纯诊断工具,不修改任何物理量。它订阅 [`crate::events::EventBus`] 的
//! [`WorldEvent::Step`] 事件,累加以下运行期统计量:
//!
//! - 已步进帧数 `steps`。
//! - 累计仿真时间 `sim_time`。
//! - 最近一步的步长 `last_dt`。
//! - 每步全局时钟(由 `Step.t` 给出)的单调检查失败次数 `clock_anomalies`
//!   (用于捕获 `dt` 符号错误或时间回退)。
//! - 自定义累加:`push_custom` 注入的外部遥测(如子系统报告的能量),
//!   记录均值/最大/最小。
//!
//! 设计:`StatsObserver` 本身是普通结构体(持有累加器),通过
//! [`attach_stats_observer`] 用 `Rc<RefCell>` 接到世界总线,使其随
//! `World::step` 自动更新;也可手动调用 [`StatsObserver::on_step`]。

use std::any::Any;
use std::cell::RefCell;
use std::rc::Rc;

use phy_math::RealField;
use num_traits::FromPrimitive;

use crate::events::{EventBus, EventKind, WorldEvent};

/// 仿真统计观测器。
#[derive(Debug, Clone)]
pub struct StatsObserver<T: RealField + Copy> {
    /// 已步进帧数。
    pub steps: u64,
    /// 累计仿真时间。
    pub sim_time: T,
    /// 最近一步步长。
    pub last_dt: T,
    /// 时钟异常(时间回退/非单调)计数。
    pub clock_anomalies: u64,
    /// 上一帧时间(用于单调检查)。
    prev_t: T,
    /// 自定义遥测样本数。
    custom_n: u64,
    /// 自定义遥测累计和。
    custom_sum: T,
    /// 自定义遥测最大值。
    custom_max: T,
    /// 自定义遥测最小值。
    custom_min: T,
}

impl<T: RealField + Copy> Default for StatsObserver<T> {
    fn default() -> Self {
        Self::new()
    }
}

impl<T: RealField + Copy> StatsObserver<T> {
    /// 创建空观测器。
    pub fn new() -> Self {
        Self {
            steps: 0,
            sim_time: T::zero(),
            last_dt: T::zero(),
            clock_anomalies: 0,
            prev_t: T::zero(),
            custom_n: 0,
            custom_sum: T::zero(),
            custom_max: T::zero(),
            custom_min: T::zero(),
        }
    }

    /// 处理一个步进事件(`t` 为该步结束后的全局时间,`dt` 为步长)。
    pub fn on_step(&mut self, t: T, dt: T) {
        self.steps += 1;
        self.sim_time += dt;
        self.last_dt = dt;
        if self.steps > 1 && t < self.prev_t {
            self.clock_anomalies += 1;
        }
        self.prev_t = t;
    }

    /// 注入一个自定义遥测样本(如某子系统的能量估计)。
    pub fn push_custom(&mut self, v: T) {
        if self.custom_n == 0 {
            self.custom_min = v;
            self.custom_max = v;
        } else {
            if v < self.custom_min {
                self.custom_min = v;
            }
            if v > self.custom_max {
                self.custom_max = v;
            }
        }
        self.custom_sum += v;
        self.custom_n += 1;
    }

    /// 自定义遥测样本均值(无样本返回 0)。
    pub fn custom_mean(&self) -> T {
        if self.custom_n == 0 {
            T::zero()
        } else {
            self.custom_sum / <T as FromPrimitive>::from_u64(self.custom_n).unwrap()
        }
    }

    /// 重置所有统计。
    pub fn reset(&mut self) {
        *self = Self::new();
    }
}

/// 把 `observer` 接到世界事件总线,使其随 `World::step` 自动累积统计。
///
/// 返回持有的 `Rc<RefCell<StatsObserver>>`(调用方可借此事后读出统计)。
/// 注意:同一 `observer` 只能 attach 一次(总线订阅为追加)。
pub fn attach_stats_observer<T: RealField + Copy>(
    bus: &mut EventBus<T>,
    observer: Rc<RefCell<StatsObserver<T>>>,
) {
    bus.subscribe(move |kind: EventKind, payload: &dyn Any| {
        if kind == EventKind::World {
            if let Some(WorldEvent::Step { t, dt }) = payload.downcast_ref::<WorldEvent<T>>() {
                observer.borrow_mut().on_step(*t, *dt);
            }
        }
    });
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::world::World;
    use std::cell::RefCell;
    use std::rc::Rc;

    #[test]
    fn observer_accumulates_from_world_steps() {
        let mut w: World<f64> = World::new();
        let obs = Rc::new(RefCell::new(StatsObserver::<f64>::new()));
        attach_stats_observer(&mut w.bus, obs.clone());
        // 步进 10 次,每次 dt=0.01。
        for _ in 0..10 {
            w.step(0.01);
        }
        let o = obs.borrow();
        assert_eq!(o.steps, 10);
        assert!((o.sim_time - 0.10).abs() < 1e-12);
        assert!((o.last_dt - 0.01).abs() < 1e-12);
        assert_eq!(o.clock_anomalies, 0);
    }

    #[test]
    fn observer_detects_clock_anomaly() {
        let mut o = StatsObserver::<f64>::new();
        o.on_step(1.0, 0.1);
        o.on_step(2.0, 0.1);
        o.on_step(1.5, 0.1); // 时间回退
        assert_eq!(o.clock_anomalies, 1);
    }

    #[test]
    fn custom_telemetry_stats() {
        let mut o = StatsObserver::<f64>::new();
        o.push_custom(1.0);
        o.push_custom(3.0);
        o.push_custom(2.0);
        assert_eq!(o.custom_n, 3);
        assert!((o.custom_mean() - 2.0).abs() < 1e-12);
        assert!((o.custom_max - 3.0).abs() < 1e-12);
        assert!((o.custom_min - 1.0).abs() < 1e-12);
    }
}
