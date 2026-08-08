//! 时间步长控制器(M19):把"帧请求的时间"切成稳定的子步推进 World。
//!
//! 不同子系统对最大稳定步长有不同要求(如刚体高速碰撞需要小 dt、SPH
//! 受 CFL 条件约束)。固定 `world.step(dt)` 在帧率抖动或请求 dt 过大时会
//! 失稳(穿透、能量爆炸)。本模块提供 `TimeController`,把任意帧 `dt` 切成
//! 若干个不超过 `max_dt` 的子步,并把剩余尾数累积到下一帧,保证数值稳定。
//!
//! 两种模式:
//! - `Fixed`:每个子步都是固定 `max_dt`(最稳,推荐科研/确定性)。
//! - `Adaptive`:子步上界 `max_dt`,但尽量用大步(尾数直接并入下一子步),
//!   适合实时游戏(帧率波动时仍平滑)。

use phy_math::RealField;

use crate::world::World;

/// 时间步长控制模式。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StepMode {
    /// 固定子步长 `max_dt`:尾数累积到下一帧,子步数 = ceil(frame_dt/max_dt)。
    /// 最稳定,子步大小恒定,适合科研/确定性回放。
    Fixed,
    /// 自适应:子步不超过 `max_dt`,但最后一个子步吸收尾数(大小可变)。
    /// 步数最少,适合实时游戏(抖动小)。
    Adaptive,
}

/// 时间步长控制器。
///
/// 用法:每帧调用 [`TimeController::advance`] 一次,传入本帧期望推进的真实
/// 时间 `frame_dt`;内部自动切片成满足 `max_dt` 约束的子步并逐个 `world.step`,
/// 多余尾数累积到下一帧(`Fixed` 模式)或并入末子步(`Adaptive` 模式)。
pub struct TimeController<T: RealField + Copy> {
    /// 单个子步允许的最大时间(超过则拆分)。
    pub max_dt: T,
    /// 累积的尾数(上一帧没花完的时间),下一帧优先花掉。
    accumulator: T,
    /// 控制模式。
    pub mode: StepMode,
    /// 时间缩放因子(>1 慢动作,<1 快进)。作用于帧 dt。
    pub time_scale: T,
    /// 统计:累计已发出的子步数(自构造以来)。
    pub substeps_total: u64,
    /// 统计:上一次 `advance` 实际发出的子步数。
    pub last_substeps: u32,
}

impl<T: RealField + Copy> TimeController<T> {
    /// 创建一个新的固定子步控制器,子步上界 `max_dt`。
    pub fn new(max_dt: T) -> Self {
        Self {
            max_dt,
            accumulator: T::zero(),
            mode: StepMode::Fixed,
            time_scale: T::one(),
            substeps_total: 0,
            last_substeps: 0,
        }
    }

    /// 创建一个自适应子步控制器。
    pub fn adaptive(max_dt: T) -> Self {
        Self {
            max_dt,
            accumulator: T::zero(),
            mode: StepMode::Adaptive,
            time_scale: T::one(),
            substeps_total: 0,
            last_substeps: 0,
        }
    }

    /// 设置时间缩放因子(>1 慢动作,<1 快进)。
    pub fn set_time_scale(&mut self, scale: T) {
        self.time_scale = scale;
    }

    /// 推进世界 `frame_dt` 个(缩放后)时间单位,切成满足约束的子步。
    ///
    /// 返回实际发出的子步数。子步大小保证 `≤ max_dt`(Fixed 模式恰好等于
    /// `max_dt`,Adaptive 模式末步 ≤ `max_dt`)。
    pub fn advance(&mut self, world: &mut World<T>, frame_dt: T) -> u32 {
        let scaled = frame_dt * self.time_scale;
        // 把本帧时间并入累积器。
        self.accumulator = self.accumulator + scaled;

        let mut count: u32 = 0;

        match self.mode {
            StepMode::Fixed => {
                // 只消费整倍数个子步,尾数留在 accumulator 等下一帧。
                let max = self.max_dt;
                while self.accumulator >= max {
                    world.step(max);
                    self.accumulator = self.accumulator - max;
                    count += 1;
                }
            }
            StepMode::Adaptive => {
                // 尽可能用大步,末步吸收剩余(若 >0)。
                let max = self.max_dt;
                while self.accumulator > T::zero() {
                    let sub = if self.accumulator > max {
                        max
                    } else {
                        self.accumulator
                    };
                    world.step(sub);
                    self.accumulator = self.accumulator - sub;
                    count += 1;
                }
            }
        }

        self.substeps_total += count as u64;
        self.last_substeps = count;
        count
    }

    /// 丢弃累积尾数(如暂停/重置后避免"补帧"爆冲)。
    pub fn flush_accumulator(&mut self) {
        self.accumulator = T::zero();
    }

    /// 当前累积尾数(只读,调试用)。
    pub fn accumulator(&self) -> T {
        self.accumulator
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::world::{Subsystem, World};
    use std::any::Any;

    struct CounterSub {
        steps: usize,
        last_dt: f64,
    }
    impl Subsystem<f64> for CounterSub {
        fn as_any(&self) -> &dyn Any {
            self
        }
        fn as_any_mut(&mut self) -> &mut dyn Any {
            self
        }
        fn step(&mut self, dt: &f64) {
            self.steps += 1;
            self.last_dt = *dt;
        }
    }

    #[test]
    fn fixed_controller_splits_into_max_dt_substeps() {
        let mut w: World<f64> = World::new();
        w.add_subsystem(Box::new(CounterSub {
            steps: 0,
            last_dt: 0.0,
        }));
        // 用二进制可精确表示的数值(0.125 是 2^-3),避免十进制字面量浮点误差。
        let mut ctrl = TimeController::new(0.125);
        // 帧 dt = 0.25 -> 2 个子步,尾数 0.0(0.25 = 2 × 0.125)。
        let n = ctrl.advance(&mut w, 0.25);
        assert_eq!(n, 2);
        assert!(ctrl.accumulator() < 1e-12);
        let c = w.get(0).unwrap().as_any().downcast_ref::<CounterSub>().unwrap();
        assert_eq!(c.steps, 2);
        assert!((c.last_dt - 0.125).abs() < 1e-12);

        // 再推进一个不整除的帧:0.0625 = 0.5 × 0.125,留尾数到下一帧。
        let n2 = ctrl.advance(&mut w, 0.0625);
        assert_eq!(n2, 0);
        assert!((ctrl.accumulator() - 0.0625).abs() < 1e-12);
    }

    #[test]
    fn fixed_controller_carries_remainder_to_next_frame() {
        let mut w: World<f64> = World::new();
        w.add_subsystem(Box::new(CounterSub {
            steps: 0,
            last_dt: 0.0,
        }));
        let mut ctrl = TimeController::new(0.125);
        ctrl.advance(&mut w, 0.0625); // 不足一步,尾 0.0625
        let n = ctrl.advance(&mut w, 0.0625); // 0.0625+0.0625=0.125 -> 1 步,尾 0
        assert_eq!(n, 1);
        assert!(ctrl.accumulator() < 1e-12);
        let c = w.get(0).unwrap().as_any().downcast_ref::<CounterSub>().unwrap();
        assert_eq!(c.steps, 1);
    }

    #[test]
    fn adaptive_controller_uses_large_last_step() {
        let mut w: World<f64> = World::new();
        w.add_subsystem(Box::new(CounterSub {
            steps: 0,
            last_dt: 0.0,
        }));
        let mut ctrl = TimeController::adaptive(0.01);
        // 0.025 -> 0.01 + 0.01 + 0.005,末步 0.005,无尾数。
        let n = ctrl.advance(&mut w, 0.025);
        assert_eq!(n, 3);
        assert!(ctrl.accumulator() < 1e-12);
        let c = w.get(0).unwrap().as_any().downcast_ref::<CounterSub>().unwrap();
        assert_eq!(c.steps, 3);
        assert!((c.last_dt - 0.005).abs() < 1e-12);
    }

    #[test]
    fn time_scale_slows_simulation() {
        let mut w: World<f64> = World::new();
        w.add_subsystem(Box::new(CounterSub {
            steps: 0,
            last_dt: 0.0,
        }));
        let mut ctrl = TimeController::new(0.01);
        ctrl.set_time_scale(0.5); // 慢动作:帧 0.02 实际只推进 0.01
        let n = ctrl.advance(&mut w, 0.02);
        // 0.02 * 0.5 = 0.01 -> 1 子步,尾 0
        assert_eq!(n, 1);
        assert!((w.time() - 0.01).abs() < 1e-12);
    }

    #[test]
    fn flush_removes_pending_accumulator() {
        let mut w: World<f64> = World::new();
        w.add_subsystem(Box::new(CounterSub {
            steps: 0,
            last_dt: 0.0,
        }));
        let mut ctrl = TimeController::new(0.01);
        ctrl.advance(&mut w, 0.005); // 不足一步,尾数 0.005
        assert!(ctrl.accumulator() > 0.0);
        ctrl.flush_accumulator();
        assert!(ctrl.accumulator() < 1e-12);
    }
}
