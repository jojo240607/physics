//! World 与 Subsystem 抽象。
//!
//! 设计要点:
//! - `World<T>` 持有 `Vec<Box<dyn Subsystem<T>>>` 子系统列表与全局时间 `t`。
//! - 每个 `Subsystem` 在 `step` 中推进自身,并可借 `couple` 与其他子系统交互。
//! - `World::step` 顺序:先所有子系统 `step`,再所有子系统 `couple`(保证单向数据依赖稳定)。

use phy_math::RealField;

/// 物理子系统接口:任何可挂在 World 上的物理规则。
///
/// M0 阶段 `step`/`couple` 仅接收时间步 `dt`,子系统自持状态;
/// World 仅负责统一推进与时间累加。
/// M5 起将引入共享状态存储(组件池)以支持跨子系统双向耦合,
/// 彼时 `step`/`couple` 可改为接收 `&World`/`&mut World`。
pub trait Subsystem<T: RealField> {
    /// 推进自身一个时间步 `dt`(以引用传入,避免泛型 move)。
    fn step(&mut self, dt: &T);

    /// 与其他子系统的耦合阶段(如流体对刚体施加浮力/阻力)。
    /// 默认空实现:无耦合的子系统无需覆写。
    fn couple(&mut self, _dt: &T) {}

    /// 子系统名称(用于调试/事件)。
    fn name(&self) -> &'static str {
        "subsystem"
    }
}

/// 仿真世界:统一驱动所有已注册子系统的多物理场容器。
pub struct World<T: RealField> {
    /// 已注册子系统。
    subsystems: Vec<Box<dyn Subsystem<T>>>,
    /// 当前仿真时间。
    t: T,
}

impl<T: RealField> Default for World<T> {
    fn default() -> Self {
        Self::new()
    }
}

impl<T: RealField> World<T> {
    /// 创建空世界。
    pub fn new() -> Self {
        Self {
            subsystems: Vec::new(),
            t: T::zero(),
        }
    }

    /// 注册一个子系统。
    pub fn add_subsystem(&mut self, s: Box<dyn Subsystem<T>>) {
        self.subsystems.push(s);
    }

    /// 当前仿真时间。
    pub fn time(&self) -> T {
        self.t.clone()
    }

    /// 已注册子系统数量。
    pub fn subsystem_count(&self) -> usize {
        self.subsystems.len()
    }

    /// 推进一个时间步 `dt`:先 step 后 couple,最后累加时间。
    pub fn step(&mut self, dt: T) {
        for s in self.subsystems.iter_mut() {
            s.step(&dt);
        }
        for s in self.subsystems.iter_mut() {
            s.couple(&dt);
        }
        self.t += dt;
    }
}
