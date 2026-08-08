//! World 与 Subsystem 抽象。
//!
//! 设计要点:
//! - `World<T>` 持有 `Vec<Box<dyn Subsystem<T>>>` 子系统列表与全局时间 `t`。
//! - 每个 `Subsystem` 在 `step` 中推进自身,并可借 `couple` 与其他子系统交互。
//! - `World::step` 顺序:先所有子系统 `step`,再所有子系统 `couple`(保证单向数据依赖稳定)。

use std::any::Any;

use phy_math::RealField;

/// 物理子系统接口:任何可挂在 World 上的物理规则。
///
/// M0 阶段 `step`/`couple` 仅接收时间步 `dt`,子系统自持状态;
/// World 仅负责统一推进与时间累加。
/// M5 起将引入共享状态存储(组件池)以支持跨子系统双向耦合,
/// 彼时 `step`/`couple` 可改为接收 `&World`/`&mut World`。
///
/// 要求 `Any` supertrait,使 `World` 可按索引取出具体子系统做渲染/调试
/// (通过 `as_any().downcast_ref::<T>()`)。
pub trait Subsystem<T: RealField>: Any {
    /// 把 `&self` 转成 `&dyn Any`,供 `World::get(i)` 后 downcast 取回具体类型渲染。
    /// 每个实现需提供 `fn as_any(&self) -> &dyn Any { self }`。
    fn as_any(&self) -> &dyn Any;

    /// 把 `&mut self` 转成 `&mut dyn Any`,供耦合阶段可变 downcast(如软体↔刚体)。
    /// 每个实现需提供 `fn as_any_mut(&mut self) -> &mut dyn Any { self }`。
    fn as_any_mut(&mut self) -> &mut dyn Any;

    /// 推进自身一个时间步 `dt`(以引用传入,避免泛型 move)。
    fn step(&mut self, dt: &T);

    /// 与其他子系统的耦合阶段(如软体↔刚体、流体↔刚体)。
    ///
    /// 接收整个 `World` 的可变引用(不含自身),可经 `world.get_mut(i)`
    /// 取出其他子系统做双向交互。`World::step` 在调用每个子系统的 `couple`
    /// 前会临时把它从 `subsystems` 中取出,避免与 `world` 内其他元素别名。
    /// 默认空实现:无耦合的子系统无需覆写。
    fn couple(&mut self, _world: &mut World<T>, _dt: &T) {}

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

    /// 不可变访问第 `i` 个子系统(用于渲染/调试 downcast)。
    pub fn get(&self, i: usize) -> Option<&Box<dyn Subsystem<T>>> {
        self.subsystems.get(i)
    }

    /// 可变访问第 `i` 个子系统。
    pub fn get_mut(&mut self, i: usize) -> Option<&mut Box<dyn Subsystem<T>>> {
        self.subsystems.get_mut(i)
    }

    /// 推进一个时间步 `dt`:先 step 后 couple,最后累加时间。
    ///
    /// 耦合阶段对第 `i` 个子系统临时 `remove` 出 `subsystems`,以 `&mut World`
    /// (不含自身)为参数调用其 `couple`,结束再 `insert` 回原位。这样 `couple`
    /// 内部可经 `world.get_mut(j)` 安全可变访问其他子系统,而无别名冲突。
    pub fn step(&mut self, dt: T) {
        for s in self.subsystems.iter_mut() {
            s.step(&dt);
        }
        let n = self.subsystems.len();
        for i in 0..n {
            let mut me = self.subsystems.remove(i);
            me.couple(self, &dt);
            self.subsystems.insert(i, me);
        }
        self.t += dt;
    }
}
