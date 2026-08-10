//! World 与 Subsystem 抽象。
//!
//! 设计要点:
//! - `World<T>` 持有 `Vec<Box<dyn Subsystem<T>>>` 子系统列表与全局时间 `t`。
//! - 每个 `Subsystem` 在 `step` 中推进自身,并可借 `couple` 与其他子系统交互。
//! - `World::step` 顺序:先所有子系统 `step`,再所有子系统 `couple`(保证单向数据依赖稳定)。

use std::any::Any;

use crate::events::{EventBus, EventKind, WorldEvent};
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
    /// 世界级事件总线(M14):子系统可在 couple 阶段发布事件,外部监听者统一消费。
    pub bus: EventBus<T>,
    /// 是否已发出 `SimStart`(首次 step 前)。
    started: bool,
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
            bus: EventBus::new(),
            started: false,
        }
    }

    /// 注册一个子系统。
    pub fn add_subsystem(&mut self, s: Box<dyn Subsystem<T>>) {
        self.subsystems.push(s);
    }

    /// 注册一个事件订阅者(见 [`EventBus::subscribe`])。
    pub fn subscribe<F>(&mut self, f: F) -> usize
    where
        F: FnMut(EventKind, &dyn Any) + 'static,
    {
        self.bus.subscribe(f)
    }

    /// 当前仿真时间。
    pub fn time(&self) -> T {
        self.t.clone()
    }

    /// 设置仿真时间(读档恢复用)。
    pub fn set_time(&mut self, t: T) {
        self.t = t;
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

    /// 临时取出第 `i` 个子系统(用于耦合阶段避免别名冲突;调用方需负责 `insert` 回原位)。
    pub fn remove(&mut self, i: usize) -> Box<dyn Subsystem<T>> {
        self.subsystems.remove(i)
    }

    /// 把取出的子系统放回第 `i` 个位置。
    pub fn insert(&mut self, i: usize, s: Box<dyn Subsystem<T>>) {
        self.subsystems.insert(i, s);
    }

    /// 推进一个时间步 `dt`:先 step 后 couple,再发 Step 事件并 flush,最后累加时间。
    ///
    /// 耦合阶段对第 `i` 个子系统临时 `remove` 出 `subsystems`,以 `&mut World`
    /// (不含自身)为参数调用其 `couple`,结束再 `insert` 回原位。这样 `couple`
    /// 内部可经 `world.get_mut(j)` 安全可变访问其他子系统,而无别名冲突。
    ///
    /// 事件序列(每个 step):首次 step 前发 `SimStart` → 各 subsystem `step` →
    /// 推进一个时间步 `dt`,但跳过 `skip[i] == true` 的子系统的 CPU `step`
    /// (其力学推进改由外部 GPU 路径完成)。`couple` 阶段对所有子系统照常执行,
    /// 以便热浮力 / 刚体碰撞等跨子系统耦合不被跳过。
    ///
    /// 通常用于 Web 演示的「运行时切换」:把流体 / 颗粒子系统的力学 step 路由到
    /// GPU compute,其余子系统仍走 CPU,耦合矩阵保持完整。
    pub fn step_skipping(&mut self, dt: T, skip: &[bool]) {
        if !self.started {
            self.started = true;
            self.bus.publish_world(WorldEvent::SimStart);
        }
        for (i, s) in self.subsystems.iter_mut().enumerate() {
            if skip.get(i).copied().unwrap_or(false) {
                continue;
            }
            s.step(&dt);
        }
        let n = self.subsystems.len();
        for i in 0..n {
            let mut me = self.subsystems.remove(i);
            me.couple(self, &dt);
            self.subsystems.insert(i, me);
        }
        self.t += dt.clone();
        self.bus.publish_world(WorldEvent::Step { t: self.t.clone(), dt });
        self.bus.flush();
    }

    /// 推进一个时间步 `dt`:先 step 后 couple,再发 Step 事件并 flush,最后累加时间。
    ///
    /// 耦合阶段对第 `i` 个子系统临时 `remove` 出 `subsystems`,以 `&mut World`
    /// (不含自身)为参数调用其 `couple`,结束再 `insert` 回原位。这样 `couple`
    /// 内部可经 `world.get_mut(j)` 安全可变访问其他子系统,而无别名冲突。
    ///
    /// 事件序列(每个 step):首次 step 前发 `SimStart` → 各 subsystem `step` →
    /// 各 subsystem `couple`(子系统可在其中经 `world.bus` 发布自定义事件)→
    /// 发 `Step{t,dt}` → `bus.flush()` 把本步累积的事件统一分发给订阅者。
    pub fn step(&mut self, dt: T) {
        self.step_skipping(dt, &[]);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    struct DummySub {
        steps: usize,
    }
    impl Subsystem<f64> for DummySub {
        fn as_any(&self) -> &dyn Any {
            self
        }
        fn as_any_mut(&mut self) -> &mut dyn Any {
            self
        }
        fn step(&mut self, _dt: &f64) {
            self.steps += 1;
        }
    }

    #[test]
    fn step_emits_simstart_then_steps() {
        use std::rc::Rc;
        use std::cell::RefCell;
        let mut w: World<f64> = World::new();
        w.add_subsystem(Box::new(DummySub { steps: 0 }));
        let events: Rc<RefCell<Vec<String>>> = Rc::new(RefCell::new(Vec::new()));
        let ev = events.clone();
        w.subscribe(move |kind, payload| {
            if kind == EventKind::World {
                if let Some(e) = payload.downcast_ref::<WorldEvent<f64>>() {
                    match e {
                        WorldEvent::SimStart => ev.borrow_mut().push("start".into()),
                        WorldEvent::Step { t, .. } => ev.borrow_mut().push(format!("step@{}", t)),
                        _ => {}
                    }
                }
            }
        });
        w.step(0.1);
        w.step(0.1);
        assert_eq!(*events.borrow(), vec!["start", "step@0.1", "step@0.2"]);
        // 子系统确实被推进了两次。
        let d = w.get(0).unwrap().as_any().downcast_ref::<DummySub>().unwrap();
        assert_eq!(d.steps, 2);
    }

    #[test]
    fn custom_event_roundtrips_through_bus() {
        use std::rc::Rc;
        use std::cell::RefCell;
        let mut w: World<f64> = World::new();
        #[derive(Debug)]
        struct MyEvent {
            tag: u32,
        }
        let seen: Rc<RefCell<u32>> = Rc::new(RefCell::new(0));
        let seen_c = seen.clone();
        w.subscribe(move |kind, payload| {
            if kind == EventKind::Custom {
                if let Some(e) = payload.downcast_ref::<MyEvent>() {
                    *seen_c.borrow_mut() = e.tag;
                }
            }
        });
        w.bus.publish_custom(MyEvent { tag: 42 });
        w.bus.flush();
        assert_eq!(*seen.borrow(), 42);
    }
}
