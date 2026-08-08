//! World 级事件总线(M14)。
//!
//! 提供轻量级发布/订阅机制,用于子系统之间解耦的通知(如碰撞事件、步进完成、
//! 自定义遥测)。`World` 持有一个 [`EventBus`],子系统可在 `couple` 阶段经
//! `world.bus_mut()` 发布事件,外部监听者经 `world.subscribe()` 注册回调统一消费。
//!
//! 设计取舍:
//! - 事件以 `&dyn Any` 承载,订阅者可按类型 `downcast_ref` 取出具体事件;
//!   同时内置常用 [`WorldEvent<T>`] 枚举供直接匹配。
//! - 回调以闭包存储,单线程仿真场景下足够;不做跨线程同步。

use std::any::Any;
use std::marker::PhantomData;

use phy_math::RealField;

/// 内置的世界事件(常用、可直接匹配的那些)。
///
/// 自定义事件可走 [`EventBus::publish`] 的 `Box<dyn Any>` 通道,订阅者 downcast 取出。
pub enum WorldEvent<T: RealField> {
    /// 世界完成一个时间步:`t` 为该步结束后的全局时间,`dt` 为步长。
    Step { t: T, dt: T },
    /// 某个子系统完成 `step`(在 `World::step` 的 step 阶段之后、`couple` 之前)。
    SubsystemStepped { name: &'static str, t: T },
    /// 仿真启动(`World` 首次 `step` 之前发出)。
    SimStart,
    /// 仿真结束(由调用方显式 `publish`)。
    SimEnd,
}

/// 轻量级事件总线。
pub struct EventBus<T: RealField> {
    /// 订阅者闭包:接收 `(event_type_tag, &dyn Any)`。
    /// `event_type_tag` 用于快速分流(见 [`EventKind`])。
    subscribers: Vec<Box<dyn FnMut(EventKind, &dyn Any)>>,
    /// 待分发事件队列(支持在回调执行期间安全 publish)。
    pending: Vec<(EventKind, Box<dyn Any>)>,
    /// `T` 仅出现在方法签名(`WorldEvent<T>`)中,用 PhantomData 占位以维持类型参数。
    _phantom: PhantomData<fn() -> T>,
}

/// 事件种类标签:用于在分发时快速分流,避免每次都 downcast 全部订阅者。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EventKind {
    /// 内置世界事件(对应 [`WorldEvent`])。
    World,
    /// 子系统自定义事件(`Box<dyn Any>`)。
    Custom,
}

impl<T: RealField> Default for EventBus<T> {
    fn default() -> Self {
        Self::new()
    }
}

impl<T: RealField> EventBus<T> {
    /// 创建空总线。
    pub fn new() -> Self {
        Self {
            subscribers: Vec::new(),
            pending: Vec::new(),
            _phantom: PhantomData,
        }
    }

    /// 注册一个订阅者。闭包接收事件种类标签与事件负载(按标签决定 downcast 目标)。
    ///
    /// 返回订阅者序号(可用于未来取消订阅,当前版本未实现 unsubscribe)。
    pub fn subscribe<F>(&mut self, f: F) -> usize
    where
        F: FnMut(EventKind, &dyn Any) + 'static,
    {
        self.subscribers.push(Box::new(f));
        self.subscribers.len() - 1
    }

    /// 发布一个内置世界事件:立即入队,待 [`flush`] 时分发。
    pub fn publish_world(&mut self, e: WorldEvent<T>) {
        self.pending.push((EventKind::World, Box::new(e)));
    }

    /// 发布一个自定义事件(任意 `Any` 类型):立即入队,待 [`flush`] 时分发。
    pub fn publish_custom<E: Any>(&mut self, e: E) {
        self.pending.push((EventKind::Custom, Box::new(e)));
    }

    /// 分发所有待处理事件给订阅者。在 `World::step` 末尾自动调用一次;
    /// 订阅回调内部若继续 `publish`,新事件进入下一轮(直至队列清空)。
    pub fn flush(&mut self) {
        // 反复取出待处理队列,直到为空(支持回调内再发布)。
        while !self.pending.is_empty() {
            let batch = std::mem::take(&mut self.pending);
            for (kind, payload) in batch {
                for sub in self.subscribers.iter_mut() {
                    sub(kind, payload.as_ref());
                }
            }
        }
    }

    /// 订阅者数量(调试用)。
    pub fn subscriber_count(&self) -> usize {
        self.subscribers.len()
    }
}
