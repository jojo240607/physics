//! 软体子系统适配:`SoftBody` 接入 `phy_core::World` 统一调度。

use std::any::Any;

use phy_core::Subsystem;
use phy_math::RealField;

use crate::body::SoftBody;

/// 软体子系统:包装 `SoftBody` 以挂在 `World` 上。
///
/// 渲染时可通过 `World::get(i).as_any().downcast_ref::<SoftSubsystem<T>>()`
/// 取回,读取 `body` 字段。
pub struct SoftSubsystem<T: RealField + Copy> {
    /// 内部软体(渲染时直接访问)。
    pub body: SoftBody<T>,
}

impl<T: RealField + Copy> SoftSubsystem<T> {
    /// 由既有 `SoftBody` 构造。
    pub fn new(body: SoftBody<T>) -> Self {
        Self { body }
    }
}

impl<T: RealField + Copy> Subsystem<T> for SoftSubsystem<T> {
    fn as_any(&self) -> &dyn Any {
        self
    }

    fn as_any_mut(&mut self) -> &mut dyn Any {
        self
    }

    fn step(&mut self, dt: &T) {
        self.body.step(*dt);
    }

    fn name(&self) -> &'static str {
        "soft"
    }
}
