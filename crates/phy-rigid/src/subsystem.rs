//! 刚体子系统适配:`RigidWorld` 接入 `phy_core::World` 统一调度。

use std::any::Any;

use phy_core::Subsystem;
use phy_math::RealField;

use crate::world::RigidWorld;

/// 刚体子系统:包装 `RigidWorld` 以挂在 `World` 上。
///
/// 渲染时可通过 `World::get(i).as_any().downcast_ref::<RigidSubsystem<T>>()`
/// 取回,读取 `world` 字段。
pub struct RigidSubsystem<T: RealField + Copy> {
    /// 内部刚体世界(渲染时直接访问)。
    pub world: RigidWorld<T>,
}

impl<T: RealField + Copy> RigidSubsystem<T> {
    /// 由既有 `RigidWorld` 构造。
    pub fn new(world: RigidWorld<T>) -> Self {
        Self { world }
    }
}

impl<T: RealField + Copy> Subsystem<T> for RigidSubsystem<T> {
    fn as_any(&self) -> &dyn Any {
        self
    }

    fn step(&mut self, dt: &T) {
        self.world.step(*dt);
    }

    fn name(&self) -> &'static str {
        "rigid"
    }
}
