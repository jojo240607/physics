//! 把 `GranularWorld` 适配为 `phy_core::Subsystem<T>`,挂入统一仿真 `World<T>`。

use std::any::Any;

use phy_core::world::Subsystem;
use phy_math::RealField;

use crate::world::GranularWorld;

/// 颗粒子系统包装。
pub struct GranularSubsystem<T: RealField + Copy> {
    /// 内部颗粒世界(渲染时直接访问)。
    pub world: GranularWorld<T>,
}

impl<T: RealField + Copy> GranularSubsystem<T> {
    /// 由既有 `GranularWorld` 构造。
    pub fn new(world: GranularWorld<T>) -> Self {
        Self { world }
    }
}

impl<T: RealField + Copy> Subsystem<T> for GranularSubsystem<T> {
    fn as_any(&self) -> &dyn Any {
        self
    }

    fn as_any_mut(&mut self) -> &mut dyn Any {
        self
    }

    fn step(&mut self, dt: &T) {
        self.world.step(*dt);
    }

    fn name(&self) -> &'static str {
        "granular"
    }
}
