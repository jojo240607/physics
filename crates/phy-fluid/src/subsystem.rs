//! 把 `FluidWorld` 适配为 `phy_core::Subsystem`,以便挂入统一的 `World<T>` 多物理场驱动。

use phy_core::Subsystem;
use phy_math::RealField;

use crate::sph::FluidWorld;

/// 流体子系统包装:持有 `FluidWorld<T>`,在 `step` 中推进 SPH 一个时间步。
///
/// 注意:`phy_core::Subsystem::step` 仅接收 `dt`,无法直接拿到刚体集合;
/// 若需刚体耦合,请在每步 `World::step` 之后手动调用 `FluidWorld::couple_bodies`,
/// 或在自己的 `couple` 阶段通过外部可变引用完成。本包装默认只做纯流体推进。
pub struct FluidSubsystem<T: RealField + Copy + num_traits::ToPrimitive> {
    /// 内部流体世界。
    pub world: FluidWorld<T>,
}

impl<T: RealField + Copy + num_traits::ToPrimitive> FluidSubsystem<T> {
    /// 由流体世界构造子系统。
    pub fn new(world: FluidWorld<T>) -> Self {
        Self { world }
    }
}

impl<T: RealField + Copy + num_traits::ToPrimitive> Subsystem<T> for FluidSubsystem<T> {
    fn step(&mut self, dt: &T) {
        self.world.step(*dt);
    }

    fn name(&self) -> &'static str {
        "fluid"
    }
}
