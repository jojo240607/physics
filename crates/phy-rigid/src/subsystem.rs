//! 刚体子系统适配:`RigidWorld` 接入 `phy_core::World` 统一调度。

use std::any::Any;

use phy_core::{Subsystem, World};
use phy_field::{HeatField, HeatFieldLike};
use phy_math::RealField;

use crate::world::RigidWorld;

/// 刚体子系统:包装 `RigidWorld` 以挂在 `World` 上。
///
/// 渲染时可通过 `World::get(i).as_any().downcast_ref::<RigidSubsystem<T>>()`
/// 取回,读取 `world` 字段。
pub struct RigidSubsystem<T: RealField + Copy + num_traits::ToPrimitive> {
    /// 内部刚体世界(渲染时直接访问)。
    pub world: RigidWorld<T>,
    /// 热浮力温度膨胀系数 β(ρ(T)=ρ0/(1+β·(T-T_ref)))。0 表示无热浮力。
    pub thermal_expansion: T,
    /// 对流换热注入强度(运动物体加热场)。0 表示不注入热源。
    pub heat_gain: T,
    /// 热浮力参考温度 T_ref(环境温度基线,ρ(T_ref)=ρ0)。0 表示以 0 为环境温度。
    pub t_ref: T,
}

impl<T: RealField + Copy + num_traits::ToPrimitive> RigidSubsystem<T> {
    /// 由既有 `RigidWorld` 构造。
    pub fn new(world: RigidWorld<T>) -> Self {
        Self {
            world,
            thermal_expansion: T::zero(),
            heat_gain: T::zero(),
            t_ref: T::zero(),
        }
    }
}

impl<T: RealField + Copy + num_traits::ToPrimitive> Subsystem<T> for RigidSubsystem<T> {
    fn as_any(&self) -> &dyn Any {
        self
    }

    fn as_any_mut(&mut self) -> &mut dyn Any {
        self
    }

    fn step(&mut self, dt: &T) {
        self.world.step(*dt);
    }

    /// 刚体↔热场双向耦合(M11)。
    ///
    /// 若 `thermal_expansion>0` 或 `heat_gain>0`,在 `World` 中动态查找 `HeatField`
    /// 子系统,经 `remove`/`insert` 安全取可变引用后调用 `RigidWorld::couple_heat`。
    fn couple(&mut self, world: &mut World<T>, dt: &T) {
        if self.thermal_expansion <= T::zero() && self.heat_gain <= T::zero() {
            return;
        }
        let n = world.subsystem_count();
        let mut heat_idx: Option<usize> = None;
        for i in 0..n {
            if let Some(s) = world.get(i) {
                if s.as_any().downcast_ref::<HeatField<T>>().is_some() {
                    heat_idx = Some(i);
                    break;
                }
            }
        }
        if let Some(hi) = heat_idx {
            let t_ref = self.t_ref;
            let mut heat_box = world.remove(hi);
            let heat = heat_box
                .as_any_mut()
                .downcast_mut::<HeatField<T>>()
                .expect("heat subsystem type mismatch");
            self.world
                .couple_heat(heat, *dt, t_ref, self.thermal_expansion, self.heat_gain);
            world.insert(hi, heat_box);
        }
    }

    fn name(&self) -> &'static str {
        "rigid"
    }
}
