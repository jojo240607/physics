//! 把 `FluidWorld` 适配为 `phy_core::Subsystem`,以便挂入统一的 `World<T>` 多物理场驱动。

use std::any::Any;

use phy_core::{Subsystem, World};
use phy_field::{HeatField, HeatFieldLike};
use phy_math::RealField;

use crate::sph::FluidWorld;

/// 流体子系统包装:持有 `FluidWorld<T>`,在 `step` 中推进 SPH 一个时间步。
///
/// 在 `couple` 阶段,若 `World` 中存在 `HeatField` 子系统且 `thermal_expansion>0`,
/// 则与之双向耦合:热浮力(流体密度随温度下降而上浮增强) + 对流热源(流动区域升温)。
/// 注意:`World::step` 在调用本 `couple` 时已把 `self`(自身)从 `World` 取出,
/// 故可安全地 `world.get_mut(heat_idx)` 取得热场可变引用,无别名冲突。
pub struct FluidSubsystem<T: RealField + Copy + num_traits::ToPrimitive> {
    /// 内部流体世界。
    pub world: FluidWorld<T>,
    /// 热浮力温度膨胀系数 β(ρ(T)=ρ0/(1+β·(T-T_ref)))。0 表示无热浮力。
    pub thermal_expansion: T,
    /// 对流换热注入强度(运动区域升温)。0 表示不注入热源。
    pub heat_gain: T,
}

impl<T: RealField + Copy + num_traits::ToPrimitive> FluidSubsystem<T> {
    /// 由流体世界构造子系统。
    pub fn new(world: FluidWorld<T>) -> Self {
        Self {
            world,
            thermal_expansion: T::zero(),
            heat_gain: T::zero(),
        }
    }
}

impl<T: RealField + Copy + num_traits::ToPrimitive> Subsystem<T> for FluidSubsystem<T> {
    fn as_any(&self) -> &dyn Any {
        self
    }

    fn as_any_mut(&mut self) -> &mut dyn Any {
        self
    }

    fn step(&mut self, dt: &T) {
        self.world.step(*dt);
    }

    fn couple(&mut self, world: &mut World<T>, dt: &T) {
        if self.thermal_expansion <= T::zero() && self.heat_gain <= T::zero() {
            return;
        }
        // 在 World 中查找热场子系统(动态,避免硬编码索引与循环依赖)。
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
            // 取热场参考温度(取为场初值不便,这里取场格点最小值作为 T_ref 近似)。
            // 更稳妥:T_ref 取热度场原点初值;这里用场中心采样作为 t_ref。
            let t_ref = {
                let h = world.get(hi).unwrap();
                let hf = h.as_any().downcast_ref::<HeatField<T>>().unwrap();
                let (nx, ny, nz) = hf.dims();
                hf.sample_trilinear(
                    nx / 2,
                    ny / 2,
                    nz / 2,
                    0.0,
                    0.0,
                    0.0,
                )
            };
            // 取出热场可变引用(此时 self 不在 World 中,无别名冲突)。
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
        "fluid"
    }
}
