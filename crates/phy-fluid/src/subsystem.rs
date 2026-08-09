//! 把 `FluidWorld` 适配为 `phy_core::Subsystem`,以便挂入统一的 `World<T>` 多物理场驱动。

use std::any::Any;

use phy_core::{Subsystem, World};
use phy_field::HeatField;
use phy_math::RealField;
use phy_rigid::RigidSubsystem;
use serde::de::DeserializeOwned;
use serde::{Deserialize, Serialize};

use crate::sph::FluidWorld;

/// 流体子系统包装:持有 `FluidWorld<T>`,在 `step` 中推进 SPH 一个时间步。
///
/// 在 `couple` 阶段完成双向耦合:
/// - **流体 ↔ 温度场**(M4e 热浮力 + M11 对流换热):`thermal_expansion>0` 或
///   `heat_gain>0` 时,在 `World` 中查找 `HeatField` 调用 `couple_heat`。
/// - **流体 ↔ 刚体**(#10 增强):浮力 + 阻力 + 动量交换。由流体侧主导——在
///   `World` 中 `remove` 刚体子系统后取 `bodies` 可变引用调用 `couple_bodies`,
///   结束放回。刚体侧不再反向耦合流体,避免两个子系统互 `remove` 造成的死锁
///   (`World::step` 调 `couple` 时已把 `self`(自身)取出,故本处可直接 `remove`
///   其它子系统而自身不会与自身冲突)。
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(bound = "T: RealField + Copy + Serialize + DeserializeOwned + nalgebra::Scalar + num_traits::ToPrimitive")]
pub struct FluidSubsystem<T: RealField + Copy + num_traits::ToPrimitive> {
    /// 内部流体世界。
    pub world: FluidWorld<T>,
    /// 热浮力温度膨胀系数 β(ρ(T)=ρ0/(1+β·(T-T_ref)))。0 表示无热浮力。
    pub thermal_expansion: T,
    /// 对流换热注入强度(运动区域升温)。0 表示不注入热源。
    pub heat_gain: T,
    /// 参考温度 T_ref(环境温度基线,对应 ρ(T_ref)=ρ0)。浮力按 ΔT=T−T_ref 计算,
    /// 暖区(ΔT>0)上举、冷区(ΔT<0)下沉。默认 0(与物理"无浮力基线"一致)。
    /// 注意:切勿取热场中心格作为 T_ref(若热场最热处恰在中心,会让所有浮力归零/反向)。
    pub t_ref: T,
    /// 刚体↔流体绕流阻力系数等效值(#10)。传入 `couple_bodies` 作为等效阻力强度
    /// (物理上为 `½ρ·Cd·A` 的合并系数,具体含义见 `FluidWorld::couple_bodies`)。
    pub drag: T,
    /// 刚体↔流体接触摩擦系数(#10)。动量交换时沿切向(相对速度)施加耗散。
    pub friction: T,
}

impl<T: RealField + Copy + num_traits::ToPrimitive> FluidSubsystem<T> {
    /// 由流体世界构造子系统。
    pub fn new(world: FluidWorld<T>) -> Self {
        Self {
            world,
            thermal_expansion: T::zero(),
            heat_gain: T::zero(),
            t_ref: T::zero(),
            drag: T::from_f64(1.0).unwrap(),
            friction: T::from_f64(0.1).unwrap(),
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
        // ---- 流体 ↔ 温度场(M4e 热浮力 + M11 对流换热)----
        if self.thermal_expansion > T::zero() || self.heat_gain > T::zero() {
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
                // 参考温度 T_ref 取子系统显式配置(默认 0,即环境温度基线),
                // 不取热场中心格采样(否则最热处落在中心时所有浮力归零/反向,见 M11 记录)。
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

        // ---- 流体 ↔ 刚体(#10 增强):浮力 + 阻力 + 动量交换 ----
        // 由流体侧主导:在 World 中 remove 刚体子系统后取 bodies 可变引用调用
        // couple_bodies,结束放回。此时 self 已被 World::step 取出,无别名冲突。
        // 刚体侧(RigidSubsystem::couple)不反向耦合流体,避免双向 remove 死锁。
        let n = world.subsystem_count();
        let mut rb_idx: Option<usize> = None;
        for i in 0..n {
            if let Some(s) = world.get(i) {
                if s.as_any().downcast_ref::<RigidSubsystem<T>>().is_some() {
                    rb_idx = Some(i);
                    break;
                }
            }
        }
        if let Some(ri) = rb_idx {
            let mut rb_box = world.remove(ri);
            if let Some(rigid_sys) = rb_box.as_any_mut().downcast_mut::<RigidSubsystem<T>>() {
                // 透传阻力(drag)与接触摩擦(friction)。couple_bodies 内部用 friction
                // 控制切向动量耗散;drag 作为等效绕流阻力强度预留(当前 SPH 用内部阻力模型)。
                self.world.couple_bodies(&mut rigid_sys.world.bodies, *dt, self.friction);
            }
            world.insert(ri, rb_box);
        }
    }

    fn name(&self) -> &'static str {
        "fluid"
    }
}
