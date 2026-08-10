//! 刚体子系统适配:`RigidWorld` 接入 `phy_core::World` 统一调度。

use std::any::Any;

use phy_core::{Subsystem, World};
use phy_field::{EmField, GravField, HeatField};
use phy_math::{RealField, Vec3};
use serde::de::DeserializeOwned;
use serde::{Deserialize, Serialize};

use crate::world::RigidWorld;

/// 刚体子系统:包装 `RigidWorld` 以挂在 `World` 上。
///
/// 渲染时可通过 `World::get(i).as_any().downcast_ref::<RigidSubsystem<T>>()`
/// 取回,读取 `world` 字段。
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(bound = "T: RealField + Copy + Serialize + DeserializeOwned + num_traits::ToPrimitive")]
pub struct RigidSubsystem<T: RealField + Copy + num_traits::ToPrimitive> {
    /// 内部刚体世界(渲染时直接访问)。
    pub world: RigidWorld<T>,
    /// 热浮力温度膨胀系数 β(ρ(T)=ρ0/(1+β·(T-T_ref)))。0 表示无热浮力。
    pub thermal_expansion: T,
    /// 对流换热注入强度(运动物体加热场)。0 表示不注入热源。
    pub heat_gain: T,
    /// 热浮力参考温度 T_ref(环境温度基线,ρ(T_ref)=ρ0)。0 表示以 0 为环境温度。
    pub t_ref: T,
    /// 电磁耦合强度(洛伦兹力缩放)。0 表示无电磁耦合。
    pub em_coupling: T,
    /// 引力耦合强度(局部引力井加速度缩放)。0 表示无引力场耦合。
    pub grav_coupling: T,
}

impl<T: RealField + Copy + num_traits::ToPrimitive> RigidSubsystem<T> {
    /// 由既有 `RigidWorld` 构造。
    pub fn new(world: RigidWorld<T>) -> Self {
        Self {
            world,
            thermal_expansion: T::zero(),
            heat_gain: T::zero(),
            t_ref: T::zero(),
            em_coupling: T::zero(),
            grav_coupling: T::zero(),
        }
    }
}

impl<T: RealField + Copy + num_traits::ToPrimitive + num_traits::Float> Subsystem<T>
    for RigidSubsystem<T>
{
    fn as_any(&self) -> &dyn Any {
        self
    }

    fn as_any_mut(&mut self) -> &mut dyn Any {
        self
    }

    fn step(&mut self, dt: &T) {
        self.world.step(*dt);
    }

    /// 刚体看门狗:扫描所有刚体的位置/速度/角速度/姿态四元数是否有限。
    fn validate(&self) -> Result<(), phy_core::WorldError> {
        for b in self.world.bodies.iter() {
            if !b.pos.x.is_finite() || !b.pos.y.is_finite() || !b.pos.z.is_finite() {
                return Err(phy_core::WorldError::NonFinite {
                    subsystem: "rigid",
                    field: "pos",
                });
            }
            if !b.vel.x.is_finite() || !b.vel.y.is_finite() || !b.vel.z.is_finite() {
                return Err(phy_core::WorldError::NonFinite {
                    subsystem: "rigid",
                    field: "vel",
                });
            }
            if !b.ang_vel.x.is_finite() || !b.ang_vel.y.is_finite() || !b.ang_vel.z.is_finite() {
                return Err(phy_core::WorldError::NonFinite {
                    subsystem: "rigid",
                    field: "ang_vel",
                });
            }
            let q = b.rot.quaternion();
            if !q.w.is_finite() || !q.i.is_finite() || !q.j.is_finite() || !q.k.is_finite() {
                return Err(phy_core::WorldError::NonFinite {
                    subsystem: "rigid",
                    field: "quat",
                });
            }
        }
        Ok(())
    }

    /// 刚体↔热场 / 刚体↔电磁场 / 刚体↔引力场 双向耦合。
    ///
    /// 若 `thermal_expansion>0` 或 `heat_gain>0`,在 `World` 中动态查找 `HeatField`
    /// 子系统,经 `remove`/`insert` 安全取可变引用后调用 `RigidWorld::couple_heat`。
    /// 若 `em_coupling>0` 且世界中存在带电刚体,动态查找 `EmField` 子系统并调用
    /// `RigidWorld::couple_em`(洛伦兹力 + 运动感应电荷)。
    /// 若 `grav_coupling>0`,动态查找 `GravField` 子系统并调用 `RigidWorld::couple_grav`
    /// (局部引力井偏转 + 运动质量沉积)。
    fn couple(&mut self, world: &mut World<T>, dt: &T) {
        // 刚体↔热场(M11)。
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
        // 刚体↔电磁场(M12)。
        if self.em_coupling > T::zero() {
            let n = world.subsystem_count();
            let mut em_idx: Option<usize> = None;
            for i in 0..n {
                if let Some(s) = world.get(i) {
                    if s.as_any().downcast_ref::<EmField<T>>().is_some() {
                        em_idx = Some(i);
                        break;
                    }
                }
            }
            if let Some(ei) = em_idx {
                let mut em_box = world.remove(ei);
                let em = em_box
                    .as_any_mut()
                    .downcast_mut::<EmField<T>>()
                    .expect("em subsystem type mismatch");
                self.world.couple_em(em, *dt, self.em_coupling);
                world.insert(ei, em_box);
            }
        }
        // 刚体↔引力场(M13)。
        if self.grav_coupling > T::zero() {
            let n = world.subsystem_count();
            let mut grav_idx: Option<usize> = None;
            for i in 0..n {
                if let Some(s) = world.get(i) {
                    if s.as_any().downcast_ref::<GravField<T>>().is_some() {
                        grav_idx = Some(i);
                        break;
                    }
                }
            }
            if let Some(gi) = grav_idx {
                let mut grav_box = world.remove(gi);
                let grav = grav_box
                    .as_any_mut()
                    .downcast_mut::<GravField<T>>()
                    .expect("grav subsystem type mismatch");
                self.world.couple_grav(grav, *dt, self.grav_coupling);
                world.insert(gi, grav_box);
            }
        }
    }

    fn name(&self) -> &'static str {
        "rigid"
    }

    fn total_momentum(&self) -> Vec3<T> {
        let mut p = Vec3::zeros();
        for b in &self.world.bodies {
            if b.inv_mass > T::zero() {
                let m = T::one() / b.inv_mass;
                p += b.vel * m;
            }
        }
        p
    }

    fn kinetic_energy(&self) -> T {
        let mut e = T::zero();
        for b in &self.world.bodies {
            if b.inv_mass > T::zero() {
                let m = T::one() / b.inv_mass;
                e += (b.vel.norm_squared() * m) / (T::one() + T::one());
                // 转动动能 ½ ωᵀ I ω,其中 I = inv_inertia_world 的逆。
                if let Some(iw_inv) = b.inv_inertia_world().clone().try_inverse() {
                    let iw = iw_inv * b.ang_vel;
                    e += (b.ang_vel.dot(&iw)) / (T::one() + T::one());
                }
            }
        }
        e
    }
}
