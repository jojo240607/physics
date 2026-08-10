//! 把 `GranularWorld` 适配为 `phy_core::Subsystem<T>`,挂入统一仿真 `World<T>`。

use std::any::Any;

use phy_core::world::Subsystem;
use phy_math::RealField;
use serde::de::DeserializeOwned;
use serde::{Deserialize, Serialize};

use crate::world::GranularWorld;

/// 颗粒子系统包装。
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(bound = "T: RealField + Copy + Serialize + DeserializeOwned + nalgebra::Scalar")]
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

impl<T: RealField + Copy + num_traits::Float> Subsystem<T> for GranularSubsystem<T> {
    fn as_any(&self) -> &dyn Any {
        self
    }

    fn as_any_mut(&mut self) -> &mut dyn Any {
        self
    }

    fn step(&mut self, dt: &T) {
        self.world.step(*dt);
    }

    /// 颗粒看门狗:扫描所有颗粒的位置/速度是否有限。
    fn validate(&self) -> Result<(), phy_core::WorldError> {
        for g in self.world.grains.iter() {
            if !g.pos.x.is_finite() || !g.pos.y.is_finite() || !g.pos.z.is_finite() {
                return Err(phy_core::WorldError::NonFinite {
                    subsystem: "granular",
                    field: "pos",
                });
            }
            if !g.vel.x.is_finite() || !g.vel.y.is_finite() || !g.vel.z.is_finite() {
                return Err(phy_core::WorldError::NonFinite {
                    subsystem: "granular",
                    field: "vel",
                });
            }
        }
        Ok(())
    }

    fn name(&self) -> &'static str {
        "granular"
    }
}
