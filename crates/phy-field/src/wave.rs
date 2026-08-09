//! 波动/电磁标量子系统(M7)。
//!
//! `WaveField` 是 `grid::ScalarField` 的 `Subsystem` 适配层;波动方程求解逻辑
//! 在 `grid::ScalarField::step_wave` 上实现。

use phy_core::Subsystem;
use phy_math::{RealField, Vec3};
use serde::de::DeserializeOwned;
use serde::{Deserialize, Serialize};

use crate::grid_geometry::GridGeometry;
use crate::grid::ScalarField;

/// 波动/电磁标量子系统。
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(bound = "T: RealField + Copy + Serialize + DeserializeOwned + nalgebra::Scalar")]
pub struct WaveField<T: RealField + Copy> {
    /// 底层标量场(电势/位移)。
    pub field: ScalarField<T>,
    /// 波速平方 c²。
    pub c2: T,
}

impl<T: RealField + Copy> WaveField<T> {
    /// 构造波场适配层。
    pub fn new(field: ScalarField<T>, c2: T) -> Self {
        Self { field, c2 }
    }
}

impl<T: RealField + Copy> GridGeometry<T> for WaveField<T> {
    fn dims(&self) -> (usize, usize, usize) {
        (self.field.nx, self.field.ny, self.field.nz)
    }
    fn origin(&self) -> Vec3<T> {
        self.field.origin
    }
    fn cell_size(&self) -> T {
        self.field.dx
    }
}

impl<T: RealField + Copy> Subsystem<T> for WaveField<T> {
    fn as_any(&self) -> &dyn std::any::Any {
        self
    }

    fn as_any_mut(&mut self) -> &mut dyn std::any::Any {
        self
    }

    fn step(&mut self, dt: &T) {
        self.field.step_wave(self.c2, *dt);
    }
    fn name(&self) -> &'static str {
        "wave"
    }
}
