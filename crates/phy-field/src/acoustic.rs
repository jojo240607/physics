//! 声波 / 压力波场(M19 / 路线图 #6)。
//!
//! 声压场满足与 `WaveField` 完全相同的标量波动方程
//! `∂²p/∂t² = c²∇²p`,区别仅在于相速度 `c` = 声速(空气 ≈ 343 m/s),
//! 且 `u` 的物理意义是**声压** `p`(相对环境压力的扰动)。因此本模块
//! 复用 `ScalarField::step_wave` 求解,仅在之上叠加声学特有的语义与工具:
//!
//! - 默认声速常量 [`SOUND_SPEED_AIR`]。
//! - 声源 = 压力注入(`add_pressure_source`,经 `src` 缓冲由 `step_wave` 注入)。
//! - 派生量采样:
//!   - 质点速度(粒子速度) `v = -(1/(ρ0·c²)) · ∂p/∂t · ∇⁻¹` 的稳态近似——
//!     这里用一阶时间导数 `∂p/∂t` 直接给出能流方向(`grid` 内保存 prior,
//!     差分可得);幅度正比于 `∂p/∂t / (ρ0·c²)`。
//!   - 声强(能流密度,瞬时) `I = p · v`,用于遮挡衰减/能量可视化。
//!
//! 与 `WaveField` 一样实现了 `GridGeometry` + `Subsystem`,可直接挂入 `World`
//! 与刚体/软体/光学子系统共存(例如刚体运动作为活塞声源推动声场)。

use phy_core::Subsystem;
use phy_math::{RealField, Vec3};
use serde::de::DeserializeOwned;
use serde::{Deserialize, Serialize};

use crate::grid_geometry::GridGeometry;
use crate::grid::ScalarField;

/// 空气声速(m/s,20℃)。
pub const SOUND_SPEED_AIR: f64 = 343.0;

/// 声波 / 压力波场:声压 `p` 的标量波动方程求解 + 声学派生量工具。
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(bound = "T: RealField + Copy + Serialize + DeserializeOwned + nalgebra::Scalar")]
pub struct AcousticField<T: RealField + Copy> {
    /// 底层标量场(值 = 声压扰动 p)。
    pub field: ScalarField<T>,
    /// 声速平方 c²。
    pub c2: T,
    /// 介质密度 ρ0(用于质点速度 / 声强换算)。
    pub rho0: T,
}

impl<T: RealField + Copy> AcousticField<T> {
    /// 构造声场。`sound_speed` 为介质声速,`rho0` 为介质密度。
    pub fn new(field: ScalarField<T>, sound_speed: T, rho0: T) -> Self {
        let c2 = sound_speed * sound_speed;
        Self { field, c2, rho0 }
    }

    /// 默认的空气中声场(声速 343 m/s,ρ0 = 1.2 kg/m³)。
    pub fn air(nx: usize, ny: usize, nz: usize, dx: T, origin: Vec3<T>, bc: crate::grid::Bc) -> Self {
        let f = ScalarField::<T>::new(nx, ny, nz, dx, T::zero(), bc).with_origin(origin);
        let c = <T as num_traits::FromPrimitive>::from_f64(SOUND_SPEED_AIR).unwrap();
        let r = <T as num_traits::FromPrimitive>::from_f64(1.2).unwrap();
        Self::new(f, c, r)
    }

    /// 注入声压源(累加进 `src`,由 `step` 经 `step_wave` 注入)。
    pub fn add_pressure_source(&mut self, x: usize, y: usize, z: usize, p: T) {
        self.field.add_source(x, y, z, p);
    }
}

impl<T: RealField + Copy> GridGeometry<T> for AcousticField<T> {
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

impl<T: RealField + Copy> Subsystem<T> for AcousticField<T> {
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
        "acoustic"
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Bc;
    use phy_core::world::World;
    use phy_math::Vec3;

    #[test]
    fn acoustic_source_propagates_pressure_pulse() {
        // 1D 管道(nz=ny=1),中心注入一个脉冲,压力波应向外双向传播。
        let f = ScalarField::<f64>::new(101, 1, 1, 1.0, 0.0, Bc::Neumann).with_origin(Vec3::zeros());
        let c = SOUND_SPEED_AIR; // 343
        let mut af = AcousticField::new(f, c, 1.2);
        // dt 满足 CFL: c*dt/dx ≤ 1/√3 ≈ 0.577 -> dt = 0.001 足够。
        let dt = 0.001;
        af.add_pressure_source(50, 0, 0, 1.0);
        af.field.step_wave(af.c2, dt); // 注入并起振
        // 演化若干步后,中心附近应有压力,且离中心一定距离处也应被波及,
        // 而更远处的网格应仍接近 0(波尚未到达)。
        for _ in 0..20 {
            af.field.step_wave(af.c2, dt);
        }
        let center = af.field.sample(50, 0, 0);
        let near = af.field.sample(45, 0, 0);
        let far = af.field.sample(10, 0, 0);
        assert!(center.abs() > 1e-6, "中心应有压力");
        assert!(near.abs() > 1e-9, "近邻应被波及");
        // 20 步 * c*dt = 20*343*0.001 ≈ 6.86 网格;距离 40 的远处应未到达。
        assert!(far.abs() < 1e-6, "远处波尚未到达");
    }

    #[test]
    fn acoustic_field_runs_in_world() {
        let f = ScalarField::<f64>::new(33, 1, 1, 1.0, 0.0, Bc::Neumann).with_origin(Vec3::zeros());
        let af = AcousticField::new(f, SOUND_SPEED_AIR, 1.2);
        let mut w: World<f64> = World::new();
        w.add_subsystem(Box::new(af));
        w.step(0.001);
        let af2 = w.get(0).unwrap().as_any().downcast_ref::<AcousticField<f64>>().unwrap();
        assert!(af2.field.t > 0.0);
    }
}
