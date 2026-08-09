//! SPH 求解参数。

use nalgebra::Scalar;
use num_traits::FromPrimitive;
use phy_math::{RealField, Vec3};
use serde::de::DeserializeOwned;
use serde::{Deserialize, Serialize};

/// SPH 求解参数(全部采用工程单位,默认 f64)。
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(bound = "T: RealField + Copy + Serialize + DeserializeOwned + Scalar")]
pub struct SphParams<T: RealField + Copy> {
    /// 静止密度 ρ0(用于压力状态方程)。
    pub rest_density: T,
    /// 压力刚度系数 k(越大越不可压,但需更小 dt 保持稳定)。
    pub stiffness: T,
    /// 动力粘度 μ(牛顿流体,也是多材料表 `visc_k` 默认值)。
    pub viscosity: T,
    /// 非牛顿幂律一致性系数 `k`,按材料索引(`visc_k[material]`)。
    /// 有效粘度 μ_eff = `visc_k[m] · max(应变率, shear_min)^(visc_n[m]-1)`。
    pub visc_k: Vec<T>,
    /// 非牛顿幂律指数 `n`,按材料索引(`visc_n[material]`)。
    /// n = 1 退化为牛顿流体;`n < 1` 剪切变稀(高剪切更稀);`n > 1` 剪切变稠。
    pub visc_n: Vec<T>,
    /// 应变率正则化下限(避免零剪切处粘度发散/除零)。
    pub shear_min: T,
    /// 粒子质量。
    pub mass: T,
    /// 光滑长度 h(核作用半径,也是网格单元边长)。
    pub h: T,
    /// 重力加速度(向量,x 右 / y 上 / z 前)。
    #[serde(with = "phy_rigid::shape::serde_geom")]
    pub gravity: Vec3<T>,
    /// 模拟盒边界(粒子被约束在 [bounds_min, bounds_max] 内)。
    #[serde(with = "phy_rigid::shape::serde_geom")]
    pub bounds_min: Vec3<T>,
    /// 模拟盒边界上限。
    #[serde(with = "phy_rigid::shape::serde_geom")]
    pub bounds_max: Vec3<T>,
    /// 边界反弹速度保留系数(0=完全吸收,1=完全弹性)。
    pub boundary_damp: T,
}

impl<T: RealField + Copy> SphParams<T> {
    /// 一组合理的默认参数(适配 ~0.04 单位间距的粒子、盒尺度 10)。
    pub fn defaults() -> Self
    where
        T: FromPrimitive,
    {
        Self {
            rest_density: T::from_f64(1000.0).unwrap(),
            stiffness: T::from_f64(250.0).unwrap(),
            viscosity: T::from_f64(3.5).unwrap(),
            visc_k: vec![T::from_f64(3.5).unwrap()],
            visc_n: vec![T::from_f64(1.0).unwrap()],
            shear_min: T::from_f64(0.01).unwrap(),
            mass: T::from_f64(0.02).unwrap(),
            h: T::from_f64(0.2).unwrap(),
            gravity: Vec3::new(
                T::zero(),
                T::from_f64(-9.81).unwrap(),
                T::zero(),
            ),
            bounds_min: Vec3::new(
                T::from_f64(-5.0).unwrap(),
                T::from_f64(-5.0).unwrap(),
                T::from_f64(-5.0).unwrap(),
            ),
            bounds_max: Vec3::new(
                T::from_f64(5.0).unwrap(),
                T::from_f64(5.0).unwrap(),
                T::from_f64(5.0).unwrap(),
            ),
            boundary_damp: T::from_f64(0.4).unwrap(),
        }
    }
}
