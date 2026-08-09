//! SPH 粒子数据结构。

use phy_math::{RealField, Vec3};
use serde::de::DeserializeOwned;
use serde::{Deserialize, Serialize};

/// 单个流体粒子。
///
/// 位置/速度/加速度采用世界坐标;密度 `rho` 与压力 `p` 为每步重算的派生量;
/// `mass` 通常为常量(同质量粒子),亦可在初始化时按目标密度设定。
///
/// 加速度拆成两部分:`acc` 仅由 SPH 近邻力(压力+粘性)每步重算;
/// `body_acc` 为体力累加器(重力 + 热浮力/刚体耦合浮力),由 `couple` 阶段写入、
/// 在 `integrate` 中与 `acc` 叠加后清零,从而不会被 `compute_forces` 覆盖。
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(bound = "T: RealField + Copy + Serialize + DeserializeOwned + nalgebra::Scalar")]
pub struct Particle<T: RealField + Copy> {
    /// 世界位置。
    #[serde(with = "phy_rigid::shape::serde_geom")]
    pub pos: Vec3<T>,
    /// 世界速度。
    #[serde(with = "phy_rigid::shape::serde_geom")]
    pub vel: Vec3<T>,
    /// SPH 近邻加速度(压力+粘性),每步由 `compute_forces` 重算。
    #[serde(with = "phy_rigid::shape::serde_geom")]
    pub acc: Vec3<T>,
    /// 体力累加器(重力 + 浮力/耦合体力),由 `couple` 写入、`integrate` 结算后清零。
    #[serde(with = "phy_rigid::shape::serde_geom")]
    pub body_acc: Vec3<T>,
    /// 当前密度。
    pub rho: T,
    /// 当前压力。
    pub p: T,
    /// 粒子质量。
    pub mass: T,
}

impl<T: RealField + Copy> Particle<T> {
    /// 以给定位置与质量创建静止粒子(其余量初始化为零/默认)。
    pub fn new(pos: Vec3<T>, mass: T) -> Self {
        Self {
            pos,
            vel: Vec3::zeros(),
            acc: Vec3::zeros(),
            body_acc: Vec3::zeros(),
            rho: T::zero(),
            p: T::zero(),
            mass,
        }
    }
}
