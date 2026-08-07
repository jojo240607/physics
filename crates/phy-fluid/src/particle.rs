//! SPH 粒子数据结构。

use phy_math::{RealField, Vec3};

/// 单个流体粒子。
///
/// 位置/速度/加速度采用世界坐标;密度 `rho` 与压力 `p` 为每步重算的派生量;
/// `mass` 通常为常量(同质量粒子),亦可在初始化时按目标密度设定。
#[derive(Debug, Clone)]
pub struct Particle<T: RealField + Copy> {
    /// 世界位置。
    pub pos: Vec3<T>,
    /// 世界速度。
    pub vel: Vec3<T>,
    /// 世界加速度(每步由受力计算)。
    pub acc: Vec3<T>,
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
            rho: T::zero(),
            p: T::zero(),
            mass,
        }
    }
}
