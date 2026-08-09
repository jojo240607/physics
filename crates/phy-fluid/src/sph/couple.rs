//! 软体质点 / 任意点质量与流体的耦合接口所用的单点描述。

use phy_math::{RealField, Vec3};

/// 软体 / 任意点质量与流体的耦合接口所用的单点描述。
///
/// 调用方填入 `pos`/`vel`/`mass`,`couple_points` 计算施加到该点的合力(浮力+阻力)
/// 写入 `force`;调用方据此更新点速度,即完成 fluid→soft 作用。soft→fluid 的
/// 反向动量由 `couple_points` 内部直接分配到邻域流体粒子,无需调用方处理。
pub struct CouplePoint<T: RealField + Copy> {
    /// 点世界坐标(只读输入)。
    pub pos: Vec3<T>,
    /// 点速度(只读输入)。
    pub vel: Vec3<T>,
    /// 点质量(只读输入)。
    pub mass: T,
    /// 输出:本步施加到该点的净力(浮力 + 阻力)。
    pub force: Vec3<T>,
}

impl<T: RealField + Copy> CouplePoint<T> {
    /// 构造一个待耦合点。
    pub fn new(pos: Vec3<T>, vel: Vec3<T>, mass: T) -> Self {
        Self {
            pos,
            vel,
            mass,
            force: Vec3::zeros(),
        }
    }
}
