//! 碰撞检测结果的数据结构。

use phy_math::{RealField, Vec3};

/// 一对物体之间的接触信息(单向:从 a 指向 b)。
#[derive(Debug, Clone)]
pub struct Contact<T: RealField + Copy> {
    /// 世界空间接触点(近似两表面中点)。
    pub point: Vec3<T>,
    /// 接触法线(由 a 指向 b,单位向量)。
    pub normal: Vec3<T>,
    /// 穿透深度(正数表示相交)。
    pub depth: T,
}

impl<T: RealField + Copy> Contact<T> {
    pub fn new(point: Vec3<T>, normal: Vec3<T>, depth: T) -> Self {
        Self {
            point,
            normal: normal.normalize(),
            depth,
        }
    }
}
