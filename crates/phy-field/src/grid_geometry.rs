//! 场网格几何接口。
//!
//! 所有离散场(热场 / 波场 / 电磁场)共享统一的网格拓扑定义,
//! 以便上层子系统按世界坐标采样、跨场耦合。

use phy_math::{RealField, Vec3};

/// 离散场的网格几何接口(共享采样/坐标换算基础设施)。
///
/// 所有离散标量/矢量场都坐落在一个均匀笛卡尔网格上;
/// 统一抽出 `dims` / `origin` / `cell_size` 三个几何参数,
/// 让 `world_to_cell` / `sample_world` 等通用函数可以作用于任意场类型。
pub trait GridGeometry<T: RealField + Copy> {
    /// 网格维度 (nx, ny, nz)。
    fn dims(&self) -> (usize, usize, usize);
    /// 网格原点(最小角)世界坐标。
    fn origin(&self) -> Vec3<T>;
    /// 单元尺寸 dx(立方网格)。
    fn cell_size(&self) -> T;
}
