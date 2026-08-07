//! phy-math: 数学基础层。
//!
//! 重导出 nalgebra 并约定全局泛型标量 `T: RealField`,
//! 使整个引擎可在 f32 / f64 间切换。

pub use nalgebra::{self as na, RealField};

/// 三维向量泛型别名。
pub type Vec3<T> = na::Vector3<T>;
/// 四元数(旋转)泛型别名。
pub type Quat<T> = na::Quaternion<T>;
/// 3x3 矩阵(惯性张量 / 旋转)泛型别名。
pub type Mat3<T> = na::Matrix3<T>;
/// 仿射变换(平移+旋转)泛型别名。
pub type Isometry3<T> = na::Isometry3<T>;

/// 重力常量(默认沿 -Y,大小由调用方以对应标量给定)。
pub fn gravity<T: RealField>() -> Vec3<T> {
    Vec3::new(T::zero(), T::from_f64(-9.81).unwrap(), T::zero())
}
