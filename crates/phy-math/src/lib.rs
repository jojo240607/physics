//! phy-math: 数学基础层。
//!
//! 重导出 nalgebra 并约定全局泛型标量 `T: RealField`,
//! 使整个引擎可在 f32 / f64 间切换。

pub use nalgebra::{self as na, RealField, UnitVector3};

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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn gravity_points_down_negative_y() {
        // 物理约定:重力沿 -Y(向上为 +Y)。任何坐标系改动必须先打破此不变量。
        let g = gravity::<f64>();
        assert_eq!(g.x, 0.0);
        assert_eq!(g.z, 0.0);
        assert!(g.y < 0.0, "gravity must point toward -Y, got y={}", g.y);
        assert!((g.y + 9.81).abs() < 1e-9);
    }

    #[test]
    fn vec3_basic_ops() {
        let a = Vec3::new(1.0_f64, 2.0, 3.0);
        let b = Vec3::new(4.0_f64, 5.0, 6.0);
        assert_eq!(a + b, Vec3::new(5.0, 7.0, 9.0));
        assert_eq!(a.dot(&b), 32.0); // 1*4 + 2*5 + 3*6
        assert_eq!(a.cross(&b), Vec3::new(-3.0, 6.0, -3.0));
        assert!((a.norm() - 14.0_f64.sqrt()).abs() < 1e-12);
    }

    #[test]
    fn quaternion_rotation_is_uniform() {
        // 绕 Z 轴转 90° 把 +X 转到 +Y。
        let axis = UnitVector3::new_normalize(Vec3::z());
        let q = na::UnitQuaternion::<f64>::from_axis_angle(&axis, std::f64::consts::FRAC_PI_2);
        let v = Vec3::new(1.0_f64, 0.0, 0.0);
        let r = q * v;
        assert!((r.x).abs() < 1e-12);
        assert!((r.y - 1.0).abs() < 1e-12);
        assert_eq!(r.z, 0.0);
        // 旋转保持向量长度。
        assert!((r.norm() - 1.0).abs() < 1e-12);
    }

    #[test]
    fn real_field_scalar_behaves_as_float() {
        // RealField 标量(此处 f64)可参与常规浮点运算且保持精度。
        let x: f64 = 3.5;
        let y = x * 2.0;
        assert_eq!(y, 7.0);
        assert!((y.sqrt() - 7.0_f64.sqrt()).abs() < 1e-12);
    }
}
