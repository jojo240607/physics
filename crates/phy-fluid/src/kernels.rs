//! SPH 光滑核函数(Müller et al. 2003)。
//!
//! 所有核均定义在 3D 下,以光滑长度 `h` 为作用半径。
//! 提供:Poly6(密度)、Spiky 梯度(压力)、Viscosity 拉普拉斯(粘性)。

use phy_math::{RealField, Vec3};

/// 核函数系数与常用幂次,预计算以避免每粒子重复求值。
pub struct Kernels<T: RealField + Copy> {
    h: T,
    h2: T,
    /// Poly6 归一化系数: 315 / (64 π h^9)
    poly6_coeff: T,
    /// Spiky 梯度系数: -45 / (π h^6)
    spiky_grad_coeff: T,
    /// 粘性拉普拉斯系数: 45 / (π h^6)
    visc_lap_coeff: T,
}

impl<T: RealField + Copy> Kernels<T> {
    /// 由光滑长度 `h` 构造并预计算系数。
    pub fn new(h: T) -> Self {
        let pi = <T as num_traits::FromPrimitive>::from_f64(std::f64::consts::PI).unwrap();
        let h2 = h * h;
        let h3 = h2 * h;
        let h6 = h3 * h3;
        let h9 = h6 * h3;
        Self {
            h,
            h2,
            poly6_coeff: <T as num_traits::FromPrimitive>::from_f64(315.0).unwrap() / (<T as num_traits::FromPrimitive>::from_f64(64.0).unwrap() * pi * h9),
            spiky_grad_coeff: -<T as num_traits::FromPrimitive>::from_f64(45.0).unwrap() / (pi * h6),
            visc_lap_coeff: <T as num_traits::FromPrimitive>::from_f64(45.0).unwrap() / (pi * h6),
        }
    }

    /// Poly6 核值 W(r),r 为两点距离。
    #[inline]
    pub fn poly6(&self, r: T) -> T {
        if r >= self.h {
            return T::zero();
        }
        let diff = self.h2 - r * r;
        self.poly6_coeff * diff * diff * diff
    }

    /// Spiky 核梯度幅值(不含方向) ∇W(r) 的大小:
    /// |∇W| = spiky_coeff * (h - r)^2 ;方向沿 (r_i - r_j)/r。
    #[inline]
    pub fn spiky_grad_mag(&self, r: T) -> T {
        if r >= self.h || r <= T::zero() {
            return T::zero();
        }
        let diff = self.h - r;
        self.spiky_grad_coeff * diff * diff
    }

    /// 粘性核拉普拉斯 ∇²W(r) = visc_coeff * (h - r)。
    #[inline]
    pub fn visc_lap(&self, r: T) -> T {
        if r >= self.h {
            return T::zero();
        }
        self.visc_lap_coeff * (self.h - r)
    }
}

/// 便捷:由两点求距离(避免重复开方路径)。
#[inline]
pub fn dist<T: RealField + Copy>(a: &Vec3<T>, b: &Vec3<T>) -> T {
    (a - b).norm()
}
