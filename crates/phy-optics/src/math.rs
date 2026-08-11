//! 光学几何/向量工具:折射(Snell)、反射、Fresnel 反射率。

use phy_math::{RealField, Vec3};

/// 归一化方向(防御除零:零向量返回 +X)。
pub fn normalize<T: RealField + Copy>(v: &Vec3<T>) -> Vec3<T> {
    let n = v.norm();
    if n > T::from_f64(1e-12).unwrap() {
        *v / n
    } else {
        Vec3::new(T::one(), T::zero(), T::zero())
    }
}

/// 反射方向(入射 `i` 与表面法线 `n` 均须为单位向量,`n` 指向入射侧)。
pub fn reflect<T: RealField + Copy>(i: &Vec3<T>, n: &Vec3<T>) -> Vec3<T> {
    let cosi = i.dot(n);
    *i - *n * (cosi * <T as num_traits::FromPrimitive>::from_f64(2.0).unwrap())
}

/// 折射方向(Snell 定律)。
///
/// `i`:单位入射方向;`n`:单位表面法线(指向入射介质侧);
/// `eta`:`n_i / n_t`(入射折射率 / 透射折射率)。
/// 返回 `None` 表示发生全内反射(TIR),应改用反射。
pub fn refract<T: RealField + Copy>(
    i: &Vec3<T>,
    n: &Vec3<T>,
    eta: T,
) -> Option<Vec3<T>> {
    let one = T::one();
    let cosi = -i.dot(n); // 入射方向与法线夹角余弦(要求 n 指向入射侧)
    let cosi2 = cosi * cosi;
    let k = one - eta * eta * (one - cosi2);
    if k < T::zero() {
        return None; // 全内反射
    }
    let t = *i * eta + *n * (eta * cosi - k.sqrt());
    Some(normalize(&t))
}

/// Schlick 近似的 Fresnel 反射率。
///
/// `cosi`:入射角余弦(取绝对值);`f0`:垂直入射反射率 `(n1-n2)²/(n1+n2)²`。
pub fn fresnel<T: RealField + Copy>(cosi: T, f0: T) -> T {
    let one = T::one();
    let tmp = one - cosi;
    let tmp2 = tmp * tmp;
    f0 + (one - f0) * tmp2 * tmp2 * tmp // f0 + (1-f0)(1-cosi)^5
}

/// 由两介质折射率算垂直入射反射率 `f0`。
pub fn f0_of<T: RealField + Copy>(n1: T, n2: T) -> T {
    let a = (n1 - n2) / (n1 + n2);
    a * a
}
