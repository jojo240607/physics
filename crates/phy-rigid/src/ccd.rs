//! 连续碰撞检测(CCD, Continuous Collision Detection)。
//!
//! 离散碰撞检测只在固定步位姿上调用 narrow-phase,高速物体在一个时间步内的位移
//! 可能大于自身(或对方)尺寸,从而"跳过"薄壁/小体 —— 即**隧穿(tunneling)**。
//!
//! 这里采用**位移受限子步化(displacement-limited substepping)** 的保守 CCD 策略:
//! 在 `step` 内把 `[0, dt]` 切成 `n` 个子步,使所有可动体在每个子步内的位移不超过其
//! 最小特征尺寸的一部分(默认一半),再在每个子步上跑既有的离散 `collide` + 速度求解。
//! 因为每子步位移远小于物体尺寸,离散检测必能在本步内捕获接触,从而杜绝隧穿。
//!
//! 该方案对"大小悬殊"的物体对同样稳健(子步数由最小特征尺寸驱动,与对方尺寸无关),
//! 且保持时间步进的确定性。另提供 `swept_sphere_sphere`(球-球闭式 TOI)与
//! `ccd_contact`(合成接触)作为可复用的底层几何工具。

use num_traits::ToPrimitive;
use phy_math::{RealField, Vec3};

use crate::contact::Contact;
use crate::shape::{Body, Shape};

/// 时间步内的相对位移最小值(小于此视为静止,避免除零)。
fn eps<T: RealField>() -> T {
    T::from_f64(1e-9).unwrap()
}

/// 计算 CCD 子步数:使最快可动体在每个子步内的位移不超过其最小特征尺寸的 `frac` 倍。
///
/// - `bodies`:当前世界全部刚体。
/// - `dt`:本帧时间步。
/// - `max_sub`:子步上限(由 `SolverParams::ccd_max_substeps` 给定;设 0 表示关闭 CCD)。
///
/// 返回 `1` 表示无需细分(退化为整步离散检测,与无 CCD 等价)。
pub fn substep_count<T: RealField + Copy + ToPrimitive>(
    bodies: &[Body<T>],
    dt: T,
    max_sub: usize,
) -> usize {
    if max_sub == 0 {
        return 1;
    }
    let frac = T::from_f64(0.5).unwrap();
    let mut max_disp = T::zero();
    let mut min_feat = T::from_f64(f64::INFINITY).unwrap();
    for b in bodies {
        if b.inv_mass <= T::zero() {
            continue; // 静态体不贡献位移,也不限制子步。
        }
        // 本步线位移(角位移引起表面点位移的保守上界也并入,避免旋转隧穿)。
        let lin = b.vel.norm() * dt;
        let r = b.shape.bounding_sphere_r();
        let ang = b.ang_vel.norm() * dt * r;
        let disp = lin + ang;
        if disp > max_disp {
            max_disp = disp;
        }
        // 最小特征尺寸:球取半径;盒取最小半边长;凸体取包围球半径(保守,会偏向更多子步)。
        let feat = match &b.shape {
            Shape::Sphere { r } => *r,
            Shape::Box { half } => half.x.min(half.y).min(half.z),
            Shape::Capsule { r, .. } => *r,
            Shape::Convex { .. } => r,
            // 高度场为静态地形，取网格尺寸作为特征尺度（不参与 CCD 保守估计）。
            Shape::Heightfield { cell, .. } => *cell,
        };
        if feat < min_feat {
            min_feat = feat;
        }
    }
    if max_disp <= eps::<T>() || !min_feat.is_finite() {
        return 1;
    }
    // needed = ceil(max_disp / (frac * min_feat)),再夹到 [1, max_sub]。
    let needed = (max_disp / (frac * min_feat)).ceil();
    let mut n = if let Some(k) = needed.to_usize() {
        k
    } else {
        1
    };
    if n < 1 {
        n = 1;
    }
    if n > max_sub {
        n = max_sub;
    }
    n
}

/// 用两体当前的包围球半径,对相对运动做扫掠,求最早接触时间(TOI)。
///
/// 方程:`|rel + vrel·t|² = (ra + rb)²`,其中 `rel = pos_b - pos_a`,`vrel = vel_b - vel_a`。
/// 返回 `Some(t)` 表示在 `t ∈ [0, dt]` 内两包围球首次接触(`t≈0` 表示初始已重叠);
/// 返回 `None` 表示在 `dt` 内不会接触。适用于球-球等尺寸相近的精确 TOI 估计。
pub fn swept_sphere_sphere<T: RealField + Copy>(
    a: &Body<T>,
    b: &Body<T>,
    dt: T,
) -> Option<T> {
    let ra = a.shape.bounding_sphere_r();
    let rb = b.shape.bounding_sphere_r();
    let rel = b.pos - a.pos;
    let vrel = b.vel - a.vel;
    let sum = ra + rb;

    let aq = vrel.dot(&vrel);
    let two = T::one() + T::one();
    let bq = two * rel.dot(&vrel);
    let cq = rel.dot(&rel) - sum * sum;

    if aq <= eps::<T>() {
        return if cq <= T::zero() {
            Some(T::zero())
        } else {
            None
        };
    }

    let disc = bq * bq - two * two * aq * cq;
    if disc < T::zero() {
        return None;
    }
    let sq = disc.sqrt();
    let t1 = (-bq - sq) / (two * aq);
    let t2 = (-bq + sq) / (two * aq);

    if t1 >= T::zero() && t1 <= dt {
        Some(t1)
    } else if t2 >= T::zero() && t2 <= dt {
        Some(T::zero())
    } else if t1 < T::zero() && t2 > T::zero() {
        Some(T::zero())
    } else {
        None
    }
}

/// 合成一个包围球接触(连心法线,由 a 指向 b,接触点取 a 表面处,深度 0)。
/// 供调用方在 TOI 位姿处尝试精确 `collide` 失败时的兜底。
pub fn ccd_contact<T: RealField + Copy>(a: &Body<T>, b: &Body<T>) -> Contact<T> {
    let ra = a.shape.bounding_sphere_r();
    let dir = b.pos - a.pos;
    let nlen = dir.norm();
    let n = if nlen > eps::<T>() {
        dir / nlen
    } else {
        Vec3::new(T::zero(), T::one(), T::zero())
    };
    let point = a.pos + n * ra;
    Contact {
        normal: n,
        point,
        depth: T::zero(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::shape::Shape;

    fn sphere(pos: Vec3<f64>, vel: Vec3<f64>, r: f64) -> Body<f64> {
        let mut b = Body::new(Shape::Sphere { r }, pos, 1.0);
        b.vel = vel;
        b
    }

    #[test]
    fn swept_sphere_head_on_hits() {
        // a 在 -5 静止,r=0.5;b 在 0 以 -10 接近(朝 -x 方向),r=0.5 → 相距 5,和半径 1 → t=0.4。
        let a = sphere(Vec3::new(-5.0, 0.0, 0.0), Vec3::zeros(), 0.5);
        let b = sphere(Vec3::new(0.0, 0.0, 0.0), Vec3::new(-10.0, 0.0, 0.0), 0.5);
        let toi = swept_sphere_sphere(&a, &b, 1.0).unwrap();
        assert!((toi - 0.4).abs() < 1e-9, "toi={}", toi);
    }

    #[test]
    fn swept_sphere_misses_when_separating() {
        // b 远离 a → 永不接触。
        let a = sphere(Vec3::new(0.0, 0.0, 0.0), Vec3::zeros(), 0.5);
        let b = sphere(Vec3::new(3.0, 0.0, 0.0), Vec3::new(10.0, 0.0, 0.0), 0.5);
        assert!(swept_sphere_sphere(&a, &b, 1.0).is_none());
    }

    #[test]
    fn substep_count_scales_with_speed() {
        let slow = sphere(Vec3::zeros(), Vec3::new(0.1, 0.0, 0.0), 0.5);
        let fast = sphere(Vec3::zeros(), Vec3::new(100.0, 0.0, 0.0), 0.5);
        let n_slow = substep_count(&[slow], 1.0 / 120.0, 8);
        let n_fast = substep_count(&[fast], 1.0 / 120.0, 8);
        assert_eq!(n_slow, 1, "慢速不应细分");
        assert!(n_fast > 1, "高速应细分, got {}", n_fast);
        assert!(n_fast <= 8);
    }

    #[test]
    fn substep_count_disabled() {
        let fast = sphere(Vec3::zeros(), Vec3::new(100.0, 0.0, 0.0), 0.5);
        assert_eq!(substep_count(&[fast], 1.0 / 120.0, 0), 1);
    }
}
