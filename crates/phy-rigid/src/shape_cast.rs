//! 形状投射 / 扫掠体检测(P2-b,游戏侧常用查询)。
//!
//! 提供 `shape_cast`(扫掠查询):把一个查询形状(球/盒/胶囊/凸体/复合)从 `from`
//! 位姿沿直线扫掠到 `to` 位姿(可带旋转),返回**第一个**与之相交的世界 body 的
//! 命中信息(命中比例、命中体索引、交点、命中表面法线)。
//!
//! 实现策略:复用既有 `narrowphase::collide` 做"某一位姿下查询形状 vs 世界 body"
//! 的离散接触判定,再对扫掠参数 `α∈[0,1]` 做**二分搜索**定位"刚好接触"的临界
//! 比例。这比完整连续 CCD 扫掠(需对运动形状做保守推进)简单且确定性强,对游戏
//! 侧"子弹/角色胶囊移动是否撞墙"等场景足够。精度由 `max_iters`(默认 20 → 约
//! `1/2^20` 分辨率)控制。
//!
//! 过滤规则与常规碰撞一致:按查询形状的 `layers`/`collision_mask` 与世界 body 的
//! `layers`/`collision_mask` 做位与;`is_sensor` 的 body 不参与阻挡(仅触发器)。

use phy_math::{na, RealField, Vec3};
use serde::de::DeserializeOwned;
use serde::{Deserialize, Serialize};

use crate::narrowphase::collide;
use crate::shape::{Body, Shape};
use num_traits::NumCast;

/// 形状投射命中结果。
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
#[serde(bound = "T: RealField + Copy + Serialize + DeserializeOwned")]
pub struct ShapeCastHit<T: RealField + Copy> {
    /// 命中点沿扫掠路径的比例 ∈ [0,1](0 = 起点即接触,1 = 到达终点仍未接触则不会返回)。
    pub fraction: T,
    /// 被命中的世界 body 全局索引。
    pub body_index: usize,
    /// 命中点世界坐标。
    #[serde(with = "crate::shape::serde_geom")]
    pub point: Vec3<T>,
    /// 命中表面世界法线(指向查询形状来向一侧,单位向量)。
    #[serde(with = "crate::shape::serde_geom")]
    pub normal: Vec3<T>,
}

/// 在 `α` 比例处把查询形状放到世界位姿(线性插值位置 + 球面插值旋转)。
fn pose_at<T: RealField + Copy>(
    from: &Vec3<T>,
    from_rot: &na::UnitQuaternion<T>,
    to: &Vec3<T>,
    to_rot: &na::UnitQuaternion<T>,
    alpha: T,
) -> (Vec3<T>, na::UnitQuaternion<T>) {
    let pos = *from + (*to - *from) * alpha;
    // 球面线性插值旋转(短弧)。
    let rot = from_rot.slerp(to_rot, alpha);
    (pos, rot)
}

/// 判断在给定 `α` 处,查询形状是否**受阻**(与任一通过过滤的世界 body 产生接触)。
fn blocked_at<T: RealField + Copy + NumCast>(
    shape: &Shape<T>,
    layers: u32,
    mask: u32,
    pos: &Vec3<T>,
    rot: &na::UnitQuaternion<T>,
    bodies: &[Body<T>],
) -> Option<(usize, Vec3<T>, Vec3<T>)> {
    let query = Body {
        shape: shape.clone(),
        pos: *pos,
        rot: *rot,
        inv_mass: T::zero(),
        layers,
        collision_mask: mask,
        ..Default::default()
    };
    for (i, b) in bodies.iter().enumerate() {
        // 过滤:碰撞层位与;传感器不阻挡。
        if b.is_sensor {
            continue;
        }
        if query.layers & b.collision_mask == 0 || b.layers & query.collision_mask == 0 {
            continue;
        }
        if let Some(c) = collide(&query, b) {
            return Some((i, c.point, c.normal));
        }
    }
    None
}

/// 形状扫掠查询:从 `from`(姿态 `from_rot`)沿直线扫掠到 `to`(姿态 `to_rot`),返回
/// 第一个受阻的世界 body。
///
/// - `shape`/`layers`/`collision_mask`:查询形状的几何与碰撞层(决定与哪些 body 交互)。
/// - `bodies`:世界 body 切片(通常传 `&world.bodies`)。
/// - `ignore`:可选忽略的 body 索引(如投射体自身),不计为阻挡。
/// - `max_iters`:二分搜索迭代次数(默认 20)。
///
/// 返回 `None` 表示整个扫掠路径畅通(到达 `to` 仍未接触)。
pub fn shape_cast<T: RealField + Copy + NumCast>(
    shape: &Shape<T>,
    from: &Vec3<T>,
    from_rot: &na::UnitQuaternion<T>,
    to: &Vec3<T>,
    to_rot: &na::UnitQuaternion<T>,
    bodies: &[Body<T>],
    layers: u32,
    mask: u32,
    ignore: Option<usize>,
    max_iters: usize,
) -> Option<ShapeCastHit<T>> {
    let zero = T::zero();
    let one = T::one();
    // 起点即受阻 → 命中比例 0。
    let start_pos = *from;
    let start_rot = *from_rot;
    if let Some((i, p, n)) = blocked_at(shape, layers, mask, &start_pos, &start_rot, bodies) {
        if ignore != Some(i) {
            return Some(ShapeCastHit {
                fraction: zero,
                body_index: i,
                point: p,
                normal: n,
            });
        }
    }

    // 扫描式定位首个受阻区间:扫掠路径可能"穿过"障碍(起点/终点均畅通,但中段受阻),
    // 故不能简单以终点畅通判为全程畅通。先粗扫找到第一个受阻采样点,再在其前一点
    // (畅通)与之间做二分精化,得到最小命中比例。
    let coarse = max_iters * 4; // 粗扫步数随精度提升。
    let mut prev_a = zero;
    let mut prev_pos = start_pos;
    let mut prev_rot = start_rot;
    for k in 1..=coarse {
        let a = T::from_f64(k as f64 / coarse as f64).unwrap();
        let (pa, ra) = pose_at(from, from_rot, to, to_rot, a);
        match blocked_at(shape, layers, mask, &pa, &ra, bodies) {
            Some((i, p, n)) if ignore != Some(i) => {
                // 在 [prev_a, a] 之间二分精化首个受阻 α。
                let mut lo = prev_a;
                let mut lo_pos = prev_pos;
                let mut lo_rot = prev_rot;
                let mut hi = a;
                let mut hit = ShapeCastHit {
                    fraction: a,
                    body_index: i,
                    point: p,
                    normal: n,
                };
                for _ in 0..max_iters {
                    let mid = (lo + hi) * (one / (one + one));
                    let (mp, mr) = pose_at(from, from_rot, to, to_rot, mid);
                    match blocked_at(shape, layers, mask, &mp, &mr, bodies) {
                        Some((j, pp, nn)) if ignore != Some(j) => {
                            hit = ShapeCastHit {
                                fraction: mid,
                                body_index: j,
                                point: pp,
                                normal: nn,
                            };
                            hi = mid;
                        }
                        _ => {
                            lo = mid;
                            lo_pos = mp;
                            lo_rot = mr;
                        }
                    }
                }
                let _ = (lo_pos, lo_rot);
                return Some(hit);
            }
            _ => {
                prev_a = a;
                prev_pos = pa;
                prev_rot = ra;
            }
        }
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn shape_cast_hits_obstacle_at_expected_fraction() {
        let shape = Shape::Sphere { r: 0.5 };
        let from = Vec3::new(0.0, 0.0, -5.0);
        let to = Vec3::new(0.0, 0.0, 5.0);
        let rot = na::UnitQuaternion::identity();
        let obstacle = Body::new(Shape::Sphere { r: 1.0 }, Vec3::zeros(), 0.0);
        let bodies = vec![obstacle];
        let hit = shape_cast::<f64>(&shape, &from, &rot, &to, &rot, &bodies, u32::MAX, u32::MAX, None, 24);
        assert!(hit.is_some(), "应命中障碍球");
        let h = hit.unwrap();
        // 期望命中比例 ≈ (−1.5 + 5)/10 = 0.35。
        assert!((h.fraction - 0.35).abs() < 1e-3, "命中比例应≈0.35,实际 {}", h.fraction);
        assert_eq!(h.body_index, 0);
        // 法线指向被命中体(障碍球)表面、朝向查询形状来向一侧:+z(查询从 −z 接近)。
        assert!(h.normal.z > 0.9, "法线应朝 +z(指向来向),实际 {:?}", h.normal);
    }

    #[test]
    fn shape_cast_misses_when_path_clear() {
        let shape = Shape::Sphere { r: 0.5 };
        let from = Vec3::new(0.0, 5.0, -5.0);
        let to = Vec3::new(0.0, 5.0, 5.0);
        let rot = na::UnitQuaternion::identity();
        // 障碍球在原点,路径在 y=5 高处掠过 → 不接触。
        let obstacle = Body::new(Shape::Sphere { r: 1.0 }, Vec3::zeros(), 0.0);
        let bodies = vec![obstacle];
        let hit = shape_cast::<f64>(&shape, &from, &rot, &to, &rot, &bodies, u32::MAX, u32::MAX, None, 24);
        assert!(hit.is_none(), "高空掠过应不命中");
    }

    #[test]
    fn shape_cast_ignores_self() {
        let shape = Shape::Sphere { r: 0.5 };
        let from = Vec3::new(0.0, 0.0, -5.0);
        let to = Vec3::new(0.0, 0.0, 5.0);
        let rot = na::UnitQuaternion::identity();
        let obstacle = Body::new(Shape::Sphere { r: 1.0 }, Vec3::zeros(), 0.0);
        let bodies = vec![obstacle];
        // 把障碍当作自身忽略 → 应视为畅通。
        let hit = shape_cast::<f64>(&shape, &from, &rot, &to, &rot, &bodies, u32::MAX, u32::MAX, Some(0), 24);
        assert!(hit.is_none(), "忽略障碍后路径应畅通");
    }
}
