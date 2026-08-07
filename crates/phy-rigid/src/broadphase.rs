//! Broad-phase: 扫描线(Sweep and Prune)粗筛。
//!
//! 为每个 Body 计算世界空间包围球,沿一个主轴上做区间重叠排序,
//! 产出潜在相交的 Body 对,交给 Narrow-phase 精算。

use phy_math::{RealField, Vec3};

use crate::shape::Body;

/// 包围球(世界中心 + 半径)。
struct Bounds<T: RealField + Copy> {
    center: Vec3<T>,
    r: T,
}

/// 对所有 body 做 SAP,返回需要 narrow-phase 的 (i, j) 索引对( i < j )。
pub fn broadphase<T: RealField + Copy>(bodies: &[Body<T>]) -> Vec<(usize, usize)> {
    let bounds: Vec<Bounds<T>> = bodies
        .iter()
        .map(|b| {
            let r = b.shape.bounding_sphere_r();
            Bounds { center: b.pos, r }
        })
        .collect();

    // 沿 X 轴做扫描线
    let mut order: Vec<usize> = (0..bounds.len()).collect();
    order.sort_by(|&i, &j| {
        let a = bounds[i].center.x - bounds[i].r;
        let b = bounds[j].center.x - bounds[j].r;
        a.partial_cmp(&b).unwrap()
    });

    let mut pairs = Vec::new();
    for i in 0..order.len() {
        let bi = order[i];
        let max_i = bounds[bi].center.x + bounds[bi].r;
        for j in (i + 1)..order.len() {
            let bj = order[j];
            let min_j = bounds[bj].center.x - bounds[bj].r;
            // 后续区间起点已超过 bi 的右端 => 后面都不可能重叠(已排序)
            if min_j > max_i {
                break;
            }
            // 完整球-球重叠测试(包围球阶段足够粗略)
            let d = bounds[bj].center - bounds[bi].center;
            let rr = bounds[bi].r + bounds[bj].r;
            if d.norm_squared() <= rr * rr {
                let (lo, hi) = if bi < bj { (bi, bj) } else { (bj, bi) };
                pairs.push((lo, hi));
            }
        }
    }
    pairs
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::shape::{Body, Shape};
    use phy_math::na;

    fn unit_sphere(pos: Vec3<f64>) -> Body<f64> {
        Body {
            shape: Shape::Sphere { r: 1.0 },
            pos,
            rot: na::UnitQuaternion::identity(),
            vel: Vec3::zeros(),
            inv_mass: 1.0,
        }
    }

    #[test]
    fn sap_finds_overlapping_pair() {
        let bodies = vec![
            unit_sphere(Vec3::new(0.0, 0.0, 0.0)),
            unit_sphere(Vec3::new(1.0, 0.0, 0.0)), // 相交
            unit_sphere(Vec3::new(10.0, 0.0, 0.0)), // 远离
        ];
        let pairs = broadphase(&bodies);
        assert_eq!(pairs.len(), 1);
        assert_eq!(pairs[0], (0, 1));
    }

    #[test]
    fn sap_no_pairs_when_far() {
        let bodies = vec![
            unit_sphere(Vec3::new(0.0, 0.0, 0.0)),
            unit_sphere(Vec3::new(100.0, 0.0, 0.0)),
        ];
        assert!(broadphase(&bodies).is_empty());
    }
}
