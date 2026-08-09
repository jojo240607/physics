//! 射线查询(M20 / 路线图 #9 车辆的前置能力)。
//!
//! 提供 `ray_cast(ray_origin, ray_dir, body)` —— 从 `ray_origin` 沿单位方向
//! `ray_dir` 投射射线,与单个 `Body` 的形状求最近相交。返回命中的参数距离
//! `t`(沿射线)、交点世界坐标、命中表面世界法线;未命中返回 `None`。
//!
//! 支持形状:
//! - `Sphere`:闭式二次方程。
//! - `Box`:把射线变换到盒的局部空间做 slab 求交(AABB),法线再转回世界。
//! - `Convex`:用 GJK 风格的"射线 vs 支撑"迭代(保守前进),命中后用最近面法线
//!   近似(本引擎车辆主要打地面 = Box/Sphere,Convex 走保守命中即可)。
//!
//! 车辆子系统据此从车轮向下打射线找地面,计算悬挂压缩量与接触法线。

use phy_math::{RealField, Vec3};
use serde::de::DeserializeOwned;
use serde::{Deserialize, Serialize};

use crate::shape::Body;
use crate::shape::Shape;

/// 单次射线命中结果。
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
#[serde(bound = "T: RealField + Copy + Serialize + DeserializeOwned")]
pub struct RayHit<T: RealField + Copy> {
    /// 沿射线的参数距离(命中点到原点的距离,因为 ray_dir 已单位化)。
    pub t: T,
    /// 命中点的世界坐标。
    #[serde(with = "crate::shape::serde_geom")]
    pub point: Vec3<T>,
    /// 命中表面的世界法线(单位向量,指向射线来向一侧)。
    #[serde(with = "crate::shape::serde_geom")]
    pub normal: Vec3<T>,
}

/// 从 `origin` 沿单位方向 `dir` 投射射线,与 `body` 求最近相交。
///
/// `dir` 必须为单位向量(调用方负责归一化);返回最近命中(`t ≥ 0`)。
pub fn ray_cast<T: RealField + Copy>(
    origin: &Vec3<T>,
    dir: &Vec3<T>,
    body: &Body<T>,
) -> Option<RayHit<T>> {
    match &body.shape {
        Shape::Sphere { r } => ray_sphere(origin, dir, body.pos, *r).map(|(t, n)| RayHit {
            t,
            point: *origin + *dir * t,
            normal: n,
        }),
        Shape::Box { half } => ray_box(origin, dir, body, *half),
        Shape::Convex { vertices, faces } => {
            // 保守近似:先把射线变换到局部空间,对每个三角形求交取最近。
            ray_convex(origin, dir, body, vertices, faces)
        }
    }
}

/// 射线 vs 球(中心 `c`,半径 `r`)。返回 `(t, 命中法线)`,未命中 None。
fn ray_sphere<T: RealField + Copy>(
    origin: &Vec3<T>,
    dir: &Vec3<T>,
    c: Vec3<T>,
    r: T,
) -> Option<(T, Vec3<T>)> {
    let oc = *origin - c;
    let b = oc.dot(dir);
    let cc = oc.dot(&oc) - r * r;
    // 判别式。
    let disc = b * b - cc;
    if disc < T::zero() {
        return None;
    }
    let sq = disc.sqrt();
    let t = if (-b - sq) >= T::zero() {
        -b - sq
    } else if (-b + sq) >= T::zero() {
        -b + sq
    } else {
        return None; // 原点在球外且射线背离。
    };
    let point = *origin + *dir * t;
    let n = (point - c).normalize();
    Some((t, n))
}

/// 射线 vs 有向盒:把射线变换到盒局部空间做 slab 求交,法线转回世界。
fn ray_box<T: RealField + Copy>(
    origin: &Vec3<T>,
    dir: &Vec3<T>,
    body: &Body<T>,
    half: Vec3<T>,
) -> Option<RayHit<T>> {
    let inv_rot = body.rot.inverse();
    let o_local = inv_rot * (*origin - body.pos);
    let d_local = inv_rot * *dir;

    let mut tmin = T::from_f64(-1e30).unwrap();
    let mut tmax = T::from_f64(1e30).unwrap();
    let mut axis = 0usize; // 记录进入轴,用于法线方向。
    let mut entry_sign = T::one(); // 进入面法线沿 axis 的符号(朝向射线来向)。

    for (i, (o, d, h)) in [(o_local.x, d_local.x, half.x), (o_local.y, d_local.y, half.y), (o_local.z, d_local.z, half.z)].iter().enumerate() {
        if d.abs() < T::from_f64(1e-12).unwrap() {
            // 射线平行于该轴:若原点在板外则无交。
            if *o < -*h || *o > *h {
                return None;
            }
        } else {
            let inv_d = T::one() / *d;
            let mut t1 = (-*h - *o) * inv_d;
            let mut t2 = (*h - *o) * inv_d;
            if t1 > t2 {
                std::mem::swap(&mut t1, &mut t2);
            }
            if t1 > tmin {
                tmin = t1;
                axis = i;
                // 记录进入轴,法线方向由局部 d 符号决定(进入面朝向射线来向)。
                entry_sign = if *d > T::zero() { -T::one() } else { T::one() };
            }
            if t2 < tmax {
                tmax = t2;
            }
            if tmin > tmax {
                return None;
            }
        }
    }

    if tmax < T::zero() {
        return None; // 盒在射线后方。
    }
    let t = if tmin >= T::zero() { tmin } else { tmax };
    if t < T::zero() {
        return None;
    }
    let point = *origin + *dir * t;
    // 局部法线沿进入轴,方向朝向射线来向(进入面外法线)。
    let mut n_local = Vec3::zeros();
    match axis {
        0 => n_local.x = entry_sign,
        1 => n_local.y = entry_sign,
        _ => n_local.z = entry_sign,
    }
    let n_world = body.rot * n_local;
    Some(RayHit { t, point, normal: n_world })
}

/// 射线 vs 凸多面体:遍历每个三角形(局部空间)求最近命中。
fn ray_convex<T: RealField + Copy>(
    origin: &Vec3<T>,
    dir: &Vec3<T>,
    body: &Body<T>,
    vertices: &[Vec3<T>],
    faces: &[[usize; 3]],
) -> Option<RayHit<T>> {
    let inv_rot = body.rot.inverse();
    let o_local = inv_rot * (*origin - body.pos);
    let d_local = inv_rot * *dir;
    let mut best: Option<(T, Vec3<T>)> = None;
    for f in faces {
        let a = vertices[f[0]];
        let b = vertices[f[1]];
        let c = vertices[f[2]];
        if let Some((t, n_local)) = ray_triangle(&o_local, &d_local, a, b, c) {
            if t >= T::zero() {
                if best.is_none() || t < best.unwrap().0 {
                    best = Some((t, n_local));
                }
            }
        }
    }
    best.map(|(t, n_local)| {
        let n_world = body.rot * n_local;
        RayHit {
            t,
            point: *origin + *dir * t,
            normal: n_world,
        }
    })
}

/// Möller–Trumbore 射线-三角形相交,返回 `(t, 命中法线)`。
fn ray_triangle<T: RealField + Copy>(
    origin: &Vec3<T>,
    dir: &Vec3<T>,
    a: Vec3<T>,
    b: Vec3<T>,
    c: Vec3<T>,
) -> Option<(T, Vec3<T>)> {
    let eps = T::from_f64(1e-9).unwrap();
    let edge1 = b - a;
    let edge2 = c - a;
    let pvec = dir.cross(&edge2);
    let det = edge1.dot(&pvec);
    if det.abs() < eps {
        return None;
    }
    let inv_det = T::one() / det;
    let tvec = *origin - a;
    let u = tvec.dot(&pvec) * inv_det;
    if u < T::zero() || u > T::one() {
        return None;
    }
    let qvec = tvec.cross(&edge1);
    let v = dir.dot(&qvec) * inv_det;
    if v < T::zero() || u + v > T::one() {
        return None;
    }
    let t = edge2.dot(&qvec) * inv_det;
    if t < T::zero() {
        return None;
    }
    let n = edge1.cross(&edge2).normalize();
    Some((t, n))
}

#[cfg(test)]
mod tests {
    use super::*;
    use phy_math::{na, Vec3};

    fn sphere_body(pos: Vec3<f64>, r: f64) -> Body<f64> {
        Body {
            shape: Shape::Sphere { r },
            pos,
            rot: na::one(),
            vel: Vec3::zeros(),
            inv_mass: 0.0,
        }
    }

    #[test]
    fn ray_hits_sphere_from_outside() {
        let b = sphere_body(Vec3::new(0.0, 0.0, 0.0), 1.0);
        let hit = ray_cast(&Vec3::new(0.0, 0.0, 5.0), &Vec3::new(0.0, 0.0, -1.0), &b);
        assert!(hit.is_some());
        let h = hit.unwrap();
        assert!((h.t - 4.0).abs() < 1e-9, "t 应为 4,得 {}", h.t);
        assert!(h.normal.z > 0.99, "法线应指向 +Z(朝向射线来向)");
    }

    #[test]
    fn ray_misses_sphere() {
        let b = sphere_body(Vec3::new(0.0, 0.0, 0.0), 1.0);
        let hit = ray_cast(&Vec3::new(5.0, 0.0, 5.0), &Vec3::new(0.0, 0.0, -1.0), &b);
        assert!(hit.is_none());
    }

    #[test]
    fn ray_hits_box_top_face() {
        let mut b = sphere_body(Vec3::new(0.0, 0.0, 0.0), 1.0);
        b.shape = Shape::Box {
            half: Vec3::new(1.0, 0.5, 1.0),
        };
        // 从上方打向盒顶 (y = +0.5)。
        let hit = ray_cast(&Vec3::new(0.0, 5.0, 0.0), &Vec3::new(0.0, -1.0, 0.0), &b);
        assert!(hit.is_some());
        let h = hit.unwrap();
        assert!((h.t - 4.5).abs() < 1e-9, "t 应为 4.5,得 {}", h.t);
        assert!(h.normal.y > 0.99, "法线应朝上 +Y");
        assert!((h.point.y - 0.5).abs() < 1e-9);
    }

    #[test]
    fn ray_misses_box_to_side() {
        let mut b = sphere_body(Vec3::new(0.0, 0.0, 0.0), 1.0);
        b.shape = Shape::Box {
            half: Vec3::new(1.0, 0.5, 1.0),
        };
        let hit = ray_cast(&Vec3::new(5.0, 5.0, 0.0), &Vec3::new(0.0, -1.0, 0.0), &b);
        assert!(hit.is_none());
    }
}
