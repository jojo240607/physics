//! 光学场景数据模型 + 射线-形状求交。
//!
//! 复用 `phy_rigid::Shape` 作为几何,叠加光学属性(表面材质 / 介质折射率),
//! 使刚体世界与光学世界共享同一套几何体描述。

use num_traits::FromPrimitive;
use phy_math::{na, RealField, Vec3};
use phy_rigid::{Body, Shape};

/// 表面材质:漫反射色 + 折射率(IOR) + 粗糙度(仅影响近似后端)。
#[derive(Debug, Clone)]
pub struct Surface<T: RealField + Copy> {
    /// 漫反射/base 颜色(RGB,0..1)。
    pub albedo: Vec3<T>,
    /// 折射率(真空为 1.0;玻璃≈1.5;水≈1.33)。
    pub ior: T,
    /// 表面粗糙度(0=镜面,1=完全漫)。实时近似后端用于随机微扰法线。
    pub roughness: T,
    /// 是否透明(参与折射)。不透明体只反射/漫反射。
    pub transparent: bool,
}

impl<T: RealField + Copy> Surface<T> {
    /// 不透明漫反射体(如地面、墙)。
    pub fn diffuse(color: Vec3<T>) -> Self {
        Self {
            albedo: color,
            ior: T::one(),
            roughness: T::one(),
            transparent: false,
        }
    }
    /// 透明折射体(如玻璃球、水)。
    pub fn glass(ior: T, tint: Vec3<T>) -> Self {
        Self {
            albedo: tint,
            ior,
            roughness: T::zero(),
            transparent: true,
        }
    }
}

/// 光学体:刚体位姿 + 形状 + 表面材质。
#[derive(Debug, Clone)]
pub struct OpticBody<T: RealField + Copy> {
    /// 底层刚体位姿(含 shape)。
    pub body: Body<T>,
    /// 表面材质。
    pub surface: Surface<T>,
}

impl<T: RealField + Copy> OpticBody<T> {
    /// 由刚体 + 表面材质构造。
    pub fn new(body: Body<T>, surface: Surface<T>) -> Self {
        Self { body, surface }
    }

    /// 把世界射线变换到局部空间求交,返回(局部 t, 局部法线)或 None。
    /// 支持 Sphere / Box / Convex 三种形状。
    pub fn intersect_local(
        &self,
        ro_world: &Vec3<T>,
        rd_world: &Vec3<T>,
    ) -> Option<(T, Vec3<T>)> {
        let inv_rot = self.body.rot.inverse();
        // 世界 -> 局部
        let ro = inv_rot * (*ro_world - self.body.pos);
        let rd = inv_rot * *rd_world;
        let rd_len = rd.norm();
        if rd_len <= T::zero() {
            return None;
        }
        let rd_n = rd / rd_len; // 局部单位方向
        let (t_local, n_local) = match &self.body.shape {
            Shape::Sphere { r } => sphere_hit(&ro, &rd_n, *r),
            Shape::Box { half } => box_hit(&ro, &rd_n, half),
            Shape::Convex { vertices, faces } => convex_hit(&ro, &rd_n, vertices, faces),
        }?;
        // 局部 t 需按世界方向长度还原(因为 rd 被旋转但长度不变,rd_len 即比例)。
        let t_world = t_local * rd_len;
        // 局部法线 -> 世界法线(仅旋转,无平移/缩放)。
        let n_world = self.body.rot * n_local;
        Some((t_world, normalize_lossy(&n_world)))
    }
}

/// 射线-球求交,返回 (t, 法线) 局部空间;无交返回 None。
/// `rd` 为单位方向。
fn sphere_hit<T: RealField + Copy>(ro: &Vec3<T>, rd: &Vec3<T>, r: T) -> Option<(T, Vec3<T>)> {
    // |ro + t rd|² = r²
    let b = ro.dot(rd);
    let c = ro.dot(ro) - r * r;
    let disc = b * b - c;
    if disc < T::zero() {
        return None;
    }
    let sq = disc.sqrt();
    let two = <T as FromPrimitive>::from_f64(2.0).unwrap();
    let t0 = -b - sq;
    let t = if t0 > T::zero() { t0 } else { -b + sq };
    if t <= T::zero() {
        return None;
    }
    let p = *ro + *rd * t;
    let n = p / r; // 球面法线
    Some((t, n))
}

/// 射线-轴对齐盒(Slab 法),`rd` 单位方向,`half` 为半长。
fn box_hit<T: RealField + Copy>(
    ro: &Vec3<T>,
    rd: &Vec3<T>,
    half: &Vec3<T>,
) -> Option<(T, Vec3<T>)> {
    let mut tmin = T::zero();
    let mut tmax = T::from_f64(1e30).unwrap();
    let mut n = Vec3::zeros();
    for axis in 0..3 {
        let o = ro[axis];
        let d = rd[axis];
        let h = half[axis];
        let (inv_d, mut sign) = if d.abs() > T::from_f64(1e-12).unwrap() {
            (T::one() / d, T::one())
        } else {
            // 平行轴:若起点在板外则无交。
            if o < -h || o > h {
                return None;
            }
            (T::zero(), T::one())
        };
        let mut t1 = (-h - o) * inv_d;
        let mut t2 = (h - o) * inv_d;
        let mut n_axis = Vec3::zeros();
        n_axis[axis] = -T::one();
        if t1 > t2 {
            std::mem::swap(&mut t1, &mut t2);
            n_axis[axis] = T::one();
            sign = -T::one();
        }
        if t1 > tmin {
            tmin = t1;
            n = n_axis * sign;
        }
        if t2 < tmax {
            tmax = t2;
        }
        if tmin > tmax {
            return None;
        }
    }
    if tmin <= T::zero() {
        return None;
    }
    Some((tmin, normalize_lossy(&n)))
}

/// 射线-凸多面体:遍历面求最近进入(简化:对每个三角形做 Möller–Trumbore,
/// 取最小正 t,法线取该三角形法线)。适合低面数凸体。
fn convex_hit<T: RealField + Copy>(
    ro: &Vec3<T>,
    rd: &Vec3<T>,
    vertices: &[Vec3<T>],
    faces: &[[usize; 3]],
) -> Option<(T, Vec3<T>)> {
    let mut best: Option<(T, Vec3<T>)> = None;
    for f in faces {
        if let Some((t, n)) = tri_hit(ro, rd, &vertices[f[0]], &vertices[f[1]], &vertices[f[2]])
        {
            if t > T::zero() {
                match best {
                    Some((bt, _)) if t >= bt => {}
                    _ => best = Some((t, n)),
                }
            }
        }
    }
    best
}

/// Möller–Trumbore 射线-三角形求交,返回 (t, 几何法线),`rd` 单位方向。
fn tri_hit<T: RealField + Copy>(
    ro: &Vec3<T>,
    rd: &Vec3<T>,
    a: &Vec3<T>,
    b: &Vec3<T>,
    c: &Vec3<T>,
) -> Option<(T, Vec3<T>)> {
    let eps = T::from_f64(1e-9).unwrap();
    let edge1 = *b - *a;
    let edge2 = *c - *a;
    let pvec = rd.cross(&edge2);
    let det = edge1.dot(&pvec);
    if det.abs() < eps {
        return None;
    }
    let inv_det = T::one() / det;
    let tvec = *ro - *a;
    let u = tvec.dot(&pvec) * inv_det;
    if u < T::zero() || u > T::one() {
        return None;
    }
    let qvec = tvec.cross(&edge1);
    let v = rd.dot(&qvec) * inv_det;
    if v < T::zero() || u + v > T::one() {
        return None;
    }
    let t = edge2.dot(&qvec) * inv_det;
    if t <= T::zero() {
        return None;
    }
    let n = edge1.cross(&edge2);
    Some((t, normalize_lossy(&n)))
}

/// 防御性归一化(零向量返回 +X)。
fn normalize_lossy<T: RealField + Copy>(v: &Vec3<T>) -> Vec3<T> {
    let n = v.norm();
    if n > T::from_f64(1e-12).unwrap() {
        *v / n
    } else {
        Vec3::new(T::one(), T::zero(), T::zero())
    }
}

/// 光学场景:背景颜色 + 环境折射率(默认真空) + 一组光学体。
#[derive(Debug, Clone)]
pub struct OpticScene<T: RealField + Copy> {
    /// 背景(无交时的颜色)。
    pub background: Vec3<T>,
    /// 环境介质折射率(默认 1.0)。
    pub env_ior: T,
    /// 场景中的光学体。
    pub bodies: Vec<OpticBody<T>>,
    /// 最大递归深度(离线后端使用)。
    pub max_depth: usize,
}

impl<T: RealField + Copy> Default for OpticScene<T> {
    fn default() -> Self {
        Self {
            background: Vec3::new(
                T::from_f64(0.05).unwrap(),
                T::from_f64(0.07).unwrap(),
                T::from_f64(0.1).unwrap(),
            ),
            env_ior: T::one(),
            bodies: Vec::new(),
            max_depth: 5,
        }
    }
}

impl<T: RealField + Copy> OpticScene<T> {
    /// 空场景。
    pub fn new() -> Self {
        Self::default()
    }

    /// 添加光学体。
    pub fn add(&mut self, b: OpticBody<T>) -> usize {
        self.bodies.push(b);
        self.bodies.len() - 1
    }

    /// 在场景中求最近交点:返回 (t, 世界法线, 命中体索引)。
    /// `ior_from`:当前射线所在介质折射率(用于判断进入/离开)。
    pub fn intersect(
        &self,
        ro: &Vec3<T>,
        rd: &Vec3<T>,
    ) -> Option<(T, Vec3<T>, usize)> {
        let mut best: Option<(T, Vec3<T>, usize)> = None;
        for (idx, body) in self.bodies.iter().enumerate() {
            if let Some((t, n)) = body.intersect_local(ro, rd) {
                match best {
                    Some((bt, _, _)) if t >= bt => {}
                    _ => best = Some((t, n, idx)),
                }
            }
        }
        best
    }
}
