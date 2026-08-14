//! Narrow-phase 精确碰撞:球/盒快速路径 + 通用 GJK(相交)/ EPA(接触)。
//!
//! - 球-球:闭式。
//! - 盒-盒: SAT(分离轴定理)。
//! - 通用凸体(含球/盒/凸多面体):GJK 判相交,EPA 求接触法线+穿透深度。

use num_traits::NumCast;
use phy_math::{na, RealField, Vec3};

use crate::contact::Contact;
use crate::shape::{Body, Shape, SubShape};

/// 泛型标量转 f64(用于索引/边界计算)。
#[allow(dead_code)]
fn to_f64<T: RealField + Copy + NumCast>(x: T) -> f64 {
    x.to_f64().unwrap_or(0.0)
}

/// 球-球快速相交:返回接触(若有)。
pub fn sphere_sphere<T: RealField + Copy>(a: &Body<T>, b: &Body<T>) -> Option<Contact<T>> {
    let (ra, rb) = match (&a.shape, &b.shape) {
        (Shape::Sphere { r: ra }, Shape::Sphere { r: rb }) => (*ra, *rb),
        _ => return None,
    };
    let d = b.pos - a.pos;
    let dist = d.norm();
    let sum = ra + rb;
    if dist >= sum {
        return None;
    }
    // 法线由 a 指向 b;退化为 +X。
    let normal = if dist > T::from_f64(1e-9).unwrap() {
        d / dist
    } else {
        Vec3::new(T::one(), T::zero(), T::zero())
    };
    let point = a.pos + normal * ra;
    Some(Contact::new(point, normal, sum - dist))
}

/// 闵可夫斯基差支持点: a.support(d) - b.support(-d)。
fn support_minkowski<T: RealField + Copy>(a: &Body<T>, b: &Body<T>, d: &Vec3<T>) -> Vec3<T> {
    let sa = a.support(d);
    let sb = b.support(&(-(*d)));
    sa - sb
}

/// GJK 判两凸体是否相交(基于支持映射的单纯形迭代)。
pub fn gjk_intersect<T: RealField + Copy>(a: &Body<T>, b: &Body<T>) -> bool {
    let mut dir = a.pos - b.pos;
    if dir.norm() < T::from_f64(1e-9).unwrap() {
        dir = Vec3::new(T::one(), T::zero(), T::zero());
    } else {
        dir = dir.normalize();
    }
    let mut simplex: Vec<Vec3<T>> = Vec::with_capacity(4);
    let s0 = support_minkowski(a, b, &dir);
    simplex.push(s0);
    dir = -s0;
    for _ in 0..64 {
        let dn = dir.norm();
        if dn < T::from_f64(1e-9).unwrap() {
            return true;
        }
        let p = support_minkowski(a, b, &dir);
        if p.dot(&dir) < T::zero() {
            return false;
        }
        simplex.push(p);
        if gjk_do_simplex(&mut simplex, &mut dir) {
            return true;
        }
    }
    false
}

/// 处理当前单纯形,更新朝原点方向 `dir`。返回 true 表示原点已被包围(相交)。
fn gjk_do_simplex<T: RealField + Copy>(simplex: &mut Vec<Vec3<T>>, dir: &mut Vec3<T>) -> bool {
    let o = Vec3::zeros();
    let n = simplex.len();
    if n == 2 {
        // 线段
        let a = simplex[1];
        let b = simplex[0];
        let ab = b - a;
        let ao = o - a;
        if ab.dot(&ao) > T::zero() {
            *dir = ab.cross(&ao).cross(&ab);
            if dir.norm() < T::from_f64(1e-12).unwrap() {
                *dir = ab.cross(&Vec3::new(T::zero(), T::one(), T::zero()));
                if dir.norm() < T::from_f64(1e-12).unwrap() {
                    *dir = ab.cross(&Vec3::new(T::zero(), T::zero(), T::one()));
                }
            }
        } else {
            *simplex = vec![a];
            *dir = ao;
        }
        false
    } else if n == 3 {
        // 三角形
        let a = simplex[2];
        let b = simplex[1];
        let c = simplex[0];
        let ab = b - a;
        let ac = c - a;
        let ao = o - a;
        let abc = ab.cross(&ac);
        if abc.cross(&ac).dot(&ao) > T::zero() {
            if ac.dot(&ao) > T::zero() {
                *simplex = vec![a, c];
                let tmp = ac.cross(&ao).cross(&ac);
                *dir = if tmp.norm() < T::from_f64(1e-12).unwrap() {
                    ac
                } else {
                    tmp
                };
            } else {
                gjk_line_reduce(simplex, a, b, ao, ab, dir);
            }
        } else if ab.cross(&abc).dot(&ao) > T::zero() {
            gjk_line_reduce(simplex, a, b, ao, ab, dir);
        } else if abc.dot(&ao) > T::zero() {
            *dir = abc;
        } else {
            *simplex = vec![a, c, b];
            *dir = -abc;
        }
        false
    } else {
        // 四面体:检查原点是否在内
        let a = simplex[3];
        let b = simplex[2];
        let c = simplex[1];
        let d = simplex[0];
        let ab = b - a;
        let ac = c - a;
        let ad = d - a;
        let ao = o - a;
        let abc = ab.cross(&ac);
        let acd = ac.cross(&ad);
        let adb = ad.cross(&ab);
        if abc.dot(&ao) > T::zero()
            || acd.dot(&ao) > T::zero()
            || adb.dot(&ao) > T::zero()
        {
            *simplex = vec![a];
            *dir = ao;
            false
        } else {
            true
        }
    }
}

fn gjk_line_reduce<T: RealField + Copy>(
    simplex: &mut Vec<Vec3<T>>,
    a: Vec3<T>,
    b: Vec3<T>,
    ao: Vec3<T>,
    ab: Vec3<T>,
    dir: &mut Vec3<T>,
) {
    if ab.dot(&ao) > T::zero() {
        *simplex = vec![a, b];
        let tmp = ab.cross(&ao).cross(&ab);
        *dir = if tmp.norm() < T::from_f64(1e-12).unwrap() {
            ab
        } else {
            tmp
        };
    } else {
        *simplex = vec![a];
        *dir = ao;
    }
}

/// EPA:在 GJK 已判相交后,从最终单纯形出发求穿透法线与深度。
/// 返回 (法线由 a 指向 b, 深度)。
pub fn epa<T: RealField + Copy>(a: &Body<T>, b: &Body<T>, simplex0: &[Vec3<T>]) -> Option<(Vec3<T>, T)> {
    let mut polytope: Vec<Vec3<T>> = simplex0.to_vec();
    let eps = T::from_f64(1e-4).unwrap();
    for _ in 0..64 {
        let (norm, dist) = closest_face_normal(&polytope)?;
        let sup = support_minkowski(a, b, &norm);
        let d = sup.dot(&norm);
        if (d - dist) < eps {
            return Some((norm, dist));
        }
        polytope.push(sup);
    }
    None
}

/// 求多面体中距原点最近面的外法线与距离(取最近者)。
fn closest_face_normal<T: RealField + Copy>(poly: &[Vec3<T>]) -> Option<(Vec3<T>, T)> {
    let o = Vec3::zeros();
    let mut best: Option<(Vec3<T>, T)> = None;
    let m = poly.len();
    for i in 0..m {
        let j = (i + 1) % m;
        let k = (i + 2) % m;
        let p0 = poly[i];
        let p1 = poly[j];
        let p2 = poly[k];
        let n = (p1 - p0).cross(&(p2 - p0));
        let len = n.norm();
        if len < T::from_f64(1e-12).unwrap() {
            continue;
        }
        let n = n / len;
        let dist = n.dot(&(p0 - o));
        let dist = if dist < T::zero() { -dist } else { dist };
        if best.as_ref().map(|b| dist < b.1).unwrap_or(true) {
            best = Some((n, dist));
        }
    }
    best
}

/// 盒-盒 SAT(分离轴定理)相交,返回接触(若有)。
pub fn box_box_sat<T: RealField + Copy>(a: &Body<T>, b: &Body<T>) -> Option<Contact<T>> {
    let (ha, hb) = match (&a.shape, &b.shape) {
        (Shape::Box { half: ha }, Shape::Box { half: hb }) => (ha, hb),
        _ => return None,
    };
    let ax = a.rot * Vec3::x();
    let ay = a.rot * Vec3::y();
    let az = a.rot * Vec3::z();
    let bx = b.rot * Vec3::x();
    let by = b.rot * Vec3::y();
    let bz = b.rot * Vec3::z();
    let a_axes = [ax, ay, az];
    let b_axes = [bx, by, bz];
    let ra = [ha.x, ha.y, ha.z];
    let rb = [hb.x, hb.y, hb.z];

    let d = b.pos - a.pos;
    let big = T::from_f64(1e30).unwrap();
    let mut min_pen = big;
    let mut min_axis = Vec3::zeros();

    // 分离容差:pen 略小于 0(恰好接触)仍视为相交,避免 touching 被误判分离
    let sep_eps = T::from_f64(-1e-6).unwrap();
    for i in 0..3 {
        let axis = a_axes[i];
        let pen = sat_penetration(axis, &d, &a_axes, &b_axes, ra, rb);
        if pen < sep_eps {
            return None;
        }
        if pen < min_pen {
            min_pen = pen;
            min_axis = axis;
        }
    }
    for i in 0..3 {
        let axis = b_axes[i];
        let pen = sat_penetration(axis, &d, &a_axes, &b_axes, ra, rb);
        if pen < sep_eps {
            return None;
        }
        if pen < min_pen {
            min_pen = pen;
            min_axis = axis;
        }
    }
    for i in 0..3 {
        for j in 0..3 {
            let axis = a_axes[i].cross(&b_axes[j]);
            if axis.norm() < T::from_f64(1e-9).unwrap() {
                continue;
            }
            let axis = axis.normalize();
            let pen = sat_penetration(axis, &d, &a_axes, &b_axes, ra, rb);
            if pen < sep_eps {
                return None;
            }
            if pen < min_pen {
                min_pen = pen;
                min_axis = axis;
            }
        }
    }

    // 法线由 a 指向 b
    let normal = if min_axis.dot(&d) < T::zero() {
        -min_axis
    } else {
        min_axis.clone()
    };
    let point = a.pos + normal.clone() * (min_pen / T::from_f64(2.0).unwrap());
    Some(Contact::new(point, normal, min_pen))
}

/// 计算沿 `axis` 的 SAT 穿透量(>0 相交, <0 分离)。
fn sat_penetration<T: RealField + Copy>(
    axis: Vec3<T>,
    d: &Vec3<T>,
    a_axes: &[Vec3<T>; 3],
    b_axes: &[Vec3<T>; 3],
    ra: [T; 3],
    rb: [T; 3],
) -> T {
    let dist = d.dot(&axis).abs();
    let r = ra[0] * a_axes[0].dot(&axis).abs()
        + ra[1] * a_axes[1].dot(&axis).abs()
        + ra[2] * a_axes[2].dot(&axis).abs()
        + rb[0] * b_axes[0].dot(&axis).abs()
        + rb[1] * b_axes[1].dot(&axis).abs()
        + rb[2] * b_axes[2].dot(&axis).abs();
    r - dist
}

/// 球-盒快速相交(解析法):将球心变换到盒的局部坐标系,夹取到盒半空间得到最近点,
/// 再按"球心在盒外/内"两种情况求接触法线与穿透深度。比 GJK/EPA 更快且对球-盒更稳健。
/// `s` 为球,`b` 为盒;返回法线由 `s` 指向 `b`。
pub fn sphere_box<T: RealField + Copy>(s: &Body<T>, b: &Body<T>) -> Option<Contact<T>> {
    let (r, half) = match (&s.shape, &b.shape) {
        (Shape::Sphere { r }, Shape::Box { half }) => (*r, *half),
        _ => return None,
    };
    // 球心在盒局部坐标中的位置。
    let local = b.rot.inverse() * (s.pos - b.pos);
    let mut closest = local;
    let mut inside = true;
    for k in 0..3 {
        if closest[k] > half[k] {
            closest[k] = half[k];
            inside = false;
        } else if closest[k] < -half[k] {
            closest[k] = -half[k];
            inside = false;
        }
    }
    if inside {
        // 球心在盒内部:沿穿透最浅的面推出。法线由盒指向球(世界系)。
        let mut axis = 0usize;
        let mut best = half[0] - local[0].abs();
        for k in 1..3 {
            let pen = half[k] - local[k].abs();
            if pen < best {
                best = pen;
                axis = k;
            }
        }
        let sign = if local[axis] >= T::zero() {
            T::one()
        } else {
            -T::one()
        };
        let mut n_local = Vec3::zeros();
        n_local[axis] = sign; // 盒→球(局部)
        let normal = b.rot * n_local; // 世界系:盒→球
        // 约定法线由 s 指向 b,即取反。
        let normal = -normal;
        let depth = r + best;
        let point = s.pos + normal * r; // 球面上接触点
        Some(Contact::new(point, normal, depth))
    } else {
        let delta = local - closest; // 盒表面最近点 → 球心(局部)
        let dist2 = delta.norm_squared();
        if dist2 > r * r {
            return None;
        }
        let dist = dist2.sqrt();
        let n_local = if dist > T::from_f64(1e-9).unwrap() {
            delta / dist
        } else {
            // 退化:球心恰在盒表面最近点,沿局部 +Z 兜底。
            Vec3::new(T::zero(), T::zero(), T::one())
        };
        // n_local 指向 盒→球;约定法线由 s 指向 b,取反。
        let normal = -(b.rot * n_local);
        let depth = r - dist;
        let point = s.pos + normal * r; // 球面接触点
        Some(Contact::new(point, normal, depth))
    }
}

/// 胶囊世界系线段端点(half 沿局部 y 轴)。
fn capsule_segment_world<T: RealField + Copy>(c: &Body<T>) -> (Vec3<T>, Vec3<T>, T) {
    let (half_height, r) = match &c.shape {
        Shape::Capsule { half_height, r } => (*half_height, *r),
        _ => return (Vec3::zeros(), Vec3::zeros(), T::zero()),
    };
    let d = c.rot * Vec3::new(T::zero(), half_height, T::zero());
    (c.pos - d, c.pos + d, r)
}

/// 点到线段(世界系)最近点与最近距离平方。
fn closest_on_segment<T: RealField + Copy>(
    p: &Vec3<T>,
    seg_a: &Vec3<T>,
    seg_b: &Vec3<T>,
) -> (Vec3<T>, T) {
    let ab = *seg_b - *seg_a;
    let ab2 = ab.norm_squared();
    let t = if ab2 > T::from_f64(1e-12).unwrap() {
        ((*p - *seg_a).dot(&ab) / ab2).clamp(T::zero(), T::one())
    } else {
        T::zero()
    };
    let q = *seg_a + ab * t;
    let d2 = (*p - q).norm_squared();
    (q, d2)
}

/// 胶囊-球快速相交:胶囊线段 + 球,点到线段最近距离 ≤ ra+rb。
/// 返回法线由胶囊指向球。
pub fn capsule_sphere<T: RealField + Copy>(cap: &Body<T>, s: &Body<T>) -> Option<Contact<T>> {
    let (ra, _rb) = match (&cap.shape, &s.shape) {
        (Shape::Capsule { r, .. }, Shape::Sphere { r: rb }) => (*r, *rb),
        _ => return None,
    };
    let (ea, eb, r) = capsule_segment_world(cap);
    let (q, d2) = closest_on_segment(&s.pos, &ea, &eb);
    let sum = r + ra; // 胶囊半径 + 球半径
    if d2 > sum * sum {
        return None;
    }
    let d = s.pos - q;
    let dist = d2.sqrt();
    let n = if dist > T::from_f64(1e-9).unwrap() {
        d / dist
    } else {
        Vec3::new(T::zero(), T::one(), T::zero())
    };
    let depth = sum - dist;
    // 接触点在球面上。
    let point = s.pos - n * ra;
    Some(Contact::new(point, n, depth))
}

/// 胶囊-胶囊快速相交:两线段最近距离 ≤ ra+rb。
/// 返回法线由 a 指向 b。
pub fn capsule_capsule<T: RealField + Copy>(a: &Body<T>, b: &Body<T>) -> Option<Contact<T>> {
    let (ra, rb) = match (&a.shape, &b.shape) {
        (Shape::Capsule { r: ra, .. }, Shape::Capsule { r: rb, .. }) => (*ra, *rb),
        _ => return None,
    };
    let (a1, a2, _) = capsule_segment_world(a);
    let (b1, b2, _) = capsule_segment_world(b);
    // 两线段最近距离(标准公式)。
    let (p, q, d2) = closest_between_segments(&a1, &a2, &b1, &b2);
    let sum = ra + rb;
    if d2 > sum * sum {
        return None;
    }
    let dist = d2.sqrt();
    // 法线约定:由 a 指向 b。p 在 a 线段、q 在 b 线段 → (q-p) 由 a→b。
    let n = if dist > T::from_f64(1e-9).unwrap() {
        (q - p) / dist
    } else {
        // 平行/重叠:沿 b→a 质心方向兜底。
        (b.pos - a.pos).normalize()
    };
    let depth = sum - dist;
    let point = p + n * ra; // a 表面接触点(朝向 b 侧)
    Some(Contact::new(point, n, depth))
}

/// 两线段(世界系)之间最近的一对点 `(p, q)` 与距离平方。
fn closest_between_segments<T: RealField + Copy>(
    a1: &Vec3<T>,
    a2: &Vec3<T>,
    b1: &Vec3<T>,
    b2: &Vec3<T>,
) -> (Vec3<T>, Vec3<T>, T) {
    let u = *a2 - *a1;
    let v = *b2 - *b1;
    let w = *a1 - *b1;
    let a = u.norm_squared();
    let b = u.dot(&v);
    let c = v.norm_squared();
    let d = u.dot(&w);
    let e = v.dot(&w);
    let det = a * c - b * b;
    let (s, t) = if det > T::from_f64(1e-12).unwrap() {
        let s = (b * e - c * d) / det;
        let t = (a * e - b * d) / det;
        let s = s.clamp(T::zero(), T::one());
        let t = t.clamp(T::zero(), T::one());
        (s, t)
    } else {
        (T::zero(), T::zero())
    };
    let p = *a1 + u * s;
    let q = *b1 + v * t;
    let d2 = (p - q).norm_squared();
    (p, q, d2)
}

/// 胶囊-盒快速相交:把胶囊线段端点的盒内最近点夹取,求胶囊线段到盒表面的最近距离。
/// 返回法线由胶囊指向盒。
pub fn capsule_box<T: RealField + Copy>(cap: &Body<T>, b: &Body<T>) -> Option<Contact<T>> {
    let (rc, half) = match (&cap.shape, &b.shape) {
        (Shape::Capsule { r, .. }, Shape::Box { half }) => (*r, *half),
        _ => return None,
    };
    let (ea, eb, _) = capsule_segment_world(cap);
    // 把胶囊两端的盒内最近点夹取。
    let closest_box = |p: Vec3<T>| -> (Vec3<T>, bool) {
        let l = b.rot.inverse() * (p - b.pos);
        let mut cl = l;
        let mut inside = true;
        for k in 0..3 {
            if cl[k] > half[k] {
                cl[k] = half[k];
                inside = false;
            } else if cl[k] < -half[k] {
                cl[k] = -half[k];
                inside = false;
            }
        }
        (b.pos + b.rot * cl, inside)
    };
    let (q_a, in_a) = closest_box(ea);
    let (q_b, in_b) = closest_box(eb);
    // 胶囊线段到盒表面最近距离 ≈ 两端点到盒最近点的最小距离。
    // 处理线段在盒内(任一端在盒内)的情形。
    if in_a || in_b {
        // 胶囊在盒内:沿穿透最浅的面推出。
        // 简化:找线段端点在盒内最浅的穿透面。
        let l = b.rot.inverse() * (cap.pos - b.pos);
        // 到最近面的距离(取绝对值,中心可在盒内或略外)。
        let mut axis = 0usize;
        let mut best = (half[0] - l[0].abs()).abs();
        for k in 1..3 {
            let pen = (half[k] - l[k].abs()).abs();
            if pen < best {
                best = pen;
                axis = k;
            }
        }
        let sign = if l[axis] >= T::zero() {
            T::one()
        } else {
            -T::one()
        };
        let mut n_local = Vec3::zeros();
        n_local[axis] = sign;
        let normal = b.rot * n_local; // 盒→胶囊
        let depth = rc + best;
        let point = cap.pos + normal * rc;
        return Some(Contact::new(point, normal, depth));
    }
    // 两端都在盒外:取到最近盒点距离最小者。
    let d_a2 = (ea - q_a).norm_squared();
    let d_b2 = (eb - q_b).norm_squared();
    let (p, q, d2) = if d_a2 <= d_b2 { (ea, q_a, d_a2) } else { (eb, q_b, d_b2) };
    if d2 > rc * rc {
        return None;
    }
    let dist = d2.sqrt();
    // 法线约定:由胶囊指向盒。p 在胶囊、q 在盒 → (q-p) 由胶囊→盒。
    let n = if dist > T::from_f64(1e-9).unwrap() {
        (q - p) / dist
    } else {
        Vec3::new(T::zero(), -T::one(), T::zero())
    };
    let depth = rc - dist;
    let point = p + n * rc; // 胶囊表面接触点(朝向盒侧)
    Some(Contact::new(point, n, depth))
}

// ===== Heightfield(高度场,静态地形)窄相 =====

/// 查询高度场在局部 xz 处的表面高度(双线性插值)。返回 None 若 xz 超出网格范围。
fn heightfield_height_at<T: RealField + Copy + NumCast>(hf: &Shape<T>, x: T, z: T) -> Option<T> {
    let (nx, nz, cell, heights) = match hf {
        Shape::Heightfield { nx, nz, cell, heights } => (*nx, *nz, *cell, heights),
        _ => return None,
    };
    // 局部 xz 范围:中心在原点,范围 [-nx/2*cell, nx/2*cell]。
    let half_x = T::from_f64(nx as f64).unwrap() * cell * T::from_f64(0.5).unwrap();
    let half_z = T::from_f64(nz as f64).unwrap() * cell * T::from_f64(0.5).unwrap();
    if x < -half_x || x > half_x || z < -half_z || z > half_z {
        return None;
    }
    // 格点坐标:局部 x 从 -half_x 到 half_x,共 nx 格点。
    let gx = (x + half_x) / cell;
    let gz = (z + half_z) / cell;
    let ix = gx.floor();
    let iz = gz.floor();
    let fx = gx - ix;
    let fz = gz - iz;
    let ix = ix.max(T::zero()).min(T::from_f64((nx - 1) as f64).unwrap());
    let iz = iz.max(T::zero()).min(T::from_f64((nz - 1) as f64).unwrap());
    let i0 = to_f64(ix.floor()).max(0.0) as usize;
    let i1 = (i0 + 1).min(nx - 1);
    let j0 = to_f64(iz.floor()).max(0.0) as usize;
    let j1 = (j0 + 1).min(nz - 1);
    let h00 = heights[i0 + nx * j0];
    let h10 = heights[i1 + nx * j0];
    let h01 = heights[i0 + nx * j1];
    let h11 = heights[i1 + nx * j1];
    // 双线性插值。
    let h0 = h00 + (h10 - h00) * fx;
    let h1 = h01 + (h11 - h01) * fx;
    Some(h0 + (h1 - h0) * fz)
}

/// 高度场 vs 动态体:查询动态体(在 xz 处)表面高度,若动态体底部低于表面则产生接触。
/// 法线约定:由高度场指向动态体(垂直向上),用于把动态体托在地形上。
/// 支持 Sphere / Box / Capsule(取各自底部最低点)。
pub fn heightfield_vs_body<T: RealField + Copy + NumCast>(hf: &Body<T>, b: &Body<T>) -> Option<Contact<T>> {
    let (nx, nz, cell, _heights) = match &hf.shape {
        Shape::Heightfield { nx, nz, cell, heights } => (*nx, *nz, *cell, heights),
        _ => return None,
    };
    let _ = (nx, nz, cell);
    // 把动态体质心变换到高度场局部系(高度场默认无旋转,世界系即可)。
    let lpos = hf.rot.inverse() * (b.pos - hf.pos);
    let x = lpos.x;
    let z = lpos.z;
    let surface_h = heightfield_height_at(&hf.shape, x, z)?;
    // 动态体底部到质心的距离(半球/半盒/半胶囊)。
    let bottom_offset = match &b.shape {
        Shape::Sphere { r } => *r,
        Shape::Capsule { half_height, r } => *half_height + *r,
        Shape::Box { half } => half.y,
        _ => return None,
    };
    // 动态体底部 y = lpos.y - bottom_offset。若低于表面则穿透。
    let bottom = lpos.y - bottom_offset;
    if bottom > surface_h {
        return None; // 未接触
    }
    // 法线由高度场指向动态体:垂直向上(+y)。
    let normal = hf.rot * Vec3::new(T::zero(), T::one(), T::zero());
    let depth = surface_h - bottom;
    // 接触点:动态体底部最低点。
    let point_local = Vec3::new(x, surface_h, z);
    let point = hf.pos + hf.rot * point_local;
    Some(Contact::new(point, normal, depth))
}

/// 构造 Compound 的子 Body(世界位姿 = 父位姿 * 子 offset/quat)。
/// 线速度含 offset 的角速度贡献,惯性/层/掩码继承父。
fn sub_body<T: RealField + Copy>(parent: &Body<T>, s: &SubShape<T>) -> Body<T> {
    let offset_world = parent.rot * s.offset;
    Body {
        shape: s.shape.clone(),
        pos: parent.pos + offset_world,
        rot: parent.rot * s.quat,
        vel: parent.vel + parent.ang_vel.cross(&offset_world),
        ang_vel: parent.ang_vel,
        inv_inertia_local: {
            let r = na::Matrix3::from(s.quat);
            r * parent.inv_inertia_local * r.transpose()
        },
        inv_mass: parent.inv_mass,
        sleeping: parent.sleeping,
        sleep_time: parent.sleep_time,
        layers: parent.layers,
        collision_mask: parent.collision_mask,
        kinematic: parent.kinematic,
        is_sensor: parent.is_sensor,
        material: parent.material,
    }
}

/// 通用 Narrow-phase 入口:优先快速路径,回退 GJK+EPA。
/// Compound 逐子形状递归(构造临时子 Body 与对方碰撞,取最深接触)。
pub fn collide<T: RealField + Copy + NumCast>(a: &Body<T>, b: &Body<T>) -> Option<Contact<T>> {
    // a 是复合体:遍历子形状,构造子 Body 与 b 碰撞,取最深。
    if let Shape::Compound { subshapes } = &a.shape {
        let mut best: Option<Contact<T>> = None;
        for s in subshapes.iter() {
            let sub = sub_body(a, s);
            if let Some(c) = collide(&sub, b) {
                if best.as_ref().map(|x| c.depth > x.depth).unwrap_or(true) {
                    best = Some(c);
                }
            }
        }
        return best;
    }
    // b 是复合体:遍历子形状,构造子 Body 与 a 碰撞,取最深(法线翻转成 a→b)。
    if let Shape::Compound { subshapes } = &b.shape {
        let mut best: Option<Contact<T>> = None;
        for s in subshapes.iter() {
            let sub = sub_body(b, s);
            if let Some(c) = collide(a, &sub) {
                // collide(a, sub) 法线由 a→子形状(=a→b),符合约定,不需翻转。
                if best.as_ref().map(|x| c.depth > x.depth).unwrap_or(true) {
                    best = Some(c);
                }
            }
        }
        return best;
    }
    if matches!(a.shape, Shape::Sphere { .. }) && matches!(b.shape, Shape::Sphere { .. }) {
        if let Some(c) = sphere_sphere(a, b) {
            return Some(c);
        }
    }
    if matches!(a.shape, Shape::Box { .. }) && matches!(b.shape, Shape::Box { .. }) {
        if let Some(c) = box_box_sat(a, b) {
            return Some(c);
        }
    }
    // 球-盒快速路径(两种顺序都覆盖)。法线约定由 a 指向 b。
    if matches!(a.shape, Shape::Sphere { .. }) && matches!(b.shape, Shape::Box { .. }) {
        if let Some(c) = sphere_box(a, b) {
            return Some(c);
        }
    }
    if matches!(a.shape, Shape::Box { .. }) && matches!(b.shape, Shape::Sphere { .. }) {
        if let Some(c) = sphere_box(b, a) {
            // sphere_box 返回法线 球→盒(=b→a),翻转成 a→b。
            return Some(Contact::new(c.point, -c.normal, c.depth));
        }
    }
    // 胶囊快速路径(解析,避免 GJK 对平滑+尖角组合的数值不稳定)。
    // capsule-capsule。
    if matches!(a.shape, Shape::Capsule { .. }) && matches!(b.shape, Shape::Capsule { .. }) {
        if let Some(c) = capsule_capsule(a, b) {
            return Some(c);
        }
    }
    // capsule-sphere(两种顺序)。
    if matches!(a.shape, Shape::Capsule { .. }) && matches!(b.shape, Shape::Sphere { .. }) {
        if let Some(c) = capsule_sphere(a, b) {
            return Some(c);
        }
    }
    if matches!(a.shape, Shape::Sphere { .. }) && matches!(b.shape, Shape::Capsule { .. }) {
        if let Some(c) = capsule_sphere(b, a) {
            // capsule_sphere 返回法线 胶囊→球(=b→a),翻转成 a→b。
            return Some(Contact::new(c.point, -c.normal, c.depth));
        }
    }
    // capsule-box(两种顺序)。
    if matches!(a.shape, Shape::Capsule { .. }) && matches!(b.shape, Shape::Box { .. }) {
        if let Some(c) = capsule_box(a, b) {
            return Some(c);
        }
    }
    if matches!(a.shape, Shape::Box { .. }) && matches!(b.shape, Shape::Capsule { .. }) {
        if let Some(c) = capsule_box(b, a) {
            // capsule_box 返回法线 胶囊→盒(=b→a),翻转成 a→b。
            return Some(Contact::new(c.point, -c.normal, c.depth));
        }
    }
    // Heightfield(静态地形)vs 动态体(Sphere/Box/Capsule)。两种顺序。
    if matches!(a.shape, Shape::Heightfield { .. })
        && matches!(b.shape, Shape::Sphere { .. } | Shape::Box { .. } | Shape::Capsule { .. })
    {
        if let Some(c) = heightfield_vs_body(a, b) {
            return Some(c);
        }
    }
    if matches!(b.shape, Shape::Heightfield { .. })
        && matches!(a.shape, Shape::Sphere { .. } | Shape::Box { .. } | Shape::Capsule { .. })
    {
        if let Some(c) = heightfield_vs_body(b, a) {
            // 法线由地形→动态体(=b→a),翻转成 a→b。
            return Some(Contact::new(c.point, -c.normal, c.depth));
        }
    }
    // Heightfield 之间或 Heightfield vs Convex:非凸不支持,返回 None。
    if matches!(a.shape, Shape::Heightfield { .. }) || matches!(b.shape, Shape::Heightfield { .. }) {
        return None;
    }
    if gjk_intersect(a, b) {
        if let Some(c) = gjk_epa_contact(a, b) {
            return Some(c);
        }
    }
    None
}

/// 跑 GJK 收集最终单纯形再 EPA 求接触。
fn gjk_epa_contact<T: RealField + Copy>(a: &Body<T>, b: &Body<T>) -> Option<Contact<T>> {
    let mut dir = a.pos - b.pos;
    if dir.norm() < T::from_f64(1e-9).unwrap() {
        dir = Vec3::new(T::one(), T::zero(), T::zero());
    } else {
        dir = dir.normalize();
    }
    let mut simplex: Vec<Vec3<T>> = Vec::with_capacity(4);
    let s0 = support_minkowski(a, b, &dir);
    simplex.push(s0);
    dir = -s0;
    for _ in 0..64 {
        let dn = dir.norm();
        if dn < T::from_f64(1e-9).unwrap() {
            break;
        }
        let p = support_minkowski(a, b, &dir);
        if p.dot(&dir) < T::zero() {
            return None;
        }
        simplex.push(p);
        if gjk_do_simplex(&mut simplex, &mut dir) {
            break;
        }
    }
    if let Some((n, depth)) = epa(a, b, &simplex) {
        let nrm = b.pos - a.pos;
        let normal = if n.dot(&nrm) < T::zero() { -n } else { n.clone() };
        let point = a.pos + normal.clone() * (depth / T::from_f64(2.0).unwrap());
        return Some(Contact::new(point, normal, depth));
    }
    None
}
