//! Narrow-phase 精确碰撞:球/盒快速路径 + 通用 GJK(相交)/ EPA(接触)。
//!
//! - 球-球:闭式。
//! - 盒-盒: SAT(分离轴定理)。
//! - 通用凸体(含球/盒/凸多面体):GJK 判相交,EPA 求接触法线+穿透深度。

use phy_math::{RealField, Vec3};

use crate::contact::Contact;
use crate::shape::{Body, Shape};

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

/// 通用 Narrow-phase 入口:优先快速路径,回退 GJK+EPA。
pub fn collide<T: RealField + Copy>(a: &Body<T>, b: &Body<T>) -> Option<Contact<T>> {
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
