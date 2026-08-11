//! 破碎 / Voronoi 碎片(路线图 #8)。
//!
//! 提供 `fracture_body`:把一个**凸**刚体按 `n` 个 Voronoi 种子点切成 `n` 个凸碎片,
//! 每个碎片用其局部顶点 + 三角面表示(`Shape::Convex`),质量按体积比例分配。
//!
//! 算法(Voronoi 图 + 半空间裁剪):
//! 1. 在物体局部包围盒内均匀/随机撒 `n` 个种子点(也可由调用方显式指定)。
//! 2. 对每个种子,其 Voronoi 细胞 = 物体原始凸多面体 ∩ 所有"与邻居中垂面"半空间。
//!    逐种子用"半空间裁剪(Sutherland–Hodgman 在凸多面面上)"迭代求交得到碎片多面体。
//! 3. 碎片质心/体积由凸多面体散度定理(体积积分)求得,质量 = 母体质心质量 × 体积比。
//!
//! 关键:本引擎刚体**已具备完整角动力学**(`Body` 含 `rot`/`ang_vel`/`inv_inertia_local`,
//! `world.advance` 用四元数导数积分姿态,`solver` 接触含角冲量)。故碎片:
//! 1) 通过 `set_inertia_from_shape()` 基于自身 Convex 几何计算惯性张量(让角动力学生效);
//! 2) 继承母本 `ang_vel`(自由旋转母体碎裂后碎片带自旋);
//! 3) 径向飞散时,偏心冲量经 `apply_impulse_at` 注入真实角自旋(角动量守恒)。
//! 与"最小侵入回避角动力学"的旧实现不同(见 PLAN §11 阶段 0 清理)。
//!
//! `fracture_body` 不修改 `RigidWorld`,只返回新 `Body` 列表(含局部→世界变换后的
//! 顶点),由调用方(如 `RigidWorld::shatter`)加入世界,避免热路径耦合。

use phy_math::{RealField, Vec3};
use num_traits::NumCast;

use crate::shape::{Body, Shape};

/// 一个 Voronoi 种子点(物体局部坐标)。
pub type Seed<T> = Vec3<T>;

/// 把一个凸多面体(局部顶点 + 三角面)按 `seeds` 切成 Voronoi 细胞。
///
/// 返回每个种子的碎片多面体(局部坐标顶点 + 重三角化面)。
/// 若 `body` 非凸(球/盒在局部空间恒为凸,可特化),先转成凸顶点-面表达。
///
/// 不变量:所有碎片体积之和 ≈ 原体积;每个碎片顶点都在其它种子中垂面之内。
pub fn fracture_convex<T: RealField + Copy + NumCast>(
    vertices: &[Vec3<T>],
    faces: &[[usize; 3]],
    seeds: &[Seed<T>],
) -> Vec<(Vec<Vec3<T>>, Vec<[usize; 3]>)> {
    // 原多面体:顶点 + 三角面(局部坐标)。对每个种子,以原始多面体为初始"细胞",
    // 逐其它种子用中垂面裁剪(带封盖),得到该种子的 Voronoi 凸细胞。
    let n = seeds.len();
    let mut cells: Vec<(Vec<Vec3<T>>, Vec<[usize; 3]>)> = Vec::with_capacity(n);
    for _ in 0..n {
        cells.push((vertices.to_vec(), faces.to_vec()));
    }
    for i in 0..n {
        for j in 0..n {
            if i == j {
                continue;
            }
            // 中垂面:法线 = seeds[j] - seeds[i],平面过中点。
            // 保留 cells[i] 中位于"靠近 seeds[i]"一侧(dot(p - mid, n) <= 0)的部分。
            let nrm = seeds[j] - seeds[i];
            let mid = (seeds[i] + seeds[j]) * T::from_f64(0.5).unwrap();
            let (cv, cf) = clip_convex_by_plane(&cells[i].0, &cells[i].1, &nrm, &mid);
            if cf.is_empty() {
                // 整个细胞被切没了(种子极近导致),用空集标记。
                cells[i] = (Vec::new(), Vec::new());
                break;
            }
            cells[i] = (cv, cf);
        }
    }
    cells
}

/// 用半空间(法线 `nrm`,过点 `mid`,保留 `dot(p-mid,nrm) <= 0` 一侧)裁剪凸多面体
/// (顶点 + 三角面),返回裁剪后的(顶点 + 面),并对切口补一个"封盖"三角形扇,
/// 保证结果仍是闭合凸多面体(体积/质心计算正确)。
fn clip_convex_by_plane<T: RealField + Copy + NumCast>(
    verts: &[Vec3<T>],
    faces: &[[usize; 3]],
    nrm: &Vec3<T>,
    mid: &Vec3<T>,
) -> (Vec<Vec3<T>>, Vec<[usize; 3]>) {
    let eps = T::from_f64(1e-9).unwrap();
    let signed = |v: &Vec3<T>| (v - mid).dot(nrm);
    let mut new_verts = verts.to_vec();
    let mut new_faces: Vec<[usize; 3]> = Vec::new();
    let mut cap_pts: Vec<Vec3<T>> = Vec::new(); // 切口与平面的交点(封盖用)

    let push_v = |nv: &mut Vec<Vec3<T>>, p: Vec3<T>| -> usize {
        if let Some(idx) = nv.iter().position(|u| (u - p).norm() < eps) {
            idx
        } else {
            nv.push(p);
            nv.len() - 1
        }
    };

    for f in faces {
        let tri = [verts[f[0]], verts[f[1]], verts[f[2]]];
        let d = [signed(&tri[0]), signed(&tri[1]), signed(&tri[2])];
        // 裁剪三角形 → 多边形(3 或 4 顶点)。
        let mut poly: Vec<Vec3<T>> = Vec::new();
        for k in 0..3 {
            let cur = k;
            let nxt = (k + 1) % 3;
            if d[cur] <= eps {
                poly.push(tri[cur]);
            }
            if (d[cur] <= eps) != (d[nxt] <= eps) {
                let denom = d[cur] - d[nxt];
                if denom.abs() > eps {
                    let t = d[cur] / denom;
                    let ip = tri[cur] + (tri[nxt] - tri[cur]) * t;
                    poly.push(ip);
                    cap_pts.push(ip);
                }
            }
        }
        // 多边形 fan 三角化(顶点 0 为扇形中心)。
        if poly.len() < 3 {
            continue; // 退化(整面被切掉/压成线):跳过。
        }
        for k in 1..poly.len() - 1 {
            let a = push_v(&mut new_verts, poly[0]);
            let b = push_v(&mut new_verts, poly[k]);
            let c = push_v(&mut new_verts, poly[k + 1]);
            new_faces.push([a, b, c]);
        }
    }

    // 封盖:把切口交点(去重)排成环,以质心为 apex 扇化。
    // 保留侧为 -nrm,故封盖外法线指向 +nrm(被切掉的一侧)。
    if cap_pts.len() >= 3 {
        // 去重交点(共享边会重复记录)。
        let eps2 = T::from_f64(1e-8).unwrap();
        let mut pts: Vec<Vec3<T>> = Vec::new();
        for p in &cap_pts {
            if !pts.iter().any(|u| (u - p).norm() < eps2) {
                pts.push(*p);
            }
        }
        if pts.len() >= 3 {
            let cen = pts.iter().fold(Vec3::zeros(), |s, v| s + *v)
                / T::from_f64(pts.len() as f64).unwrap();
            // 在平面内选一组正交基 (u, w),使 (u, w, n) 右手系:w = u × n。
            let n = *nrm;
            let mut u = if n.x.abs() < n.y.abs() && n.x.abs() < n.z.abs() {
                Vec3::new(T::zero(), n.z, -n.y)
            } else if n.y.abs() < n.z.abs() {
                Vec3::new(n.z, T::zero(), -n.x)
            } else {
                Vec3::new(n.y, -n.x, T::zero())
            };
            u = u.normalize();
            let w = n.cross(&u); // 使 (u, w, n) 右手系:CCW 扇化封盖外法指向 +nrm
            let mut ring: Vec<Vec3<T>> = pts.clone();
            ring.sort_by(|p, q| {
                let ap = (p - cen).dot(&u);
                let bp = (p - cen).dot(&w);
                let aq = (q - cen).dot(&u);
                let bq = (q - cen).dot(&w);
                bp.atan2(ap).partial_cmp(&bq.atan2(aq)).unwrap_or(std::cmp::Ordering::Equal)
            });
            let base = push_v(&mut new_verts, cen);
            let mut prev = push_v(&mut new_verts, ring[0]);
            for k in 1..ring.len() {
                let cur = push_v(&mut new_verts, ring[k]);
                // 环按 (u,w) 平面 CCW 排序(从 +nrm 看),扇化三角形外法自然指向 +nrm。
                new_faces.push([base, prev, cur]);
                prev = cur;
            }
        }
    }

    (new_verts, new_faces)
}

#[inline]
fn to_f64<T: RealField + Copy + NumCast>(x: T) -> f64 {
    x.to_f64().unwrap_or(0.0)
}

/// 凸多面体的体积与质心(散度定理:对凸多面体,体积 = Σ (v0·(e1×e2))/6,
/// 质心 = Σ (v0+v1+v2)·(v0·(e1×e2))/24 / volume)。
pub fn convex_volume_centroid<T: RealField + Copy + NumCast>(
    verts: &[Vec3<T>],
    faces: &[[usize; 3]],
) -> (T, Vec3<T>) {
    let zero = T::zero();
    let mut vol = zero;
    let mut cen = Vec3::zeros();
    for f in faces {
        let (i, j, k) = (f[0], f[1], f[2]);
        if i >= verts.len() || j >= verts.len() || k >= verts.len() {
            continue;
        }
        let a = verts[i];
        let bb = verts[j];
        let c = verts[k];
        let cross = bb.cross(&c);
        let tet_vol = a.dot(&cross) / T::from_f64(6.0).unwrap();
        vol += tet_vol;
        cen += (a + bb + c) * (tet_vol / T::from_f64(4.0).unwrap());
    }
    if vol > zero {
        cen /= vol;
    } else {
        // 退化:用顶点平均兜底。
        cen = verts.iter().fold(Vec3::zeros(), |s, v| s + *v) / T::from_f64(verts.len() as f64).unwrap();
        vol = vol.abs();
    }
    (vol, cen)
}

/// 把一个刚体按 Voronoi 碎裂成多个凸碎片刚体。
///
/// - `n`:碎片数量(若 `seeds` 为 `None`,在局部包围盒内均匀 jitter 撒点)。
/// - `impulse`:作用于母体质心的冲量(世界坐标),碎片初速 = 母体质心速度 +
///   径向爆裂(`(frag_centroid_world - body_centroid_world).normalize() * spread` 的变形)。
///   实际采用:每个碎片获得母体质心速度 + `impulse * frag_inv_mass`(与母体质心受同等
///   冲量一致),再由 `radial` 叠加一个沿碎片-母体质心方向的离散速度,体现"碎裂飞散"。
/// - `radial`:径向飞散速度尺度(0 = 纯继承线速度,无角/径向扩散)。
///
/// 返回新碎片 `Body`(世界坐标已烘焙到 `pos`/`rot`/`shape` 顶点)。母本体仍保留在
/// 世界中(调用方应自行移除或保留为不碎主体)。
pub fn fracture_body<T: RealField + Copy + NumCast>(
    body: &Body<T>,
    n: usize,
    seeds: Option<&[Seed<T>]>,
    radial: T,
) -> Vec<Body<T>> {
    // 母体检索为凸表达(球/盒/凸均转凸顶点-面)。
    let (local_verts, local_faces) = body_to_convex_local(body);

    // 决定种子点。
    let seeds = match seeds {
        Some(s) => s.to_vec(),
        None => scatter_seeds(&local_verts, n),
    };

    // 原始凸多面体体积(用于质量比)。
    let (orig_vol, _) = convex_volume_centroid(&local_verts, &local_faces);

    // 逐种子切 Voronoi 细胞。
    let cells = fracture_convex(&local_verts, &local_faces, &seeds);

    let m_total = if body.inv_mass > T::zero() {
        T::one() / body.inv_mass
    } else {
        T::one() // 静态体碎裂后给单位质量(游戏向)。
    };

    let body_centroid_world = body.pos; // 凸体局部质心近似原点(对盒/球成立)。

    let mut frags = Vec::new();
    for (cell_verts, cell_faces) in cells {
        if cell_verts.len() < 4 {
            continue; // 退化细胞(体积≈0)跳过。
        }
        let (vol, cl) = convex_volume_centroid(&cell_verts, &cell_faces);
        if vol <= T::from_f64(1e-9).unwrap() {
            continue;
        }
        // 碎片质量按体积比。
        let m_frag = if orig_vol > T::zero() {
            m_total * (vol / orig_vol)
        } else {
            m_total / T::from_f64(n as f64).unwrap()
        };
        let inv_mass = if m_frag > T::zero() {
            T::one() / m_frag
        } else {
            T::zero()
        };
        // 碎片局部质心 → 世界。
        let cl_world = body.rot * cl + body.pos;
        // 碎片形状顶点:相对碎片质心的局部坐标(旋转继承母体,已含世界方向)。
        let shape_verts: Vec<Vec3<T>> = cell_verts
            .iter()
            .map(|v| body.rot * (v - cl))
            .collect();
        // 构造碎片体:先按自身几何算惯性张量(角动力学生效),再继承母本自旋。
        let mut frag = Body {
            shape: Shape::Convex {
                vertices: shape_verts,
                faces: cell_faces,
            },
            pos: cl_world,
            rot: body.rot, // 继承母朝向。
            vel: body.vel, // 继承母体质心线速度。
            ang_vel: body.ang_vel, // 继承母本角速度(碎片带自旋)。
            inv_mass,
            ..Default::default()
        };
        frag.set_inertia_from_shape(); // 基于 Convex 几何写入 inv_inertia_local。
        // 径向飞散:线速度沿"碎片质心相对母体质心方向"注入;角自旋由碎面不对称扭力
        // 产生 —— 用相对碎片质心的确定性偏心(r_offset)经 `apply_impulse_at` 注入角动量,
        // 幅度正比于 radial 与碎片尺度(物理上:碎裂瞬间相邻碎块挤压给碎片一个扭转冲量)。
        if radial > T::zero() {
            let dir = cl_world - body_centroid_world;
            let len = dir.norm();
            if len > T::from_f64(1e-9).unwrap() {
                let ndir = dir / len;
                frag.vel += ndir * radial; // 过质心线冲量 → 纯平移飞散。
                // 确定性偏心:由碎片包围盒半长派生一个与径向正交的偏置向量(非 0)。
                let half = frag_inertia_half(&cell_verts);
                let r_offset = Vec3::new(half.y, half.z, half.x) * T::from_f64(0.25).unwrap();
                // 角冲量方向取径向与偏置的叉积(产生绕碎片质心的自转)。
                let jt = ndir.cross(&r_offset).normalize() * (radial * inv_mass);
                frag.apply_impulse_at(jt, r_offset); // r_offset 非 0 → 产生角自旋。
            }
        }
        frags.push(frag);
    }
    frags
}

/// 由碎片局部顶点求包围盒半长(惯性张量对角估计用)。
fn frag_inertia_half<T: RealField + Copy + NumCast>(verts: &[Vec3<T>]) -> Vec3<T> {
    if verts.is_empty() {
        return Vec3::zeros();
    }
    let mut h = Vec3::new(T::zero(), T::zero(), T::zero());
    for v in verts {
        h.x = if v.x.abs() > h.x { v.x.abs() } else { h.x };
        h.y = if v.y.abs() > h.y { v.y.abs() } else { h.y };
        h.z = if v.z.abs() > h.z { v.z.abs() } else { h.z };
    }
    h
}

/// 把任意 `Body` 转为局部凸顶点 + 三角面。
fn body_to_convex_local<T: RealField + Copy + NumCast>(body: &Body<T>) -> (Vec<Vec3<T>>, Vec<[usize; 3]>) {
    match &body.shape {
        Shape::Sphere { r } => {
            // 用 6 面体(立方体)逼近(球体碎裂成方块碎片即可,游戏向)。
            let h = *r;
            let v = vec![
                Vec3::new(-h, -h, -h),
                Vec3::new(h, -h, -h),
                Vec3::new(h, h, -h),
                Vec3::new(-h, h, -h),
                Vec3::new(-h, -h, h),
                Vec3::new(h, -h, h),
                Vec3::new(h, h, h),
                Vec3::new(-h, h, h),
            ];
            let f = vec![
                [0, 1, 2],
                [0, 2, 3], // -z
                [4, 6, 5],
                [4, 7, 6], // +z
                [0, 5, 1],
                [0, 4, 5], // -y
                [2, 6, 7],
                [2, 7, 3], // +y
                [1, 6, 2],
                [1, 5, 6], // +x
                [3, 7, 4],
                [3, 4, 0], // -x
            ];
            (v, f)
        }
        Shape::Box { half } => {
            let (hx, hy, hz) = (half.x, half.y, half.z);
            let v = vec![
                Vec3::new(-hx, -hy, -hz),
                Vec3::new(hx, -hy, -hz),
                Vec3::new(hx, hy, -hz),
                Vec3::new(-hx, hy, -hz),
                Vec3::new(-hx, -hy, hz),
                Vec3::new(hx, -hy, hz),
                Vec3::new(hx, hy, hz),
                Vec3::new(-hx, hy, hz),
            ];
            let f = vec![
                [0, 1, 2],
                [0, 2, 3],
                [4, 6, 5],
                [4, 7, 6],
                [0, 5, 1],
                [0, 4, 5],
                [2, 6, 7],
                [2, 7, 3],
                [1, 6, 2],
                [1, 5, 6],
                [3, 7, 4],
                [3, 4, 0],
            ];
            (v, f)
        }
        Shape::Convex { vertices, faces } => (vertices.clone(), faces.clone()),
    }
}

/// 在顶点包围盒内均匀网格 + jitter 撒 `n` 个种子点。
fn scatter_seeds<T: RealField + Copy + NumCast>(verts: &[Vec3<T>], n: usize) -> Vec<Seed<T>> {
    if verts.is_empty() {
        return Vec::new();
    }
    let mut min = verts[0];
    let mut max = verts[0];
    for v in verts {
        min = min.inf(v);
        max = max.sup(v);
    }
    let span = max - min;
    let min_f = [
        to_f64(min.x),
        to_f64(min.y),
        to_f64(min.z),
    ];
    let span_f = [
        to_f64(span.x),
        to_f64(span.y),
        to_f64(span.z),
    ];
    // 沿最长边的立方体网格近似,取 ceil(cbrt(n)) 为每边点数。
    let per = (n as f64).cbrt().ceil() as usize;
    let per = per.max(1);
    let mut seeds = Vec::new();
    let mut count = 0usize;
    for i in 0..per {
        for j in 0..per {
            for k in 0..per {
                if count >= n {
                    break;
                }
                let fp = per as f64;
                let jit = 0.15;
                let rx = (rand01(count * 3) - 0.5) * 2.0 * jit;
                let ry = (rand01(count * 3 + 1) - 0.5) * 2.0 * jit;
                let rz = (rand01(count * 3 + 2) - 0.5) * 2.0 * jit;
                // 映射到 (1/(per+1) .. per/(per+1)) 内边距,确保种子严格在物体内部。
                let sx = (i as f64 + 1.0 + rx) / (fp + 1.0);
                let sy = (j as f64 + 1.0 + ry) / (fp + 1.0);
                let sz = (k as f64 + 1.0 + rz) / (fp + 1.0);
                let p = Vec3::new(
                    T::from_f64(min_f[0] + span_f[0] * sx).unwrap(),
                    T::from_f64(min_f[1] + span_f[1] * sy).unwrap(),
                    T::from_f64(min_f[2] + span_f[2] * sz).unwrap(),
                );
                seeds.push(p);
                count += 1;
            }
        }
    }
    // 若网格点不足 n,补内边距随机点。
    while seeds.len() < n {
        let rx = 0.1 + rand01(seeds.len() * 7 + 1) * 0.8;
        let ry = 0.1 + rand01(seeds.len() * 7 + 2) * 0.8;
        let rz = 0.1 + rand01(seeds.len() * 7 + 3) * 0.8;
        seeds.push(Vec3::new(
            T::from_f64(min_f[0] + span_f[0] * rx).unwrap(),
            T::from_f64(min_f[1] + span_f[1] * ry).unwrap(),
            T::from_f64(min_f[2] + span_f[2] * rz).unwrap(),
        ));
    }
    seeds
}

/// 确定性伪随机 [0,1)(避免引入 rand 依赖,纯几何用)。
fn rand01(x: usize) -> f64 {
    // 整数哈希(xorshift)→ 取低 24 位归一化到 [0,1)。
    let mut s = (x.wrapping_add(0x9E3779B9)).wrapping_mul(0x85EBCA6B);
    s ^= s >> 13;
    s = s.wrapping_mul(0xC2B2AE35);
    s ^= s >> 16;
    ((s >> 8) & 0xFFFFFF) as f64 / (1u64 << 24) as f64
}

#[cfg(test)]
mod tests {
    use super::*;
    use phy_math::{na, Vec3};

    #[test]
    fn fracture_box_yields_n_fragments() {
        // 一个 2×2×2 盒切成 4 块,碎块数应接近 4(退化细胞极少)。
        let body = Body {
            shape: Shape::Box {
                half: Vec3::new(1.0, 1.0, 1.0),
            },
            pos: Vec3::new(0.0, 0.0, 0.0),
            rot: na::one(),
            vel: Vec3::zeros(),
            inv_mass: 1.0,
        
            ..Default::default()
        };
        let frags = fracture_body(&body, 4, None, 0.0);
        assert!(frags.len() >= 3, "应切出至少 3 块,得 {}", frags.len());
        // 每块都是 Convex 且顶点数 >= 4。
        for f in &frags {
            if let Shape::Convex { vertices, .. } = &f.shape {
                assert!(vertices.len() >= 4);
            } else {
                panic!("碎片必须是 Convex");
            }
        }
    }

    #[test]
    fn fragment_mass_sums_to_parent() {
        // 质量守恒:碎片质量之和 ≈ 母体质心质量。
        let m = 8.0;
        let body = Body {
            shape: Shape::Box {
                half: Vec3::new(1.0, 1.0, 1.0),
            },
            pos: Vec3::new(0.0, 0.0, 0.0),
            rot: na::one(),
            vel: Vec3::zeros(),
            inv_mass: 1.0 / m,
        
            ..Default::default()
        };
        let frags = fracture_body(&body, 8, None, 0.0);
        let sum: f64 = frags.iter().map(|f| 1.0 / f.inv_mass).sum();
        assert!((sum - m).abs() / m < 0.05, "碎片质量之和应≈母质量,得 {}", sum);
    }

    #[test]
    fn fracture_adds_radial_velocity() {
        // 径向飞散应使碎片获得非零速度(母本静止)。
        let body = Body {
            shape: Shape::Box {
                half: Vec3::new(1.0, 1.0, 1.0),
            },
            pos: Vec3::new(0.0, 0.0, 0.0),
            rot: na::one(),
            vel: Vec3::zeros(),
            inv_mass: 1.0,
        
            ..Default::default()
        };
        let frags = fracture_body(&body, 6, None, 2.0);
        let max_speed = frags.iter().map(|f| f.vel.norm()).fold(0.0_f64, f64::max);
        assert!(max_speed > 1.0, "径向碎裂应赋予碎片飞散速度,得 {}", max_speed);
    }

    #[test]
    fn fracture_convex_preserves_volume_sum() {
        // Voronoi 切分:碎片体积之和 ≈ 原体积(2×2×2 = 8)。
        let verts = vec![
            Vec3::new(-1.0, -1.0, -1.0),
            Vec3::new(1.0, -1.0, -1.0),
            Vec3::new(1.0, 1.0, -1.0),
            Vec3::new(-1.0, 1.0, -1.0),
            Vec3::new(-1.0, -1.0, 1.0),
            Vec3::new(1.0, -1.0, 1.0),
            Vec3::new(1.0, 1.0, 1.0),
            Vec3::new(-1.0, 1.0, 1.0),
        ];
        let faces = vec![
            [0, 1, 2],
            [0, 2, 3],
            [4, 6, 5],
            [4, 7, 6],
            [0, 5, 1],
            [0, 4, 5],
            [2, 6, 7],
            [2, 7, 3],
            [1, 6, 2],
            [1, 5, 6],
            [3, 7, 4],
            [3, 4, 0],
        ];
        let seeds = vec![
            Vec3::new(-0.5, -0.5, -0.5),
            Vec3::new(0.5, -0.5, -0.5),
            Vec3::new(-0.5, 0.5, -0.5),
            Vec3::new(0.5, 0.5, -0.5),
            Vec3::new(-0.5, -0.5, 0.5),
            Vec3::new(0.5, -0.5, 0.5),
            Vec3::new(-0.5, 0.5, 0.5),
            Vec3::new(0.5, 0.5, 0.5),
        ];
        let cells = fracture_convex(&verts, &faces, &seeds);
        let mut total = 0.0_f64;
        for (cv, cf) in &cells {
            let (v, _) = convex_volume_centroid(cv, cf);
            total += v;
        }
        assert!(
            (total - 8.0_f64).abs() < 0.3_f64,
            "碎片体积之和应≈8,得 {}",
            total
        );
    }

    #[test]
    fn fragment_inherits_parent_spin() {
        // 母本带角速度 → 碎片应继承非零 ang_vel(角动力学已具备)。
        use phy_math::na;
        let mut body = Body {
            shape: Shape::Box {
                half: Vec3::new(1.0, 1.0, 1.0),
            },
            pos: Vec3::new(0.0, 0.0, 0.0),
            rot: na::one(),
            vel: Vec3::zeros(),
            inv_mass: 1.0,
            ..Default::default()
        };
        body.set_inertia_from_shape();
        body.ang_vel = Vec3::new(0.0, 5.0, 0.0); // 母本绕 y 轴自旋。
        let frags = fracture_body(&body, 6, None, 0.0);
        assert!(!frags.is_empty(), "应切出碎片");
        for f in &frags {
            // 碎片应继承母本角速度(非零),且惯性张量已被写入(可响应角冲量)。
            assert!(
                f.ang_vel.norm() > 1e-6,
                "碎片应继承母本自旋,得 {:?}",
                f.ang_vel
            );
            assert!(
                f.inv_inertia_local.norm() > 1e-9,
                "碎片应已计算惯性张量,得 {:?}",
                f.inv_inertia_local
            );
        }
    }

    #[test]
    fn radial_shatter_imparts_angular_spin() {
        // 径向飞散:偏心冲量应给静止碎片注入非零角速度(角动量守恒,而非纯平移)。
        let body = Body {
            shape: Shape::Box {
                half: Vec3::new(1.0, 1.0, 1.0),
            },
            pos: Vec3::new(0.0, 0.0, 0.0),
            rot: na::one(),
            vel: Vec3::zeros(),
            ang_vel: Vec3::zeros(), // 母本不自转。
            inv_mass: 1.0,
            ..Default::default()
        };
        let frags = fracture_body(&body, 6, None, 2.0);
        // 至少应有一部分碎片获得非零角速度(偏心碎片)。
        let spun = frags.iter().any(|f| f.ang_vel.norm() > 1e-6);
        assert!(spun, "径向碎裂应让偏心碎片获得角自旋");
    }
}
