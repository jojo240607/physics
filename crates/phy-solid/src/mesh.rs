//! 规则盒网格构造与悬臂梁专用构造器。
//!
//! `SolidWorld::from_box` 生成任意规则盒的 Tet 网格。这里提供:
//! - [`MeshParams`]:盒网格参数(尺寸、分段、原点、弹性模量/泊松比、材料密度);
//! - [`cantilever_box`]:生成一个沿 +x 方向伸出、左端 (x = lo.x) 全固定的悬臂梁体;
//! - [`apply_tip_load`]/[`tip_displacement`]:自由端加载与位移读取(与解析柔度对照)。
//!
//! 习惯:x 为梁长方向,左端固定面 `x == lo.x`,外载在自由端 (x = hi.x) 施加。

use phy_math::{RealField, Vec3};

use crate::fem::{Node, SolidWorld, Tet};

/// 规则盒网格参数。
#[derive(Clone, Debug)]
pub struct MeshParams<T: RealField> {
    /// 盒左下后角。
    pub lo: Vec3<T>,
    /// 盒右上前角。
    pub hi: Vec3<T>,
    /// 沿各轴单元段数 (>=1)。
    pub segs: (usize, usize, usize),
    /// 杨氏模量 E。
    pub young: T,
    /// 泊松比 nu。
    pub poisson: T,
    /// 名义材料密度(动力松弛用)。
    pub rho: T,
    /// 是否固定 x==lo.x 面(做悬臂支座)。
    pub fix_x_min: bool,
}

impl<T: RealField> Default for MeshParams<T> {
    fn default() -> Self {
        Self {
            lo: Vec3::zeros(),
            hi: Vec3::new(T::one(), T::from_f64(0.1).unwrap(), T::from_f64(0.1).unwrap()),
            segs: (8, 1, 1),
            young: T::from_f64(1.0e9).unwrap(),
            poisson: T::from_f64(0.3).unwrap(),
            rho: T::from_f64(1000.0).unwrap(),
            fix_x_min: true,
        }
    }
}

/// 构造一个规则盒 Tet 网格(各轴 >=1 段)。
pub fn box_mesh<
    T: RealField + Copy + num_traits::ToPrimitive + num_traits::FromPrimitive,
>(
    p: &MeshParams<T>,
) -> SolidWorld<T> {
    let mut w = SolidWorld::new(p.young, p.poisson, p.rho);
    w.from_box(p.lo, p.hi, p.segs.0, p.segs.1, p.segs.2, p.fix_x_min);
    w
}

/// 构造一根沿 +x 伸出的悬臂梁:左端 (x==lo.x) 全固定,自由端在 x=hi.x。
pub fn cantilever_box<
    T: RealField + Copy + num_traits::ToPrimitive + num_traits::FromPrimitive,
>(
    p: &MeshParams<T>,
) -> SolidWorld<T> {
    let mut p2 = p.clone();
    p2.fix_x_min = true;
    box_mesh(&p2)
}

/// 取自由端 (x 最大) 处位移最大的节点位移(用于与解析柔度对照)。
pub fn tip_displacement<
    T: RealField + Copy + num_traits::ToPrimitive + num_traits::FromPrimitive,
>(
    w: &SolidWorld<T>,
    axis: usize,
) -> T {
    let xmax = w.nodes.iter().map(|n| n.x0.x).fold(T::zero(), |a, b| {
        if b > a {
            b
        } else {
            a
        }
    });
    let mut best = T::zero();
    let mut best_node: Option<&Node<T>> = None;
    for n in w.nodes.iter() {
        if (n.x0.x - xmax).abs() <= T::from_f64(1e-9).unwrap() {
            let d = n.u.norm();
            if d > best {
                best = d;
                best_node = Some(n);
            }
        }
    }
    match best_node {
        Some(n) => n.u[axis],
        None => T::zero(),
    }
}

/// 在自由端 (x 最大) 所有自由节点上叠加指定位移方向的外载,返回受力节点数。
///
/// 注:`SolidWorld::step` 会在每步把 `node.f` 重置为重力再累加单元内力,
/// 因此载荷必须通过 `add_load`(在 each step 之前) 或这里一次性加到 `node.f`;
/// 为稳健起见,这里把载荷写入节点 `f`,并假定随后立即 `solve_equilibrium`
/// (静力求解不重置 `f`,直接以当前 `f` 为右端)。若要动力松弛,需每步重新加载。
pub fn apply_tip_load<
    T: RealField + Copy + num_traits::ToPrimitive + num_traits::FromPrimitive,
>(
    w: &mut SolidWorld<T>,
    load: Vec3<T>,
) -> usize {
    let xmax = w.nodes.iter().map(|n| n.x0.x).fold(T::zero(), |a, b| {
        if b > a {
            b
        } else {
            a
        }
    });
    let thr = T::from_f64(1e-9).unwrap();
    let mut count = 0usize;
    for (i, n) in w.nodes.iter_mut().enumerate() {
        if !n.fixed && (n.x0.x - xmax).abs() <= thr {
            if i < w.load.len() {
                w.load[i] += load;
            }
            count += 1;
        }
    }
    count
}

/// 取第 i 个四面体(便于渲染/调试)。
pub fn tet(w: &SolidWorld<f64>, i: usize) -> Option<&Tet> {
    w.tets.get(i)
}
