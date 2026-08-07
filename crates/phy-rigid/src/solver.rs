//! 约束求解:顺序冲量法(Sequential Impulse, 累积冲量版)。
//!
//! 对每个接触迭代求解法向冲量与摩擦冲量,使物体间不穿透且符合恢复系数。
//! 采用标准 SI 的**累积冲量**形式:每次迭代施加增量 dλ 并把 λ 钳制为非负,
//! 避免非累积版本在堆叠场景中产生的反向过冲与抖动。

use phy_math::{RealField, Vec3};

use crate::contact::Contact;
use crate::shape::Body;

/// 求解参数。
#[derive(Debug, Clone)]
pub struct SolverParams<T: RealField> {
    /// 恢复系数 (0 = 完全非弹性, 1 = 完全弹性)。
    pub restitution: T,
    /// 摩擦系数。
    pub friction: T,
    /// 速度求解迭代次数。
    pub iterations: usize,
}

impl<T: RealField> Default for SolverParams<T> {
    fn default() -> Self {
        Self {
            restitution: T::from_f64(0.0).unwrap(),
            friction: T::from_f64(0.5).unwrap(),
            iterations: 20,
        }
    }
}

/// 一个接触在求解期的状态(含参与 body 索引、接触几何、累积冲量)。
pub struct ContactConstraint<T: RealField + Copy> {
    pub a: usize,
    pub b: usize,
    pub contact: Contact<T>,
    /// 累积法向冲量(非负)。
    pub normal_impulse: T,
    /// 累积切向冲量(向量)。
    pub tangent_impulse: Vec3<T>,
}

impl<T: RealField + Copy> ContactConstraint<T> {
    pub fn new(a: usize, b: usize, contact: Contact<T>) -> Self {
        Self {
            a,
            b,
            contact,
            normal_impulse: T::zero(),
            tangent_impulse: Vec3::zeros(),
        }
    }
}

/// 用顺序冲量法求解所有接触,就地修改 bodies 的速度。
pub fn solve_velocity<T: RealField + Copy>(
    bodies: &mut [Body<T>],
    constraints: &mut [ContactConstraint<T>],
    params: &SolverParams<T>,
) {
    let e = params.restitution;
    let mu = params.friction;

    for _ in 0..params.iterations {
        for c in constraints.iter_mut() {
            if std::env::var("PHY_DEBUG").is_ok() && c.contact.depth > T::from_f64(0.0).unwrap() {
                eprintln!(
                    "[SOLVER] a={} b={} n=({:.3},{:.3},{:.3}) depth={:.3} va=({:.3},{:.3},{:.3}) vb=({:.3},{:.3},{:.3})",
                    c.a, c.b,
                    c.contact.normal.x, c.contact.normal.y, c.contact.normal.z,
                    c.contact.depth,
                    bodies[c.a].vel.x, bodies[c.a].vel.y, bodies[c.a].vel.z,
                    bodies[c.b].vel.x, bodies[c.b].vel.y, bodies[c.b].vel.z,
                );
            }
            let n = c.contact.normal;
            let inv_sum = bodies[c.a].inv_mass + bodies[c.b].inv_mass;
            if inv_sum <= T::zero() {
                continue;
            }

            // ---- 法向(累积增量) ----
            let rel_v = bodies[c.b].vel - bodies[c.a].vel;
            let vn = rel_v.dot(&n);
            // 增量冲量:把法向相对速度消除(含恢复系数)
            let dlambda = -(T::one() + e) * vn / inv_sum;
            let new_lambda = if c.normal_impulse + dlambda > T::zero() {
                c.normal_impulse + dlambda
            } else {
                T::zero()
            };
            let applied = new_lambda - c.normal_impulse;
            c.normal_impulse = new_lambda;
            let imp = n * applied;
            bodies[c.a].vel -= imp * bodies[c.a].inv_mass;
            bodies[c.b].vel += imp * bodies[c.b].inv_mass;

            // ---- 摩擦(切向, 受库仑锥限制) ----
            let rel_v = bodies[c.b].vel - bodies[c.a].vel;
            let vn_now = rel_v.dot(&n);
            let tangent = rel_v - n * vn_now;
            let tlen = tangent.norm();
            if tlen > T::from_f64(1e-9).unwrap() {
                let t = tangent / tlen;
                let vt = rel_v.dot(&t);
                let dlt = -vt / inv_sum;
                // 库仑摩擦: |λ_t| <= μ * λ_n
                let max_lt = mu * c.normal_impulse;
                let new_lt = if c.tangent_impulse.norm() + dlt.abs() > max_lt {
                    if dlt > T::zero() {
                        max_lt
                    } else {
                        -max_lt
                    }
                } else {
                    c.tangent_impulse.norm() + dlt
                };
                let applied_t = new_lt - c.tangent_impulse.norm();
                c.tangent_impulse = t * new_lt;
                let imp_t = t * applied_t;
                bodies[c.a].vel -= imp_t * bodies[c.a].inv_mass;
                bodies[c.b].vel += imp_t * bodies[c.b].inv_mass;
            }
        }
    }
}

/// 用伪速度(split impulse)求解位置修正,使物体分离而不污染真实速度。
///
/// `pseudo` 为每个 body 的伪速度(初值 0),求解后调用方用 `pos += pseudo*dt` 修正位置。
/// 约束目标:沿法线的相对伪速度 = `beta * depth / dt`(把穿透在 dt 内消除)。
pub fn solve_position<T: RealField + Copy>(
    bodies: &[Body<T>],
    constraints: &[ContactConstraint<T>],
    pseudo: &mut [Vec3<T>],
    beta_over_dt: T,
) {
    // 位置层独立累积冲量
    let mut lambda: Vec<T> = vec![T::zero(); constraints.len()];
    for _ in 0..constraints.len().max(1) * 4 {
        let mut max_vn = T::zero();
        for (idx, c) in constraints.iter().enumerate() {
            let n = c.contact.normal;
            let inv_sum = bodies[c.a].inv_mass + bodies[c.b].inv_mass;
            if inv_sum <= T::zero() {
                continue;
            }
            let rel_p = pseudo[c.b] - pseudo[c.a];
            let vn = rel_p.dot(&n);
            // 目标:相对伪速度应等于 beta*depth/dt(正值表示分离)
            let target = beta_over_dt * c.contact.depth;
            let dlambda = (target - vn) / inv_sum;
            let new_lambda = if lambda[idx] + dlambda > T::zero() {
                lambda[idx] + dlambda
            } else {
                T::zero()
            };
            let applied = new_lambda - lambda[idx];
            lambda[idx] = new_lambda;
            pseudo[c.a] -= n * applied * bodies[c.a].inv_mass;
            pseudo[c.b] += n * applied * bodies[c.b].inv_mass;
            let vn_now = (pseudo[c.b] - pseudo[c.a]).dot(&n);
            let diff = (target - vn_now).abs();
            max_vn = if diff > max_vn { diff } else { max_vn };
        }
        if max_vn < T::from_f64(1e-4).unwrap() {
            break;
        }
    }
}
