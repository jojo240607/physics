//! 约束求解:顺序冲量法(Sequential Impulse, 累积冲量版)。
//!
//! 对每个接触迭代求解法向冲量与摩擦冲量,使物体间不穿透且符合恢复系数。
//! 采用标准 SI 的**累积冲量**形式:每次迭代施加增量 dλ 并把 λ 钳制为非负,
//! 避免非累积版本在堆叠场景中产生的反向过冲与抖动。

use phy_math::{Mat3, RealField, Vec3};
use serde::de::DeserializeOwned;
use serde::{Deserialize, Serialize};

use crate::contact::Contact;
use crate::shape::Body;

/// 求解参数。
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(bound = "T: RealField + Copy + Serialize + DeserializeOwned")]
pub struct SolverParams<T: RealField> {
    /// 恢复系数 (0 = 完全非弹性, 1 = 完全弹性)。
    pub restitution: T,
    /// 摩擦系数。
    pub friction: T,
    /// 速度求解**法向**迭代次数。
    pub iterations: usize,
    /// 速度求解**摩擦**迭代次数(B1:摩擦收敛独立于法向,避免堆叠时摩擦欠收敛)。
    /// 设为 0 时回退到与 `iterations` 相同。
    pub friction_iterations: usize,
    /// 位置修正允许的残余穿透容差(slop,米)。B1:静止堆叠时不为每帧微小穿透
    /// 全部修正,避免"过度弹开"抖动。仅修正 `depth - slop` 部分。
    pub position_slop: T,
    /// CCD 子步上限(连续碰撞检测:每步把位置积分拆成至多这么多个子步以避免隧穿)。
    /// 设 0 关闭 CCD(退化为离散碰撞,高速会隧穿)。
    pub ccd_max_substeps: usize,
    /// B1 休眠:线速度平方阈值(低于此且角速度也低视为"近静止")。
    pub sleep_lin_vel2: T,
    /// B1 休眠:角速度平方阈值(rad²/s²)。
    pub sleep_ang_vel2: T,
    /// B1 休眠:持续近静止达此时长(秒)后置 sleeping=true。
    pub sleep_time: T,
}

impl<T: RealField> Default for SolverParams<T> {
    fn default() -> Self {
        Self {
            restitution: T::from_f64(0.0).unwrap(),
            friction: T::from_f64(0.5).unwrap(),
            iterations: 20,
            friction_iterations: 20,
            position_slop: T::from_f64(1e-3).unwrap(), // 1mm 残余穿透容差
            ccd_max_substeps: 8,
            sleep_lin_vel2: T::from_f64(1e-2).unwrap(), // 线速 ~0.1 m/s
            sleep_ang_vel2: T::from_f64(1e-2).unwrap(), // 角速 ~0.1 rad/s
            sleep_time: T::from_f64(0.5).unwrap(),      // 持续 0.5s 静止即休眠
        }
    }
}

/// 一个接触在求解期的状态(含参与 body 索引、接触几何、累积冲量)。
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(bound = "T: RealField + Copy + Serialize + DeserializeOwned")]
pub struct ContactConstraint<T: RealField + Copy> {
    pub a: usize,
    pub b: usize,
    pub contact: Contact<T>,
    /// 累积法向冲量(非负)。
    pub normal_impulse: T,
    /// 累积切向冲量(向量)。
    #[serde(with = "crate::shape::serde_geom")]
    pub tangent_impulse: Vec3<T>,
    /// 是否已做 warm-start(初值已从上一帧恢复并施加)。
    pub warm_started: bool,
}

impl<T: RealField + Copy> ContactConstraint<T> {
    pub fn new(a: usize, b: usize, contact: Contact<T>) -> Self {
        Self {
            a,
            b,
            contact,
            normal_impulse: T::zero(),
            tangent_impulse: Vec3::zeros(),
            warm_started: false,
        }
    }
}

/// 用顺序冲量法求解所有接触,就地修改 bodies 的线速度与角速度。
///
/// 含转动:接触点相对质心的力臂 `r` 产生角冲量,有效质量计入 `nᵀ·I⁻¹·(r×n)` 项。
pub fn solve_velocity<T: RealField + Copy>(
    bodies: &mut [Body<T>],
    constraints: &mut [ContactConstraint<T>],
    params: &SolverParams<T>,
) {
    let e = params.restitution;
    let mu = params.friction;

    // D3 warm-start:把上一帧累积的接触冲量作为初值打入速度,减少迭代收敛次数、
    // 消除静止堆叠抖动(Box2D 同款)。仅对 `warm_started` 的约束施加一次(迭代前)。
    for c in constraints.iter_mut() {
        if !c.warm_started {
            continue;
        }
        let n = c.contact.normal;
        let ra = c.contact.point - bodies[c.a].pos;
        let rb = c.contact.point - bodies[c.b].pos;
        let pn = n * c.normal_impulse;
        bodies[c.a].apply_impulse_at(-pn, ra);
        bodies[c.b].apply_impulse_at(pn, rb);
        let pt = c.tangent_impulse;
        bodies[c.a].apply_impulse_at(-pt, ra);
        bodies[c.b].apply_impulse_at(pt, rb);
    }

    // B1: 法向求解在主迭代中完成。
    for _ in 0..params.iterations {
        for c in constraints.iter_mut() {
            let n = c.contact.normal;
            let ia = bodies[c.a].inv_inertia_world();
            let ib = bodies[c.b].inv_inertia_world();
            let ra = c.contact.point - bodies[c.a].pos;
            let rb = c.contact.point - bodies[c.b].pos;
            // 有效质量(法向): m_eff = 1 / (effInvMa + effInvMb + nᵀ(Ia⁻¹(ra×n)×ra + Ib⁻¹(rb×n)×rb))
            // B2: 运动学体 eff_inv_mass=0,不被接触冲量驱动(但按自身 vel 主动移动并推开动态体)。
            let inv_sum = bodies[c.a].eff_inv_mass() + bodies[c.b].eff_inv_mass();
            if inv_sum <= T::zero() {
                continue;
            }
            let angular_term = |r: Vec3<T>, inv: Mat3<T>| -> T {
                let rn = r.cross(&n);
                let t = inv * rn;
                rn.dot(&t)
            };
            let denom = inv_sum + angular_term(ra, ia) + angular_term(rb, ib);
            if denom <= T::from_f64(1e-12).unwrap() {
                continue;
            }

            // ---- 法向(累积增量) ----
            // 相对速度含角速度项: v_rel = (vb + ωb×rb) - (va + ωa×ra)
            let va = bodies[c.a].vel + bodies[c.a].ang_vel.cross(&ra);
            let vb = bodies[c.b].vel + bodies[c.b].ang_vel.cross(&rb);
            let rel_v = vb - va;
            let vn = rel_v.dot(&n);
            // 增量冲量:把法向相对速度消除(含恢复系数)
            let dlambda = -(T::one() + e) * vn / denom;
            let new_lambda = if c.normal_impulse + dlambda > T::zero() {
                c.normal_impulse + dlambda
            } else {
                T::zero()
            };
            let applied = new_lambda - c.normal_impulse;
            c.normal_impulse = new_lambda;
            let imp = n * applied;
            bodies[c.a].apply_impulse_at(-imp, ra);
            bodies[c.b].apply_impulse_at(imp, rb);
        }
    }

    // B1: 摩擦独立子迭代(收敛上限 = friction_iterations)。法向冲量已固定,
    // 此阶段只调整切向冲量使其收敛到库仑锥,不回写体速度之外的新法向分量。
    let fiter = if params.friction_iterations == 0 {
        params.iterations
    } else {
        params.friction_iterations
    };
    for _ in 0..fiter {
        for c in constraints.iter_mut() {
            let n = c.contact.normal;
            let ia = bodies[c.a].inv_inertia_world();
            let ib = bodies[c.b].inv_inertia_world();
            let ra = c.contact.point - bodies[c.a].pos;
            let rb = c.contact.point - bodies[c.b].pos;
            let inv_sum = bodies[c.a].eff_inv_mass() + bodies[c.b].eff_inv_mass();
            if inv_sum <= T::zero() {
                continue;
            }
            let angular_term = |r: Vec3<T>, inv: Mat3<T>| -> T {
                let rn = r.cross(&n);
                let t = inv * rn;
                rn.dot(&t)
            };
            let va = bodies[c.a].vel + bodies[c.a].ang_vel.cross(&ra);
            let vb = bodies[c.b].vel + bodies[c.b].ang_vel.cross(&rb);
            let rel_v = vb - va;
            let vn_now = rel_v.dot(&n);
            let tangent = rel_v - n * vn_now;
            let tlen = tangent.norm();
            if tlen > T::from_f64(1e-9).unwrap() {
                let t = tangent / tlen;
                let vt = rel_v.dot(&t);
                let denom_t = inv_sum
                    + angular_term(ra, ia)
                    + angular_term(rb, ib);
                let dlt = if denom_t > T::from_f64(1e-12).unwrap() {
                    -vt / denom_t
                } else {
                    T::zero()
                };
                // 库仑摩擦: |λ_t| <= μ * λ_n(法向冲量已在此阶段固定)
                let max_lt = mu * c.normal_impulse;
                let cur_lt = c.tangent_impulse.norm();
                let new_lt = if cur_lt + dlt.abs() > max_lt {
                    if dlt > T::zero() {
                        max_lt
                    } else {
                        -max_lt
                    }
                } else {
                    cur_lt + dlt
                };
                let applied_t = new_lt - cur_lt;
                c.tangent_impulse = t * new_lt;
                let imp_t = t * applied_t;
                bodies[c.a].apply_impulse_at(-imp_t, ra);
                bodies[c.b].apply_impulse_at(imp_t, rb);
            }
        }
    }
}

/// 用伪速度(split impulse)求解位置修正,使物体分离而不污染真实速度。
///
/// `pseudo` / `ang_pseudo` 为每个 body 的线/角伪速度(初值 0),求解后调用方用
/// `pos += pseudo*dt` / 姿态 += 0.5*(0,ang_pseudo)⊗q*dt 修正。
/// 约束目标:沿法线的相对伪速度 = `beta * depth / dt`(把穿透在 dt 内消除)。
pub fn solve_position<T: RealField + Copy>(
    bodies: &[Body<T>],
    constraints: &[ContactConstraint<T>],
    pseudo: &mut [Vec3<T>],
    ang_pseudo: &mut [Vec3<T>],
    beta_over_dt: T,
    slop: T,
) {
    // 位置层独立累积冲量
    let mut lambda: Vec<T> = vec![T::zero(); constraints.len()];
    for _ in 0..constraints.len().max(1) * 4 {
        let mut max_vn = T::zero();
        for (idx, c) in constraints.iter().enumerate() {
            let n = c.contact.normal;
            let ia = bodies[c.a].inv_inertia_world();
            let ib = bodies[c.b].inv_inertia_world();
            let ra = c.contact.point - bodies[c.a].pos;
            let rb = c.contact.point - bodies[c.b].pos;
            let inv_sum = bodies[c.a].eff_inv_mass() + bodies[c.b].eff_inv_mass();
            if inv_sum <= T::zero() {
                continue;
            }
            let angular_term = |r: Vec3<T>, inv: Mat3<T>| -> T {
                let rn = r.cross(&n);
                let t = inv * rn;
                rn.dot(&t)
            };
            let denom = inv_sum + angular_term(ra, ia) + angular_term(rb, ib);
            if denom <= T::from_f64(1e-12).unwrap() {
                continue;
            }
            // 相对伪速度含角伪速度项。
            let va = pseudo[c.a] + ang_pseudo[c.a].cross(&ra);
            let vb = pseudo[c.b] + ang_pseudo[c.b].cross(&rb);
            let rel_p = vb - va;
            let vn = rel_p.dot(&n);
            // 目标:相对伪速度应等于 beta*(depth - slop)/dt(正值表示分离)。
            // B1: 残余穿透 < slop 的不修正,避免静止堆叠逐帧过度弹开而抖动。
            let corr = if c.contact.depth > slop {
                c.contact.depth - slop
            } else {
                T::zero()
            };
            let target = beta_over_dt * corr;
            let dlambda = (target - vn) / denom;
            let new_lambda = if lambda[idx] + dlambda > T::zero() {
                lambda[idx] + dlambda
            } else {
                T::zero()
            };
            let applied = new_lambda - lambda[idx];
            lambda[idx] = new_lambda;
            let imp = n * applied;
            // 线伪速度(B2:运动学体 eff_inv_mass=0,不参与穿透修正,位置只由自身 vel 决定)
            pseudo[c.a] -= imp * bodies[c.a].eff_inv_mass();
            pseudo[c.b] += imp * bodies[c.b].eff_inv_mass();
            // 角伪速度: ω̃ += I⁻¹ (r × imp)
            let ta = ia * ra.cross(&(-imp));
            let tb = ib * rb.cross(&imp);
            ang_pseudo[c.a] += ta;
            ang_pseudo[c.b] += tb;
            let va_now = pseudo[c.a] + ang_pseudo[c.a].cross(&ra);
            let vb_now = pseudo[c.b] + ang_pseudo[c.b].cross(&rb);
            let vn_now = (vb_now - va_now).dot(&n);
            let diff = (target - vn_now).abs();
            max_vn = if diff > max_vn { diff } else { max_vn };
        }
        if max_vn < T::from_f64(1e-4).unwrap() {
            break;
        }
    }
}
