//! 约束求解:顺序冲量法(Sequential Impulse, 累积冲量版)。
//!
//! 对每个接触迭代求解法向冲量与摩擦冲量,使物体间不穿透且符合恢复系数。
//! 采用标准 SI 的**累积冲量**形式:每次迭代施加增量 dλ 并把 λ 钳制为非负,
//! 避免非累积版本在堆叠场景中产生的反向过冲与抖动。

use nalgebra::{Matrix3, Vector3};
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

/// 3×3 有效质量矩阵(法向 + 两个切向),供 D3 块求解器(`solve_velocity`)使用。
///
/// 相对速度由冲量 P 引起的变化满足 `Δv = K · P`,其中
/// `K = Σ_i (1/m_i)·I + I_i⁻¹ (r_i r_iᵀ − |r_i|²·I)`(i=a,b),
/// 即每个体的"速度/冲量"有效质量矩阵。块求解器在 (n, t1, t2) 正交基下求解,
/// 故把世界系 K 投影到该基:`K_basis = Bᵀ · K_world · B`(B 的列即 n/t1/t2)。
fn build_k3<T: RealField + Copy>(
    ba: &Body<T>,
    bb: &Body<T>,
    ra: &Vec3<T>,
    rb: &Vec3<T>,
    n: &Vec3<T>,
    t1: &Vec3<T>,
    t2: &Vec3<T>,
) -> Matrix3<T> {
    let inv_ma = ba.eff_inv_mass();
    let inv_mb = bb.eff_inv_mass();
    let ia = ba.inv_inertia_world();
    let ib = bb.inv_inertia_world();
    let ra2 = ra.dot(ra);
    let rb2 = rb.dot(rb);
    // 体 a: I⁻¹(r rᵀ − |r|²I) + (1/m)I
    let mut ka = ia * (ra * ra.transpose()) - ia * ra2;
    ka += Mat3::identity() * inv_ma;
    // 体 b:同上
    let mut kb = ib * (rb * rb.transpose()) - ib * rb2;
    kb += Mat3::identity() * inv_mb;
    let k_world = ka + kb;
    // 投影到 (n, t1, t2) 基
    let b = Matrix3::new(
        n.x, t1.x, t2.x,
        n.y, t1.y, t2.y,
        n.z, t1.z, t2.z,
    );
    b.transpose() * k_world * b
}

/// 由法向 `n` 构造一组单位正交切向基 (t1, t2),其中 `t2 = n × t1`。
fn tangent_basis<T: RealField + Copy>(n: &Vec3<T>) -> (Vec3<T>, Vec3<T>) {
    // 选一个与 n 不平行的参考轴构造 t1。
    let ref1 = Vec3::new(T::one(), T::zero(), T::zero());
    let mut t1 = ref1 - n * n.dot(&ref1);
    if t1.norm() < T::from_f64(1e-9).unwrap() {
        // n 几乎沿 X:改用 Y 轴。
        let ref2 = Vec3::new(T::zero(), T::one(), T::zero());
        t1 = ref2 - n * n.dot(&ref2);
    }
    t1 = t1.normalize();
    let t2 = n.cross(&t1);
    (t1, t2)
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

    // D3 块求解器(Box2D 同款):法向 + 两个切向摩擦方向耦合进 3×3 有效质量矩阵,
    // 在**单一迭代循环**内联合求解(而非先解完法向再解摩擦)。好处:摩擦锥在每步迭代都
    // 基于最新法向冲量夹紧,静止堆叠/斜坡上物体更稳定,不再出现法向与摩擦相互拉扯的抖动。
    for _ in 0..params.iterations {
        for c in constraints.iter_mut() {
            let n = c.contact.normal;
            // 由法向构造正交切向基(Contact 只存法向,切向基数值求解时现算)。
            let (t1, t2) = tangent_basis(&n);

            let inv_sum = bodies[c.a].eff_inv_mass() + bodies[c.b].eff_inv_mass();
            if inv_sum <= T::zero() {
                continue;
            }

            let ra = c.contact.point - bodies[c.a].pos;
            let rb = c.contact.point - bodies[c.b].pos;
            // 3×3 有效质量矩阵 K(Box2D 块求解核心),来自 build_K3:
            //   K = Σ 1/m_i * (E₃ - [r×](I_i⁻¹)[r×]ᵀ) 对 i=a,b
            // 各列对应 (法向, 切向1, 切向2)。
            let k3 = build_k3(&bodies[c.a], &bodies[c.b], &ra, &rb, &n, &t1, &t2);
            let mass = Matrix3::new(
                k3[(0, 0)], k3[(0, 1)], k3[(0, 2)],
                k3[(1, 0)], k3[(1, 1)], k3[(1, 2)],
                k3[(2, 0)], k3[(2, 1)], k3[(2, 2)],
            );
            // 奇异(K 退化,如运动学体只一个运动分量)→ 跳过本约束。
            let inv_mass = match mass.try_inverse() {
                Some(im) => im,
                None => continue,
            };

            // 累积冲量初值(warm-start 已把上一帧值打入速度)
            let mut pn = c.normal_impulse;
            let mut pt1 = c.tangent_impulse.dot(&t1);
            let mut pt2 = c.tangent_impulse.dot(&t2);

            // 相对速度(含角速度项)
            let va = bodies[c.a].vel + bodies[c.a].ang_vel.cross(&ra);
            let vb = bodies[c.b].vel + bodies[c.b].ang_vel.cross(&rb);
            let rel_v = vb - va;
            let vn = rel_v.dot(&n);
            let vt1 = rel_v.dot(&t1);
            let vt2 = rel_v.dot(&t2);

            // 3×3 块求解:目标冲量 = -K⁻¹ · v_rel(把相对速度消除;恢复系数在法向侧加)
            let col = Vector3::new((T::one() + e) * vn, vt1, vt2);
            let p = inv_mass * col;
            let mut d_pn = -p[0];
            let mut d_pt1 = -p[1];
            let mut d_pt2 = -p[2];

            // 法向夹紧(累积非负)
            let new_pn = pn + d_pn;
            if new_pn < T::zero() {
                d_pn = -pn;
                pn = T::zero();
            } else {
                pn = new_pn;
            }

            // 摩擦锥夹紧(基于最新法向冲量,块求解核心):
            // 切向冲量幅值 |(pt1,pt2)| <= μ * pn
            let mut new_pt1 = pt1 + d_pt1;
            let mut new_pt2 = pt2 + d_pt2;
            let max_f = mu * pn;
            let fmag2 = new_pt1 * new_pt1 + new_pt2 * new_pt2;
            if fmag2 > max_f * max_f && fmag2 > T::zero() {
                let scale = max_f / fmag2.sqrt();
                new_pt1 *= scale;
                new_pt2 *= scale;
                d_pt1 = new_pt1 - pt1;
                d_pt2 = new_pt2 - pt2;
                pt1 = new_pt1;
                pt2 = new_pt2;
            } else {
                pt1 = new_pt1;
                pt2 = new_pt2;
            }

            // 应用本次增量冲量
            let pvec = n * d_pn + t1 * d_pt1 + t2 * d_pt2;
            bodies[c.a].apply_impulse_at(-pvec, ra);
            bodies[c.b].apply_impulse_at(pvec, rb);

            // 写回累积冲量(供 warm-start 缓存 / 渲染 / 位置层使用)
            c.normal_impulse = pn;
            c.tangent_impulse = t1 * pt1 + t2 * pt2;
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
