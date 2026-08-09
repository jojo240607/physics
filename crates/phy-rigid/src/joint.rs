//! 关节约束(M18):把若干刚体连成链条/摆/机械结构的等式约束。
//!
//! 采用与 `solver.rs` 接触求解**同构**的顺序冲量法(累积冲量版),在 `RigidWorld::step`
//! 的速度层与位置层各求解一遍,使约束与碰撞共存且不互相破坏。
//!
//! 当前支持的关节(均为纯位置等式约束,不约束转动 —— 球窝本就允许自由旋转):
//! - `Ball`(球窝):两刚体各自的局部锚点 `pa`/`pb` 的世界位置应重合(3 自由度转动放开)。
//!   链路/摆/机械臂的基础。
//! - `Distance`(定长杆):两锚点世界距离应保持为 `rest`(单标量约束)。绳/杆/弹簧链。
//!
//! 约束在速度层消除相对漂移速度、在位置层用 split-impulse 伪速度把残余误差投影掉
//! (不污染真实速度,与接触的位置修正一致)。

use phy_math::{RealField, Vec3};
use serde::de::DeserializeOwned;
use serde::{Deserialize, Serialize};

use crate::shape::Body;

/// 关节参数(刚度:0=软约束,1=硬约束;用于位置层投影比例)。
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
#[serde(bound = "T: RealField + Copy + Serialize + DeserializeOwned")]
pub struct JointParams<T: RealField> {
    /// 位置层投影比例(类比接触求解的 `beta`):每步纠正多少比例的距离误差。
    pub beta: T,
}

impl<T: RealField> Default for JointParams<T> {
    fn default() -> Self {
        Self {
            beta: T::from_f64(0.2).unwrap(),
        }
    }
}

/// 一个关节在求解期的状态。
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(bound = "T: RealField + Copy + Serialize + DeserializeOwned")]
pub struct JointConstraint<T: RealField + Copy> {
    /// 关联刚体 a 索引。
    pub a: usize,
    /// 关联刚体 b 索引。
    pub b: usize,
    /// 关节几何。
    pub joint: Joint<T>,
    /// 累积冲量(标量;Ball 视作沿误差方向的累积,这里统一用标量沿当前误差方向施加)。
    pub lambda: T,
}

/// 关节几何定义。
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
#[serde(bound = "T: RealField + Copy + Serialize + DeserializeOwned")]
pub enum Joint<T: RealField + Copy> {
    /// 球窝:局部锚点 `pa`(a 上)/`pb`(b 上)世界位置应重合。
    Ball {
        /// a 上的局部锚点。
        #[serde(with = "crate::shape::serde_geom")]
        pa: Vec3<T>,
        /// b 上的局部锚点。
        #[serde(with = "crate::shape::serde_geom")]
        pb: Vec3<T>,
    },
    /// 定长杆:两锚点世界距离保持为 `rest`。
    Distance {
        /// a 上的局部锚点。
        #[serde(with = "crate::shape::serde_geom")]
        pa: Vec3<T>,
        /// b 上的局部锚点。
        #[serde(with = "crate::shape::serde_geom")]
        pb: Vec3<T>,
        /// 目标距离。
        rest: T,
    },
}

impl<T: RealField + Copy> Joint<T> {
    /// 计算两锚点的世界坐标(局部锚经刚体位姿变换)。
    fn world_anchors(&self, bodies: &[Body<T>], a: usize, b: usize) -> (Vec3<T>, Vec3<T>) {
        let (pa, pb) = match self {
            Joint::Ball { pa, pb } => (*pa, *pb),
            Joint::Distance { pa, pb, .. } => (*pa, *pb),
        };
        let wa = bodies[a].pos + bodies[a].rot * pa;
        let wb = bodies[b].pos + bodies[b].rot * pb;
        (wa, wb)
    }

    /// 约束误差向量 `C = wa - wb`(Distance 下取其沿连线方向的分量方向)。
    /// 返回 `(error_vec, error_len)`:误差方向与带符号误差长度。
    fn error(&self, bodies: &[Body<T>], a: usize, b: usize) -> (Vec3<T>, T) {
        let (wa, wb) = self.world_anchors(bodies, a, b);
        match self {
            Joint::Ball { .. } => {
                let e = wa - wb;
                (e, e.norm())
            }
            Joint::Distance { rest, .. } => {
                let d = wa - wb;
                let len = d.norm();
                if len > T::from_f64(1e-9).unwrap() {
                    let dir = d / len;
                    // 带符号误差:当前长度 - 目标长度。
                    (dir, len - *rest)
                } else {
                    (Vec3::zeros(), T::zero())
                }
            }
        }
    }
}

impl<T: RealField + Copy> JointConstraint<T> {
    pub fn new(a: usize, b: usize, joint: Joint<T>) -> Self {
        Self {
            a,
            b,
            joint,
            lambda: T::zero(),
        }
    }
}

/// 速度层求解:消除沿约束误差方向的相对速度(顺序冲量,累积冲量版)。
pub fn solve_joints_velocity<T: RealField + Copy>(
    bodies: &mut [Body<T>],
    joints: &mut [JointConstraint<T>],
    iterations: usize,
) {
    for _ in 0..iterations {
        for j in joints.iter_mut() {
            let (dir, err_len) = j.joint.error(bodies, j.a, j.b);
            let dlen = dir.norm();
            if dlen < T::from_f64(1e-9).unwrap() {
                continue; // 无定义方向(两锚点重合且为 Distance)。
            }
            let n = dir / dlen; // 单位约束轴。
            let inv_sum = bodies[j.a].inv_mass + bodies[j.b].inv_mass;
            if inv_sum <= T::zero() {
                continue;
            }
            // 当前沿约束轴的相对速度。
            let rel_v = bodies[j.b].vel - bodies[j.a].vel;
            let vn = rel_v.dot(&n);
            // 增量冲量:消除该相对速度(无恢复系数,硬约束)。
            let dlambda = -vn / inv_sum;
            let new_lambda = if j.lambda + dlambda > T::zero() {
                j.lambda + dlambda
            } else {
                T::zero()
            };
            let applied = new_lambda - j.lambda;
            j.lambda = new_lambda;
            let imp = n * applied;
            bodies[j.a].vel -= imp * bodies[j.a].inv_mass;
            bodies[j.b].vel += imp * bodies[j.b].inv_mass;
            // err_len 仅用于位置层,此处未用,避免未使用警告。
            let _ = err_len;
        }
    }
}

/// 位置层求解:split-impulse 伪速度投影,把残余距离误差消除而不污染真实速度。
///
/// 关节仅约束平动(`ang_pseudo` 透传但本函数不修改它)。
/// 调用方用 `pos += pseudo*dt` 修正位置。
pub fn solve_joints_position<T: RealField + Copy>(
    bodies: &[Body<T>],
    joints: &[JointConstraint<T>],
    pseudo: &mut [Vec3<T>],
    _ang_pseudo: &mut [Vec3<T>],
    beta_over_dt: T,
) {
    // 位置层独立累积冲量。
    let mut lambda: Vec<T> = vec![T::zero(); joints.len()];
    for _ in 0..joints.len().max(1) * 4 {
        let mut max_err = T::zero();
        for (idx, j) in joints.iter().enumerate() {
            let (dir, err_len) = j.joint.error(bodies, j.a, j.b);
            let dlen = dir.norm();
            if dlen < T::from_f64(1e-9).unwrap() {
                continue;
            }
            let n = dir / dlen;
            let inv_sum = bodies[j.a].inv_mass + bodies[j.b].inv_mass;
            if inv_sum <= T::zero() {
                continue;
            }
            // 当前沿约束轴的相对伪速度。
            let rel_p = pseudo[j.b] - pseudo[j.a];
            let vn = rel_p.dot(&n);
            // 目标:伪速度应把误差在 dt 内消除 —— 沿约束轴的目标相对伪速度 = beta*err_len/dt。
            let target = beta_over_dt * err_len;
            let dlambda = (target - vn) / inv_sum;
            let new_lambda = if lambda[idx] + dlambda > T::zero() {
                lambda[idx] + dlambda
            } else {
                T::zero()
            };
            let applied = new_lambda - lambda[idx];
            lambda[idx] = new_lambda;
            pseudo[j.a] -= n * applied * bodies[j.a].inv_mass;
            pseudo[j.b] += n * applied * bodies[j.b].inv_mass;
            let vn_now = (pseudo[j.b] - pseudo[j.a]).dot(&n);
            let diff = (target - vn_now).abs();
            max_err = if diff > max_err { diff } else { max_err };
        }
        if max_err < T::from_f64(1e-4).unwrap() {
            break;
        }
    }
}
