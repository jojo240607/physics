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

use phy_math::{Mat3, RealField, Vec3};
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
    /// 焊点:两刚体相对位姿完全锁死(6 DOF 全约束:3 平动 + 3 转动)。
    /// 用于把碎片焊回、强化结构、或构造刚性组合件。
    Weld {
        /// a 上的局部锚点。
        #[serde(with = "crate::shape::serde_geom")]
        pa: Vec3<T>,
        /// b 上的局部锚点。
        #[serde(with = "crate::shape::serde_geom")]
        pb: Vec3<T>,
    },
    /// 铰链:两锚点重合 + 两铰链轴对齐,沿轴自由旋转(1 转动 DOF,约束 3 平动 + 2 转动)。
    /// 车辆车轮、门、齿轮、机械臂关节的基础。
    /// 可选 `motor_vel`(目标角速度,rad/s)驱动沿轴旋转,`max_motor_torque` 限制最大驱动扭矩。
    Hinge {
        /// a 上的局部锚点。
        #[serde(with = "crate::shape::serde_geom")]
        pa: Vec3<T>,
        /// b 上的局部锚点。
        #[serde(with = "crate::shape::serde_geom")]
        pb: Vec3<T>,
        /// a 上的局部铰链轴(单位向量,定义旋转轴)。
        #[serde(with = "crate::shape::serde_geom")]
        axis_a: Vec3<T>,
        /// b 上的局部铰链轴(单位向量)。
        #[serde(with = "crate::shape::serde_geom")]
        axis_b: Vec3<T>,
        /// Motor 目标角速度(rad/s),0 = 无驱动(纯自由铰链)。
        motor_vel: T,
        /// Motor 最大驱动扭矩(冲量上限)。
        max_motor_torque: T,
    },
    /// 滑块(棱柱):两锚点沿公共轴对齐(3 转动锁死)+ 沿轴 1 平动自由(约束 2 垂直平动)。
    /// 液压杆、抽屉、活塞、线性滑轨。
    /// 可选 `motor_vel`(目标线速度, m/s)驱动沿轴移动,`max_motor_force` 限制最大驱动力。
    Prismatic {
        /// a 上的局部锚点。
        #[serde(with = "crate::shape::serde_geom")]
        pa: Vec3<T>,
        /// b 上的局部锚点。
        #[serde(with = "crate::shape::serde_geom")]
        pb: Vec3<T>,
        /// a 上的局部滑动轴(单位向量)。
        #[serde(with = "crate::shape::serde_geom")]
        axis_a: Vec3<T>,
        /// Motor 目标线速度(m/s),0 = 无驱动(纯自由滑块)。
        motor_vel: T,
        /// Motor 最大驱动力(冲量上限)。
        max_motor_force: T,
    },
}

impl<T: RealField + Copy> Joint<T> {
    /// 计算两锚点的世界坐标(局部锚经刚体位姿变换)。
    fn world_anchors(&self, bodies: &[Body<T>], a: usize, b: usize) -> (Vec3<T>, Vec3<T>) {
        let (pa, pb) = match self {
            Joint::Ball { pa, pb } => (*pa, *pb),
            Joint::Distance { pa, pb, .. } => (*pa, *pb),
            Joint::Weld { pa, pb } => (*pa, *pb),
            Joint::Hinge { pa, pb, .. } => (*pa, *pb),
            Joint::Prismatic { pa, pb, .. } => (*pa, *pb),
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
            Joint::Weld { .. } => {
                let e = wa - wb;
                (e, e.norm())
            }
            Joint::Hinge { .. } => {
                let e = wa - wb;
                (e, e.norm())
            }
            Joint::Prismatic { .. } => {
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
///
/// - `Ball` / `Distance`:标量平动约束(锚点分离速度沿误差方向消零)。
/// - `Weld`:锁死 6 DOF —— 3 平动(点约束,含角速度项)+ 3 转动(角对齐,消除相对角速度)。
pub fn solve_joints_velocity<T: RealField + Copy>(
    bodies: &mut [Body<T>],
    joints: &mut [JointConstraint<T>],
    iterations: usize,
) {
    for _ in 0..iterations {
        for j in joints.iter_mut() {
            match j.joint {
                Joint::Weld { pa, pb } => {
                    solve_weld_velocity(bodies, j.a, j.b, pa, pb);
                    continue;
                }
                Joint::Hinge { pa, pb, axis_a, axis_b, motor_vel, max_motor_torque } => {
                    solve_hinge_velocity(
                        bodies, j.a, j.b, pa, pb, axis_a, axis_b, motor_vel, max_motor_torque,
                    );
                    continue;
                }
                Joint::Prismatic { pa, pb, axis_a, motor_vel, max_motor_force } => {
                    solve_prismatic_velocity(
                        bodies, j.a, j.b, pa, pb, axis_a, motor_vel, max_motor_force,
                    );
                    continue;
                }
                _ => {}
            }
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

/// Weld 速度求解:锁死 6 DOF。
/// 3 个平动点约束(锚点分离速度消零,含角速度项)+ 3 个转动约束(相对角速度消零)。
fn solve_weld_velocity<T: RealField + Copy>(
    bodies: &mut [Body<T>],
    a: usize,
    b: usize,
    pa: Vec3<T>,
    pb: Vec3<T>,
) {
    let wa = bodies[a].pos + bodies[a].rot * pa;
    let wb = bodies[b].pos + bodies[b].rot * pb;
    let ra = wa - bodies[a].pos;
    let rb = wb - bodies[b].pos;
    let ia = bodies[a].inv_inertia_world();
    let ib = bodies[b].inv_inertia_world();

    // 基轴(世界系)。
    let basis = [
        Vec3::new(T::one(), T::zero(), T::zero()),
        Vec3::new(T::zero(), T::one(), T::zero()),
        Vec3::new(T::zero(), T::zero(), T::one()),
    ];

    // 3 平动点约束:消除锚点沿每个基轴的分离速度(含角速度项)。
    for &n in basis.iter() {
        let inv_ma = bodies[a].eff_inv_mass();
        let inv_mb = bodies[b].eff_inv_mass();
        if inv_ma + inv_mb <= T::zero() {
            continue;
        }
        let ang = |r: Vec3<T>, inv: Mat3<T>| -> T {
            let rn = r.cross(&n);
            let t = inv * rn;
            rn.dot(&t)
        };
        let inv_sum = inv_ma + inv_mb + ang(ra, ia) + ang(rb, ib);
        if inv_sum <= T::from_f64(1e-12).unwrap() {
            continue;
        }
        let va = bodies[a].vel + bodies[a].ang_vel.cross(&ra);
        let vb = bodies[b].vel + bodies[b].ang_vel.cross(&rb);
        let vn = (vb - va).dot(&n);
        let dlambda = -vn / inv_sum;
        let imp = n * dlambda;
        bodies[a].apply_impulse_at(-imp, ra);
        bodies[b].apply_impulse_at(imp, rb);
    }

    // 3 转动约束:消除沿每个基轴的相对角速度。
    for &n in basis.iter() {
        let inv_rot = n.dot(&(ia * n)) + n.dot(&(ib * n));
        if inv_rot <= T::from_f64(1e-12).unwrap() {
            continue;
        }
        let wn = (bodies[b].ang_vel - bodies[a].ang_vel).dot(&n);
        let dlambda = -wn / inv_rot;
        let imp = n * dlambda;
        bodies[a].ang_vel -= ia * imp;
        bodies[b].ang_vel += ib * imp;
    }
}

/// 位置层求解:split-impulse 伪速度投影,把残余距离误差消除而不污染真实速度。
///
/// - `Ball` / `Distance`:仅约束平动。
/// - `Weld`:3 平动点约束 + 3 转动角对齐(修正 `ang_pseudo`),消除残余位姿误差。
/// 调用方用 `pos += pseudo*dt` 与 `rot` 的角伪速度积分修正位置。
pub fn solve_joints_position<T: RealField + Copy>(
    bodies: &[Body<T>],
    joints: &[JointConstraint<T>],
    pseudo: &mut [Vec3<T>],
    ang_pseudo: &mut [Vec3<T>],
    beta_over_dt: T,
) {
    // 位置层独立累积冲量。
    let mut lambda: Vec<T> = vec![T::zero(); joints.len()];
    for _ in 0..joints.len().max(1) * 4 {
        let mut max_err = T::zero();
        for (idx, j) in joints.iter().enumerate() {
            // Weld/Hinge 的转动角对齐:消除相对角伪速度(把两体角对齐)。
            match j.joint {
                Joint::Weld { .. } => {
                    solve_weld_position_angle(bodies, j.a, j.b, ang_pseudo, beta_over_dt);
                }
                Joint::Hinge { axis_a, .. } => {
                    solve_hinge_position_angle(bodies, j.a, j.b, axis_a, ang_pseudo);
                }
                Joint::Prismatic { .. } => {
                    // 滑块锁死 3 转动 → 用 Weld 的 3 基轴角对齐。
                    solve_weld_position_angle(bodies, j.a, j.b, ang_pseudo, beta_over_dt);
                }
                _ => {}
            }
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

/// Hinge 速度求解:约束 3 平动(点约束,锚点重合)+ 2 转动(铰链轴对齐,保留沿轴自由旋转),
/// 可选 Motor 驱动沿轴旋转(目标角速度)。
fn solve_hinge_velocity<T: RealField + Copy>(
    bodies: &mut [Body<T>],
    a: usize,
    b: usize,
    pa: Vec3<T>,
    pb: Vec3<T>,
    axis_a_local: Vec3<T>,
    _axis_b_local: Vec3<T>,
    motor_vel: T,
    max_motor_torque: T,
) {
    let wa = bodies[a].pos + bodies[a].rot * pa;
    let wb = bodies[b].pos + bodies[b].rot * pb;
    let ra = wa - bodies[a].pos;
    let rb = wb - bodies[b].pos;
    let ia = bodies[a].inv_inertia_world();
    let ib = bodies[b].inv_inertia_world();
    // 世界系铰链轴。
    let axis_world = (bodies[a].rot * axis_a_local).normalize();

    // 1) 3 平动点约束(锚点重合)。
    let basis = [
        Vec3::new(T::one(), T::zero(), T::zero()),
        Vec3::new(T::zero(), T::one(), T::zero()),
        Vec3::new(T::zero(), T::zero(), T::one()),
    ];
    for &n in basis.iter() {
        let inv_ma = bodies[a].eff_inv_mass();
        let inv_mb = bodies[b].eff_inv_mass();
        if inv_ma + inv_mb <= T::zero() {
            continue;
        }
        let ang = |r: Vec3<T>, inv: Mat3<T>| -> T {
            let rn = r.cross(&n);
            let t = inv * rn;
            rn.dot(&t)
        };
        let inv_sum = inv_ma + inv_mb + ang(ra, ia) + ang(rb, ib);
        if inv_sum <= T::from_f64(1e-12).unwrap() {
            continue;
        }
        let va = bodies[a].vel + bodies[a].ang_vel.cross(&ra);
        let vb = bodies[b].vel + bodies[b].ang_vel.cross(&rb);
        let vn = (vb - va).dot(&n);
        let dlambda = -vn / inv_sum;
        let imp = n * dlambda;
        bodies[a].apply_impulse_at(-imp, ra);
        bodies[b].apply_impulse_at(imp, rb);
    }

    // 2) 2 转动约束:消除相对角速度沿"垂直于铰链轴"的分量(保留沿轴自由旋转)。
    //    用两个与轴正交的基向量 u1/u2(选与轴不共线的参考轴做叉积)。
    let ref_vec = if axis_world.z.abs() < T::from_f64(0.9).unwrap() {
        Vec3::new(T::zero(), T::zero(), T::one())
    } else {
        Vec3::new(T::one(), T::zero(), T::zero())
    };
    let u1 = ref_vec.cross(&axis_world).normalize();
    let u2 = axis_world.cross(&u1).normalize();
    for &u in [u1, u2].iter() {
        let inv_rot = u.dot(&(ia * u)) + u.dot(&(ib * u));
        if inv_rot <= T::from_f64(1e-12).unwrap() {
            continue;
        }
        let wn = (bodies[b].ang_vel - bodies[a].ang_vel).dot(&u);
        let dlambda = -wn / inv_rot;
        let imp = u * dlambda;
        bodies[a].ang_vel -= ia * imp;
        bodies[b].ang_vel += ib * imp;
    }

    // 3) Motor:沿铰链轴施加目标角速度(驱动两体相对旋转),扭矩受限。
    if motor_vel.abs() > T::zero() && max_motor_torque > T::zero() {
        let inv_rot = axis_world.dot(&(ia * axis_world)) + axis_world.dot(&(ib * axis_world));
        if inv_rot > T::from_f64(1e-12).unwrap() {
            // 当前沿轴的相对角速度 vs 目标。
            let wn = (bodies[b].ang_vel - bodies[a].ang_vel).dot(&axis_world);
            // 需要的角冲量(目标速度 - 当前速度)。
            let mut dlambda = (motor_vel - wn) / inv_rot;
            // 扭矩上限:每步冲量限制在 max_motor_torque*dt 内。这里用单帧上限近似。
            let limit = max_motor_torque;
            if dlambda > limit {
                dlambda = limit;
            } else if dlambda < -limit {
                dlambda = -limit;
            }
            let imp = axis_world * dlambda;
            bodies[a].ang_vel -= ia * imp;
            bodies[b].ang_vel += ib * imp;
        }
    }
}

/// Weld 位置层转动对齐:消除两体沿每个基轴的相对角伪速度,使焊点保持角对齐。
fn solve_weld_position_angle<T: RealField + Copy>(
    bodies: &[Body<T>],
    a: usize,
    b: usize,
    ang_pseudo: &mut [Vec3<T>],
    beta_over_dt: T,
) {
    let ia = bodies[a].inv_inertia_world();
    let ib = bodies[b].inv_inertia_world();
    let basis = [
        Vec3::new(T::one(), T::zero(), T::zero()),
        Vec3::new(T::zero(), T::one(), T::zero()),
        Vec3::new(T::zero(), T::zero(), T::one()),
    ];
    let _ = beta_over_dt;
    for &n in basis.iter() {
        let inv_rot = n.dot(&(ia * n)) + n.dot(&(ib * n));
        if inv_rot <= T::from_f64(1e-12).unwrap() {
            continue;
        }
        let wn = (ang_pseudo[b] - ang_pseudo[a]).dot(&n);
        if wn.abs() < T::from_f64(1e-9).unwrap() {
            continue;
        }
        let dlambda = -wn / inv_rot;
        let imp = n * dlambda;
        ang_pseudo[a] -= ia * imp;
        ang_pseudo[b] += ib * imp;
    }
}

/// Prismatic(滑块)速度求解:约束 2 个垂直滑动轴的平动(保留沿轴 1 平动自由)
/// + 3 转动全锁(两体角对齐),可选 Motor 沿滑动轴驱动。
fn solve_prismatic_velocity<T: RealField + Copy>(
    bodies: &mut [Body<T>],
    a: usize,
    b: usize,
    pa: Vec3<T>,
    pb: Vec3<T>,
    axis_a_local: Vec3<T>,
    motor_vel: T,
    max_motor_force: T,
) {
    let wa = bodies[a].pos + bodies[a].rot * pa;
    let wb = bodies[b].pos + bodies[b].rot * pb;
    let ra = wa - bodies[a].pos;
    let rb = wb - bodies[b].pos;
    let ia = bodies[a].inv_inertia_world();
    let ib = bodies[b].inv_inertia_world();
    let axis_world = (bodies[a].rot * axis_a_local).normalize();

    // 滑动轴的正交基:两个垂直平动约束轴 + 滑动轴本身。
    let ref_vec = if axis_world.z.abs() < T::from_f64(0.9).unwrap() {
        Vec3::new(T::zero(), T::zero(), T::one())
    } else {
        Vec3::new(T::one(), T::zero(), T::zero())
    };
    let u1 = ref_vec.cross(&axis_world).normalize();
    let u2 = axis_world.cross(&u1).normalize();

    // 1) 2 平动约束:沿 u1/u2 消除锚点分离速度(保留沿轴平动)。
    for &n in [u1, u2].iter() {
        let inv_ma = bodies[a].eff_inv_mass();
        let inv_mb = bodies[b].eff_inv_mass();
        if inv_ma + inv_mb <= T::zero() {
            continue;
        }
        let ang = |r: Vec3<T>, inv: Mat3<T>| -> T {
            let rn = r.cross(&n);
            let t = inv * rn;
            rn.dot(&t)
        };
        let inv_sum = inv_ma + inv_mb + ang(ra, ia) + ang(rb, ib);
        if inv_sum <= T::from_f64(1e-12).unwrap() {
            continue;
        }
        let va = bodies[a].vel + bodies[a].ang_vel.cross(&ra);
        let vb = bodies[b].vel + bodies[b].ang_vel.cross(&rb);
        let vn = (vb - va).dot(&n);
        let dlambda = -vn / inv_sum;
        let imp = n * dlambda;
        bodies[a].apply_impulse_at(-imp, ra);
        bodies[b].apply_impulse_at(imp, rb);
    }

    // 2) 3 转动约束:两体完全角对齐(锁死相对旋转)。
    let basis = [
        Vec3::new(T::one(), T::zero(), T::zero()),
        Vec3::new(T::zero(), T::one(), T::zero()),
        Vec3::new(T::zero(), T::zero(), T::one()),
    ];
    for &n in basis.iter() {
        let inv_rot = n.dot(&(ia * n)) + n.dot(&(ib * n));
        if inv_rot <= T::from_f64(1e-12).unwrap() {
            continue;
        }
        let wn = (bodies[b].ang_vel - bodies[a].ang_vel).dot(&n);
        let dlambda = -wn / inv_rot;
        let imp = n * dlambda;
        bodies[a].ang_vel -= ia * imp;
        bodies[b].ang_vel += ib * imp;
    }

    // 3) Motor:沿滑动轴施加目标线速度(推动两体沿轴相对移动),力受限。
    if motor_vel.abs() > T::zero() && max_motor_force > T::zero() {
        let inv_ma = bodies[a].eff_inv_mass();
        let inv_mb = bodies[b].eff_inv_mass();
        let ang = |r: Vec3<T>, inv: Mat3<T>| -> T {
            let rn = r.cross(&axis_world);
            let t = inv * rn;
            rn.dot(&t)
        };
        let inv_sum = inv_ma + inv_mb + ang(ra, ia) + ang(rb, ib);
        if inv_sum > T::from_f64(1e-12).unwrap() {
            let va = bodies[a].vel + bodies[a].ang_vel.cross(&ra);
            let vb = bodies[b].vel + bodies[b].ang_vel.cross(&rb);
            let vn = (vb - va).dot(&axis_world);
            let mut dlambda = (motor_vel - vn) / inv_sum;
            if dlambda > max_motor_force {
                dlambda = max_motor_force;
            } else if dlambda < -max_motor_force {
                dlambda = -max_motor_force;
            }
            let imp = axis_world * dlambda;
            bodies[a].apply_impulse_at(-imp, ra);
            bodies[b].apply_impulse_at(imp, rb);
        }
    }
}

/// Hinge 位置层转动对齐:消除两体相对角伪速度沿**垂直于铰链轴**的分量(保留沿轴自由旋转)。
fn solve_hinge_position_angle<T: RealField + Copy>(
    bodies: &[Body<T>],
    a: usize,
    b: usize,
    axis_a_local: Vec3<T>,
    ang_pseudo: &mut [Vec3<T>],
) {
    let ia = bodies[a].inv_inertia_world();
    let ib = bodies[b].inv_inertia_world();
    let axis_world = (bodies[a].rot * axis_a_local).normalize();
    let ref_vec = if axis_world.z.abs() < T::from_f64(0.9).unwrap() {
        Vec3::new(T::zero(), T::zero(), T::one())
    } else {
        Vec3::new(T::one(), T::zero(), T::zero())
    };
    let u1 = ref_vec.cross(&axis_world).normalize();
    let u2 = axis_world.cross(&u1).normalize();
    for &u in [u1, u2].iter() {
        let inv_rot = u.dot(&(ia * u)) + u.dot(&(ib * u));
        if inv_rot <= T::from_f64(1e-12).unwrap() {
            continue;
        }
        let wn = (ang_pseudo[b] - ang_pseudo[a]).dot(&u);
        if wn.abs() < T::from_f64(1e-9).unwrap() {
            continue;
        }
        let dlambda = -wn / inv_rot;
        let imp = u * dlambda;
        ang_pseudo[a] -= ia * imp;
        ang_pseudo[b] += ib * imp;
    }
}
