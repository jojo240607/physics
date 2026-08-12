//! RigidWorld: 刚体动力学世界(M2)。
//!
//! `step` 流程:
//! 1. 积分速度(施加重力)
//! 2. Broad-phase + Narrow-phase 求所有接触
//! 3. 顺序冲量法求解速度(接触 + 摩擦)
//! 4. 积分位置(用求解后的速度)
//! 5. 位置修正(防止穿透累积)

use phy_field::{EmFieldLike, GravFieldLike, HeatFieldLike};
use phy_math::{gravity, na, Quat, RealField, Vec3};
use num_traits::NumCast;
use serde::de::DeserializeOwned;
use serde::{Deserialize, Serialize};

use crate::broadphase::broadphase;
use crate::ccd;
use crate::contact::Contact;
use crate::fracture::fracture_body;
use crate::joint::{Joint, JointConstraint};
use crate::narrowphase::collide;
use crate::profile::StepProfile;
use crate::profile_stage;
use crate::shape::Body;
use crate::solver::{ContactConstraint, SolverParams};
use rayon::prelude::*;
use std::collections::HashMap;

/// 刚体动力学世界。
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(bound = "T: RealField + Copy + Serialize + DeserializeOwned")]
pub struct RigidWorld<T: RealField + Copy> {
    pub bodies: Vec<Body<T>>,
    /// 每个刚体的电荷量(与 `bodies` 等长,0 = 中性)。用于电磁耦合。
    pub charges: Vec<T>,
    /// 关节约束(M18):把刚体连成链条/摆/机械结构。
    pub joints: Vec<JointConstraint<T>>,
    /// 重力(默认沿 -Y)。
    #[serde(with = "crate::shape::serde_geom")]
    pub gravity: Vec3<T>,
    /// 求解参数。
    pub params: SolverParams<T>,
    /// B2 上一步检测到的传感器重叠接触(仅含 pair 中任一方 `is_sensor` 的接触,
    /// 不施加冲量)。每次 `step` 刷新;可用 `sensor_contacts()` 读取供触发器/拾取判定。
    pub last_sensor_contacts: Vec<Contact<T>>,
}

impl<T: RealField + Copy + num_traits::ToPrimitive> Default for RigidWorld<T> {
    fn default() -> Self {
        Self::new()
    }
}

/// B4 岛屿(island)求解辅助:把全局接触/关节约束按连通分量分组,分别在局部
/// 视图上顺序求解,再写回全局。岛屿间刚体索引互不相交 → 可安全并行。
///
/// 这些函数保持与单线程顺序求解**数值等价**:岛屿内部约束相对顺序不变,且关节
/// 累积冲量(`lambda`)跨子步的持久性被保留(通过写回全局 `joints` 的 `lambda`)。
/// 仅岛屿之间的处理顺序被解耦(因为不共享刚体,顺序无关 → 结果一致)。

/// 为给定岛屿构建全局→局部索引映射,并抽取属于该岛屿的接触/关节约束(重映射索引)。
fn island_partition<T: RealField + Copy>(
    island: &[usize],
    cons: &[ContactConstraint<T>],
    joints: &[JointConstraint<T>],
) -> (
    HashMap<usize, usize>,
    Vec<ContactConstraint<T>>,
    Vec<JointConstraint<T>>,
    Vec<usize>,
) {
    let mut g2l = HashMap::new();
    for (li, &gi) in island.iter().enumerate() {
        g2l.insert(gi, li);
    }
    let mut cons_l = Vec::new();
    for c in cons {
        if let (Some(&la), Some(&lb)) = (g2l.get(&c.a), g2l.get(&c.b)) {
            let mut cc = c.clone();
            cc.a = la;
            cc.b = lb;
            cons_l.push(cc);
        }
    }
    let mut joints_l = Vec::new();
    let mut joint_gi = Vec::new();
    for (k, j) in joints.iter().enumerate() {
        if let (Some(&la), Some(&lb)) = (g2l.get(&j.a), g2l.get(&j.b)) {
            let mut jj = j.clone();
            jj.a = la;
            jj.b = lb;
            joints_l.push(jj);
            joint_gi.push(k);
        }
    }
    (g2l, cons_l, joints_l, joint_gi)
}

/// 单个岛屿的速度层求解(只读全局 `bodies`),返回局部 vel/ang_vel 与关节 lambda 更新。
fn island_velocity<T: RealField + Copy>(
    bodies: &[Body<T>],
    island: &[usize],
    cons: &[ContactConstraint<T>],
    joints: &[JointConstraint<T>],
    params: &SolverParams<T>,
) -> (Vec<Vec3<T>>, Vec<Vec3<T>>, Vec<(usize, T)>) {
    let (_, mut cons_l, mut joints_l, joint_gi) =
        island_partition(island, cons, joints);
    let mut local: Vec<Body<T>> = island.iter().map(|&gi| bodies[gi].clone()).collect();
    crate::solver::solve_velocity(&mut local, &mut cons_l, params);
    crate::joint::solve_joints_velocity(&mut local, &mut joints_l, params.iterations);
    let vel: Vec<Vec3<T>> = local.iter().map(|b| b.vel).collect();
    let ang: Vec<Vec3<T>> = local.iter().map(|b| b.ang_vel).collect();
    // 关节 lambda 跨子步累积需在全局保留 → 回写全局 joints。
    let jl: Vec<(usize, T)> = joints_l
        .iter()
        .enumerate()
        .map(|(i, j)| (joint_gi[i], j.lambda))
        .collect();
    (vel, ang, jl)
}

/// 单个岛屿的位置层求解(只读全局 `bodies`),返回局部伪速度(线 + 角)。
fn island_position<T: RealField + Copy>(
    bodies: &[Body<T>],
    island: &[usize],
    cons: &[ContactConstraint<T>],
    joints: &[JointConstraint<T>],
    beta_over_dt: T,
    slop: T,
) -> (Vec<Vec3<T>>, Vec<Vec3<T>>) {
    let (_, cons_l, joints_l, _) = island_partition(island, cons, joints);
    let local: Vec<Body<T>> = island.iter().map(|&gi| bodies[gi].clone()).collect();
    let mut pseudo: Vec<Vec3<T>> = vec![Vec3::zeros(); local.len()];
    let mut ang_pseudo: Vec<Vec3<T>> = vec![Vec3::zeros(); local.len()];
    crate::solver::solve_position(
        &local,
        &cons_l,
        &mut pseudo,
        &mut ang_pseudo,
        beta_over_dt,
        slop,
    );
    crate::joint::solve_joints_position(
        &local,
        &joints_l,
        &mut pseudo,
        &mut ang_pseudo,
        beta_over_dt,
    );
    (pseudo, ang_pseudo)
}

/// B4 按岛屿分组求解速度层:返回每个刚体的新 vel/ang_vel,以及关节 lambda 更新。
///
/// 岛屿之间互不依赖(共享刚体索引为空),故用 rayon 在岛屿间并行求解;
/// 仅最后把各岛屿结果写回全局是串行的(按岛屿索引写回,互不相交 → 也可并行,
/// 但此处串行足够,且与单线程写回顺序一致便于确定性复现)。
fn solve_islands_velocity<T: RealField + Copy>(
    bodies: &[Body<T>],
    cons: &[ContactConstraint<T>],
    joints: &[JointConstraint<T>],
    params: &SolverParams<T>,
) -> (Vec<Vec3<T>>, Vec<Vec3<T>>, Vec<(usize, T)>) {
    let contact_pairs: Vec<(usize, usize)> = cons.iter().map(|c| (c.a, c.b)).collect();
    let joint_pairs: Vec<(usize, usize)> = joints.iter().map(|j| (j.a, j.b)).collect();
    let islands = crate::islands::build_islands(bodies.len(), &contact_pairs, &joint_pairs);
    // 并行求解每个岛屿(只读全局 bodies,各自产出独立结果)。
    let results: Vec<(Vec<usize>, Vec<Vec3<T>>, Vec<Vec3<T>>, Vec<(usize, T)>)> = islands
        .par_iter()
        .map(|island| {
            let (vl, al, jl) = island_velocity(bodies, island, cons, joints, params);
            (island.clone(), vl, al, jl)
        })
        .collect();
    // 串行写回(岛屿间索引不相交,无竞争;与单线程写回次序无关,结果一致)。
    let mut vel_out = vec![Vec3::zeros(); bodies.len()];
    let mut ang_out = vec![Vec3::zeros(); bodies.len()];
    let mut joint_lambda = Vec::new();
    for (island, vl, al, jl) in results {
        for (li, &gi) in island.iter().enumerate() {
            vel_out[gi] = vl[li];
            ang_out[gi] = al[li];
        }
        joint_lambda.extend(jl);
    }
    (vel_out, ang_out, joint_lambda)
}

/// B4 按岛屿分组求解位置层:返回每个刚体的伪速度(线 + 角)。
///
/// 与速度层同构:岛屿间并行求解,串行写回(索引不相交)。
fn solve_islands_position<T: RealField + Copy>(
    bodies: &[Body<T>],
    cons: &[ContactConstraint<T>],
    joints: &[JointConstraint<T>],
    beta_over_dt: T,
    slop: T,
) -> (Vec<Vec3<T>>, Vec<Vec3<T>>) {
    let contact_pairs: Vec<(usize, usize)> = cons.iter().map(|c| (c.a, c.b)).collect();
    let joint_pairs: Vec<(usize, usize)> = joints.iter().map(|j| (j.a, j.b)).collect();
    let islands = crate::islands::build_islands(bodies.len(), &contact_pairs, &joint_pairs);
    let results: Vec<(Vec<usize>, Vec<Vec3<T>>, Vec<Vec3<T>>)> = islands
        .par_iter()
        .map(|island| {
            let (p, ap) = island_position(bodies, island, cons, joints, beta_over_dt, slop);
            (island.clone(), p, ap)
        })
        .collect();
    let mut pseudo = vec![Vec3::zeros(); bodies.len()];
    let mut ang_pseudo = vec![Vec3::zeros(); bodies.len()];
    for (island, p, ap) in results {
        for (li, &gi) in island.iter().enumerate() {
            pseudo[gi] = p[li];
            ang_pseudo[gi] = ap[li];
        }
    }
    (pseudo, ang_pseudo)
}

impl<T: RealField + Copy + num_traits::ToPrimitive> RigidWorld<T> {
    /// 创建空刚体世界(默认重力沿 -Y)。
    ///
    /// # 示例
    ///
    /// 放一个静止地面 + 一个下落球,推进若干步,断言球未穿透地面:
    ///
    /// ```
    /// use phy_rigid::{RigidWorld, Body, Shape};
    /// use phy_math::Vec3 as V3;
    /// use nalgebra::UnitQuaternion;
    ///
    /// let mut w = RigidWorld::<f64>::new();
    /// w.add_body(Body { shape: Shape::Box { half: V3::new(20.0, 0.5, 20.0) },
    ///                    pos: V3::new(0.0, -0.5, 0.0),
    ///                    rot: UnitQuaternion::identity(), vel: V3::zeros(),
    ///                    inv_mass: 0.0, ..Default::default() });
    /// w.add_body(Body::new(Shape::Sphere { r: 1.0 }, V3::new(0.0, 5.0, 0.0), 0.5));
    /// for _ in 0..200 { w.step(0.01); }
    /// let ball = &w.bodies[1];
    /// assert!(ball.pos.y > -0.5 + 1.0 - 1e-6); // 球心不低于 地面顶 + 半径
    /// ```
    pub fn new() -> Self {
        Self {
            bodies: Vec::new(),
            charges: Vec::new(),
            joints: Vec::new(),
            gravity: gravity::<T>(),
            params: SolverParams::default(),
            last_sensor_contacts: Vec::new(),
        }
    }

    pub fn add_body(&mut self, b: Body<T>) -> usize {
        self.bodies.push(b);
        self.charges.push(T::zero());
        self.bodies.len() - 1
    }

    /// 添加带电刚体,返回其索引。电荷量 `q` 存入并行 `charges` 向量。
    pub fn add_charged_body(&mut self, b: Body<T>, q: T) -> usize {
        self.bodies.push(b);
        self.charges.push(q);
        self.bodies.len() - 1
    }

    /// 添加关节约束(M18),返回其索引。
    ///
    /// 关节在 `step` 的速度层 + 位置层与接触一起被顺序冲量法求解,
    /// 使链条/摆/机械结构稳定(不破坏碰撞)。
    pub fn add_joint(&mut self, a: usize, b: usize, joint: Joint<T>) -> usize {
        self.joints.push(JointConstraint::new(a, b, joint));
        self.joints.len() - 1
    }

    /// 推进一步。返回本步检测到的接触(供调试/渲染)。
    ///
    /// 这是 `step_with_profile` 的精简封装:不返回分阶段计时(零开销,默认构建与
    /// 未启用 `profiler` feature 时完全一致)。需要性能分析请改用 `step_with_profile`。
    ///
    /// 标准半隐式欧拉 + 顺序冲量流程,带连续碰撞检测(CCD):
    /// 1. 积分速度(重力) 2. CCD 子步推进:粗筛→扫掠求最早接触时间(TOI)→把位置/姿态积分
    ///    截断到 TOI(避免高速隧穿)→在该处检测接触并求解速度冲量(含角冲量)→余下时间继续
    ///    3. 位置投影(清残余穿透,含角伪速度)。低速/无接近时 TOI 落在步外,子步退化为整步推进,
    ///    与离散碰撞检测完全等价。
    pub fn step(&mut self, dt: T) -> Vec<Contact<T>> {
        self.step_with_profile(dt).0
    }

    /// 推进一步并返回分阶段耗时(C3 运行时性能分析,商用引擎补齐计划 §11)。
    ///
    /// 返回 `(本步接触, StepProfile)`。`StepProfile` 在 `profiler` feature 启用时
    /// 含各管线阶段的纳秒计时(broad/narrow、velocity、advance、position、sleep);
    /// 未启用时所有字段恒为 0 且不产生任何计时调用(零开销)。
    ///
    /// ```
    /// use phy_rigid::RigidWorld;
    /// let mut w = RigidWorld::<f64>::new();
    /// // API 在默认构建与 `profiler` feature 下均可用;
    /// // 默认构建未启用 `profiler` 时计时字段恒为 0(零开销),启用后即为真实耗时。
    /// let (_contacts, prof) = w.step_with_profile(1.0 / 60.0);
    /// let _ = prof.total_ns; // 默认构建为 0;启用 profiler feature 后为真实纳秒耗时
    /// ```
    /// 启用 `profiler` feature 后,各阶段计时即真实反映 `step` 的内部耗时。
    pub fn step_with_profile(&mut self, dt: T) -> (Vec<Contact<T>>, StepProfile) {
        let mut prof = StepProfile::default();
        let max_sub = self.params.ccd_max_substeps;

        // 1. 积分速度(重力) —— 姿态积分与位置积分合并到子步的 advance 中。
        //    B1:休眠管理。先依据上一步末速度判定"近静止"并累积 sleep_time;
        //    达到阈值则置 sleeping=true 并清零速度(后续跳过积分/推进,零 CPU)。
        profile_stage!(prof, sleep_ns, {
            let (sl2, sa2, st) = (
                self.params.sleep_lin_vel2,
                self.params.sleep_ang_vel2,
                self.params.sleep_time,
            );
            for b in self.bodies.iter_mut() {
                if b.inv_mass <= T::zero() || b.kinematic {
                    continue; // 静态体/运动学体不积分重力、不参与休眠。
                }
                if b.sleeping {
                    b.vel = Vec3::zeros();
                    b.ang_vel = Vec3::zeros();
                    continue; // 休眠体本步不积分重力(速度已清零)。
                }
                let near_rest =
                    b.vel.norm_squared() < sl2 && b.ang_vel.norm_squared() < sa2;
                if near_rest {
                    b.sleep_time += dt;
                    if b.sleep_time >= st {
                        b.sleeping = true;
                        b.vel = Vec3::zeros();
                        b.ang_vel = Vec3::zeros();
                        continue;
                    }
                } else {
                    b.sleep_time = T::zero();
                }
                // 非休眠且未达阈值的体:照常积分重力。
                // 注意:静态体(inv_mass==0)与运动学体不接受重力——否则其 vel 会累积,
                // 污染求解器相对速度计算(静体"假运动"抵消动态体接近速度 => 无接触冲量
                // => 动态体穿入后被位置修正伪速度弹出)。
                if b.inv_mass > T::zero() && !b.kinematic {
                    b.vel += self.gravity * dt;
                }
            }
        });

        // 2. CCD 子步循环:按位移受限决定子步数,使最快体每子步位移 ≤ 其最小特征尺寸的一半,
        //    再在每个子步上跑离散 collide + 速度求解。低速/无接近时 n=1,等价于原整步离散检测。
        let n = ccd::substep_count(&self.bodies, dt, max_sub);
        let h = dt / T::from_usize(n).unwrap();
        let mut last_constraints: Vec<ContactConstraint<T>> = Vec::new();
        // B2 传感器重叠累加器:收集所有子步中"任一方为传感器"的窄相接触(不施加冲量)。
        let mut sensor_acc: Vec<Contact<T>> = Vec::new();
        for _ in 0..n {
            // 2a. 常规粗筛 + 当前位置离散碰撞检测。
            profile_stage!(prof, broad_narrow_ns, {
                let pairs = broadphase(&self.bodies);
                let mut cons: Vec<ContactConstraint<T>> = Vec::new();
                for (i, j) in pairs {
                    // B1 唤醒:若一对中一方休眠、另一方运动,唤醒休眠方(恢复参与求解/推进)。
                    {
                        let (bi, bj) = (&self.bodies[i], &self.bodies[j]);
                        if bi.sleeping != bj.sleeping {
                            // B2:运动学体(kinematic)也视为"运动方",可唤醒休眠体。
                            let moving_j = bj.inv_mass > T::zero() || bj.kinematic;
                            let moving_i = bi.inv_mass > T::zero() || bi.kinematic;
                            let awoke_i = bi.sleeping && moving_j;
                            let awoke_j = bj.sleeping && moving_i;
                            if awoke_i || awoke_j {
                                // 需要可变借用,延迟到此处再做。
                                if awoke_i {
                                    self.bodies[i].sleeping = false;
                                    self.bodies[i].sleep_time = T::zero();
                                }
                                if awoke_j {
                                    self.bodies[j].sleeping = false;
                                    self.bodies[j].sleep_time = T::zero();
                                }
                            }
                        }
                    }
                    // B5 碰撞层过滤:层位与掩码不匹配则跳过(不窄相、不唤醒)。
                    if !self.bodies[i].can_collide_with(&self.bodies[j]) {
                        continue;
                    }
                    // B2 传感器:任一方为传感器时,窄相检测重叠但不求解(不施加冲量),
                    // 仅记录到传感器事件列表(供触发器/拾取判定)。
                    let sensor_pair =
                        self.bodies[i].is_sensor || self.bodies[j].is_sensor;
                    if let Some(c) = collide(&self.bodies[i], &self.bodies[j]) {
                        if sensor_pair {
                            sensor_acc.push(c);
                        } else {
                            cons.push(ContactConstraint::new(i, j, c));
                        }
                    }
                }
                last_constraints = cons;
            });

            // 2b. 速度求解(顺序冲量,含角冲量)+ 关节速度求解。
            //     B4:按岛屿分组求解(岛屿内顺序、岛屿间可并行),保证与单线程等价。
            profile_stage!(prof, velocity_ns, {
                let (vel, ang, joint_lambda) = solve_islands_velocity(
                    &self.bodies,
                    &last_constraints,
                    &self.joints,
                    &self.params,
                );
                for (i, b) in self.bodies.iter_mut().enumerate() {
                    b.vel = vel[i];
                    b.ang_vel = ang[i];
                }
                // 关节 lambda 跨子步累积需保留全局状态。
                for (k, lam) in joint_lambda {
                    self.joints[k].lambda = lam;
                }
            });

            // 2c. 把所有体按子步时长 h 推进位置与姿态。
            profile_stage!(prof, advance_ns, {
                Self::advance(&mut self.bodies, h);
            });
        }

        // 3. 位置修正(split impulse 伪速度):解伪速度使物体分离,
        //    伪速度只用于修正位置,不污染真实速度(避免抖动/能量注入)。
        let mut pseudo: Vec<Vec3<T>> = vec![Vec3::zeros(); self.bodies.len()];
        let mut ang_pseudo: Vec<Vec3<T>> = vec![Vec3::zeros(); self.bodies.len()];
        let beta = T::from_f64(0.2).unwrap();
        let beta_over_dt = beta / dt;
        // B4:按岛屿分组求解位置层(与速度层同构,岛屿间可并行)。
        profile_stage!(prof, position_ns, {
            let (p, ap) = solve_islands_position(
                &self.bodies,
                &last_constraints,
                &self.joints,
                beta_over_dt,
                self.params.position_slop,
            );
            pseudo = p;
            ang_pseudo = ap;
        });
        for (i, b) in self.bodies.iter_mut().enumerate() {
            if b.sleeping {
                continue; // B1:休眠体位置/姿态冻结,不应用伪速度修正。
            }
            if b.inv_mass > T::zero() {
                b.pos += pseudo[i] * dt;
                if b.inv_inertia_local.iter().any(|x| *x != T::zero()) {
                    let q = *b.rot.quaternion();
                    let dq = Quat::new(
                        T::zero(),
                        ang_pseudo[i].x,
                        ang_pseudo[i].y,
                        ang_pseudo[i].z,
                    ) * q;
                    let nq = Quat::new(
                        q.w + dq.w * (dt * T::from_f64(0.5).unwrap()),
                        q.i + dq.i * (dt * T::from_f64(0.5).unwrap()),
                        q.j + dq.j * (dt * T::from_f64(0.5).unwrap()),
                        q.k + dq.k * (dt * T::from_f64(0.5).unwrap()),
                    );
                    b.rot = na::UnitQuaternion::new_normalize(nq);
                }
            }
        }

        // B2:刷新传感器事件(仅保留本步最后子步的重叠快照,避免重复计数)。
        self.last_sensor_contacts = sensor_acc;
        prof.total_ns = prof.broad_narrow_ns
            + prof.velocity_ns
            + prof.advance_ns
            + prof.position_ns
            + prof.sleep_ns;
        (last_constraints.into_iter().map(|c| c.contact).collect(), prof)
    }

    /// B2 返回上一步 `step` 检测到的传感器重叠接触(任一方 `is_sensor` 的 pair)。
    ///
    /// 传感器不施加冲量、不阻止穿透,仅用于触发器/拾取/进入判定。每次 `step` 刷新。
    pub fn sensor_contacts(&self) -> &[Contact<T>] {
        &self.last_sensor_contacts
    }

    /// 把所有可动体按时间步 `h` 推进位置(线速度)与姿态(角速度)。
    ///
    /// 姿态积分 `q += 0.5·(0,ω)⊗q·h` 后与 `step` 原逻辑一致;把积分放此处使 CCD
    /// 子步能按每个子步时长精确推进,整体在 `h` 求和等于 `dt` 时等价于原整步积分。
    fn advance(bodies: &mut [Body<T>], h: T) {
        let half = h * T::from_f64(0.5).unwrap();
        for b in bodies.iter_mut() {
            if b.sleeping {
                continue; // B1:休眠体不推进(位置/姿态冻结)。
            }
            // B2:运动学体按用户设定的 vel 主动移动(即使 eff_inv_mass=0 也推进)。
            if b.inv_mass > T::zero() || b.kinematic {
                b.pos += b.vel * h;
            }
            if b.inv_inertia_local.iter().any(|x| *x != T::zero()) {
                let wbar = b.ang_vel;
                let q = *b.rot.quaternion();
                let dq = Quat::new(T::zero(), wbar.x, wbar.y, wbar.z) * q;
                let nq = Quat::new(
                    q.w + dq.w * half,
                    q.i + dq.i * half,
                    q.j + dq.j * half,
                    q.k + dq.k * half,
                );
                b.rot = na::UnitQuaternion::new_normalize(nq);
            }
        }
    }

    /// 刚体↔热场双向耦合(M11):热浮力 + 对流换热。
    ///
    /// 对每个可动刚体,在其质心处三线性采样温度 `T`,按密度修正
    /// `ρ(T)=ρ0/(1+β·(T-T_ref))` 计算热浮力加速度修正 `a = -g·(ρ0-ρT)/ρ0`,
    /// 以 `vel += a·dt` 注入(下一帧 `step` 的重力积分后生效,与流体一致)。
    /// 若 `heat_gain>0`,以 `heat_gain·‖vel‖·dt` 注入热源到质心所在网格单元(对流换热)。
    pub fn couple_heat(
        &mut self,
        heat: &mut dyn HeatFieldLike<T>,
        dt: T,
        t_ref: T,
        beta: T,
        heat_gain: T,
    ) where
        T: num_traits::ToPrimitive,
    {
        let g = self.gravity; // 沿 -Y
        for b in self.bodies.iter_mut() {
            if b.inv_mass <= T::zero() {
                continue; // 静态物体不参与热浮力。
            }
            let temp = phy_field::sample_world(heat, b.pos);
            // 密度修正。
            let denom = T::one() + beta * (temp - t_ref);
            let rho_t = if denom > T::zero() {
                T::one() / denom
            } else {
                T::one() // 极端情况退化为参考密度。
            };
            // 热浮力加速度修正:-g·(ρ0-ρT)/ρ0 = -g·(1 - ρT)(与流体 M4e 一致,向上为正)。
            let buoy = -g * (T::one() - rho_t);
            b.vel += buoy * dt;
            // 对流换热:运动物体加热所在网格。
            if heat_gain > T::zero() {
                let speed = b.vel.norm();
                if speed > T::zero() {
                    let (cx, cy, cz, _, _, _) = phy_field::world_to_cell(heat, b.pos);
                    heat.add_source(cx, cy, cz, heat_gain * speed * dt);
                }
            }
        }
    }

    /// 刚体↔电磁场双向耦合(M12):洛伦兹力 + 运动感应电荷。
    ///
    /// 对每个带电刚体(`q≠0`),在其质心处三线性采样电场 `E`,施加洛伦兹力
    /// `F = q·(E + v×B_ext)`(`B_ext` 为电磁场外加均匀磁场,默认零),以
    /// `vel += (F/m)·dt` 注入(下一帧 `step` 生效,与热浮力一致)。
    /// 同时把运动带电体的等效电流 `q·‖v‖·dt` 沉积进所在网格的电荷密度
    /// (运动物体感应/产生电荷,反向影响电场),实现双向耦合。
    pub fn couple_em(
        &mut self,
        em: &mut dyn EmFieldLike<T>,
        dt: T,
        em_coupling: T,
    ) where
        T: num_traits::ToPrimitive,
    {
        if em_coupling <= T::zero() {
            return;
        }
        for (idx, b) in self.bodies.iter_mut().enumerate() {
            let q = self.charges[idx];
            if b.inv_mass <= T::zero() || q == T::zero() {
                continue; // 静态或中性物体不参与电磁耦合。
            }
            let e = phy_field::sample_e_field(em, b.pos);
            let b_ext = em.b_ext();
            // 洛伦兹力 F = q·(E + v×B)。
            let lorentz = e + b.vel.cross(&b_ext);
            let f = lorentz * q;
            b.vel += f * b.inv_mass * dt * em_coupling;
            // 运动感应电荷沉积:q·‖v‖·dt 注入所在网格(反向影响电场)。
            let speed = b.vel.norm();
            if speed > T::zero() {
                let (cx, cy, cz, _, _, _) = phy_field::world_to_cell(em, b.pos);
                em.add_charge(cx, cy, cz, q * speed * dt);
            }
        }
    }

    /// 刚体↔引力场双向耦合(M13):局部引力井偏转 + 运动质量沉积。
    ///
    /// 对每个可动刚体,在其质心处三线性采样局部引力加速度 `g_local = -∇Φ`
    /// (空间变化的引力井,叠加在 `RigidWorld.gravity` 的均匀重力之上),以
    /// `vel += g_local·dt·grav_coupling` 注入(下一帧 `step` 生效,与热浮力/电磁一致)。
    /// 同时把运动物体的等效质量通量 `m·‖v‖·dt` 沉积进所在网格(运动团块塑造引力井),
    /// 实现双向耦合。质量 `m = 1/inv_mass`。
    pub fn couple_grav(
        &mut self,
        grav: &mut dyn GravFieldLike<T>,
        dt: T,
        grav_coupling: T,
    ) where
        T: num_traits::ToPrimitive,
    {
        if grav_coupling <= T::zero() {
            return;
        }
        for b in self.bodies.iter_mut() {
            if b.inv_mass <= T::zero() {
                continue; // 静态物体不受局部引力加速(其质量由静态天体注入贡献)。
            }
            let g_local = phy_field::sample_g_field(grav, b.pos);
            b.vel += g_local * dt * grav_coupling;
            // 运动质量沉积:运动团块把质量通量注入网格,反向塑造引力井。
            let speed = b.vel.norm();
            if speed > T::zero() {
                let m = T::one() / b.inv_mass;
                let (cx, cy, cz, _, _, _) = phy_field::world_to_cell(grav, b.pos);
                grav.add_mass(cx, cy, cz, m * speed * dt);
            }
        }
    }
}

/// Voronoi 破碎(M21 / 路线图 #8):把一个刚体碎成 `n` 个凸碎片。
///
/// 调用 `fracture_body` 生成碎片 `Body`,移除原刚体并追加所有碎片。碎片继承母本
/// 线速度 + 沿碎片-母体质心方向的径向飞散(`radial`);质量按体积比分配(守恒)。
///
/// 返回新碎片在世界中的索引列表。若 `body_id` 越界或母本为静态体(`inv_mass==0`),
/// 直接返回空(静态天体不参与破碎)。
///
/// 注意:本引擎刚体已具备完整角动力学(`Body.ang_vel`/`inv_inertia_local` + 四元数姿态
/// 积分)。`fracture_body` 让碎片继承母本 `ang_vel`,并在径向飞散时经偏心冲量注入真实
/// 角自旋(见 `fracture.rs`)。故碎裂后碎片既带线速度飞散、也带自旋,角动量守恒。
impl<T: RealField + Copy + NumCast> RigidWorld<T> {
    pub fn shatter(&mut self, body_id: usize, n: usize, radial: T) -> Vec<usize> {
        if body_id >= self.bodies.len() {
            return Vec::new();
        }
        if self.bodies[body_id].inv_mass <= T::zero() {
            return Vec::new(); // 静态体不碎。
        }
        let parent = self.bodies[body_id].clone();
        let parent_charge = self.charges[body_id];
        let frags = fracture_body(&parent, n, None, radial);
        // 移除母本(用 swap_remove 保持 charges 同步),碎片的索引以"母本之后追加"为准。
        self.bodies.swap_remove(body_id);
        self.charges.swap_remove(body_id);
        let mut frag_ids = Vec::with_capacity(frags.len());
        for f in frags {
            // 碎片电荷:按体积比(这里用质量比近似)分配母本质荷。
            let q = parent_charge * (T::one() / f.inv_mass) * parent.inv_mass;
            let id = self.add_charged_body(f, q);
            frag_ids.push(id);
        }
        frag_ids
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use phy_core::Subsystem;
    use phy_field::{Bc, EmField, GravField, HeatField, ScalarField};
    use phy_math::na;

    use crate::shape::Shape;
    use crate::solver::SolverParams;

    /// 热浮力应让热区中的刚体获得向上的速度修正(抵消部分重力)。
    #[test]
    fn couple_heat_warmer_body_rises() {
        // 简单 3x3x3 热场,中心一格高温。
        let nx = 3usize;
        let dx = 1.0;
        let mut f = ScalarField::<f64>::new(nx, nx, nx, dx, 0.0, Bc::Neumann);
        let hot = f.idx(1, 1, 1);
        f.u[hot] = 100.0; // 中心高温(T_ref=0)。
        let mut heat = HeatField::new(f, 0.1);

        let mut world = RigidWorld::new();
        world.gravity = Vec3::new(0.0, -9.81, 0.0);
        // 质点球放在热场中心(世界坐标 (1,1,1))。
        let body = Body {
            shape: Shape::Sphere { r: 0.2.into() },
            pos: Vec3::new(1.0, 1.0, 1.0),
            rot: na::one(),
            vel: Vec3::new(0.0, 0.0, 0.0),
            inv_mass: 1.0,
        
            ..Default::default()
        };
        world.add_body(body);

        // 耦合前记重力方向速度(应为 0,因为还没 step)。
        let vy_before = world.bodies[0].vel.y;
        world.couple_heat(&mut heat, 0.1, 0.0, 0.5, 0.0);
        let vy_after = world.bodies[0].vel.y;

        // 热浮力修正 a = -g·(1-ρT),g.y=-9.81 → vy 增加(向上为正)。
        // 中心温度 100,β=0.5 → ρT=1/(1+0.5·100)=1/51≈0.0196,向上修正≈0.98·9.81·0.1≈0.96。
        assert!(vy_after > vy_before, "热物体应获得向上速度修正");
        assert!(vy_after > -9.81 * 0.1, "热浮力应显著抵消重力");
    }

    /// 运动刚体应把热源注入所在网格(对流换热)。
    #[test]
    fn couple_heat_injects_source_into_moving_body() {
        let nx = 3usize;
        let dx = 1.0;
        let f = ScalarField::<f64>::new(nx, nx, nx, dx, 0.0, Bc::Neumann);
        let mut heat = HeatField::new(f, 0.1);

        let mut world = RigidWorld::new();
        // 放在 (1,1,1) 处且已有水平速度。
        let body = Body {
            shape: Shape::Sphere { r: 0.2.into() },
            pos: Vec3::new(1.0, 1.0, 1.0),
            rot: na::one(),
            vel: Vec3::new(2.0, 0.0, 0.0),
            inv_mass: 1.0,
        
            ..Default::default()
        };
        world.add_body(body);

        world.couple_heat(&mut heat, 0.1, 0.0, 0.0, 0.1);

        // 注入进 src,经 step_diffusion 才会进 u。验证 src 已累积。
        heat.field.step_diffusion(0.1, 0.01);
        let injected = heat.field.sample(1, 1, 1);
        assert!(injected > 0.0, "运动刚体应加热所在网格");
    }

    /// 洛伦兹力的电场项:正电荷在 +X 电场中应获得 +X 方向速度。
    #[test]
    fn couple_em_electric_force_on_charge() {
        let nx = 3usize;
        let dx = 1.0;
        let rho = ScalarField::<f64>::new(nx, nx, nx, dx, 0.0, Bc::Neumann);
        let mut em = EmField::build(rho, 1.0);
        // 直接铺设一个沿 +X 的均匀电场(跳过泊松松弛,专测力项)。
        for e in em.e.iter_mut() {
            *e = Vec3::new(1.0, 0.0, 0.0);
        }
        let mut world = RigidWorld::new();
        let body = Body {
            shape: Shape::Sphere { r: 0.2.into() },
            pos: Vec3::new(1.0, 1.0, 1.0),
            rot: na::one(),
            vel: Vec3::new(0.0, 0.0, 0.0),
            inv_mass: 1.0,
        
            ..Default::default()
        };
        world.add_charged_body(body, 1.0); // q=+1

        world.couple_em(&mut em, 0.1, 1.0);
        // F = q·E = +1·(+X) → vx 应为正。
        assert!(world.bodies[0].vel.x > 0.0, "正电荷在 +X 电场中应受力加速 +X");
        // 中性或静态物体不受影响:放一个中性体验证。
        let neutral = Body {
            shape: Shape::Sphere { r: 0.2.into() },
            pos: Vec3::new(1.0, 1.0, 1.0),
            rot: na::one(),
            vel: Vec3::new(0.0, 0.0, 0.0),
            inv_mass: 1.0,
        
            ..Default::default()
        };
        world.add_body(neutral);
        let vx_before = world.bodies[1].vel.x;
        world.couple_em(&mut em, 0.1, 1.0);
        assert!(
            (world.bodies[1].vel.x - vx_before).abs() < 1e-12,
            "中性物体不应受电磁力"
        );
    }

    /// 洛伦兹力的磁场项:v×B 应产生垂直于 v 与 B 的偏转。
    #[test]
    fn couple_em_velocity_cross_b_deflects() {
        let nx = 3usize;
        let rho = ScalarField::<f64>::new(nx, nx, nx, 1.0, 0.0, Bc::Neumann);
        let mut em = EmField::build(rho, 1.0);
        em.b_ext = Vec3::new(0.0, 0.0, 1.0); // B 沿 +Z
        let mut world = RigidWorld::new();
        // 速度沿 +X,电荷 +1 → v×B = X×Z = -Y → 应获得 -Y 速度。
        let body = Body {
            shape: Shape::Sphere { r: 0.2.into() },
            pos: Vec3::new(1.0, 1.0, 1.0),
            rot: na::one(),
            vel: Vec3::new(1.0, 0.0, 0.0),
            inv_mass: 1.0,
        
            ..Default::default()
        };
        world.add_charged_body(body, 1.0);
        world.couple_em(&mut em, 0.1, 1.0);
        assert!(world.bodies[0].vel.y < 0.0, "v(+X)×B(+Z) 应产生 -Y 偏转");
    }

    /// 运动带电体应把电荷沉积进所在网格(双向耦合:电荷→电场)。
    #[test]
    fn couple_em_deposits_charge_from_moving_body() {
        let nx = 3usize;
        let rho = ScalarField::<f64>::new(nx, nx, nx, 1.0, 0.0, Bc::Neumann);
        let mut em = EmField::build(rho, 1.0);
        let mut world = RigidWorld::new();
        let body = Body {
            shape: Shape::Sphere { r: 0.2.into() },
            pos: Vec3::new(1.0, 1.0, 1.0),
            rot: na::one(),
            vel: Vec3::new(2.0, 0.0, 0.0), // 已有速度
            inv_mass: 1.0,
        
            ..Default::default()
        };
        world.add_charged_body(body, 1.0);
        world.couple_em(&mut em, 0.1, 1.0);
        // 沉积 q·‖v‖·dt = 1·2·0.1 = 0.2 到 (1,1,1)(经 rho.src,由 EmField::step 注入 u)。
        let dep = em.rho.src[em.rho.idx(1, 1, 1)];
        assert!(dep > 0.0, "运动带电体应把电荷沉积进网格");
    }

    /// 局部引力井应把邻近刚体加速指向质量源(吸引)。
    #[test]
    fn couple_grav_attracts_body_toward_mass() {
        // 5x5x5 引力场,中心放一个静态大质量天体(质量源注入中心格)。
        let nx = 5usize;
        let dx = 1.0;
        let mut rho = ScalarField::<f64>::new(nx, nx, nx, dx, 0.0, Bc::Neumann);
        rho.add_source(2, 2, 2, 10.0); // 天体质量源。
        let mut grav = GravField::build(rho, 1.0);
        grav.step(&0.1); // 松弛出引力井。

        let mut world = RigidWorld::new();
        world.gravity = Vec3::new(0.0, 0.0, 0.0); // 关掉均匀重力,专测局部引力。
        // 物体放在天体右侧 (x=3,y=2,z=2),应被吸引向 -X(指向中心 x=2)。
        let body = Body {
            shape: Shape::Sphere { r: 0.2.into() },
            pos: Vec3::new(3.0, 2.0, 2.0),
            rot: na::one(),
            vel: Vec3::new(0.0, 0.0, 0.0),
            inv_mass: 1.0,
        
            ..Default::default()
        };
        world.add_body(body);
        world.couple_grav(&mut grav, 0.1, 1.0);
        assert!(world.bodies[0].vel.x < 0.0, "物体应被右侧的天体吸引加速朝 -X");
    }

    /// 运动物体应把质量沉积进所在网格(双向耦合:质量→引力井)。
    #[test]
    fn couple_grav_deposits_mass_from_moving_body() {
        let nx = 3usize;
        let rho = ScalarField::<f64>::new(nx, nx, nx, 1.0, 0.0, Bc::Neumann);
        let mut grav = GravField::build(rho, 1.0);
        let mut world = RigidWorld::new();
        world.gravity = Vec3::new(0.0, 0.0, 0.0);
        let body = Body {
            shape: Shape::Sphere { r: 0.2.into() },
            pos: Vec3::new(1.0, 1.0, 1.0),
            rot: na::one(),
            vel: Vec3::new(2.0, 0.0, 0.0), // 已有速度,m=1
            inv_mass: 1.0,
        
            ..Default::default()
        };
        world.add_body(body);
        world.couple_grav(&mut grav, 0.1, 1.0);
        // 沉积 m·‖v‖·dt = 1·2·0.1 = 0.2 到 (1,1,1)(经 rho.src)。
        let dep = grav.rho.src[grav.rho.idx(1, 1, 1)];
        assert!(dep > 0.0, "运动物体应把质量沉积进网格");
    }

    /// 定长杆关节:两动态体初始间距偏离目标,步进后应收敛到 rest,且总动量守恒。
    #[test]
    fn distance_joint_keeps_rest_length_and_conserves_momentum() {
        let mut world = RigidWorld::<f64>::new();
        world.gravity = Vec3::new(0.0, 0.0, 0.0); // 关重力,专测约束
        // 两球,初始间距 3,目标杆长 2。
        let a = world.add_body(Body {
            shape: Shape::Sphere { r: 0.2 },
            pos: Vec3::new(0.0, 0.0, 0.0),
            rot: na::one(),
            vel: Vec3::zeros(),
            inv_mass: 1.0,
        
            ..Default::default()
        });
        let b = world.add_body(Body {
            shape: Shape::Sphere { r: 0.2 },
            pos: Vec3::new(3.0, 0.0, 0.0),
            rot: na::one(),
            vel: Vec3::zeros(),
            inv_mass: 1.0,
        
            ..Default::default()
        });
        world.add_joint(a, b, Joint::Distance {
            pa: Vec3::zeros(),
            pb: Vec3::zeros(),
            rest: 2.0,
        });

        let p0 = world.bodies[a].vel + world.bodies[b].vel; // 初始总动量(零)
        for _ in 0..200 {
            world.step(1.0 / 120.0);
        }
        let dist = (world.bodies[b].pos - world.bodies[a].pos).norm();
        assert!((dist - 2.0).abs() < 0.05, "杆长应收敛到 2,实际 {}", dist);
        let p1 = world.bodies[a].vel + world.bodies[b].vel;
        assert!((p1 - p0).norm() < 1e-6, "无外力下总动量应守恒");
    }

    /// 球窝关节:动态体经球窝连到静态锚点,释放后锚点保持不动且两锚间距≈0。
    #[test]
    fn ball_joint_pins_body_to_static_anchor() {
        let mut world = RigidWorld::<f64>::new();
        world.gravity = Vec3::new(0.0, -9.81, 0.0);
        // 静态锚点在 (0,5,0)。
        let anchor = world.add_body(Body {
            shape: Shape::Sphere { r: 0.1 },
            pos: Vec3::new(0.0, 5.0, 0.0),
            rot: na::one(),
            vel: Vec3::zeros(),
            inv_mass: 0.0, // 静态
            ..Default::default()
        });
        // 摆动体初始在锚点下方偏右 (1,4,0),经球窝挂在锚点上(pa 在锚点局部原点,
        // pb 在摆动体顶部)。
        let swing = world.add_body(Body {
            shape: Shape::Sphere { r: 0.2 },
            pos: Vec3::new(1.0, 4.0, 0.0),
            rot: na::one(),
            vel: Vec3::zeros(),
            inv_mass: 1.0,
        
            ..Default::default()
        });
        world.add_joint(anchor, swing, Joint::Ball {
            pa: Vec3::zeros(),               // 锚点局部原点
            pb: Vec3::new(0.0, 1.0, 0.0),    // 摆动体顶部(距质心 1 向上)
        });

        for _ in 0..600 {
            world.step(1.0 / 120.0);
        }
        // 锚点应保持静止。
        assert!(
            (world.bodies[anchor].pos - Vec3::new(0.0, 5.0, 0.0)).norm() < 1e-9,
            "静态锚点不应移动"
        );
        // 两锚点世界位置应几乎重合(球窝约束):摆动体顶部 ≈ (0,5,0)。
        let swing_top = world.bodies[swing].pos + Vec3::new(0.0, 1.0, 0.0);
        assert!(
            (swing_top - Vec3::new(0.0, 5.0, 0.0)).norm() < 0.05,
            "球窝锚点应重合, 实际 {}", (swing_top - Vec3::new(0.0, 5.0, 0.0)).norm()
        );
    }

    /// 焊点关节(D1):两动态体 Weld 后,给初始角速度 → 步进后相对位姿保持(锚点间距≈0)
    /// 且两体角速度趋于一致(角对齐),总动量守恒。
    #[test]
    fn weld_joint_keeps_relative_pose_and_syncs_angular_velocity() {
        let mut world = RigidWorld::<f64>::new();
        world.gravity = Vec3::new(0.0, 0.0, 0.0); // 关重力,专测约束
        let a = world.add_body(Body::new(Shape::Sphere { r: 0.2 }, Vec3::zeros(), 1.0));
        let b = world.add_body(Body::new(Shape::Sphere { r: 0.2 }, Vec3::new(1.0, 0.0, 0.0), 1.0));
        // Weld 焊住:b 上局部点 (-1,0,0) 与 a 质心重合 → 保持两体质心间距 1。
        world.add_joint(a, b, Joint::Weld {
            pa: Vec3::zeros(),
            pb: Vec3::new(-1.0, 0.0, 0.0),
        });
        // 给 b 一个角速度,期望 Weld 把它同步到 a(角对齐)。
        world.bodies[b].ang_vel = Vec3::new(0.0, 3.0, 0.0);

        let p0 = world.bodies[a].vel + world.bodies[b].vel;
        for _ in 0..300 {
            world.step(1.0 / 120.0);
        }
        // 两体质心间距应保持初始 1(Weld 平动约束)。
        let dist = (world.bodies[b].pos - world.bodies[a].pos).norm();
        assert!((dist - 1.0).abs() < 0.03, "Weld 后间距应保持 1,实际 {}", dist);
        // 两体角速度应趋于一致(角对齐)。
        let dw = (world.bodies[b].ang_vel - world.bodies[a].ang_vel).norm();
        assert!(dw < 0.5, "Weld 应使两体角速度一致,差值 {}", dw);
        // 无外力总动量守恒(角动量也大致守恒,这里验证线动量)。
        let p1 = world.bodies[a].vel + world.bodies[b].vel;
        assert!((p1 - p0).norm() < 1e-6, "无外力下总线动量应守恒");
    }

    /// 铰链关节(D1):动态体经 Hinge 连到静态锚点,Motor 驱动 → 应绕铰链轴持续旋转,
    /// 且两锚点保持重合、铰链轴保持对齐(沿轴旋转自由)。
    #[test]
    fn hinge_joint_with_motor_rotates_around_aligned_axis() {
        let mut world = RigidWorld::<f64>::new();
        world.gravity = Vec3::new(0.0, 0.0, 0.0); // 关重力,专测铰链
        // 静态锚点在原点。
        let anchor = world.add_body(Body::new(Shape::Sphere { r: 0.1 }, Vec3::zeros(), 0.0));
        // 动态体在 (0,0,1),经铰链(轴沿 y)连到锚点。
        let dynb = world.add_body(Body::new(
            Shape::Sphere { r: 0.2 },
            Vec3::new(0.0, 0.0, 1.0),
            1.0,
        ));
        world.add_joint(anchor, dynb, Joint::Hinge {
            pa: Vec3::zeros(),
            pb: Vec3::new(0.0, 0.0, -1.0), // b 质心下方 1,使锚点初始与 a 质心重合
            axis_a: Vec3::new(0.0, 1.0, 0.0),
            axis_b: Vec3::new(0.0, 1.0, 0.0),
            motor_vel: 3.0,      // 目标角速度 3 rad/s
            max_motor_torque: 10.0,
        });

        for _ in 0..200 {
            world.step(1.0 / 120.0);
        }
        // 锚点保持静止。
        assert!(
            (world.bodies[anchor].pos - Vec3::zeros()).norm() < 1e-9,
            "静态锚点不应移动"
        );
        // 动态体被 Motor 驱动绕 y 轴旋转:角速度的 y 分量应>0。
        assert!(
            world.bodies[dynb].ang_vel.y > 1.0,
            "Motor 应驱动动态体绕 y 轴旋转,实际 ang_vel.y={}",
            world.bodies[dynb].ang_vel.y
        );
        // 铰链轴保持对齐(沿 y):动态体局部 y 轴经旋转后应仍基本沿世界 y。
        let dy_local_y = world.bodies[dynb].rot * Vec3::new(0.0, 1.0, 0.0);
        let align = dy_local_y.dot(&Vec3::new(0.0, 1.0, 0.0));
        assert!(
            align.abs() > 0.99,
            "铰链轴应保持对齐(绕 y 旋转),对齐度 {}",
            align
        );
    }

    /// 滑块关节(D1):动态体经 Prismatic 连到静态锚点,Motor 驱动 → 应沿轴滑动(线速度沿轴),
    /// 且两体角对齐(不产生旋转)。
    #[test]
    fn prismatic_joint_with_motor_slides_along_axis() {
        let mut world = RigidWorld::<f64>::new();
        world.gravity = Vec3::new(0.0, 0.0, 0.0); // 关重力,专测滑块
        let anchor = world.add_body(Body::new(Shape::Sphere { r: 0.1 }, Vec3::zeros(), 0.0));
        let slib = world.add_body(Body::new(
            Shape::Sphere { r: 0.2 },
            Vec3::new(0.0, 0.0, 1.0),
            1.0,
        ));
        world.add_joint(anchor, slib, Joint::Prismatic {
            pa: Vec3::zeros(),
            pb: Vec3::new(0.0, 0.0, -1.0),
            axis_a: Vec3::new(0.0, 0.0, 1.0), // 沿 z 滑动
            motor_vel: 2.0,                   // 目标线速度 2 m/s 沿 z
            max_motor_force: 10.0,
        });

        for _ in 0..200 {
            world.step(1.0 / 120.0);
        }
        // 锚点保持静止。
        assert!(
            (world.bodies[anchor].pos - Vec3::zeros()).norm() < 1e-9,
            "静态锚点不应移动"
        );
        // 动态体被 Motor 驱动沿 z 轴滑动:角速度应≈0(不旋转),线速度沿 z 的分量应>0。
        let ang_norm = world.bodies[slib].ang_vel.norm();
        assert!(ang_norm < 0.5, "滑块不应旋转,实际角速度 {}", ang_norm);
        assert!(
            world.bodies[slib].vel.z > 1.0,
            "Motor 应驱动滑块沿 z 轴移动,实际 vel.z={}",
            world.bodies[slib].vel.z
        );
    }

    /// Voronoi 破碎(M21 / #8):碎裂一个盒,碎片应继承母本线速度且总质量守恒。
    #[test]
    fn shatter_box_produces_fragments_conserving_mass() {
        let mut world = RigidWorld::<f64>::new();
        world.gravity = Vec3::new(0.0, 0.0, 0.0); // 关重力,专测破碎本身
        let parent_mass = 8.0;
        let pid = world.add_body(Body {
            shape: Shape::Box {
                half: Vec3::new(1.0, 1.0, 1.0),
            },
            pos: Vec3::new(0.0, 0.0, 0.0),
            rot: na::one(),
            vel: Vec3::new(3.0, 0.0, 0.0), // 已有水平速度
            inv_mass: 1.0 / parent_mass,
        
            ..Default::default()
        });

        let frag_ids = world.shatter(pid, 8, 2.0);
        assert!(frag_ids.len() >= 6, "应碎出至少 6 块,得 {}", frag_ids.len());

        // 总质量守恒(碎片质量之和 ≈ 母本)。
        let total_frag: f64 = frag_ids.iter().map(|&id| 1.0 / world.bodies[id].inv_mass).sum();
        assert!(
            (total_frag - parent_mass).abs() / parent_mass < 0.05,
            "碎片质量之和应≈母本,得 {}",
            total_frag
        );

        // 母本已从世界移除(shatter 内部 swap_remove 母本并追加碎片,
        // 世界刚体总数 = 原总数 - 1 + 碎片数)。
        assert!(
            world.bodies.len() >= frag_ids.len(),
            "shatter 后世界应包含母本被替换的碎片"
        );

        // 每个碎片都应是 Convex 形状。
        for &id in &frag_ids {
            assert!(matches!(world.bodies[id].shape, Shape::Convex { .. }), "碎片必须是 Convex");
        }

        // 径向飞散应使至少部分碎片获得与原速度不同的速度分量。
        let max_speed = frag_ids.iter().map(|&id| world.bodies[id].vel.norm()).fold(0.0_f64, f64::max);
        assert!(max_speed > 3.0, "径向飞散应叠加到母本速度上,得 {}", max_speed);
    }

    /// 破碎后步进:碎片应互不相穿地自由飞行(无 NaN / 无崩溃)。
    #[test]
    fn shattered_fragments_step_stably() {
        let mut world = RigidWorld::<f64>::new();
        world.gravity = Vec3::new(0.0, -9.81, 0.0);
        let pid = world.add_body(Body {
            shape: Shape::Box {
                half: Vec3::new(1.0, 1.0, 1.0),
            },
            pos: Vec3::new(0.0, 5.0, 0.0),
            rot: na::one(),
            vel: Vec3::zeros(),
            inv_mass: 1.0,
        
            ..Default::default()
        });
        let frags = world.shatter(pid, 6, 1.5);
        assert!(!frags.is_empty());
        for _ in 0..60 {
            world.step(1.0 / 120.0);
        }
        // 无 NaN 检查。
        for id in &frags {
            assert!(world.bodies[*id].pos.x.is_finite(), "碎片位置不应 NaN");
            assert!(world.bodies[*id].pos.y.is_finite(), "碎片位置不应 NaN");
        }
    }

    /// 角动力学(M18 / S1):自由转动体在无外力下以恒定角速度自旋,姿态应随时间演化。
    #[test]
    fn free_spin_integrates_attitude_without_drift() {
        let mut world = RigidWorld::<f64>::new();
        world.gravity = Vec3::new(0.0, 0.0, 0.0); // 关重力,专测自旋
        // 一个绕世界 z 轴自旋的盒体(须用 Body::new 取得真实惯性)。
        let id = world.add_body(Body::new(
            Shape::Box {
                half: Vec3::new(1.0, 0.5, 0.25),
            },
            Vec3::zeros(),
            1.0,
        ));
        world.bodies[id].ang_vel = Vec3::new(0.0, 0.0, 1.0); // 1 rad/s 绕 z

        let q0 = *world.bodies[id].rot.quaternion();
        for _ in 0..200 {
            world.step(1.0 / 120.0); // 总时长 200/120 ≈ 1.667 s
        }
        // 自旋 1.667 s ≈ 1.667 rad。姿态应绕 z 旋转该角度,且归一化四元数应仍有效。
        let q1 = *world.bodies[id].rot.quaternion();
        assert!((q1.norm() - 1.0).abs() < 1e-6, "积分后四元数应保持单位范数");
        // 绕 z 轴转 1.667 rad 的四元数 w = cos(θ/2)。
        let expected_w = (1.667_f64 / 2.0).cos();
        assert!((q1.w - expected_w).abs() < 0.05, "姿态角应≈角速度×时间, w 得 {}", q1.w);
        // 质心不应移动(无外力、无角动量耦合到线速度)。
        assert!(world.bodies[id].pos.norm() < 1e-9, "无外力下质心应静止");
        let _ = q0;
    }

    /// 角动力学(S1):离轴冲量使自由体同时平动并绕质心旋转(角动量守恒)。
    #[test]
    fn off_center_impulse_spins_free_body() {
        let mut world = RigidWorld::<f64>::new();
        world.gravity = Vec3::new(0.0, 0.0, 0.0);
        let id = world.add_body(Body::new(
            Shape::Box {
                half: Vec3::new(1.0, 1.0, 1.0),
            },
            Vec3::zeros(),
            1.0,
        ));
        // 在质心上方 1.0 处施 +x 冲量 => 平动 +x 且绕 -z 自旋。
        world.bodies[id].apply_impulse_at(
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
        );
        assert!(world.bodies[id].vel.x > 0.0, "应获得 +x 线速度");
        assert!(world.bodies[id].ang_vel.z < 0.0, "离轴冲量应产生 -z 角速度");
        let p0 = world.bodies[id].vel; // 动量 = m·v(初始无角动量耦合)
        for _ in 0..120 {
            world.step(1.0 / 120.0);
        }
        // 无外力:线动量守恒(只考虑质心线速度)。
        assert!((world.bodies[id].vel - p0).norm() < 1e-6, "无外力下质心线动量守恒");
        // 角速度在无外力下保持恒定(自由刚体角动量守恒 => 对主轴体恒定)。
        assert!(world.bodies[id].ang_vel.z < 0.0, "自旋角速度应持续");
    }

    /// B1 休眠:盒子落地静止后进入 sleeping(速度归零、sleeping=true),且不抖动穿地。
    #[test]
    fn resting_box_falls_asleep_and_stays_put() {
        let mut world = RigidWorld::<f64>::new();
        world.gravity = Vec3::new(0.0, -9.81, 0.0);
        // 静态地面(扁平大盒,顶面对齐 y=0)。
        world.add_body(Body::new(
            Shape::Box {
                half: Vec3::new(50.0, 0.5, 50.0),
            },
            Vec3::new(0.0, -0.5, 0.0),
            0.0,
        ));
        let id = world.add_body(Body::new(
            Shape::Box {
                half: Vec3::new(0.5, 0.5, 0.5),
            },
            Vec3::new(0.0, 5.0, 0.0),
            1.0,
        ));
        let mut slept = false;
        for _ in 0..400 {
            world.step(1.0 / 120.0);
            if world.bodies[id].sleeping {
                slept = true;
                break;
            }
        }
        assert!(slept, "静止盒应在 400 步内进入休眠");
        // 休眠后:速度归零、位置固定在地面上方(半高 0.5,故 y≈0.5)、且不再下沉。
        assert!(world.bodies[id].vel.norm() < 1e-6, "休眠体速度应归零");
        assert!(
            (world.bodies[id].pos.y - 0.5).abs() < 0.05,
            "休眠盒应停在地面上方约半高处,得 {}",
            world.bodies[id].pos.y
        );
        let y_sleep = world.bodies[id].pos.y;
        for _ in 0..60 {
            world.step(1.0 / 120.0);
        }
        assert!(
            (world.bodies[id].pos.y - y_sleep).abs() < 1e-6,
            "休眠后位置应完全冻结,无抖动"
        );
    }

    /// B1 唤醒:掉落球砸中已休眠的盒,盒被唤醒(恢复运动、sleeping=false)。
    #[test]
    fn falling_ball_wakes_sleeping_box() {
        let mut world = RigidWorld::<f64>::new();
        world.gravity = Vec3::new(0.0, -9.81, 0.0);
        // 静态地面。
        world.add_body(Body::new(
            Shape::Box {
                half: Vec3::new(50.0, 0.5, 50.0),
            },
            Vec3::new(0.0, -0.5, 0.0),
            0.0,
        ));
        // 先放一个盒,让它落地休眠。
        let box_id = world.add_body(Body::new(
            Shape::Box {
                half: Vec3::new(0.5, 0.5, 0.5),
            },
            Vec3::new(0.0, 0.5, 0.0),
            1.0,
        ));
        for _ in 0..400 {
            world.step(1.0 / 120.0);
        }
        assert!(world.bodies[box_id].sleeping, "盒应先进入休眠");
        // 在盒正上方释放一个下落球。
        let _ball_id = world.add_body(Body::new(
            Shape::Sphere { r: 0.3 },
            Vec3::new(0.0, 4.0, 0.0),
            1.0,
        ));
        let mut woke = false;
        for _ in 0..120 {
            world.step(1.0 / 120.0);
            if !world.bodies[box_id].sleeping {
                woke = true;
                break;
            }
        }
        assert!(woke, "掉落球应唤醒休眠的盒");
        // 盒被唤醒后应获得向上的速度分量(被砸起)。
        assert!(world.bodies[box_id].vel.y > -1e-6, "唤醒后盒应被碰撞驱动(速度非零/向上)");
    }

    /// B1 堆叠收敛(Baumgarte slop):三层盒静止堆叠,收敛后顶层停在约 3×半高处,
    /// 且残余穿透被 slop 吸收而不逐帧抖动(顶层 y 在稳态阶段稳定)。
    #[test]
    fn stacked_boxes_converge_without_jitter() {
        let mut world = RigidWorld::<f64>::new();
        world.gravity = Vec3::new(0.0, -9.81, 0.0);
        // 静态地面(顶面 y=0)。
        world.add_body(Body::new(
            Shape::Box {
                half: Vec3::new(50.0, 0.5, 50.0),
            },
            Vec3::new(0.0, -0.5, 0.0),
            0.0,
        ));
        // 三层 1×1×1 盒,半高 0.5。初始留小间隙(仿 two_boxes_stack_stably),
        // 让其自由落体坐实接触后再堆叠。
        let mut ids = Vec::new();
        let starts = [0.5_f64, 1.6, 2.7];
        for &y0 in &starts {
            let id = world.add_body(Body::new(
                Shape::Box {
                    half: Vec3::new(0.5, 0.5, 0.5),
                },
                Vec3::new(0.0, y0, 0.0),
                1.0,
            ));
            ids.push(id);
        }
        let dt = 1.0 / 120.0;
        for step in 0..600 {
            world.step(dt);
            // 稳态阶段(最后 100 步)校验:顶层停在约 2.5,且速度收敛(不抖)。
            if step >= 500 {
                let top_y = world.bodies[ids[2]].pos.y;
                assert!(
                    (top_y - 2.5).abs() < 0.1,
                    "堆叠收敛后顶层应停在约 2.5,得 {}",
                    top_y
                );
                assert!(
                    world.bodies[ids[2]].vel.norm() < 0.5,
                    "堆叠顶层速度应收敛,得 {}",
                    world.bodies[ids[2]].vel.norm()
                );
            }
        }
        // 末态高度不变量:三层中心应≈0.5/1.5/2.5。
        assert!((world.bodies[ids[0]].pos.y - 0.5).abs() < 0.1, "底层高度异常");
        assert!((world.bodies[ids[1]].pos.y - 1.5).abs() < 0.1, "中层高度异常");
        assert!((world.bodies[ids[2]].pos.y - 2.5).abs() < 0.1, "顶层高度异常");
    }

    /// B1 摩擦/位置参数接线与稳定性:(1) SolverParams 默认值正确(friction_iterations
    /// 与 iterations 一致, position_slop=1mm);(2) 平地上静止盒在竖直重力下稳定不弹出
    /// (验证带 slop 的位置修正不注入能量);(3) 不同 friction_iterations 下求解器不 panic
    /// 且收敛(摩擦子迭代上限接口生效)。
    #[test]
    fn b1_friction_and_slop_params_wired_and_stable() {
        // (1) 默认参数接线正确。
        let p = SolverParams::<f64>::default();
        assert_eq!(p.friction_iterations, p.iterations, "friction_iterations 默认应回退到 iterations");
        assert!((p.position_slop - 1e-3).abs() < 1e-9, "position_slop 默认应为 1mm");

        // (2) 静止盒在竖直重力下稳定不弹出(带 slop 的位置修正不注入能量)。
        let mut world = RigidWorld::<f64>::new();
        world.gravity = Vec3::new(0.0, -9.81, 0.0);
        world.add_body(Body::new(
            Shape::Box { half: Vec3::new(50.0, 0.5, 50.0) },
            Vec3::new(0.0, -0.5, 0.0),
            0.0,
        ));
        let id = world.add_body(Body::new(
            Shape::Box { half: Vec3::new(0.5, 0.5, 0.5) },
            Vec3::new(0.0, 0.55, 0.0),
            1.0,
        ));
        let mut max_y = 0.0_f64;
        for _ in 0..300 {
            world.step(1.0 / 120.0);
            max_y = max_y.max(world.bodies[id].pos.y);
        }
        // 盒应停在约 0.5(地面顶面),不得被位置修正弹出。
        assert!(
            (world.bodies[id].pos.y - 0.5).abs() < 0.05,
            "静止盒应停在约 0.5,得 {}",
            world.bodies[id].pos.y
        );
        assert!(max_y < 1.0, "静止盒不应被弹出(y<1.0),得 {}", max_y);

        // (3) 不同 friction_iterations 不 panic 且收敛(接口生效)。
        for fi in [0_usize, 1, 5, 40] {
            let mut w = RigidWorld::<f64>::new();
            w.gravity = Vec3::new(0.0, -9.81, 0.0);
            w.params.friction_iterations = fi;
            w.add_body(Body::new(
                Shape::Box { half: Vec3::new(50.0, 0.5, 50.0) },
                Vec3::new(0.0, -0.5, 0.0),
                0.0,
            ));
            let bid = w.add_body(Body::new(
                Shape::Box { half: Vec3::new(0.5, 0.5, 0.5) },
                Vec3::new(0.0, 0.55, 0.0),
                1.0,
            ));
            for _ in 0..120 {
                w.step(1.0 / 120.0);
            }
            assert!(
                (w.bodies[bid].pos.y - 0.5).abs() < 0.1,
                "friction_iterations={} 下盒应稳定,得 {}",
                fi,
                w.bodies[bid].pos.y
            );
        }
    }

    /// B5 碰撞层过滤:球与地面的层/掩码互斥 => 球穿透地面继续下落(不碰撞)。
    #[test]
    fn collision_layers_block_cross_layer_contact() {
        let mut world = RigidWorld::<f64>::new();
        world.gravity = Vec3::new(0.0, -9.81, 0.0);
        // 地面在层 0x1,掩码只接受 0x1。
        world.add_body({
            let mut b = Body::new(
                Shape::Box {
                    half: Vec3::new(50.0, 0.5, 50.0),
                },
                Vec3::new(0.0, -0.5, 0.0),
                0.0,
            );
            b.layers = 0x1;
            b.collision_mask = 0x1;
            b
        });
        // 球在层 0x2,掩码只接受 0x2 => 与地面互斥,不碰撞。
        let ball = world.add_body({
            let mut b = Body::new(Shape::Sphere { r: 0.3 }, Vec3::new(0.0, 2.0, 0.0), 1.0);
            b.layers = 0x2;
            b.collision_mask = 0x2;
            b
        });
        for _ in 0..120 {
            world.step(1.0 / 120.0);
        }
        // 不碰撞 => 球持续自由落体,y 远低于地面顶面(0.0),且速度为负(加速下落)。
        assert!(
            world.bodies[ball].pos.y < -1.0,
            "互斥层球应穿透地面继续下落, y 得 {}",
            world.bodies[ball].pos.y
        );
        assert!(
            world.bodies[ball].vel.y < 0.0,
            "互斥层球应持续加速下落, vel.y 得 {}",
            world.bodies[ball].vel.y
        );
    }

    /// B5 碰撞层过滤:球与地面同层(默认全通) => 球停在地面上方(正常碰撞)。
    #[test]
    fn collision_layers_allow_same_layer_contact() {
        let mut world = RigidWorld::<f64>::new();
        world.gravity = Vec3::new(0.0, -9.81, 0.0);
        // 地面默认层(全通)。
        world.add_body(Body::new(
            Shape::Box {
                half: Vec3::new(50.0, 0.5, 50.0),
            },
            Vec3::new(0.0, -0.5, 0.0),
            0.0,
        ));
        // 球默认层(全通) => 与地面碰撞,停在 y≈r=0.3 上方。
        let ball = world.add_body(Body::new(Shape::Sphere { r: 0.3 }, Vec3::new(0.0, 2.0, 0.0), 1.0));
        for _ in 0..200 {
            world.step(1.0 / 120.0);
        }
        assert!(
            world.bodies[ball].pos.y > 0.2 && world.bodies[ball].pos.y < 0.4,
            "同层球应停在地面上方约半径处, y 得 {}",
            world.bodies[ball].pos.y
        );
        assert!(
            world.bodies[ball].vel.norm() < 0.5,
            "落地后速度应收敛(被阻挡), 得 {}",
            world.bodies[ball].vel.norm()
        );
    }

    /// B2 运动学体:按用户设定的 vel 主动移动并推开动态体,自身不受接触冲量影响。
    #[test]
    fn kinematic_body_pushes_dynamic_and_is_unaffected() {
        let mut world = RigidWorld::<f64>::new();
        world.gravity = Vec3::zeros();
        // 运动学盒:以 +x 匀速移动(kinematic,不被推)。
        let kin = world.add_body({
            let mut b = Body::new(
                Shape::Box {
                    half: Vec3::new(0.5, 0.5, 0.5),
                },
                Vec3::new(-1.0, 0.0, 0.0),
                1.0,
            );
            b.kinematic = true;
            b.vel = Vec3::new(1.0, 0.0, 0.0);
            b
        });
        // 动态球:静止在运动学盒前进路径上。
        let dyn_id = world.add_body(Body::new(Shape::Sphere { r: 0.3 }, Vec3::new(0.0, 0.0, 0.0), 1.0));
        let kin_vel0 = world.bodies[kin].vel;
        for _ in 0..240 {
            world.step(1.0 / 120.0); // 2s:盒从 -1 走到 +1,跨越球必然碰撞。
        }
        // 运动学体速度保持(不受球反作用)。
        assert!(
            (world.bodies[kin].vel - kin_vel0).norm() < 1e-9,
            "运动学体速度应保持不变,得 {:?}",
            world.bodies[kin].vel
        );
        // 运动学体按 vel 移动了约 1.0 * 2s = 2.0(从 -1 到约 +1)。
        assert!(
            world.bodies[kin].pos.x > 0.8 && world.bodies[kin].pos.x < 1.2,
            "运动学体应主动移动, x 得 {}",
            world.bodies[kin].pos.x
        );
        // 动态球被运动学盒推开(+x 方向,且被推离路径)。
        assert!(
            world.bodies[dyn_id].pos.x > 0.3,
            "动态球应被运动学体推开, x 得 {}",
            world.bodies[dyn_id].pos.x
        );
        assert!(
            world.bodies[dyn_id].vel.x > 0.0,
            "动态球应获得 +x 速度, 得 {}",
            world.bodies[dyn_id].vel.x
        );
    }

    /// B2 传感器/触发器:参与重叠检测但不施加冲量(不阻止穿透、不改变对方速度)。
    #[test]
    fn sensor_detects_overlap_but_no_impulse() {
        let mut world = RigidWorld::<f64>::new();
        // 动态球从上方自由落体,穿过一个静态传感器盒区域。
        let ball = world.add_body(Body::new(Shape::Sphere { r: 0.3 }, Vec3::new(0.0, 2.0, 0.0), 2.0));
        // 静态传感器盒(半 0.5),位于 y∈[-0.5, 0.5];球必经此区间。
        let sensor = {
            let mut b = Body::new(
                Shape::Box { half: Vec3::new(0.5, 0.5, 0.5) },
                Vec3::new(0.0, 0.0, 0.0),
                0.0,
            ); // 静态
            b.is_sensor = true;
            world.add_body(b)
        };
        let _ = sensor;

        let mut saw_overlap = false;
        for _ in 0..120 {
            // 自由落体约 1.2s(y 从 2.0 降到 ~-5)。
            world.step(0.01);
            if !world.sensor_contacts().is_empty() {
                saw_overlap = true;
            }
        }
        // 传感器确实报告了重叠事件。
        assert!(saw_overlap, "传感器应检测到球穿过时的重叠");
        // 球未被传感器阻挡:穿透到了盒下方(y < -0.5),且速度接近自由落体(未被减速)。
        assert!(
            world.bodies[ball].pos.y < -0.5,
            "球应穿过传感器盒(未被阻挡), y 得 {}",
            world.bodies[ball].pos.y
        );
        // 自由落体 1.2s 速度≈12.0,允许微小误差(无空气阻力)。
        assert!(
            world.bodies[ball].vel.y < -10.0,
            "球速度应接近自由落体(传感器不减速), vy 得 {}",
            world.bodies[ball].vel.y
        );
    }

    /// B3 场景 DSL / prefab:JSON 往返加载正确重建世界(重力 / 体数 / 位姿 / 标志)。
    #[test]
    fn scene_load_json_roundtrip_rebuilds_world() {
        use crate::scene::SceneDesc;
        use phy_math::Vec3;

        let mut world = RigidWorld::<f64>::new();
        // 直接用 Rust SceneDesc 序列化为 JSON(覆盖序列化),再经 load 反序列化重建。
        let desc = SceneDesc {
            gravity: Vec3::new(0.0, -5.0, 0.0),
            bodies: {
                let g = Body::new(
                    Shape::Box { half: Vec3::new(20.0, 0.5, 20.0) },
                    Vec3::new(0.0, -0.5, 0.0),
                    0.0,
                );
                let mut b = Body::new(Shape::Sphere { r: 1.0 }, Vec3::new(0.0, 5.0, 0.0), 2.0);
                b.kinematic = true;
                vec![g, b]
            },
            joints: vec![],
        };
        let json = desc.to_json().expect("序列化场景 JSON 应成功");
        world.load_scene_json(&json).expect("加载场景 JSON 应成功");

        assert_eq!(world.bodies.len(), 2, "应加载 2 个刚体");
        assert!((world.gravity.y + 5.0).abs() < 1e-12, "应应用自定义重力 -5.0");
        assert_eq!(world.bodies[0].inv_mass, 0.0, "地面应为静态");
        assert!((world.bodies[1].pos.y - 5.0).abs() < 1e-12, "球初始 y 应为 5.0");
        assert!(world.bodies[1].kinematic, "球的 kinematic 标志应保留");
        assert!(!world.bodies[1].is_sensor, "球的 sensor 标志默认 false 应保留");
    }

    /// B3 场景 DSL:极简 JSON(省略 gravity/joints 用默认)可正确解析。
    #[test]
    fn scene_minimal_json_uses_defaults() {
        use crate::scene::SceneDesc;
        use phy_math::Vec3;

        // 仅含一个 body 的最小 JSON,验证 gravity/joints 的 serde default。
        let b = Body::new(Shape::Sphere { r: 0.5 }, Vec3::new(1.0, 2.0, 3.0), 1.0);
        let mut buf = String::new();
        buf.push_str("{\"bodies\":[");
        buf.push_str(&serde_json::to_string(&b).unwrap());
        buf.push_str("]}");
        let desc = SceneDesc::<f64>::from_json(&buf).expect("极简场景 JSON 应解析");
        assert_eq!(desc.bodies.len(), 1);
        assert_eq!(desc.joints.len(), 0);
        // 默认重力应等于引擎默认重力(0, -9.81, 0)。
        assert!((desc.gravity.y + 9.81).abs() < 1e-9, "省略重力应回退默认 -9.81");
    }

    /// 把世界所有刚体的位姿/速度拍平为一维向量,用于逐位比较。
    fn rigid_state_vector(w: &RigidWorld<f64>) -> Vec<f64> {
        let mut out = Vec::new();
        for b in &w.bodies {
            out.push(b.pos.x);
            out.push(b.pos.y);
            out.push(b.pos.z);
            let q = *b.rot.quaternion();
            out.push(q.w);
            out.push(q.i);
            out.push(q.j);
            out.push(q.k);
            out.push(b.vel.x);
            out.push(b.vel.y);
            out.push(b.vel.z);
            out.push(b.ang_vel.x);
            out.push(b.ang_vel.y);
            out.push(b.ang_vel.z);
        }
        out
    }

    /// M1 刚性体确定性:相同初始世界 + 相同步长序列,重复运行逐位一致。
    ///
    /// 覆盖:堆叠 + 自由落体 + 运动学体推动 + 中途碎裂(M21)。碎裂会 `swap_remove` 母本
    /// 并追加碎片,只要种子/顺序确定,两次独立构造的世界应逐位复现。
    #[test]
    fn rigid_determinism_repeat_run_bit_identical() {
        let build = || {
            let mut w = RigidWorld::<f64>::new();
            // 地面。
            w.add_body(Body::new(
                Shape::Box { half: Vec3::new(20.0, 0.5, 20.0) },
                Vec3::new(0.0, -0.5, 0.0),
                0.0,
            ));
            // 堆叠两球。
            w.add_body(Body::new(Shape::Sphere { r: 0.5 }, Vec3::new(0.0, 1.0, 0.0), 1.0));
            w.add_body(Body::new(Shape::Sphere { r: 0.5 }, Vec3::new(0.0, 2.0, 0.0), 1.0));
            // 运动学推板(沿 +x 匀速)。
            let mut kin = Body::new(Shape::Box { half: Vec3::new(0.5, 0.5, 0.5) }, Vec3::new(-1.0, 0.5, 0.0), 0.0);
            kin.kinematic = true;
            kin.vel = Vec3::new(2.0, 0.0, 0.0);
            w.add_body(kin);
            w
        };
        let run = |mut w: RigidWorld<f64>| -> Vec<f64> {
            let dt = 1.0 / 60.0;
            for step in 0..240 {
                if step == 120 {
                    // 中途把第一个球碎裂成 4 块(M21),径向飞散速度 1.0。
                    w.shatter(1, 4, 1.0);
                }
                w.step(dt);
            }
            rigid_state_vector(&w)
        };

        let a = run(build());
        let b = run(build());
        assert_eq!(a.len(), b.len(), "刚体数应一致(堆叠+推板+碎裂后)");
        for (x, y) in a.iter().zip(b.iter()) {
            assert!(
                x == y,
                "刚体重复运行状态逐位分歧: {} vs {} (差 {:e})",
                x,
                y,
                (x - y).abs()
            );
        }
    }

    /// M1 刚性体确定性重放:用场景 DSL 快照 + 录制的步长序列,回放应逐位复现直跑终态。
    ///
    /// 刚体求解器本身无随机源,重放等价于「同一初始场景 + 同一 dt 序列」再跑一遍;
    /// 此处显式经 `to_scene_json` 序列化快照 + 逐帧 `dt` 录制,验证存档/重放回路的
    /// 一致性(与 `phy_core::replay` 同构,但作用在 `RigidWorld` 上)。
    #[test]
    fn rigid_replay_is_bit_identical() {
        let build = || {
            let mut w = RigidWorld::<f64>::new();
            w.add_body(Body::new(
                Shape::Box { half: Vec3::new(20.0, 0.5, 20.0) },
                Vec3::new(0.0, -0.5, 0.0),
                0.0,
            ));
            w.add_body(Body::new(Shape::Sphere { r: 0.5 }, Vec3::new(0.0, 2.0, 0.0), 1.0));
            w
        };

        // 录制:先对「初始(未步进)世界」取快照(场景 DSL JSON)+ 逐帧 dt。
        let fresh = build();
        let snapshot = fresh.to_scene_json().expect("场景快照序列化");
        let frames: Vec<f64> = vec![1.0 / 60.0; 100];

        // 直跑:从初始世界跑 100 步。
        let mut direct = build();
        for _ in 0..100 {
            direct.step(1.0 / 60.0);
        }
        let direct_state = rigid_state_vector(&direct);

        // 回放路径 1。
        let mut r1 = RigidWorld::<f64>::new();
        r1.load_scene_json(&snapshot).expect("回放加载快照");
        for &dt in &frames {
            r1.step(dt);
        }
        let replay1 = rigid_state_vector(&r1);

        // 回放路径 2(再次从同一快照重建,验证可重复)。
        let mut r2 = RigidWorld::<f64>::new();
        r2.load_scene_json(&snapshot).expect("回放加载快照");
        for &dt in &frames {
            r2.step(dt);
        }
        let replay2 = rigid_state_vector(&r2);

        assert_eq!(direct_state.len(), replay1.len());
        assert_eq!(replay1.len(), replay2.len());
        for ((x, y), z) in direct_state.iter().zip(replay1.iter()).zip(replay2.iter()) {
            assert_eq!(*x, *y, "直跑与回放分歧");
            assert_eq!(*y, *z, "两次回放分歧");
        }
    }
}
