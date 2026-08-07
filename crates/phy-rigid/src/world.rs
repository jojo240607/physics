//! RigidWorld: 刚体动力学世界(M2)。
//!
//! `step` 流程:
//! 1. 积分速度(施加重力)
//! 2. Broad-phase + Narrow-phase 求所有接触
//! 3. 顺序冲量法求解速度(接触 + 摩擦)
//! 4. 积分位置(用求解后的速度)
//! 5. 位置修正(防止穿透累积)

use phy_math::{gravity, RealField, Vec3};

use crate::broadphase::broadphase;
use crate::contact::Contact;
use crate::narrowphase::collide;
use crate::shape::Body;
use crate::solver::{solve_position, solve_velocity, ContactConstraint, SolverParams};

/// 刚体动力学世界。
pub struct RigidWorld<T: RealField + Copy> {
    pub bodies: Vec<Body<T>>,
    /// 重力(默认沿 -Y)。
    pub gravity: Vec3<T>,
    /// 求解参数。
    pub params: SolverParams<T>,
}

impl<T: RealField + Copy> Default for RigidWorld<T> {
    fn default() -> Self {
        Self::new()
    }
}

impl<T: RealField + Copy> RigidWorld<T> {
    pub fn new() -> Self {
        Self {
            bodies: Vec::new(),
            gravity: gravity::<T>(),
            params: SolverParams::default(),
        }
    }

    pub fn add_body(&mut self, b: Body<T>) -> usize {
        self.bodies.push(b);
        self.bodies.len() - 1
    }

    /// 推进一步。返回本步检测到的接触(供调试/渲染)。
    ///
    /// 标准半隐式欧拉 + 顺序冲量流程:
    /// 1. 积分速度(重力) 2. detect(当前位置) 3. 求解速度冲量
    /// 4. 积分位置(用求解后速度) 5. 位置投影(清残余穿透)
    pub fn step(&mut self, dt: T) -> Vec<Contact<T>> {
        // 1. 积分速度(重力)
        for b in self.bodies.iter_mut() {
            if b.inv_mass > T::zero() {
                b.vel += self.gravity * dt;
            }
        }

        // 2. 碰撞检测(当前位置)
        let pairs = broadphase(&self.bodies);
        let mut constraints: Vec<ContactConstraint<T>> = Vec::new();
        for (i, j) in pairs {
            if let Some(c) = collide(&self.bodies[i], &self.bodies[j]) {
                constraints.push(ContactConstraint::new(i, j, c));
            }
        }

        // 3. 速度求解(顺序冲量)
        solve_velocity(&mut self.bodies, &mut constraints, &self.params);

        // 4. 积分位置(用求解后速度)
        for b in self.bodies.iter_mut() {
            if b.inv_mass > T::zero() {
                b.pos += b.vel * dt;
            }
        }

        // 5. 位置修正(split impulse 伪速度):解伪速度使物体分离,
        //    伪速度只用于修正位置,不污染真实速度(避免抖动/能量注入)。
        let mut pseudo: Vec<Vec3<T>> = vec![Vec3::zeros(); self.bodies.len()];
        let beta = T::from_f64(0.2).unwrap();
        let beta_over_dt = beta / dt;
        solve_position(
            &self.bodies,
            &constraints,
            &mut pseudo,
            beta_over_dt,
        );
        for (i, b) in self.bodies.iter_mut().enumerate() {
            if b.inv_mass > T::zero() {
                b.pos += pseudo[i] * dt;
            }
        }

        constraints.into_iter().map(|c| c.contact).collect()
    }
}
