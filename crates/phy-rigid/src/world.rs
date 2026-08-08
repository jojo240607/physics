//! RigidWorld: 刚体动力学世界(M2)。
//!
//! `step` 流程:
//! 1. 积分速度(施加重力)
//! 2. Broad-phase + Narrow-phase 求所有接触
//! 3. 顺序冲量法求解速度(接触 + 摩擦)
//! 4. 积分位置(用求解后的速度)
//! 5. 位置修正(防止穿透累积)

use phy_field::EmFieldLike;
use phy_field::HeatFieldLike;
use phy_math::{gravity, RealField, Vec3};

use crate::broadphase::broadphase;
use crate::contact::Contact;
use crate::narrowphase::collide;
use crate::shape::Body;
use crate::solver::{solve_position, solve_velocity, ContactConstraint, SolverParams};

/// 刚体动力学世界。
pub struct RigidWorld<T: RealField + Copy> {
    pub bodies: Vec<Body<T>>,
    /// 每个刚体的电荷量(与 `bodies` 等长,0 = 中性)。用于电磁耦合。
    pub charges: Vec<T>,
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
            charges: Vec::new(),
            gravity: gravity::<T>(),
            params: SolverParams::default(),
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
}

#[cfg(test)]
mod tests {
    use super::*;
    use phy_field::{Bc, EmField, HeatField, ScalarField};
    use phy_math::na;

    use crate::shape::Shape;

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
        let mut rho = ScalarField::<f64>::new(nx, nx, nx, dx, 0.0, Bc::Neumann);
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
        let mut rho = ScalarField::<f64>::new(nx, nx, nx, 1.0, 0.0, Bc::Neumann);
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
        };
        world.add_charged_body(body, 1.0);
        world.couple_em(&mut em, 0.1, 1.0);
        assert!(world.bodies[0].vel.y < 0.0, "v(+X)×B(+Z) 应产生 -Y 偏转");
    }

    /// 运动带电体应把电荷沉积进所在网格(双向耦合:电荷→电场)。
    #[test]
    fn couple_em_deposits_charge_from_moving_body() {
        let nx = 3usize;
        let mut rho = ScalarField::<f64>::new(nx, nx, nx, 1.0, 0.0, Bc::Neumann);
        let mut em = EmField::build(rho, 1.0);
        let mut world = RigidWorld::new();
        let body = Body {
            shape: Shape::Sphere { r: 0.2.into() },
            pos: Vec3::new(1.0, 1.0, 1.0),
            rot: na::one(),
            vel: Vec3::new(2.0, 0.0, 0.0), // 已有速度
            inv_mass: 1.0,
        };
        world.add_charged_body(body, 1.0);
        world.couple_em(&mut em, 0.1, 1.0);
        // 沉积 q·‖v‖·dt = 1·2·0.1 = 0.2 到 (1,1,1)(经 rho.src,由 EmField::step 注入 u)。
        let dep = em.rho.src[em.rho.idx(1, 1, 1)];
        assert!(dep > 0.0, "运动带电体应把电荷沉积进网格");
    }
}
