//! RigidWorld: 刚体动力学世界(M2)。
//!
//! `step` 流程:
//! 1. 积分速度(施加重力)
//! 2. Broad-phase + Narrow-phase 求所有接触
//! 3. 顺序冲量法求解速度(接触 + 摩擦)
//! 4. 积分位置(用求解后的速度)
//! 5. 位置修正(防止穿透累积)

use phy_field::{EmFieldLike, GravFieldLike, HeatFieldLike};
use phy_math::{gravity, RealField, Vec3};

use crate::broadphase::broadphase;
use crate::contact::Contact;
use crate::joint::{solve_joints_position, solve_joints_velocity, Joint, JointConstraint};
use crate::narrowphase::collide;
use crate::shape::Body;
use crate::solver::{solve_position, solve_velocity, ContactConstraint, SolverParams};

/// 刚体动力学世界。
pub struct RigidWorld<T: RealField + Copy> {
    pub bodies: Vec<Body<T>>,
    /// 每个刚体的电荷量(与 `bodies` 等长,0 = 中性)。用于电磁耦合。
    pub charges: Vec<T>,
    /// 关节约束(M18):把刚体连成链条/摆/机械结构。
    pub joints: Vec<JointConstraint<T>>,
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
            joints: Vec::new(),
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
        // 3b. 关节速度求解(与接触同构的顺序冲量,消除关节相对漂移速度)。
        solve_joints_velocity(&mut self.bodies, &mut self.joints, self.params.iterations);

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
        // 5b. 关节位置投影(split-impulse 伪速度):把残余关节距离误差消除而不污染真实速度。
        solve_joints_position(&self.bodies, &self.joints, &mut pseudo, beta_over_dt);
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

#[cfg(test)]
mod tests {
    use super::*;
    use phy_core::Subsystem;
    use phy_field::{Bc, EmField, GravField, HeatField, ScalarField};
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
        });
        let b = world.add_body(Body {
            shape: Shape::Sphere { r: 0.2 },
            pos: Vec3::new(3.0, 0.0, 0.0),
            rot: na::one(),
            vel: Vec3::zeros(),
            inv_mass: 1.0,
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
        });
        // 摆动体初始在锚点下方偏右 (1,4,0),经球窝挂在锚点上(pa 在锚点局部原点,
        // pb 在摆动体顶部)。
        let swing = world.add_body(Body {
            shape: Shape::Sphere { r: 0.2 },
            pos: Vec3::new(1.0, 4.0, 0.0),
            rot: na::one(),
            vel: Vec3::zeros(),
            inv_mass: 1.0,
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
}
