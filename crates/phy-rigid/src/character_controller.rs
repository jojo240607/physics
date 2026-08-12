//! D4 角色控制器:基于 kinematic 胶囊体 + 手动碰撞检测的地面移动/跳跃。
//!
//! 角色用 `Shape::Capsule` 的 kinematic 体表示(不被接触冲量驱动,由本控制器直接控制速度),
//! `update` 每帧:重力积分 → 水平移动 → 与地形(Box/Heightfield/Capsule/Compound)碰撞检测,
//! 依据接触法线把角色停在表面(落地 vel.y=0 / 撞墙 vel.xz 抵消),再交给 `RigidWorld::step`
//! 推进并推开动态体。D1(D4 依赖 Hinge)+D2(Capsule)完成后,可再接 ragdoll。
use crate::narrowphase::collide;
use crate::shape::{Body, Shape};
use crate::world::RigidWorld;
use num_traits::NumCast;
use phy_math::{RealField, Vec3};

/// 角色控制器:控制一个 kinematic 胶囊体在场景中移动/跳跃。
#[derive(Debug, Clone)]
pub struct CharacterController<T: RealField + Copy> {
    /// 胶囊半高(沿局部 y 轴)。
    pub half_height: T,
    /// 胶囊半径。
    pub radius: T,
    /// 水平移动速度(m/s)。
    pub speed: T,
    /// 跳跃初速度(m/s, >0 才可跳)。
    pub jump_speed: T,
    /// 绑定的 kinematic 体 id。
    pub body_id: Option<usize>,
    /// 是否着地(上一帧)。
    pub grounded: bool,
    /// 重力加速度(默认 -9.81,可覆盖)。
    pub gravity: T,
    /// 角色自身的垂直速度状态(m/s,跨帧累积重力;不写回 body 的 vel)。
    pub vel_y: T,
}

impl<T: RealField + Copy + NumCast> Default for CharacterController<T> {
    fn default() -> Self {
        Self {
            half_height: T::from_f64(1.0).unwrap(),
            radius: T::from_f64(0.4).unwrap(),
            speed: T::from_f64(4.0).unwrap(),
            jump_speed: T::zero(),
            body_id: None,
            grounded: false,
            gravity: T::from_f64(-9.81).unwrap(),
            vel_y: T::zero(),
        }
    }
}

impl<T: RealField + Copy + NumCast> CharacterController<T> {
    /// 创建并绑定一个 kinematic 胶囊体到 world。
    pub fn new(world: &mut RigidWorld<T>, pos: Vec3<T>) -> Self {
        let id = world.add_body(Body {
            shape: Shape::Capsule {
                half_height: Self::default().half_height,
                r: Self::default().radius,
            },
            pos,
            rot: phy_math::na::UnitQuaternion::identity(),
            inv_mass: T::zero(), // kinematic:反质量 0
            kinematic: true,
            ..Default::default()
        });
        Self {
            body_id: Some(id),
            ..Default::default()
        }
    }

    /// 当前角色位置(若无绑定返回原点)。
    pub fn position(&self, world: &RigidWorld<T>) -> Vec3<T> {
        match self.body_id {
            Some(id) => world.bodies[id].pos,
            None => Vec3::zeros(),
        }
    }

    /// 角色是否着地。
    pub fn is_grounded(&self) -> bool {
        self.grounded
    }

    /// 更新一帧:水平移动 `move_dir`(xz 方向),`want_jump` 触发跳跃。
    /// 计算期望位移 → 与地形碰撞检测并修正位置(落地/撞墙/爬坡) → **直接写入角色 pos**,
    /// 并设 vel=0 避免 world.step 的 kinematic 双重积分;step 只处理"角色位置重叠推开动态体"。
    pub fn update(
        &mut self,
        world: &mut RigidWorld<T>,
        dt: T,
        move_dir: Vec3<T>,
        want_jump: bool,
    ) {
        let id = match self.body_id {
            Some(id) => id,
            None => return,
        };
        // 水平期望位移(归一化 move_dir,0 则不动)。
        let mut dx = T::zero();
        let mut dz = T::zero();
        if move_dir.norm_squared() > T::from_f64(1e-9).unwrap() {
            let n = move_dir.normalize();
            dx = n.x * self.speed * dt;
            dz = n.z * self.speed * dt;
        }
        // 垂直:未着地累积重力速度;着地归零。跳跃覆盖。
        // 注:着地时若 vel_y 归零,本帧 dy=0 不再下穿,slide 无法确认接触 → grounded 会丢失。
        // 故着地时额外施加一个微小向下探测位移(probe),让 slide 仍能检测到接触并维持 grounded;
        // 若角色已走下台阶(探测不再穿透),slide 返回未着地 → 自然开始下落。
        let probe = T::from_f64(0.02).unwrap();
        if self.grounded {
            self.vel_y = T::zero();
        } else {
            self.vel_y = self.vel_y + self.gravity * dt;
        }
        if want_jump && self.grounded && self.jump_speed > T::zero() {
            self.vel_y = self.jump_speed;
            self.grounded = false;
        }
        let dy = if self.grounded {
            // 着地态:用探测位移确认接触,vel_y 保持 0。
            -probe
        } else {
            self.vel_y * dt
        };
        // slide:检测角色移动后的穿透,修正位置(落地/撞墙)。
        let (final_pos, grounded) = self.slide(world, id, dx, dy, dz);
        world.bodies[id].pos = final_pos;
        // vel 设 0:角色位置已由 update 直接控制,step 不额外积分 kinematic 体。
        world.bodies[id].vel = Vec3::zeros();
        if grounded {
            self.vel_y = T::zero(); // 落地停止下落
        }
        self.grounded = grounded;
    }

    /// 检测角色从当前位置移动 (dx,dy,dz) 后是否与场景穿透,沿接触法线推出修正位置。
    /// 法线朝上置 grounded。返回 (修正后位置, 是否着地)。
    fn slide(
        &self,
        world: &RigidWorld<T>,
        id: usize,
        dx: T,
        dy: T,
        dz: T,
    ) -> (Vec3<T>, bool) {
        let mut pos = world.bodies[id].pos + Vec3::new(dx, dy, dz);
        let mut grounded = false;
        // 多轮迭代把角色推出穿透(最多 4 轮)。
        for _ in 0..4 {
            let mut pen: Option<(Vec3<T>, T)> = None;
            for (j, other) in world.bodies.iter().enumerate() {
                if j == id {
                    continue;
                }
                let role = Body {
                    shape: Shape::Capsule {
                        half_height: self.half_height,
                        r: self.radius,
                    },
                    pos,
                    rot: phy_math::na::UnitQuaternion::identity(),
                    inv_mass: T::zero(),
                    kinematic: true,
                    ..Default::default()
                };
                if let Some(c) = collide(&role, other) {
                    if c.depth > T::zero()
                        && pen.as_ref().map(|(_, d)| c.depth > *d).unwrap_or(true)
                    {
                        // 推出方向 = -c.normal(把角色从 other 推出)。
                        pen = Some((-c.normal, c.depth));
                    }
                }
            }
            match pen {
                Some((n, depth)) => {
                    pos = pos + n * depth;
                    // 推出方向朝上 → 角色在下方体之上 → 着地。
                    if n.y > T::from_f64(0.5).unwrap() {
                        grounded = true;
                    }
                }
                None => break,
            }
        }
        (pos, grounded)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::shape::Shape;
    use phy_math::Vec3;

    /// 静态地面。
    fn ground<T: RealField + Copy + NumCast>(world: &mut RigidWorld<T>) {
        let h = T::from_f64(0.5).unwrap();
        world.add_body(Body {
            shape: Shape::Box {
                half: Vec3::new(T::from_f64(50.0).unwrap(), h, T::from_f64(50.0).unwrap()),
            },
            pos: Vec3::new(T::zero(), -h, T::zero()),
            rot: phy_math::na::UnitQuaternion::identity(),
            inv_mass: T::zero(),
            ..Default::default()
        });
    }

    /// 角色从空中下落,应停在地面(grounded),且 y 稳定在胶囊半高+半径处。
    #[test]
    fn character_falls_to_ground_and_grounded() {
        let mut world = RigidWorld::<f64>::new();
        world.gravity = Vec3::new(0.0, -9.81, 0.0);
        ground(&mut world);
        let mut cc = CharacterController::new(&mut world, Vec3::new(0.0, 3.0, 0.0));
        cc.half_height = 1.0;
        cc.radius = 0.4;
        let dt = 1.0 / 120.0;
        for _ in 0..300 {
            cc.update(&mut world, dt, Vec3::zeros(), false);
            world.step(dt);
        }
        assert!(cc.grounded, "角色应着地");
        // 角色底部 = pos.y - (h+r) = 地面顶 0.0;pos.y 应 ≈ 1.4。
        let y = cc.position(&world).y;
        assert!(
            (y - 1.4).abs() < 0.1,
            "角色应停在地面上(pos.y≈1.4),实际 {}",
            y
        );
    }

    /// 着地后跳跃:y 应先上升(跳起)再回落到地面。
    #[test]
    fn character_can_jump() {
        let mut world = RigidWorld::<f64>::new();
        world.gravity = Vec3::new(0.0, -9.81, 0.0);
        ground(&mut world);
        let mut cc = CharacterController::new(&mut world, Vec3::new(0.0, 3.0, 0.0));
        cc.half_height = 1.0;
        cc.radius = 0.4;
        cc.jump_speed = 5.0;
        let dt = 1.0 / 120.0;
        // 先落地。
        for _ in 0..300 {
            cc.update(&mut world, dt, Vec3::zeros(), false);
            world.step(dt);
        }
        let y0 = cc.position(&world).y;
        assert!(cc.grounded, "落地后应着地");
        // 跳一次。
        let mut max_y = y0;
        for _ in 0..120 {
            cc.update(&mut world, dt, Vec3::zeros(), true);
            world.step(dt);
            max_y = max_y.max(cc.position(&world).y);
        }
        assert!(max_y > y0 + 0.5, "跳跃应使角色上升, max_y={} 起跳 y={}", max_y, y0);
        // 跳后应回落(最终 y 回到地面附近)。
        let y_end = cc.position(&world).y;
        assert!(
            (y_end - y0).abs() < 0.3,
            "跳后应回落, y_end={} 起跳 y0={}",
            y_end,
            y0
        );
    }

    /// 水平移动:给 move_dir(+x),角色应沿 x 移动(地面高度不变)。
    #[test]
    fn character_moves_horizontally() {
        let mut world = RigidWorld::<f64>::new();
        world.gravity = Vec3::new(0.0, -9.81, 0.0);
        ground(&mut world);
        let mut cc = CharacterController::new(&mut world, Vec3::new(0.0, 3.0, 0.0));
        cc.half_height = 1.0;
        cc.radius = 0.4;
        cc.speed = 2.0;
        let dt = 1.0 / 120.0;
        // 先落地。
        for _ in 0..200 {
            cc.update(&mut world, dt, Vec3::zeros(), false);
            world.step(dt);
        }
        let y0 = cc.position(&world).y;
        // 沿 +x 移动 1 秒。
        for _ in 0..120 {
            cc.update(&mut world, dt, Vec3::new(1.0, 0.0, 0.0), false);
            world.step(dt);
        }
        let x = cc.position(&world).x;
        assert!(x > 1.0, "角色应沿 +x 移动, 实际 x={}", x);
        // 高度应保持(在地面上走,不坠落)。
        let y = cc.position(&world).y;
        assert!((y - y0).abs() < 0.2, "移动时高度应保持, y={} 起 y0={}", y, y0);
    }
}

