//! 射线投射车辆(raycast vehicle)(M20 / 路线图 #9)。
//!
//! 经典 Bullet 式 `btRaycastVehicle` 的精简实现:车身是一个普通刚体
//! (加入 `RigidWorld`),每个车轮从悬挂顶端沿车身局部 -Y 向下打射线找地面,
//! 命中后按悬挂压缩量施加弹簧-阻尼力(沿地面法线)托住车身,并在接触点施加
//! 轮胎力(纵向引擎/刹车 + 横向转向摩擦)。所有力都注入到**车身刚体**的速度
//! (`vel += F/m · dt`,与 `couple_*` 系列同款力注入方式),车身再参与常规碰撞。
//!
//! 本引擎刚体未建模角速度,故车辆姿态由悬挂在四角的对称托举自然维持直立
//! (悬挂力在四轮均匀分布),轮胎力作为线性加速度作用于车身质心。足以支撑
//! 驾驶/加速/刹车/转向的动力学行为,可作为游戏向车辆基础。
//!
//! 用法:`Vehicle::new(chassis_idx, wheels)` 构造,每帧 `world.step` 之前调用
//! `vehicle.update(&mut world, dt)` 注入悬挂 + 轮胎力,然后用控制输入
//! (`set_engine` / `set_steering` / `set_brake`)驱动。

use phy_math::{RealField, Vec3};
use serde::de::DeserializeOwned;
use serde::{Deserialize, Serialize};

use crate::raycast::ray_cast;
use crate::world::RigidWorld;

/// 单个车轮的静态参数与运行时状态。
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(bound = "T: RealField + Copy + Serialize + DeserializeOwned")]
pub struct Wheel<T: RealField + Copy> {
    /// 车轮在车身局部坐标的悬挂顶端(射线起点)。
    #[serde(with = "crate::shape::serde_geom")]
    pub anchor: Vec3<T>,
    /// 悬挂自然长度(从顶端到轮心的静止距离)。
    pub suspension_rest: T,
    /// 悬挂刚度(弹簧系数 k)。
    pub suspension_k: T,
    /// 悬挂阻尼系数(c)。
    pub suspension_c: T,
    /// 车轮半径(用于把轮心落到地面之上 + 接触判定)。
    pub radius: T,
    /// 轮胎纵向抓地(引擎/刹车力缩放)。
    pub traction: T,
    /// 轮胎横向抓地(转向摩擦缩放)。
    pub grip: T,
    /// 当前悬挂压缩量(运行时,= rest - 实际长度,≥0 表示受压)。只读。
    pub compression: T,
    /// 当前是否接地(运行时)。只读。
    pub grounded: bool,
}

impl<T: RealField + Copy> Wheel<T> {
    /// 构造一个车轮。
    pub fn new(
        anchor: Vec3<T>,
        suspension_rest: T,
        suspension_k: T,
        suspension_c: T,
        radius: T,
        traction: T,
        grip: T,
    ) -> Self {
        Self {
            anchor,
            suspension_rest,
            suspension_k,
            suspension_c,
            radius,
            traction,
            grip,
            compression: T::zero(),
            grounded: false,
        }
    }
}

/// 射线投射车辆。
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(bound = "T: RealField + Copy + Serialize + DeserializeOwned")]
pub struct Vehicle<T: RealField + Copy> {
    /// 车身刚体在 `RigidWorld.bodies` 中的索引。
    pub chassis: usize,
    /// 车轮列表。
    pub wheels: Vec<Wheel<T>>,
    /// 引擎力(纵向驱动,正值前进)。由 `set_engine` 设置。
    pub engine: T,
    /// 转向角(弧度,正值右转)。由 `set_steering` 设置。
    pub steering: T,
    /// 刹车力(纵向减速)。由 `set_brake` 设置。
    pub brake: T,
    /// 车身局部"前进"方向(默认 +X),用于把引擎力映射到世界。
    #[serde(with = "crate::shape::serde_geom")]
    pub forward_axis: Vec3<T>,
}

impl<T: RealField + Copy> Vehicle<T> {
    /// 构造车辆。
    pub fn new(chassis: usize, wheels: Vec<Wheel<T>>) -> Self {
        Self {
            chassis,
            wheels,
            engine: T::zero(),
            steering: T::zero(),
            brake: T::zero(),
            forward_axis: Vec3::new(T::one(), T::zero(), T::zero()),
        }
    }

    /// 设置引擎输出(纵向驱动力,世界单位质量·加速度)。
    pub fn set_engine(&mut self, f: T) {
        self.engine = f;
    }
    /// 设置转向角(弧度)。
    pub fn set_steering(&mut self, s: T) {
        self.steering = s;
    }
    /// 设置刹车力(纵向减速加速度量级)。
    pub fn set_brake(&mut self, b: T) {
        self.brake = b;
    }

    /// 推进车辆一帧:对每个车轮 raycast 找地面,施加悬挂 + 轮胎力到车身。
    ///
    /// 必须在 `world.step(dt)` **之前**调用(力注入到 `vel`,由 step 的重力积分
    /// 之后、碰撞之前生效 —— 与 `couple_*` 顺序一致)。
    pub fn update(&mut self, world: &mut RigidWorld<T>, dt: T) {
        let cid = self.chassis;
        if world.bodies[cid].inv_mass <= T::zero() {
            return; // 静态车身不参与。
        }
        let chassis_pos = world.bodies[cid].pos;
        let chassis_rot = world.bodies[cid].rot;
        let chassis_inv_mass = world.bodies[cid].inv_mass;
        let m = T::one() / chassis_inv_mass; // 车身质量(用于把"减速度"转成力)。

        let down_local = Vec3::new(T::zero(), -T::one(), T::zero());
        let down_world = chassis_rot * down_local;

        let mut total_force = Vec3::zeros();

        for w in self.wheels.iter_mut() {
            // 射线起点 = 车身变换后的悬挂顶端;只向下打射线找最近的静态地面。
            let origin = chassis_rot * w.anchor + chassis_pos;
            if let Some(hit) = Self::ray_cast_ground(world, &origin, &down_world) {
                // 命中:计算悬挂实际长度 = 命中距离 - 轮半径(轮心在地面上方 radius)。
                let susp_len = hit.t - w.radius;
                if susp_len < w.suspension_rest {
                    let compression = w.suspension_rest - susp_len;
                    w.compression = compression;
                    w.grounded = true;
                    // 悬挂弹簧力(沿地面法线向上托举)。
                    let spring = w.suspension_k * compression;
                    // 阻尼:车身竖直速度沿法线分量。
                    let v_at = world.bodies[cid].vel;
                    let vn = v_at.dot(&hit.normal);
                    let damper = w.suspension_c * vn;
                    let susp_force = hit.normal * (spring - damper);

                    // 轮胎力(投影到地面切平面):
                    // 纵向 = 引擎/刹车沿车身前进方向(投影到地面)。
                    let fwd = (chassis_rot * self.forward_axis).normalize();
                    let fwd_ground = fwd - hit.normal * fwd.dot(&hit.normal);
                    let fwd_ground = if fwd_ground.norm() > T::from_f64(1e-6).unwrap() {
                        fwd_ground.normalize()
                    } else {
                        fwd_ground
                    };
                    let mut tire = fwd_ground * (self.engine * w.traction);
                    // 刹车:抵消车身沿 fwd_ground 的速度分量。乘以质量 m 使刹车成为
                    // "减速度"系数(与质量无关),否则对重车身几乎无效。
                    if self.brake > T::zero() {
                        let vlong = v_at.dot(&fwd_ground);
                        tire -= fwd_ground * (vlong * self.brake * m);
                    }

                    // 横向(转向)摩擦:把车身侧向速度朝转向方向引导。
                    let right = fwd_ground.cross(&hit.normal); // 地面右向。
                    let right = if right.norm() > T::from_f64(1e-6).unwrap() {
                        right.normalize()
                    } else {
                        right
                    };
                    let vlat = v_at.dot(&right);
                    // 转向产生期望横向加速度 = steering * 纵向速度(自行车模型)。
                    let vlong_now = v_at.dot(&fwd_ground);
                    let desired_lat = self.steering * vlong_now;
                    let lat_force = (desired_lat - vlat) * w.grip;
                    tire += right * lat_force;

                    total_force += susp_force + tire;
                } else {
                    w.compression = T::zero();
                    w.grounded = false;
                }
            } else {
                w.compression = T::zero();
                w.grounded = false;
            }
        }

        // 注入到车身速度(与 couple_* 同款)。
        world.bodies[cid].vel += total_force * chassis_inv_mass * dt;
    }

    /// 在所有静态(`inv_mass==0`)刚体中找最近命中射线的地面,返回命中结果。
    /// 仅对静态物体打射线(地面/墙体),动态物体由常规碰撞处理。
    fn ray_cast_ground(world: &RigidWorld<T>, origin: &Vec3<T>, dir: &Vec3<T>) -> Option<crate::raycast::RayHit<T>> {
        let mut best: Option<crate::raycast::RayHit<T>> = None;
        for b in world.bodies.iter() {
            if b.inv_mass <= T::zero() {
                if let Some(h) = ray_cast(origin, dir, b) {
                    if best.is_none() || h.t < best.unwrap().t {
                        best = Some(h);
                    }
                }
            }
        }
        best
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::shape::{Body, Shape};
    use phy_math::na;

    fn ground_box() -> Body<f64> {
        Body {
            shape: Shape::Box {
                half: Vec3::new(1000.0, 0.5, 1000.0),
            },
            pos: Vec3::new(0.0, -0.5, 0.0),
            rot: na::one(),
            vel: Vec3::zeros(),
            inv_mass: 0.0, // 静态地面
            ..Default::default()
        }
    }

    fn chassis() -> Body<f64> {
        Body {
            shape: Shape::Box {
                half: Vec3::new(1.0, 0.25, 0.5),
            },
            pos: Vec3::new(0.0, 1.0, 0.0), // 悬空在地面之上。
            rot: na::one(),
            vel: Vec3::zeros(),
            inv_mass: 1.0 / 500.0, // 500 kg 车身。
            ..Default::default()
        }
    }

    #[test]
    fn vehicle_suspension_holds_chassis_above_ground() {
        let mut world = RigidWorld::<f64>::new();
        world.gravity = Vec3::new(0.0, -9.81, 0.0);
        let gid = world.add_body(ground_box());
        let cid = world.add_body(chassis());

        // 4 轮:四角,悬挂自然长度 0.6,刚度足够托住车身。
        let rest = 0.6;
        let k = 8000.0;
        let c = 800.0;
        let r = 0.3;
        let wheels = vec![
            Wheel::new(Vec3::new(-0.9, -0.25, 0.4), rest, k, c, r, 1.0, 1.0),
            Wheel::new(Vec3::new(0.9, -0.25, 0.4), rest, k, c, r, 1.0, 1.0),
            Wheel::new(Vec3::new(-0.9, -0.25, -0.4), rest, k, c, r, 1.0, 1.0),
            Wheel::new(Vec3::new(0.9, -0.25, -0.4), rest, k, c, r, 1.0, 1.0),
        ];
        let mut veh = Vehicle::new(cid, wheels);

        // 演化:每帧先 update(注入悬挂力)再 step。
        let dt = 1.0 / 120.0;
        for _ in 0..600 {
            veh.update(&mut world, dt);
            world.step(dt);
        }
        // 稳态:车身应被悬挂托在地面上方(底盘底约在 y≈ground_top + ride_height)。
        let chassis_y = world.bodies[cid].pos.y;
        // 底盘半高 0.25 + 轮半径 0.3 + 悬挂受压但 > 0。最低应明显高于地面顶 (0)。
        assert!(chassis_y > 0.3, "车身应被悬挂托离地面, y={}", chassis_y);
        assert!(chassis_y < 1.5, "车身不应飞起, y={}", chassis_y);
        // 竖直速度应基本收敛(不抖动发散)。
        assert!(world.bodies[cid].vel.y.abs() < 2.0, "竖直速度应收敛");
        // 至少应有轮接地。
        assert!(veh.wheels.iter().any(|w| w.grounded), "应有车轮接地");
        let _ = gid;
    }

    #[test]
    fn vehicle_engine_drives_forward() {
        let mut world = RigidWorld::<f64>::new();
        world.gravity = Vec3::new(0.0, -9.81, 0.0);
        world.add_body(ground_box());
        let cid = world.add_body(chassis());

        let rest = 0.6;
        let k = 8000.0;
        let c = 800.0;
        let r = 0.3;
        let wheels = vec![
            Wheel::new(Vec3::new(-0.9, -0.25, 0.4), rest, k, c, r, 1.0, 1.0),
            Wheel::new(Vec3::new(0.9, -0.25, 0.4), rest, k, c, r, 1.0, 1.0),
            Wheel::new(Vec3::new(-0.9, -0.25, -0.4), rest, k, c, r, 1.0, 1.0),
            Wheel::new(Vec3::new(0.9, -0.25, -0.4), rest, k, c, r, 1.0, 1.0),
        ];
        let mut veh = Vehicle::new(cid, wheels);
        veh.set_engine(3000.0); // 正向引擎力。

        let dt = 1.0 / 120.0;
        // 先稳定着地。
        for _ in 0..200 {
            veh.update(&mut world, dt);
            world.step(dt);
        }
        let x0 = world.bodies[cid].pos.x;
        let vx0 = world.bodies[cid].vel.x;
        for _ in 0..200 {
            veh.update(&mut world, dt);
            world.step(dt);
        }
        let x1 = world.bodies[cid].pos.x;
        let vx1 = world.bodies[cid].vel.x;
        // 引擎应驱动车身沿 +X 前进(vx 为正、x 增加)。
        assert!(vx1 > 0.0, "引擎应使车速沿 +X 为正, vx={}", vx1);
        assert!(x1 > x0, "引擎应使车身前进, dx={}", x1 - x0);
        let _ = vx0;
    }

    #[test]
    fn vehicle_brake_slows_down() {
        let mut world = RigidWorld::<f64>::new();
        world.gravity = Vec3::new(0.0, -9.81, 0.0);
        world.add_body(ground_box());
        let cid = world.add_body(chassis());

        let rest = 0.6;
        let k = 8000.0;
        let c = 800.0;
        let r = 0.3;
        let wheels = vec![
            Wheel::new(Vec3::new(-0.9, -0.25, 0.4), rest, k, c, r, 1.0, 1.0),
            Wheel::new(Vec3::new(0.9, -0.25, 0.4), rest, k, c, r, 1.0, 1.0),
            Wheel::new(Vec3::new(-0.9, -0.25, -0.4), rest, k, c, r, 1.0, 1.0),
            Wheel::new(Vec3::new(0.9, -0.25, -0.4), rest, k, c, r, 1.0, 1.0),
        ];
        let mut veh = Vehicle::new(cid, wheels);
        veh.set_engine(3000.0);

        let dt = 1.0 / 120.0;
        for _ in 0..400 {
            veh.update(&mut world, dt);
            world.step(dt);
        }
        let v_fast = world.bodies[cid].vel.x;
        // 踩刹车。
        veh.set_brake(5.0);
        veh.set_engine(0.0);
        for _ in 0..200 {
            veh.update(&mut world, dt);
            world.step(dt);
        }
        let v_slow = world.bodies[cid].vel.x;
        assert!(v_slow < v_fast, "刹车应使车速下降, {} -> {}", v_fast, v_slow);
        assert!(v_slow < v_fast * 0.5, "刹车应大幅减速");
    }
}
