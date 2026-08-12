//! B3 场景描述 DSL / prefab(商用游戏引擎补齐计划 §11)。
//!
//! 作者侧用一份 JSON 描述一整个关卡/关卡片段:重力向量、刚体列表、关节列表,
//! 经 `RigidWorld::load_scene_json` 一次性替换世界内容(等价于"加载 prefab")。
//! 也可经 `RigidWorld::to_scene_json` 把当前世界导出为 JSON 存档(往返一致)。
//!
//! 复用已 serde 化的 `Body<T>` / `JointConstraint<T>`,因此描述与引擎内部数据同构,
//! 无需镜像字段、不会信息丢失(姿态四元数、逆惯性张量、碰撞层、kinematic/sensor
//! 标志等全部保留)。

use phy_math::{gravity, RealField, Vec3};
use serde::de::DeserializeOwned;
use serde::{Deserialize, Serialize};

use crate::character_controller::CharacterController;
use crate::joint::JointConstraint;
use num_traits::NumCast;
use crate::shape::Body;
use crate::world::RigidWorld;

/// D4 角色出生点:场景 DSL 中声明一个受控 kinematic 胶囊体(角色)。
///
/// 加载时由 `RigidWorld::load_scene_json` 实例化为 `CharacterController` 并绑定到世界;
/// 运行时由游戏循环每帧 `world.character.as_mut().unwrap().update(&mut world, dt, dir, jump)`
/// 再 `world.step(dt)`。所有字段省略则用 `CharacterController::default()` 值。
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(bound = "T: RealField + Copy + Serialize + DeserializeOwned")]
pub struct CharacterSpawn<T: RealField + Copy> {
    /// 出生位置(胶囊中心)。
    #[serde(with = "crate::shape::serde_geom")]
    pub pos: Vec3<T>,
    /// 胶囊半高(沿局部 y 轴),省略 = 1.0。
    #[serde(default = "f_one")]
    pub half_height: T,
    /// 胶囊半径,省略 = 0.4。
    #[serde(default = "f_point_four")]
    pub radius: T,
    /// 水平移动速度(m/s),省略 = 4.0。
    #[serde(default = "f_four")]
    pub speed: T,
    /// 跳跃初速度(m/s,>0 才可跳),省略 = 0(不可跳)。
    #[serde(default = "f_zero")]
    pub jump_speed: T,
    /// 重力加速度(覆盖世界重力对角色的影响),省略 = -9.81。
    #[serde(default = "f_neg_nine_eight_one")]
    pub gravity: T,
}

fn f_one<T: RealField + Copy>() -> T { T::from_f64(1.0).unwrap() }
fn f_point_four<T: RealField + Copy>() -> T { T::from_f64(0.4).unwrap() }
fn f_four<T: RealField + Copy>() -> T { T::from_f64(4.0).unwrap() }
fn f_zero<T: RealField + Copy>() -> T { T::zero() }
fn f_neg_nine_eight_one<T: RealField + Copy>() -> T { T::from_f64(-9.81).unwrap() }

/// 场景描述(作者侧 DSL 的根节点)。
///
/// JSON 形态示例:
/// ```json
/// {
///   "gravity": [0.0, -9.81, 0.0],
///   "bodies": [
///     { "shape": {"Box":{"half":[20.0,0.5,20.0]}}, "pos":[0.0,-0.5,0.0],
///       "rot":[1,0,0,0], "vel":[0,0,0], "ang_vel":[0,0,0],
///       "inv_inertia_local":[0,0,0,0,0,0,0,0,0], "inv_mass":0.0,
///       "sleeping":false, "sleep_time":0.0, "layers":4294967295,
///       "collision_mask":4294967295, "kinematic":false, "is_sensor":false },
///     { "shape": {"Sphere":{"r":1.0}}, "pos":[0.0,5.0,0.0], "rot":[1,0,0,0],
///       "vel":[0,0,0], "ang_vel":[0,0,0], "inv_inertia_local":[...],
///       "inv_mass":2.0, "sleeping":false, "sleep_time":0.0,
///       "layers":4294967295, "collision_mask":4294967295,
///       "kinematic":false, "is_sensor":false }
///   ],
///   "joints": [],
///   "character_spawn": { "pos":[0.0,3.0,0.0], "speed":5.0, "jump_speed":6.0 }
/// }
/// ```
/// `gravity` 与 `joints` 可省略(默认引擎重力、空关节);`character_spawn` 省略则无角色。
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(bound = "T: RealField + Copy + Serialize + DeserializeOwned")]
pub struct SceneDesc<T: RealField + Copy> {
    /// 世界重力(省略则用引擎默认重力)。
    #[serde(default = "gravity_field", with = "crate::shape::serde_geom")]
    pub gravity: Vec3<T>,
    /// 刚体列表(顺序即 `RigidWorld::bodies` 索引,关节据此引用)。
    pub bodies: Vec<Body<T>>,
    /// 关节列表(引用 `bodies` 的索引)。
    #[serde(default)]
    pub joints: Vec<JointConstraint<T>>,
    /// D4 角色出生点(省略则无角色)。
    #[serde(default)]
    pub character_spawn: Option<CharacterSpawn<T>>,
}

fn gravity_field<T: RealField + Copy>() -> Vec3<T> {
    gravity::<T>()
}

impl<T: RealField + Copy + Serialize + DeserializeOwned> SceneDesc<T> {
    /// 从 JSON 字符串解析场景描述。
    pub fn from_json(s: &str) -> Result<Self, serde_json::Error> {
        serde_json::from_str(s)
    }

    /// 序列化为 JSON 字符串(便于存档/调试)。
    pub fn to_json(&self) -> Result<String, serde_json::Error> {
        serde_json::to_string_pretty(self)
    }
}

impl<T: RealField + Copy> RigidWorld<T> {
    /// B3 从 JSON 场景描述**替换**整个世界内容(加载 prefab / 关卡)。
    ///
    /// 清空现有 bodies/joints/charges,按描述重建;`gravity` 设为场景指定值。
    /// 电荷向量(`charges`)按 body 数量填零(场景描述不表达电荷,保持默认中性)。
    ///
    /// 返回 `Err` 若 JSON 解析失败(世界保持调用前状态 — 因为先解析后替换)。
    pub fn load_scene_json(&mut self, json: &str) -> Result<(), serde_json::Error>
    where
        T: Serialize + DeserializeOwned + NumCast,
    {
        let desc: SceneDesc<T> = SceneDesc::from_json(json)?;
        self.bodies = desc.bodies;
        self.charges = vec![T::zero(); self.bodies.len()];
        self.joints = desc.joints;
        self.gravity = desc.gravity;
        self.last_sensor_contacts.clear();
        // D4 角色:按 character_spawn 实例化并绑定 kinematic 胶囊体。
        self.character = desc.character_spawn.map(|sp| {
            let mut cc = CharacterController::new(self, sp.pos);
            cc.half_height = sp.half_height;
            cc.radius = sp.radius;
            cc.speed = sp.speed;
            cc.jump_speed = sp.jump_speed;
            cc.gravity = sp.gravity;
            cc
        });
        Ok(())
    }

    /// B3 把当前世界导出为场景 JSON(存档 / 关卡往返)。
    ///
    /// 注意:仅导出描述性状态(bodies / joints / gravity);运行时瞬态
    /// (`last_sensor_contacts`、求解器内部伪速度等)不进存档。
    pub fn to_scene_json(&self) -> Result<String, serde_json::Error>
    where
        T: Serialize + DeserializeOwned,
    {
        let character_spawn = self.character.as_ref().map(|cc| CharacterSpawn {
            pos: self.bodies[cc.body_id.unwrap()].pos,
            half_height: cc.half_height,
            radius: cc.radius,
            speed: cc.speed,
            jump_speed: cc.jump_speed,
            gravity: cc.gravity,
        });
        let desc = SceneDesc {
            gravity: self.gravity,
            bodies: self.bodies.clone(),
            joints: self.joints.clone(),
            character_spawn,
        };
        desc.to_json()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::shape::Shape;
    use phy_math::Vec3;

    /// D4 场景 DSL:`character_spawn` 加载后实例化为 `world.character`,且 `to_scene_json`
    /// 能反向还原出生点(往返一致)。用 Rust 构造 `SceneDesc` 再序列化,避开手写 serde 细节。
    #[test]
    fn scene_dsl_character_spawn_roundtrip() {
        // 构造场景:静态地面 + 角色出生点。
        let ground = Body {
            shape: Shape::Box { half: Vec3::new(20.0, 0.5, 20.0) },
            pos: Vec3::new(0.0, -0.5, 0.0),
            inv_mass: 0.0,
            ..Default::default()
        };
        let desc = SceneDesc {
            gravity: Vec3::new(0.0, -9.81, 0.0),
            bodies: vec![ground],
            joints: vec![],
            character_spawn: Some(CharacterSpawn {
                pos: Vec3::new(0.0, 3.0, 0.0),
                half_height: 1.0,
                radius: 0.4,
                speed: 5.0,
                jump_speed: 6.0,
                gravity: -9.81,
            }),
        };
        // 序列化 → 反序列化往返(验证 DSL 自洽)。
        let json = desc.to_json().expect("序列化应成功");
        let desc2: SceneDesc<f64> = SceneDesc::from_json(&json).expect("重新解析应成功");
        let sp = desc2.character_spawn.expect("应含 character_spawn");
        assert!((sp.pos.y - 3.0).abs() < 1e-9, "pos.y 应还原为 3.0");
        assert!((sp.speed - 5.0).abs() < 1e-9, "speed 应还原为 5.0");
        assert!((sp.jump_speed - 6.0).abs() < 1e-9, "jump_speed 应还原为 6.0");

        // 用 JSON 加载到世界:实例化为角色控制器。
        let mut world = RigidWorld::<f64>::new();
        world.load_scene_json(&json).expect("加载场景 JSON 应成功");
        let cc = world.character.as_ref().expect("应创建角色控制器");
        assert_eq!(cc.body_id, Some(1), "角色体应绑定为第 2 个 body(index 1)");
        assert!((cc.speed - 5.0).abs() < 1e-9, "speed 应为 5.0");
        assert!((cc.jump_speed - 6.0).abs() < 1e-9, "jump_speed 应为 6.0");
        assert!(!cc.grounded, "初始未着地");

        // 反向导出:character_spawn 应被还原。
        let out = world.to_scene_json().expect("导出应成功");
        let desc3: SceneDesc<f64> = SceneDesc::from_json(&out).expect("重新解析应成功");
        let sp3 = desc3.character_spawn.expect("应含 character_spawn");
        assert!((sp3.pos.y - 3.0).abs() < 1e-9, "pos.y 应还原为 3.0");

        // 角色能落地(端到端跑几步,无 NaN)。
        let mut cc = world.character.take().unwrap();
        for _ in 0..240 {
            cc.update(&mut world, 1.0 / 120.0, Vec3::zeros(), false);
            world.step(1.0 / 120.0);
        }
        assert!(cc.grounded, "角色应落到地面");
        assert!(cc.position(&world).y.is_finite(), "位置应有限(无 NaN)");
    }
}
