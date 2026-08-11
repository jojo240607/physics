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

use crate::joint::JointConstraint;
use crate::shape::Body;
use crate::world::RigidWorld;

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
///   "joints": []
/// }
/// ```
/// `gravity` 与 `joints` 可省略(默认引擎重力、空关节)。
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
        T: Serialize + DeserializeOwned,
    {
        let desc: SceneDesc<T> = SceneDesc::from_json(json)?;
        self.bodies = desc.bodies;
        self.charges = vec![T::zero(); self.bodies.len()];
        self.joints = desc.joints;
        self.gravity = desc.gravity;
        self.last_sensor_contacts.clear();
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
        let desc = SceneDesc {
            gravity: self.gravity,
            bodies: self.bodies.clone(),
            joints: self.joints.clone(),
        };
        desc.to_json()
    }
}
