//! 世界存档 / 读档(用 serde JSON 持久化子系统状态)。
//!
//! `World<T>` 持有 `Vec<Box<dyn Subsystem<T>>>`(`Any` trait object),serde 无法直接
//! 序列化 trait object。本模块用枚举派发(`SubArchive<T>`)绕过:存档时把每个子系统
//! `downcast` 到具体类型,序列化时带上类型标签;读档时按标签重建并装回 `World`。
//!
//! 支持的具体子系统(随路由图扩展而扩展):
//! - `RigidSubsystem`(刚体)
//! - `SoftSubsystem`(软体)
//! - `SolidSubsystem`(连续介质 FEM)
//! - `GranularSubsystem`(颗粒)
//! - `FluidSubsystem`(SPH 流体)
//! - `HeatField` / `EmField` / `GravField` / `AcousticField` / `WaveField`(连续场)
//! - `OpticSubsystem`(光学)

use phy_core::Subsystem;
use phy_core::World;
use phy_field::{AcousticField, EmField, GravField, HeatField, WaveField};
use phy_fluid::FluidSubsystem;
use phy_granular::subsystem::GranularSubsystem;
use phy_math::RealField;
use phy_optics::OpticSubsystem;
use phy_rigid::RigidSubsystem;
use phy_soft::SoftSubsystem;
use phy_solid::SolidSubsystem;
use serde::de::DeserializeOwned;
use serde::{Deserialize, Serialize};

/// 各子系统存档镜像(带类型标签,供 JSON 枚举派发)。
///
/// 注意:此处**只 derive `Serialize`/`Deserialize`**(用 `#[serde(bound)]` 统一约束),
/// 不 derive `Debug` —— 否则 `Debug` 会用默认约束(`RigidSubsystem<T>` 要求 `T:
/// ToPrimitive`),与 serde bound 脱节而报错。改为手写 `Debug` impl。
#[derive(Serialize, Deserialize)]
#[serde(bound(
    serialize = "T: RealField + Copy + Serialize + DeserializeOwned + Default + nalgebra::Scalar + num_traits::ToPrimitive",
    deserialize = "T: RealField + Copy + Serialize + DeserializeOwned + Default + nalgebra::Scalar + num_traits::ToPrimitive"
))]
pub enum SubArchive<T: RealField + Copy + num_traits::ToPrimitive> {
    Rigid(RigidSubsystem<T>),
    Soft(SoftSubsystem<T>),
    Solid(SolidSubsystem<T>),
    Granular(GranularSubsystem<T>),
    Fluid(FluidSubsystem<T>),
    Heat(HeatField<T>),
    Em(EmField<T>),
    Grav(GravField<T>),
    Acoustic(AcousticField<T>),
    Wave(WaveField<T>),
    Optic(OpticSubsystem<T>),
}

impl<T: RealField + Copy + Serialize + DeserializeOwned + nalgebra::Scalar + num_traits::ToPrimitive> std::fmt::Debug
    for SubArchive<T>
{
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let name = match self {
            SubArchive::Rigid(_) => "Rigid",
            SubArchive::Soft(_) => "Soft",
            SubArchive::Solid(_) => "Solid",
            SubArchive::Granular(_) => "Granular",
            SubArchive::Fluid(_) => "Fluid",
            SubArchive::Heat(_) => "Heat",
            SubArchive::Em(_) => "Em",
            SubArchive::Grav(_) => "Grav",
            SubArchive::Acoustic(_) => "Acoustic",
            SubArchive::Wave(_) => "Wave",
            SubArchive::Optic(_) => "Optic",
        };
        write!(f, "SubArchive::{}", name)
    }
}

/// 顶层存档格式:版本 + 仿真时间 + 子系统列表。
#[derive(Serialize, Deserialize)]
#[serde(bound(
    serialize = "T: RealField + Copy + Serialize + DeserializeOwned + Default + nalgebra::Scalar + num_traits::ToPrimitive",
    deserialize = "T: RealField + Copy + Serialize + DeserializeOwned + Default + nalgebra::Scalar + num_traits::ToPrimitive"
))]
pub struct WorldArchive<T: RealField + Copy + num_traits::ToPrimitive> {
    /// 格式版本(便于将来兼容性判断)。
    pub version: u32,
    /// 仿真时间。
    pub time: T,
    /// 各子系统存档。
    pub subsystems: Vec<SubArchive<T>>,
}

impl<T: RealField + Copy + Serialize + DeserializeOwned + nalgebra::Scalar + num_traits::ToPrimitive> std::fmt::Debug
    for WorldArchive<T>
{
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "WorldArchive{{version={}, subsystems={}}}",
            self.version,
            self.subsystems.len()
        )
    }
}

/// 把 `World` 中可识别的具体子系统序列化进存档枚举列表。
///
/// 无法识别(unknown)的子系统会被静默跳过(向后兼容,不会 panic)。
pub fn archive_world<T>(world: &World<T>) -> WorldArchive<T>
where
    T: RealField
        + Copy
        + Serialize
        + DeserializeOwned
        + Default
        + nalgebra::Scalar
        + num_traits::ToPrimitive,
{
    let mut subs = Vec::new();
    let n = world.subsystem_count();
    for i in 0..n {
        let s = world.get(i).expect("subsystem index in range");
        let any = s.as_any();
        if let Some(r) = any.downcast_ref::<RigidSubsystem<T>>() {
            subs.push(SubArchive::Rigid(r.clone()));
        } else if let Some(r) = any.downcast_ref::<SoftSubsystem<T>>() {
            subs.push(SubArchive::Soft(r.clone()));
        } else if let Some(r) = any.downcast_ref::<SolidSubsystem<T>>() {
            subs.push(SubArchive::Solid(r.clone()));
        } else if let Some(r) = any.downcast_ref::<GranularSubsystem<T>>() {
            subs.push(SubArchive::Granular(r.clone()));
        } else if let Some(r) = any.downcast_ref::<FluidSubsystem<T>>() {
            subs.push(SubArchive::Fluid(r.clone()));
        } else if let Some(r) = any.downcast_ref::<HeatField<T>>() {
            subs.push(SubArchive::Heat(r.clone()));
        } else if let Some(r) = any.downcast_ref::<EmField<T>>() {
            subs.push(SubArchive::Em(r.clone()));
        } else if let Some(r) = any.downcast_ref::<GravField<T>>() {
            subs.push(SubArchive::Grav(r.clone()));
        } else if let Some(r) = any.downcast_ref::<AcousticField<T>>() {
            subs.push(SubArchive::Acoustic(r.clone()));
        } else if let Some(r) = any.downcast_ref::<WaveField<T>>() {
            subs.push(SubArchive::Wave(r.clone()));
        } else if let Some(r) = any.downcast_ref::<OpticSubsystem<T>>() {
            subs.push(SubArchive::Optic(r.clone()));
        }
        // 未知类型:跳过。
    }
    WorldArchive {
        version: 1,
        time: world.time(),
        subsystems: subs,
    }
}

/// 把存档重建为 `World<T>`(按枚举标签还原各子系统)。
pub fn unarchive_world<T>(arch: WorldArchive<T>) -> World<T>
where
    T: RealField
        + Copy
        + Serialize
        + DeserializeOwned
        + Default
        + nalgebra::Scalar
        + num_traits::ToPrimitive
        + num_traits::Float,
{
    let mut world = World::new();
    world.set_time(arch.time);
    for sub in arch.subsystems {
        let boxed: Box<dyn Subsystem<T>> = match sub {
            SubArchive::Rigid(s) => Box::new(s),
            SubArchive::Soft(s) => Box::new(s),
            SubArchive::Solid(s) => Box::new(s),
            SubArchive::Granular(s) => Box::new(s),
            SubArchive::Fluid(s) => Box::new(s),
            SubArchive::Heat(s) => Box::new(s),
            SubArchive::Em(s) => Box::new(s),
            SubArchive::Grav(s) => Box::new(s),
            SubArchive::Acoustic(s) => Box::new(s),
            SubArchive::Wave(s) => Box::new(s),
            SubArchive::Optic(s) => Box::new(s),
        };
        world.add_subsystem(boxed);
    }
    world
}

/// 序列化世界到 JSON 字符串。
pub fn save_world_json<T>(world: &World<T>) -> String
where
    T: RealField
        + Copy
        + Serialize
        + DeserializeOwned
        + Default
        + nalgebra::Scalar
        + num_traits::ToPrimitive
        + num_traits::Float,
{
    let arch = archive_world(world);
    serde_json::to_string_pretty(&arch).expect("serialize world archive")
}

/// 从 JSON 字符串反序列化世界。
pub fn load_world_json<T>(json: &str) -> World<T>
where
    T: RealField
        + Copy
        + Serialize
        + DeserializeOwned
        + Default
        + nalgebra::Scalar
        + num_traits::ToPrimitive
        + num_traits::Float,
{
    let arch: WorldArchive<T> =
        serde_json::from_str(json).expect("deserialize world archive");
    unarchive_world(arch)
}

/// 保存世界到文件(UTF-8 JSON)。
pub fn save_world<T>(world: &World<T>, path: &std::path::Path) -> std::io::Result<()>
where
    T: RealField
        + Copy
        + Serialize
        + DeserializeOwned
        + Default
        + nalgebra::Scalar
        + num_traits::ToPrimitive
        + num_traits::Float,
{
    let json = save_world_json(world);
    std::fs::write(path, json)
}

/// 从文件加载世界。
pub fn load_world<T>(path: &std::path::Path) -> World<T>
where
    T: RealField
        + Copy
        + Serialize
        + DeserializeOwned
        + Default
        + nalgebra::Scalar
        + num_traits::ToPrimitive
        + num_traits::Float,
{
    let json = std::fs::read_to_string(path).expect("read archive file");
    load_world_json(&json)
}
