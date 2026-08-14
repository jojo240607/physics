//! # phy-sdk —— 物理引擎集成 SDK
//!
//! > ⚠️ **稳定性**: 当前为 `0.x` 阶段, API 仍可能变动。业务接入请锁定精确版本
//! > (如 `phy-sdk = "=0.1.0"`),避免因小版本升级导致代码不兼容。
//!
//! 这是面向**业务集成**与 **AI vibe-coding** 的便利层(M4 里程碑),目标是把散落在
//! 多个 `phy-*` crate 的子系统拼装细节屏蔽掉,让接入方(包括 LLM 代码生成)拿到一个
//! **统一入口 + 开箱即用工厂**:
//!
//! - 通过 [`PhysicsBuilder`] 一行声明要启用哪些子系统,自动创建内部世界并挂载;
//! - 通过 [`WorldExt`] (见 [`spawn_sphere`] / [`WorldExt::rigid`]) 按**意图**直接造物体,
//!   无需记忆子系统索引或手写 downcast;
//! - 通过 [`World::step`] / [`World::step_checked`] 驱动仿真;
//! - 通过 [`save_world`] / [`load_world`] 做存档(兼容 `phy-io` 的 ron 格式);
//! - 运行时可用 [`get_as`] / [`get_as_mut`] (底层逃生口) 或 [`WorldExt`] 取回强类型句柄。
//!
//! 所有物理算法实现仍由各 `phy-*` crate 负责,本 crate **只做重导出与便捷封装**,
//! 不重新实现任何物理,保证与底层 crate 行为字节级一致。
//!
//! # 坐标与单位契约 (AI 务必遵守)
//!
//! 全库统一约定,所有 `spawn_*` 与取回的位置/姿态数据都遵守:
//!
//! - **坐标系**: 右手系, **+Y 轴向上**, 单位 **米 (m)**。
//! - **角度**: 四元数 **(w, x, y, z)**, 含义为 *机体(局部) -> 世界* 的旋转。
//! - **速度 / 角速度**: 世界系, 单位 **m/s** 与 **rad/s**。
//! - **重力**: 默认沿 **-Y**, 大小 **9.81 m/s²** (可改 `world.rigid().gravity`)。
//!
//! # AI 推荐用法 (vibe-coding 友好)
//!
//! 直接 `use phy_sdk::WorldExt;` 后即可用意图式 API,无需关心索引:
//!
//! ```
//! use phy_sdk::{PhysicsBuilder, World, WorldExt, Vec3};
//!
//! // 1) 声明并构建世界(只需刚体)。
//! let mut world: World<f64> = PhysicsBuilder::new().rigid().build();
//!
//! // 2) 直接按意图造物体,无需记索引/downcast (返回 BodyId, 可忽略)。
//! world.spawn_floor(Vec3::new(20.0, 0.5, 20.0), -0.5);      // 地板在 y=-0.5
//! world.spawn_sphere(Vec3::new(0.0, 3.0, 0.0), 0.5, 1.0);   // 下落球
//! let ids = world.spawn_sphere_grid(                       // 5³ = 125 个下落球网格
//!     Vec3::new(-1.0, 0.0, -1.0), 5, 0.3, 0.1, 0.1);
//! assert_eq!(ids.len(), 125);
//!
//! // 3) 推进仿真。
//! for _ in 0..30 { world.step(1.0 / 60.0); }
//!
//! // 4) 读回任意物体位姿 (pos.xyz 米, rot.wxyz 四元数)。
//! let r = world.rigid().unwrap();
//! let ball = &r.world.bodies[ids[0].0];
//! assert!(ball.pos.y < 3.0, "球应已下落");
//! ```
//!
//! ## 多子系统组合
//!
//! ```
//! use phy_sdk::{PhysicsBuilder, World, WorldExt};
//! use phy_sdk::fluid::FluidSubsystem;
//! use phy_sdk::granular::GranularSubsystem;
//!
//! // 同时启用流体 + 颗粒(同一 World 内多子系统各自独立 step)。
//! let mut world: World<f64> =
//!     PhysicsBuilder::new().fluid().granular().build();
//!
//! for _ in 0..10 {
//!     world.step(1.0 / 60.0);
//! }
//!
//! // 用具名取回,无需记索引顺序。
//! assert!(world.fluid().is_some());
//! assert!(world.granular().is_some());
//! ```
//!
//! ## 存档(ron/JSON)
//!
//! ```
//! use phy_sdk::{PhysicsBuilder, World, save_world_json, load_world_json};
//!
//! let world: World<f64> = PhysicsBuilder::new().granular().build();
//! let s = save_world_json(&world); // 序列化到字符串(也支持 save_world 写文件)
//! let restored: World<f64> = load_world_json(&s); // 从字符串反序列化
//! // 序列化往返一致即证明存档/读档闭环。
//! assert_eq!(save_world_json(&restored), s);
//! ```
//!
//! ## 看门狗与数值稳定性
//!
//! 生产环境建议用 [`World::step_checked`],它会在任一子系统产生 `NaN`/`Inf` 或卡死
//! (`delta_t` 推进为 0)时返回 [`WorldError`],而非静默产出坏数据:
//!
//! ```
//! use phy_sdk::{PhysicsBuilder, World, WorldExt};
//!
//! let mut world: World<f64> = PhysicsBuilder::new().fluid().build();
//! if let Err(e) = world.step_checked(1.0 / 60.0) {
//!     eprintln!("仿真异常: {:?}", e);
//! }
//! ```

use std::any::Any;

// ===== 重导出:核心类型与 trait =====
pub use phy_core::{Subsystem, World, WorldError};

// ===== AI 友好层所需的类型导入 (供下方 `WorldExt` impl 使用) =====
pub use phy_math::Vec3;
use crate::field::HeatField;
use crate::fluid::FluidSubsystem;
use crate::granular::GranularSubsystem;
use crate::optics::OpticSubsystem;
use crate::rigid::{Body, RigidSubsystem, Shape};
use crate::soft::SoftSubsystem;
use crate::solid::SolidSubsystem;

// ===== 命名子模块:各子系统强类型句柄(供业务按域名引用) =====
pub mod fluid {
    //! 流体(SPH/WCSPH)。
    pub use phy_fluid::{FluidSubsystem, FluidWorld, SphParams};
}
pub mod rigid {
    //! 刚体。
    //!
    //! # 角色控制器 (D4)
    //! 用 kinematic 胶囊体做带碰撞 slide 的角色移动:先创建,再每帧用玩家输入驱动
    //! `update`,最后 `step`。角色不会因碰撞被推开(kinematic),由控制器手动处理 slide。
    //!
    //! ```
    //! use phy_sdk::{PhysicsBuilder, World, get_as_mut};
    //! use phy_sdk::rigid::{RigidSubsystem, RigidWorld, CharacterController};
    //! use phy_math::Vec3;
    //!
    //! let mut world: World<f64> = PhysicsBuilder::new().rigid().build();
    //! let mut rigid = get_as_mut::<RigidSubsystem<f64>>(&mut world, 0)
    //!     .expect("刚体子系统");
    //! // 静态地面
    //! rigid.world.add_body(phy_rigid::shape::Body {
    //!     shape: phy_rigid::shape::Shape::Box { half: Vec3::new(20.0, 0.5, 20.0) },
    //!     pos: Vec3::new(0.0, -0.5, 0.0),
    //!     inv_mass: 0.0,
    //!     ..Default::default()
    //! });
    //! // 创建角色(kinematic 胶囊)
    //! let mut cc = CharacterController::new(&mut rigid.world, Vec3::new(0.0, 3.0, 0.0));
    //! cc.speed = 5.0; cc.jump_speed = 6.0;
    //! // 每帧:玩家输入驱动 + step(此处模拟静止下落)
    //! for _ in 0..240 {
    //!     cc.update(&mut rigid.world, 1.0 / 120.0, Vec3::zeros(), false);
    //!     rigid.world.step(1.0 / 120.0);
    //! }
    //! assert!(cc.grounded, "角色应落到地面");
    //! ```
    pub use phy_rigid::shape::{Body, Shape};
    pub use phy_rigid::{CharacterController, RigidSubsystem, RigidWorld};
}
pub mod granular {
    //! 颗粒(粉末/PBD)。
    pub use phy_granular::subsystem::GranularSubsystem;
    pub use phy_granular::world::{GranularWorld, Grain};
}
pub mod soft {
    //! 软体(可变形)。
    pub use phy_soft::{SoftBody, SoftSubsystem};
}
pub mod field {
    //! 标量场/传热(注意:`HeatField` 本身即 `Subsystem`,无额外 wrapper)。
    pub use phy_field::{Bc, HeatField, ScalarField};
}
pub mod optics {
    //! 光学/波动。
    pub use phy_optics::{OpticScene, OpticSubsystem, Precision};
}
pub mod solid {
    //! 可破坏实体(FEM 连续介质)。
    pub use phy_solid::{SolidSubsystem, SolidWorld};
}

// 存档 API(文件版 + 字符串/JSON 版,后者更便于嵌入与测试)。
pub use phy_io::{load_world, load_world_json, save_world, save_world_json};

/// 把 `World` 中第 `idx` 个子系统取回为强类型 `&T`(G4 集成常用:拿到句柄做细粒度控制)。
///
/// 失败(索引越界或类型不符)返回 `None`,不会 panic —— 便于业务侧优雅降级。
///
/// # 示例
/// ```
/// use phy_sdk::{PhysicsBuilder, World, get_as, granular::GranularSubsystem};
/// let world: World<f64> = PhysicsBuilder::new().granular().build();
/// let g = get_as::<GranularSubsystem<f64>>(&world, 0).expect("颗粒子系统");
/// assert_eq!(g.world.grains.len(), 0); // 尚未灌粒子
/// ```
pub fn get_as<'a, T: Any>(world: &'a World<f64>, idx: usize) -> Option<&'a T> {
    world
        .get(idx)
        .and_then(|sub| sub.as_any().downcast_ref::<T>())
}

/// 把 `World` 中第 `idx` 个子系统取回为强类型 `&mut T`。
///
/// 失败返回 `None`,不会 panic。
pub fn get_as_mut<'a, T: Any>(world: &'a mut World<f64>, idx: usize) -> Option<&'a mut T> {
    world
        .get_mut(idx)
        .and_then(|sub| sub.as_any_mut().downcast_mut::<T>())
}

/// 集成 SDK 的统一构建入口(M4 核心 API)。
///
/// 调用方按"声明式"启用要用的子系统,`build()` 自动创建内部世界并**只挂载被启用**
/// 的子系统。各子系统在 `World.subsystems` 中的索引**等于声明命中顺序**,可用
/// [`get_as`]/[`get_as_mut`] 取回强类型句柄。
#[derive(Clone, Default)]
pub struct PhysicsBuilder {
    fluid: bool,
    rigid: bool,
    granular: bool,
    soft: bool,
    field: bool,
    optics: bool,
    solid: bool,
}

// ===========================================================================
// AI 友好层 (vibe-coding 适配): 具名取回 + 意图式 spawn + 坐标契约
// ===========================================================================
//
// 设计动机: 本 SDK 第一消费者可能是 LLM。AI 容易搞错"索引 0 = 第一个声明
// 的子系统"、"向下转型取句柄"、"不透明指针"等人类习惯。下面这组 API 让 AI
// 直接按"意图"操作,无需记忆索引或泛型 downcast。
//
// 坐标契约 (所有 spawn / 取回 API 一律遵守, 写在此处 AI 可检索):
//   * 坐标系: 右手系, +Y 轴向上, 单位米(m)。
//   * 角度: 四元数 (w, x, y, z), 含义为 机体(局部) -> 世界 的旋转。
//   * 速度/角速度: 世界系, 单位 m/s 与 rad/s。
//   * 重力: 默认沿 -Y, 大小 9.81 m/s² (可改 `rigid().gravity`)。

/// 刚体 / 颗粒等物体的稳定句柄。
///
/// 由 spawn 类方法返回, AI 无需自己保存 `add_body` 的整数索引。
/// 内部即 `usize`(物体在所属子系统中的位置), 但用新类型避免误用。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct BodyId(pub usize);

/// 给 `World<f64>` 增加的 AI 友好方法集。
///
/// 用法: `use phy_sdk::WorldExt;` 后, `world.rigid()` / `world.spawn_sphere(...)`
/// 等方法即可直接调用, 无需 `get_as::<RigidSubsystem<f64>>(&mut world, 0)`。
///
/// # 坐标契约
/// 本 trait 所有方法遵守: **右手系, +Y 向上, 单位米(m); 四元数 (w,x,y,z) 为
/// 机体->世界旋转; 速度 m/s, 角速度 rad/s; 重力沿 -Y, 9.81 m/s²**。
pub trait WorldExt {
    // ---- 具名取回 (替代索引取回) ----
    /// 取回刚体子系统句柄 (若世界含刚体子系统)。
    fn rigid(&self) -> Option<&RigidSubsystem<f64>>;
    /// 可变取回刚体子系统句柄。
    fn rigid_mut(&mut self) -> Option<&mut RigidSubsystem<f64>>;
    /// 取回流体子系统句柄。
    fn fluid(&self) -> Option<&FluidSubsystem<f64>>;
    /// 可变取回流体子系统句柄。
    fn fluid_mut(&mut self) -> Option<&mut FluidSubsystem<f64>>;
    /// 取回颗粒子系统句柄。
    fn granular(&self) -> Option<&GranularSubsystem<f64>>;
    /// 可变取回颗粒子系统句柄。
    fn granular_mut(&mut self) -> Option<&mut GranularSubsystem<f64>>;
    /// 取回软体子系统句柄。
    fn soft(&self) -> Option<&SoftSubsystem<f64>>;
    /// 可变取回软体子系统句柄。
    fn soft_mut(&mut self) -> Option<&mut SoftSubsystem<f64>>;
    /// 取回标量场/传热子系统句柄。
    fn field(&self) -> Option<&HeatField<f64>>;
    /// 可变取回标量场/传热子系统句柄。
    fn field_mut(&mut self) -> Option<&mut HeatField<f64>>;
    /// 取回光学子系统句柄。
    fn optics(&self) -> Option<&OpticSubsystem<f64>>;
    /// 可变取回光学子系统句柄。
    fn optics_mut(&mut self) -> Option<&mut OpticSubsystem<f64>>;
    /// 取回可破坏实体子系统句柄。
    fn solid(&self) -> Option<&SolidSubsystem<f64>>;
    /// 可变取回可破坏实体子系统句柄。
    fn solid_mut(&mut self) -> Option<&mut SolidSubsystem<f64>>;

    // ---- 意图式 spawn (刚体) ----
    /// 在地面 y = `center_y` 处放一块静态地板, 半尺寸 `half` (即 x/z 边长 = 2*half)。
    ///
    /// 返回地板 `BodyId`。地板质量无限 (inv_mass=0), 不受重力与碰撞冲量驱动。
    /// 坐标: 右手系 +Y 向上, 米。
    fn spawn_floor(&mut self, half: Vec3<f64>, center_y: f64) -> Option<BodyId>;
    /// 在世界坐标 `pos` 处生成一个动态球 (半径 `r`, 质量 `mass` 千克), 初速 0。
    ///
    /// 返回 `BodyId` (后续可用 `rigid_mut().world.body(id.0)` 取回)。坐标: 米, +Y 向上。
    fn spawn_sphere(&mut self, pos: Vec3<f64>, r: f64, mass: f64) -> Option<BodyId>;
    /// 在世界坐标 `pos` 处生成一个动态长方体 (半尺寸 `half`, 质量 `mass` 千克), 初速 0。
    ///
    /// 坐标: 米, +Y 向上。
    fn spawn_box(&mut self, pos: Vec3<f64>, half: Vec3<f64>, mass: f64) -> Option<BodyId>;
    /// 生成静态球 (半径 `r`, inv_mass=0), 常用于障碍/边界。
    fn spawn_static_sphere(&mut self, pos: Vec3<f64>, r: f64) -> Option<BodyId>;
    /// 生成静态长方体 (半尺寸 `half`, inv_mass=0)。
    fn spawn_static_box(&mut self, pos: Vec3<f64>, half: Vec3<f64>) -> Option<BodyId>;
    /// 在 `pos` 周围网格化生成 `count`×`count`×`count` 个下落球 (间距 `spacing`, 各质量 `mass`)。
    ///
    /// 返回每个球的 `BodyId` 列表。适合"一堆弹跳球"场景。
    fn spawn_sphere_grid(
        &mut self,
        pos: Vec3<f64>,
        count: usize,
        spacing: f64,
        r: f64,
        mass: f64,
    ) -> Vec<BodyId>;
}

impl WorldExt for World<f64> {
    fn rigid(&self) -> Option<&RigidSubsystem<f64>> {
        for i in 0..self.subsystem_count() {
            if let Some(s) = self.get(i) {
                if let Some(r) = s.as_any().downcast_ref::<RigidSubsystem<f64>>() {
                    return Some(r);
                }
            }
        }
        None
    }
    fn rigid_mut(&mut self) -> Option<&mut RigidSubsystem<f64>> {
        let mut idx = None;
        for i in 0..self.subsystem_count() {
            if let Some(s) = self.get(i) {
                if s.as_any().is::<RigidSubsystem<f64>>() {
                    idx = Some(i);
                    break;
                }
            }
        }
        idx.and_then(|i| self.get_mut(i).and_then(|s| s.as_any_mut().downcast_mut()))
    }
    fn fluid(&self) -> Option<&FluidSubsystem<f64>> {
        for i in 0..self.subsystem_count() {
            if let Some(s) = self.get(i) {
                if let Some(r) = s.as_any().downcast_ref::<FluidSubsystem<f64>>() {
                    return Some(r);
                }
            }
        }
        None
    }
    fn fluid_mut(&mut self) -> Option<&mut FluidSubsystem<f64>> {
        let mut idx = None;
        for i in 0..self.subsystem_count() {
            if let Some(s) = self.get(i) {
                if s.as_any().is::<FluidSubsystem<f64>>() {
                    idx = Some(i);
                    break;
                }
            }
        }
        idx.and_then(|i| self.get_mut(i).and_then(|s| s.as_any_mut().downcast_mut()))
    }
    fn granular(&self) -> Option<&GranularSubsystem<f64>> {
        for i in 0..self.subsystem_count() {
            if let Some(s) = self.get(i) {
                if let Some(r) = s.as_any().downcast_ref::<GranularSubsystem<f64>>() {
                    return Some(r);
                }
            }
        }
        None
    }
    fn granular_mut(&mut self) -> Option<&mut GranularSubsystem<f64>> {
        let mut idx = None;
        for i in 0..self.subsystem_count() {
            if let Some(s) = self.get(i) {
                if s.as_any().is::<GranularSubsystem<f64>>() {
                    idx = Some(i);
                    break;
                }
            }
        }
        idx.and_then(|i| self.get_mut(i).and_then(|s| s.as_any_mut().downcast_mut()))
    }
    fn soft(&self) -> Option<&SoftSubsystem<f64>> {
        for i in 0..self.subsystem_count() {
            if let Some(s) = self.get(i) {
                if let Some(r) = s.as_any().downcast_ref::<SoftSubsystem<f64>>() {
                    return Some(r);
                }
            }
        }
        None
    }
    fn soft_mut(&mut self) -> Option<&mut SoftSubsystem<f64>> {
        let mut idx = None;
        for i in 0..self.subsystem_count() {
            if let Some(s) = self.get(i) {
                if s.as_any().is::<SoftSubsystem<f64>>() {
                    idx = Some(i);
                    break;
                }
            }
        }
        idx.and_then(|i| self.get_mut(i).and_then(|s| s.as_any_mut().downcast_mut()))
    }
    fn field(&self) -> Option<&HeatField<f64>> {
        for i in 0..self.subsystem_count() {
            if let Some(s) = self.get(i) {
                if let Some(r) = s.as_any().downcast_ref::<HeatField<f64>>() {
                    return Some(r);
                }
            }
        }
        None
    }
    fn field_mut(&mut self) -> Option<&mut HeatField<f64>> {
        let mut idx = None;
        for i in 0..self.subsystem_count() {
            if let Some(s) = self.get(i) {
                if s.as_any().is::<HeatField<f64>>() {
                    idx = Some(i);
                    break;
                }
            }
        }
        idx.and_then(|i| self.get_mut(i).and_then(|s| s.as_any_mut().downcast_mut()))
    }
    fn optics(&self) -> Option<&OpticSubsystem<f64>> {
        for i in 0..self.subsystem_count() {
            if let Some(s) = self.get(i) {
                if let Some(r) = s.as_any().downcast_ref::<OpticSubsystem<f64>>() {
                    return Some(r);
                }
            }
        }
        None
    }
    fn optics_mut(&mut self) -> Option<&mut OpticSubsystem<f64>> {
        let mut idx = None;
        for i in 0..self.subsystem_count() {
            if let Some(s) = self.get(i) {
                if s.as_any().is::<OpticSubsystem<f64>>() {
                    idx = Some(i);
                    break;
                }
            }
        }
        idx.and_then(|i| self.get_mut(i).and_then(|s| s.as_any_mut().downcast_mut()))
    }
    fn solid(&self) -> Option<&SolidSubsystem<f64>> {
        for i in 0..self.subsystem_count() {
            if let Some(s) = self.get(i) {
                if let Some(r) = s.as_any().downcast_ref::<SolidSubsystem<f64>>() {
                    return Some(r);
                }
            }
        }
        None
    }
    fn solid_mut(&mut self) -> Option<&mut SolidSubsystem<f64>> {
        let mut idx = None;
        for i in 0..self.subsystem_count() {
            if let Some(s) = self.get(i) {
                if s.as_any().is::<SolidSubsystem<f64>>() {
                    idx = Some(i);
                    break;
                }
            }
        }
        idx.and_then(|i| self.get_mut(i).and_then(|s| s.as_any_mut().downcast_mut()))
    }

    fn spawn_floor(&mut self, half: Vec3<f64>, center_y: f64) -> Option<BodyId> {
        let sub = self.rigid_mut()?;
        let id = sub.world.add_body(Body::new(
            Shape::Box { half },
            Vec3::new(0.0, center_y, 0.0),
            0.0,
        ));
        Some(BodyId(id))
    }
    fn spawn_sphere(&mut self, pos: Vec3<f64>, radius: f64, mass: f64) -> Option<BodyId> {
        let inv = if mass <= 0.0 { 0.0 } else { 1.0 / mass };
        let sub = self.rigid_mut()?;
        let id = sub
            .world
            .add_body(Body::new(Shape::Sphere { r: radius }, pos, inv));
        Some(BodyId(id))
    }
    fn spawn_box(&mut self, pos: Vec3<f64>, half: Vec3<f64>, mass: f64) -> Option<BodyId> {
        let inv = if mass <= 0.0 { 0.0 } else { 1.0 / mass };
        let sub = self.rigid_mut()?;
        let id = sub
            .world
            .add_body(Body::new(Shape::Box { half }, pos, inv));
        Some(BodyId(id))
    }
    fn spawn_static_sphere(&mut self, pos: Vec3<f64>, radius: f64) -> Option<BodyId> {
        let sub = self.rigid_mut()?;
        let id = sub
            .world
            .add_body(Body::new(Shape::Sphere { r: radius }, pos, 0.0));
        Some(BodyId(id))
    }
    fn spawn_static_box(&mut self, pos: Vec3<f64>, half: Vec3<f64>) -> Option<BodyId> {
        let sub = self.rigid_mut()?;
        let id = sub
            .world
            .add_body(Body::new(Shape::Box { half }, pos, 0.0));
        Some(BodyId(id))
    }
    fn spawn_sphere_grid(
        &mut self,
        pos: Vec3<f64>,
        count: usize,
        spacing: f64,
        radius: f64,
        mass: f64,
    ) -> Vec<BodyId> {
        let mut ids = Vec::with_capacity(count * count * count);
        let inv = if mass <= 0.0 { 0.0 } else { 1.0 / mass };
        if let Some(sub) = self.rigid_mut() {
            for i in 0..count {
                for j in 0..count {
                    for k in 0..count {
                        let p = Vec3::new(
                            pos.x + i as f64 * spacing,
                            pos.y + j as f64 * spacing,
                            pos.z + k as f64 * spacing,
                        );
                        let id = sub
                            .world
                            .add_body(Body::new(Shape::Sphere { r: radius }, p, inv));
                        ids.push(BodyId(id));
                    }
                }
            }
        }
        ids
    }
}

impl PhysicsBuilder {
    /// 新建空构建器(不含任何子系统)。
    pub fn new() -> Self {
        Self::default()
    }

    /// 启用流体子系统(初始为空世界,后续经句柄 `fill_box` 灌粒子)。
    pub fn fluid(mut self) -> Self {
        self.fluid = true;
        self
    }

    /// 启用刚体子系统。
    pub fn rigid(mut self) -> Self {
        self.rigid = true;
        self
    }

    /// 启用颗粒(粉末)子系统。
    pub fn granular(mut self) -> Self {
        self.granular = true;
        self
    }

    /// 启用软体(可变形)子系统。
    pub fn soft(mut self) -> Self {
        self.soft = true;
        self
    }

    /// 启用标量场/传热子系统(默认挂一个 16³ 热扩散场,稳态用)。
    pub fn field(mut self) -> Self {
        self.field = true;
        self
    }

    /// 启用光学/波动子系统。
    pub fn optics(mut self) -> Self {
        self.optics = true;
        self
    }

    /// 启用可破坏实体(FEM)子系统。
    pub fn solid(mut self) -> Self {
        self.solid = true;
        self
    }

    /// 构建统一 `World<f64>` 并挂载所有已声明子系统。
    ///
    /// 索引顺序固定为 `fluid → rigid → granular → soft → field → optics → solid`,
    /// 仅包含被启用的子系统。
    pub fn build(self) -> World<f64> {
        let mut world = World::default();
        if self.fluid {
            use crate::fluid::{FluidSubsystem, FluidWorld, SphParams};
            let fw = FluidWorld::new(SphParams::defaults());
            world.add_subsystem(Box::new(FluidSubsystem::new(fw)));
        }
        if self.rigid {
            use crate::rigid::{RigidSubsystem, RigidWorld};
            world.add_subsystem(Box::new(RigidSubsystem::new(RigidWorld::new())));
        }
        if self.granular {
            use crate::granular::{GranularSubsystem, GranularWorld};
            world.add_subsystem(Box::new(GranularSubsystem::new(GranularWorld::new())));
        }
        if self.soft {
            use crate::soft::{SoftBody, SoftSubsystem};
            world.add_subsystem(Box::new(SoftSubsystem::new(SoftBody::new(0.0))));
        }
        if self.field {
            use crate::field::{Bc, HeatField, ScalarField};
            let grid = ScalarField::new(16, 16, 16, 1.0, 0.0, Bc::Neumann);
            world.add_subsystem(Box::new(HeatField::new(grid, 0.1)));
        }
        if self.optics {
            use crate::optics::{OpticScene, OpticSubsystem, Precision};
            world.add_subsystem(Box::new(OpticSubsystem::new(
                OpticScene::new(),
                Precision::Offline,
            )));
        }
        if self.solid {
            use crate::solid::{SolidSubsystem, SolidWorld};
            world.add_subsystem(Box::new(SolidSubsystem::new(SolidWorld::new(
                1.0, 1.0, 1.0,
            ))));
        }
        world
    }
}
