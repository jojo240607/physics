//! 固体连续介质力学:FEM 线性四面体弹性(M23, 路线图 #5)。
//!
//! 提供:
//! - [`fem`]:线性四面体单元(常数应变)的刚度装配与静力平衡 / 动力松弛求解;
//! - [`mesh`]:规则盒网格(Tet 剖分)与悬臂梁专用构造器;
//! - [`subsystem`]:把 `SolidWorld` 适配为 `phy_core::Subsystem`,挂入统一 `World<T>`。
//!
//! 设计目标:小变形弹性,结果与材料力学解析解(梁弯曲柔度 EI)可直接对照。

mod fem;
mod mesh;
mod subsystem;

#[cfg(test)]
mod tests;

pub use fem::{Node, SolidWorld, Tet};
pub use mesh::{cantilever_box, MeshParams};
pub use subsystem::SolidSubsystem;
