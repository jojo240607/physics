//! SPH 流体世界与求解器。
//!
//! 采用 Müller 2003 的标准弱可压缩 SPH:
//! - Poly6 核估算密度,理想气体状态方程 p = k (ρ - ρ0) 得压力;
//! - 对称压力力 + 粘性力 + 重力作为加速度来源;
//! - 半隐式欧拉积分;盒状边界带阻尼反弹;
//! - 均匀网格(单元 = 光滑长度 h)做邻居查询,保证线性复杂度;
//! - `couple_bodies` 提供与刚体的双向耦合(浮力 / 阻力 / 动量交换)。
//!
//! 源文件按职责拆分(一个类一个文件):
//! - [`params`]:SPH 求解参数 `SphParams`;
//! - [`grid`]:均匀空间哈希网格 `Grid`(邻居查询);
//! - [`couple`]:软体质点耦合接口 `CouplePoint`;
//! - [`world`]:`FluidWorld` 求解器主体与刚体 / 热场 / 软体质点耦合。

mod couple;
mod grid;
mod params;
mod world;
mod gpu_flat;

#[cfg(test)]
mod tests;

pub use couple::CouplePoint;
pub use gpu_flat::SphFlatData;
pub use params::SphParams;
pub use world::FluidWorld;
