//! # phy-fluid
//!
//! 弱可压缩 SPH(光滑粒子流体动力学)求解器(Müller et al. 2003),
//! 支持与 `phy-rigid` 刚体的双向耦合(浮力 / 阻力 / 动量交换)。
//!
//! 设计对齐姊妹 crate `phy-rigid`:提供自持的 `FluidWorld<T>` 世界,
//! 亦可经 `FluidSubsystem` 挂入 `phy_core::World<T>` 多物理场统一驱动。

mod kernels;
mod particle;
mod sph;
mod subsystem;

pub use kernels::{dist, Kernels};
pub use particle::Particle;
pub use sph::{CouplePoint, FluidWorld, SphParams};
pub use subsystem::FluidSubsystem;
