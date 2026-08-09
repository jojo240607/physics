//! 把 `SolidWorld` 适配为 `phy_core::Subsystem<T>`,挂入统一仿真 `World<T>`。
//!
//! 固体求解是准静态的:每个 `step` 先做一步动态松弛(显式阻尼推进),
//! 再在其中调用静力平衡求解作为约束校正。对帧率无关的小变形弹性,
//! `step` 后即可得到稳定位移场,供渲染/耦合读取。

use std::any::Any;

use phy_math::{RealField, Vec3};
use serde::de::DeserializeOwned;
use serde::{Deserialize, Serialize};

use crate::fem::SolidWorld;
use phy_core::world::Subsystem;

/// 固体子系统包装。
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(
    bound = "T: RealField + Copy + Serialize + DeserializeOwned + num_traits::ToPrimitive + num_traits::FromPrimitive"
)]
pub struct SolidSubsystem<
    T: RealField + Copy + num_traits::ToPrimitive + num_traits::FromPrimitive,
> {
    /// 内部固体世界。
    pub world: SolidWorld<T>,
    /// 单步耦合前的额外预备(预留;此处恒为 None 占位)。
    _marker: std::marker::PhantomData<T>,
}

impl<
    T: RealField + Copy + num_traits::ToPrimitive + num_traits::FromPrimitive,
> SolidSubsystem<T> {
    /// 由一个固体世界构造子系统。
    pub fn new(world: SolidWorld<T>) -> Self {
        Self {
            world,
            _marker: std::marker::PhantomData,
        }
    }

    /// 设置每节点重力向量(写入内部 `gravity`,由 `step` 累加为外力)。
    pub fn set_gravity(&mut self, g: Vec3<T>) {
        self.world.gravity = g;
    }

    /// 读取某节点当前位移(用于渲染)。
    pub fn displacement_at(&self, i: usize) -> Option<Vec3<T>> {
        self.world.nodes.get(i).map(|n| n.u)
    }
}

impl<
    T: RealField + Copy + num_traits::ToPrimitive + num_traits::FromPrimitive,
> Subsystem<T> for SolidSubsystem<T> {
    fn as_any(&self) -> &dyn Any {
        self
    }
    fn as_any_mut(&mut self) -> &mut dyn Any {
        self
    }

    fn step(&mut self, dt: &T) {
        let dt = *dt;
        self.world.step(dt);
    }

    fn couple(&mut self, _world: &mut phy_core::world::World<T>, _dt: &T) {
        // 固体当前为独立子系统,无跨子系统耦合;预留接口。
    }

    fn name(&self) -> &'static str {
        "solid"
    }
}
