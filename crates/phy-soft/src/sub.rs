//! 软体子系统适配:`SoftBody` 接入 `phy_core::World` 统一调度。

use std::any::Any;

use phy_core::{Subsystem, World};
use phy_math::RealField;
use phy_rigid::RigidSubsystem;

use crate::body::SoftBody;

/// 软体子系统:包装 `SoftBody` 以挂在 `World` 上。
///
/// 渲染时可通过 `World::get(i).as_any().downcast_ref::<SoftSubsystem<T>>()`
/// 取回,读取 `body` 字段。
pub struct SoftSubsystem<T: RealField + Copy> {
    /// 内部软体(渲染时直接访问)。
    pub body: SoftBody<T>,
}

impl<T: RealField + Copy> SoftSubsystem<T> {
    /// 由既有 `SoftBody` 构造。
    pub fn new(body: SoftBody<T>) -> Self {
        Self { body }
    }
}

impl<T: RealField + Copy> Subsystem<T> for SoftSubsystem<T> {
    fn as_any(&self) -> &dyn Any {
        self
    }

    fn as_any_mut(&mut self) -> &mut dyn Any {
        self
    }

    fn step(&mut self, dt: &T) {
        self.body.step(*dt);
    }

    /// 软体↔刚体双向耦合:对每个刚体把软体质点推出,并把可动刚体推开。
    ///
    /// 通过 `world.get_mut(rigid_idx)` 取出刚体子系统(两阶段避免同时双可变借用),
    /// 复用 `SoftBody::collide_body`(不可变刚体 → 软体推出 + 净冲量,再施加到刚体)。
    fn couple(&mut self, world: &mut World<T>, _dt: &T) {
        // 找到刚体子系统下标(只需一个;多刚体子系统场景下取第一个即可)。
        let ridx = match world.get(RIGID_IDX) {
            Some(s) if s.as_any().downcast_ref::<RigidSubsystem<T>>().is_some() => RIGID_IDX,
            _ => return,
        };
        let count = world
            .get(ridx)
            .and_then(|s| s.as_any().downcast_ref::<RigidSubsystem<T>>())
            .map(|r| r.world.bodies.len())
            .unwrap_or(0);
        for i in 0..count {
            // 阶段一:克隆刚体快照(释放对 world 的不可变借用),可变借软体算碰撞。
            let body = match world
                .get(ridx)
                .and_then(|s| s.as_any().downcast_ref::<RigidSubsystem<T>>())
            {
                Some(r) => r.world.bodies[i].clone(),
                None => return,
            };
            let impulse = self.body.collide_body(&body);
            // 阶段二:可变借刚体施加冲量。
            if let Some(r) = world
                .get_mut(ridx)
                .and_then(|s| s.as_any_mut().downcast_mut::<RigidSubsystem<T>>())
            {
                r.world.bodies[i].apply_impulse(impulse);
            }
        }
    }

    fn name(&self) -> &'static str {
        "soft"
    }
}

/// 刚体子系统在 `World` 中的约定下标(与 `phy-demo` 的 `IDX_RIGID` 一致)。
const RIGID_IDX: usize = 1;
