//! 软体子系统适配:`SoftBody` 接入 `phy_core::World` 统一调度。

use std::any::Any;

use phy_core::{Subsystem, World};
use phy_field::HeatField;
use phy_fluid::{CouplePoint, FluidSubsystem};
use phy_math::RealField;
use phy_rigid::RigidSubsystem;

use crate::body::SoftBody;

/// 软体子系统:包装 `SoftBody` 以挂在 `World` 上。
///
/// 渲染时可通过 `World::get(i).as_any().downcast_ref::<SoftSubsystem<T>>()`
/// 取回,读取 `body` 字段。
pub struct SoftSubsystem<T: RealField + Copy + num_traits::ToPrimitive> {
    /// 内部软体(渲染时直接访问)。
    pub body: SoftBody<T>,
    /// 与流体耦合的阻力系数(越大越倾向跟随局部流速)。
    pub fluid_drag: T,
    /// 软体质点的等效密度(用于把质量换算成排开体积以算阿基米德浮力)。
    pub soft_density: T,
    /// 热浮力温度膨胀系数 β(ρ(T)=ρ0/(1+β·(T-T_ref)))。0 表示无热浮力。
    pub thermal_expansion: T,
    /// 对流换热注入强度(运动质点加热场)。0 表示不注入热源。
    pub heat_gain: T,
    /// 热浮力参考温度 T_ref(环境温度基线,ρ(T_ref)=ρ0)。0 表示以 0 为环境温度。
    pub t_ref: T,
}

impl<T: RealField + Copy + num_traits::ToPrimitive> SoftSubsystem<T> {
    /// 由既有 `SoftBody` 构造。
    pub fn new(body: SoftBody<T>) -> Self {
        Self {
            body,
            fluid_drag: T::from_f64(3.0).unwrap(),
            soft_density: T::from_f64(1000.0).unwrap(),
            thermal_expansion: T::zero(),
            heat_gain: T::zero(),
            t_ref: T::zero(),
        }
    }
}

impl<T: RealField + Copy + num_traits::ToPrimitive> Subsystem<T> for SoftSubsystem<T> {
    fn as_any(&self) -> &dyn Any {
        self
    }

    fn as_any_mut(&mut self) -> &mut dyn Any {
        self
    }

    fn step(&mut self, dt: &T) {
        self.body.step(*dt);
    }

    /// 软体↔刚体 / 软体↔流体 双向耦合。
    ///
    /// 先在 `World` 中动态查找刚体与流体子系统的下标(不依赖注册顺序),
    /// 然后分别做:(1) 软体↔刚体碰撞推出 + 反向冲量;(2) 软体↔流体浮力 + 阻力 +
    /// 动量交换。`world.get/get_mut` 的安全调用避免了同时双可变借用。
    fn couple(&mut self, world: &mut World<T>, dt: &T) {
        // 动态查找刚体/流体子系统下标(遍历到 get 返回 None 为止,避免依赖 len 的 T 约束)。
        let mut i = 0;
        let mut rigid_idx = None;
        let mut fluid_idx = None;
        while let Some(s) = world.get(i) {
            if s.as_any().downcast_ref::<RigidSubsystem<T>>().is_some() {
                rigid_idx = Some(i);
            } else if s.as_any().downcast_ref::<FluidSubsystem<T>>().is_some() {
                fluid_idx = Some(i);
            }
            i += 1;
        }

        // (1) 软体↔刚体:逐个刚体把质点推出,并把可动刚体推开。
        if let Some(ridx) = rigid_idx {
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
                    None => break,
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

        // (2) 软体↔流体:浮力 + 阻力(soft→fluid 反向动量由 FluidWorld 内部分配)。
        if let Some(fidx) = fluid_idx {
            // 打包软体质点为 CouplePoint,交给 FluidWorld 计算合力并就地交换动量。
            let mut pts: Vec<CouplePoint<T>> = self
                .body
                .particles
                .iter()
                .map(|p| CouplePoint::new(p.pos, p.vel, T::one() / p.inv_mass))
                .collect();
            if let Some(f) = world
                .get_mut(fidx)
                .and_then(|s| s.as_any_mut().downcast_mut::<FluidSubsystem<T>>())
            {
                f.world
                    .couple_points(&mut pts, *dt, self.fluid_drag, self.soft_density);
            }
            // 把合力写回软体质点:更新速度 + 累加力(供下一帧 velocity-Verlet 使用)。
            for (p, cp) in self.body.particles.iter_mut().zip(pts.iter()) {
                if p.inv_mass > T::zero() && cp.force.norm() > T::zero() {
                    let f = cp.force;
                    p.vel += f * p.inv_mass * *dt;
                    p.force += f;
                }
            }
        }

        // (3) 软体↔热场:热浮力 + 对流换热(M11)。
        if self.thermal_expansion > T::zero() || self.heat_gain > T::zero() {
            let mut heat_idx = None;
            let mut i = 0;
            while let Some(s) = world.get(i) {
                if s.as_any().downcast_ref::<HeatField<T>>().is_some() {
                    heat_idx = Some(i);
                    break;
                }
                i += 1;
            }
            if let Some(hi) = heat_idx {
                let t_ref = self.t_ref;
                let mut heat_box = world.remove(hi);
                if let Some(heat) = heat_box.as_any_mut().downcast_mut::<HeatField<T>>() {
                    self.body.couple_heat(
                        heat,
                        *dt,
                        t_ref,
                        self.thermal_expansion,
                        self.heat_gain,
                    );
                }
                world.insert(hi, heat_box);
            }
        }
    }

    fn name(&self) -> &'static str {
        "soft"
    }
}
