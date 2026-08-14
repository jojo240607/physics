//! 把 `OpticScene` 适配为 `phy_core::Subsystem`,并提供一个离线成像辅助。

use phy_core::Subsystem;
use phy_math::{RealField, Vec3};
use phy_rigid::RigidSubsystem;
use serde::de::DeserializeOwned;
use serde::{Deserialize, Serialize};
use rayon::prelude::*;

use crate::renderer::{Approx, Renderer, Whitted};
use crate::scene::OpticScene;

/// 光学后端精度开关。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[non_exhaustive]
pub enum Precision {
    /// 离线 Whitted 递归追踪(高保真)。
    Offline,
    /// 实时近似(单次折射 + 阴影)。
    Realtime,
}

/// 光学子系统:持有场景与精度模式,可挂入统一 `World<T>`。
///
/// 光场本身不随时间演进(`step` 为空),但保留接口以便将来接入时变介质/动画光源。
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(bound = "T: RealField + Copy + Serialize + DeserializeOwned + nalgebra::Scalar + num_traits::ToPrimitive")]
pub struct OpticSubsystem<T: RealField + Copy + num_traits::ToPrimitive> {
    /// 场景。
    pub scene: OpticScene<T>,
    /// 当前精度模式。
    pub precision: Precision,
    /// 光学↔世界(刚体)耦合强度:>0 时每步把"来自刚体"的光学体位姿同步到最新刚体位姿。
    /// 这实现了光线对运动刚体的正确求交(物理驱动的光学场景)。
    pub optic_coupling: T,
}

impl<T: RealField + Copy + num_traits::ToPrimitive> OpticSubsystem<T> {
    /// 构造。
    pub fn new(scene: OpticScene<T>, precision: Precision) -> Self {
        Self {
            scene,
            precision,
            optic_coupling: T::zero(),
        }
    }

    /// 切换精度。
    pub fn set_precision(&mut self, p: Precision) {
        self.precision = p;
    }

    /// 用相机参数渲染一帧到 `buf`(宽×高 RGB f64,行优先)。
    /// 简单针孔相机:`eye` 看向 `target`,`fov` 为垂直视场(弧度)。
    ///
    /// S7 并行后端:每个像素的 `trace` 相互独立(只读 `scene`/`eye`/方向),
    /// 故用 rayon 并行遍历全部像素,结果写入并行色缓冲后回填 `buf`。
    /// 像素结果仅由 (x,y) 决定,与遍历顺序无关,并行渲染与串行逐像素完全一致。
    pub fn render_camera(
        &self,
        buf: &mut [Vec3<T>],
        width: usize,
        height: usize,
        eye: &Vec3<T>,
        target: &Vec3<T>,
        up: &Vec3<T>,
        fov: T,
    ) {
        // S7 并行后端:每个像素的追踪相互独立,故按精度等级分发到具体的
        // `Whitted`/`Approx`(二者均为零字段单元结构体,天然 `Sync`),再在 rayon
        // 并行闭包中传入 `&R: Renderer<T> + Sync`,避开 `dyn Renderer` 不可跨线程
        // 共享的限制。像素结果仅由坐标决定,并行渲染与串行逐像素完全一致。
        match self.precision {
            Precision::Offline => self.render_parallel::<Whitted>(
                &Whitted, buf, width, height, eye, target, up, fov,
            ),
            Precision::Realtime => self.render_parallel::<Approx>(
                &Approx, buf, width, height, eye, target, up, fov,
            ),
        }
    }

    /// 泛型并行渲染核心:`R` 为具体渲染后端(`Whitted`/`Approx`),`Sync` 以便
    /// 在 rayon 闭包中按 `&R` 共享。
    fn render_parallel<R: Renderer<T> + Sync>(
        &self,
        renderer: &R,
        buf: &mut [Vec3<T>],
        width: usize,
        height: usize,
        eye: &Vec3<T>,
        target: &Vec3<T>,
        up: &Vec3<T>,
        fov: T,
    ) {
        // 相机基向量(右手)。
        let fwd = *target - *eye;
        let fwd_len = fwd.norm();
        let fwd = if fwd_len > T::from_f64(1e-12).unwrap() {
            fwd / fwd_len
        } else {
            Vec3::new(T::zero(), T::zero(), -T::one())
        };
        let right = fwd.cross(up);
        let right = if right.norm() > T::from_f64(1e-12).unwrap() {
            right / right.norm()
        } else {
            Vec3::new(T::one(), T::zero(), T::zero())
        };
        let true_up = right.cross(&fwd);
        let aspect = (width as f64) / (height as f64);
        let tan_h = (fov * T::from_f64(0.5).unwrap()).tan();
        let tan_w = tan_h * T::from_f64(aspect).unwrap();

        // 并行逐像素追踪(每像素仅依赖其坐标,无共享可变状态)。
        let colors: Vec<Vec3<T>> = (0..width * height)
            .into_par_iter()
            .map(|idx| {
                let x = idx % width;
                let y = idx / width;
                // NDC [-1,1],y 向下。
                let u = (T::from_f64(2.0 * (x as f64) / (width as f64 - 1.0) - 1.0).unwrap())
                    * tan_w;
                let v = (T::from_f64(1.0 - 2.0 * (y as f64) / (height as f64 - 1.0)).unwrap())
                    * tan_h;
                let dir = fwd + right * u + true_up * v;
                let dir = if dir.norm() > T::from_f64(1e-12).unwrap() {
                    dir / dir.norm()
                } else {
                    fwd
                };
                renderer.trace(&self.scene, eye, &dir)
            })
            .collect();
        buf.copy_from_slice(&colors);
    }
}

impl<T: RealField + Copy + num_traits::ToPrimitive> Subsystem<T> for OpticSubsystem<T> {
    fn as_any(&self) -> &dyn std::any::Any {
        self
    }

    fn as_any_mut(&mut self) -> &mut dyn std::any::Any {
        self
    }

    fn step(&mut self, _dt: &T) {
        // 光场稳态:不随时间演进。
    }

    fn name(&self) -> &'static str {
        "optics"
    }

    /// 光学↔世界(刚体)耦合(M16):若 `optic_coupling>0`,在 `World` 中查找 `RigidSubsystem`,
    /// 把每个标记为"来自刚体"的光学体(`source_rigid_idx` 非空)的位姿/形状,
    /// 同步到该刚体当前的最新状态。这样光线追踪能正确反映运动刚体(如下落的玻璃球)。
    fn couple(&mut self, world: &mut phy_core::World<T>, _dt: &T) {
        if self.optic_coupling <= T::zero() {
            return;
        }
        let n = world.subsystem_count();
        let mut rigid_idx: Option<usize> = None;
        for i in 0..n {
            if let Some(s) = world.get(i) {
                if s.as_any().downcast_ref::<RigidSubsystem<T>>().is_some() {
                    rigid_idx = Some(i);
                    break;
                }
            }
        }
        let ri = match rigid_idx {
            Some(ri) => ri,
            None => return,
        };
        let mut rigid_box = world.remove(ri);
        let rigid = rigid_box
            .as_any_mut()
            .downcast_mut::<RigidSubsystem<T>>()
            .expect("rigid subsystem type mismatch");
        for ob in self.scene.bodies.iter_mut() {
            if let Some(idx) = ob.source_rigid_idx {
                if let Some(rb) = rigid.world.bodies.get(idx) {
                    ob.sync_from_rigid(rb);
                }
            }
        }
        world.insert(ri, rigid_box);
    }
}
