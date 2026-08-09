//! 热场子系统(M7)。
//!
//! `HeatField` 是 `grid::ScalarField` 的 `Subsystem` 适配层,并实现了
//! `HeatFieldLike` 接口供流体/软体耦合访问(三线性温度采样 + 热源注入)。

use phy_core::Subsystem;
use phy_math::{RealField, Vec3};
use serde::de::DeserializeOwned;
use serde::{Deserialize, Serialize};

use crate::grid_geometry::GridGeometry;
use crate::grid::ScalarField;

/// 热场子系统接口(供 `World` 动态查找 + 跨场耦合)。
pub trait HeatFieldLike<T: RealField + Copy>: GridGeometry<T> + std::any::Any {
    /// 三线性插值温度,给定最近格点索引 (cx,cy,cz) 和 [0,1] 单元内偏移 (tx,ty,tz)。
    fn sample_trilinear(
        &self,
        cx: usize,
        cy: usize,
        cz: usize,
        tx: f64,
        ty: f64,
        tz: f64,
    ) -> T
    where
        T: num_traits::ToPrimitive;
    /// 向网格单元 (cx,cy,cz) 累加一个热源项。
    fn add_source(&mut self, cx: usize, cy: usize, cz: usize, q: T);
    /// 安装/清除对流速度场采样器(世界坐标 → 速度矢量)。
    /// 安装后 `step` 改为扩散-对流混合求解,构成热浮力↔对流闭环。
    fn set_vel_sampler(&mut self, f: Option<Box<dyn Fn(Vec3<T>) -> Vec3<T>>>);
}

/// 热扩散子系统。
#[derive(Serialize, Deserialize)]
#[serde(bound = "T: RealField + Copy + Serialize + DeserializeOwned + nalgebra::Scalar")]
pub struct HeatField<T: RealField + Copy> {
    /// 底层标量场(温度)。
    pub field: ScalarField<T>,
    /// 热扩散率 α。
    pub alpha: T,
    /// 上次步进的稳定性数 r=α·dt/dx²(>1/6 不可信)。
    pub last_r: T,
    /// 可选对流速度场采样器(世界坐标 → 速度)。由流体耦合写入,
    /// 使热场在 `step` 时做扩散-对流(而非纯扩散),构成热浮力↔对流闭环。
    /// 闭包不可序列化,存档时跳过,加载后由耦合方重新安装。
    #[serde(skip)]
    pub vel_sampler: Option<Box<dyn Fn(Vec3<T>) -> Vec3<T>>>,
}

impl<T: RealField + Copy + std::fmt::Debug> std::fmt::Debug for HeatField<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("HeatField")
            .field("field", &self.field)
            .field("alpha", &self.alpha)
            .field("last_r", &self.last_r)
            .field("vel_sampler", &self.vel_sampler.is_some())
            .finish()
    }
}

impl<T: RealField + Copy> Clone for HeatField<T> {
    fn clone(&self) -> Self {
        HeatField {
            field: self.field.clone(),
            alpha: self.alpha,
            last_r: self.last_r,
            vel_sampler: None,
        }
    }
}

impl<T: RealField + Copy + num_traits::ToPrimitive> HeatField<T> {
    /// 构造热场适配层。
    pub fn new(field: ScalarField<T>, alpha: T) -> Self {
        Self {
            field,
            alpha,
            last_r: T::zero(),
            vel_sampler: None,
        }
    }

    /// 安装对流速度场采样器(典型来自 SPH 速度场)。
    /// 传入 `None` 可恢复为纯扩散。
    pub fn set_vel_sampler(&mut self, f: Option<Box<dyn Fn(Vec3<T>) -> Vec3<T>>>) {
        self.vel_sampler = f;
    }
}

impl<T: RealField + Copy> GridGeometry<T> for HeatField<T> {
    fn dims(&self) -> (usize, usize, usize) {
        (self.field.nx, self.field.ny, self.field.nz)
    }
    fn origin(&self) -> Vec3<T> {
        self.field.origin
    }
    fn cell_size(&self) -> T {
        self.field.dx
    }
}

impl<T: RealField + Copy + num_traits::ToPrimitive> HeatFieldLike<T> for HeatField<T> {
    fn sample_trilinear(
        &self,
        cx: usize,
        cy: usize,
        cz: usize,
        tx: f64,
        ty: f64,
        tz: f64,
    ) -> T
    where
        T: num_traits::ToPrimitive,
    {
        let fx = T::from_f64(tx).unwrap_or(T::zero());
        let fy = T::from_f64(ty).unwrap_or(T::zero());
        let fz = T::from_f64(tz).unwrap_or(T::zero());
        let c000 = self.field.sample(cx, cy, cz);
        let c100 = self.field.sample((cx + 1).min(self.field.nx - 1), cy, cz);
        let c010 = self.field.sample(cx, (cy + 1).min(self.field.ny - 1), cz);
        let c110 = self.field.sample(
            (cx + 1).min(self.field.nx - 1),
            (cy + 1).min(self.field.ny - 1),
            cz,
        );
        let c001 = self.field.sample(cx, cy, (cz + 1).min(self.field.nz - 1));
        let c101 = self
            .field
            .sample((cx + 1).min(self.field.nx - 1), cy, (cz + 1).min(self.field.nz - 1));
        let c011 = self
            .field
            .sample(cx, (cy + 1).min(self.field.ny - 1), (cz + 1).min(self.field.nz - 1));
        let c111 = self.field.sample(
            (cx + 1).min(self.field.nx - 1),
            (cy + 1).min(self.field.ny - 1),
            (cz + 1).min(self.field.nz - 1),
        );
        let x00 = c000 + (c100 - c000) * fx;
        let x10 = c010 + (c110 - c010) * fx;
        let x01 = c001 + (c101 - c001) * fx;
        let x11 = c011 + (c111 - c011) * fx;
        let y0 = x00 + (x10 - x00) * fy;
        let y1 = x01 + (x11 - x01) * fy;
        y0 + (y1 - y0) * fz
    }
    fn add_source(&mut self, cx: usize, cy: usize, cz: usize, q: T) {
        self.field.add_source(cx, cy, cz, q);
    }
    fn set_vel_sampler(&mut self, f: Option<Box<dyn Fn(Vec3<T>) -> Vec3<T>>>) {
        self.vel_sampler = f;
    }
}

impl<T: RealField + Copy + num_traits::ToPrimitive> Subsystem<T> for HeatField<T> {
    fn as_any(&self) -> &dyn std::any::Any {
        self
    }

    fn as_any_mut(&mut self) -> &mut dyn std::any::Any {
        self
    }

    fn step(&mut self, dt: &T) {
        if let Some(vel) = &self.vel_sampler {
            // 扩散-对流混合步:热浮力驱动的羽流会被自身产生的速度场带走。
            self.last_r = self.field.step_advection_diffusion(vel, self.alpha, *dt);
        } else {
            self.last_r = self.field.step_diffusion(self.alpha, *dt);
        }
    }
    fn name(&self) -> &'static str {
        "heat"
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Bc;

    #[test]
    fn heat_step_advects_with_installed_velocity() {
        // 安装速度采样器后,热场 step 应做扩散-对流,温度峰随流平移(热浮力↔对流闭环基础)。
        let mut field = ScalarField::<f64>::new(16, 16, 16, 0.5, 0.0, Bc::Neumann);
        let ci = field.idx(4, 8, 8);
        field.u[ci] = 1.0;
        let mut heat = HeatField::new(field, 0.0); // α=0 纯平流
        // 沿 +x 匀速,dt=1 => 平移 1 个 dx(0.5)
        heat.set_vel_sampler(Some(Box::new(|_p: Vec3<f64>| Vec3::new(0.5, 0.0, 0.0))));
        heat.step(&1.0);
        // 峰应出现在 i≈5
        let new_ci = heat.field.idx(5.min(15), 8, 8);
        assert!(
            heat.field.u[new_ci] > 0.8,
            "加热场对流后峰应平移到 i=5, 实测={}",
            heat.field.u[new_ci]
        );
        assert!(
            heat.field.u[ci] < 0.2,
            "原位置温度应被对流带走, 实测={}",
            heat.field.u[ci]
        );
    }

    #[test]
    fn heat_step_without_sampler_is_pure_diffusion() {
        // 无速度场时退化为纯扩散(非平流):中心峰只扩散不外移。
        let mut field = ScalarField::<f64>::new(16, 16, 16, 0.5, 0.0, Bc::Neumann);
        let ci = field.idx(8, 8, 8);
        field.u[ci] = 1.0;
        let mut heat = HeatField::new(field, 0.05);
        heat.step(&0.1); // r = 0.05*0.1/0.25 < 1/6
        let sum: f64 = heat.field.u.iter().sum();
        assert!(sum > 0.9, "纯扩散不丢失质量, 总量应≈原值, sum={}", sum);
    }
}
