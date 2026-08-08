//! 热场子系统(M7)。
//!
//! `HeatField` 是 `grid::ScalarField` 的 `Subsystem` 适配层,并实现了
//! `HeatFieldLike` 接口供流体/软体耦合访问(三线性温度采样 + 热源注入)。

use phy_core::Subsystem;
use phy_math::{RealField, Vec3};

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
}

/// 热扩散子系统。
pub struct HeatField<T: RealField + Copy> {
    /// 底层标量场(温度)。
    pub field: ScalarField<T>,
    /// 热扩散率 α。
    pub alpha: T,
    /// 上次步进的稳定性数 r=α·dt/dx²(>1/6 不可信)。
    pub last_r: T,
}

impl<T: RealField + Copy> HeatField<T> {
    /// 构造热场适配层。
    pub fn new(field: ScalarField<T>, alpha: T) -> Self {
        Self {
            field,
            alpha,
            last_r: T::zero(),
        }
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

impl<T: RealField + Copy> HeatFieldLike<T> for HeatField<T> {
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
}

impl<T: RealField + Copy> Subsystem<T> for HeatField<T> {
    fn as_any(&self) -> &dyn std::any::Any {
        self
    }

    fn as_any_mut(&mut self) -> &mut dyn std::any::Any {
        self
    }

    fn step(&mut self, dt: &T) {
        self.last_r = self.field.step_diffusion(self.alpha, *dt);
    }
    fn name(&self) -> &'static str {
        "heat"
    }
}
