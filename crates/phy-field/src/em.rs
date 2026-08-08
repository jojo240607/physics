//! 电磁场子系统(M12)。
//!
//! `EmField` 持有电荷密度场 `rho` 与电势工作场 `phi`,通过 Jacobi 松弛求解泊松方程
//! ∇²φ = -ρ/ε,再由 `E = -∇φ` 中心差分得到电场矢量 `e`。外加均匀磁场 `b_ext` 用于
//! 洛伦兹力的 `v×B` 项。每步 `step` 重新求解 `E`,支持刚体↔电磁双向耦合。

use std::any::Any;

use phy_core::Subsystem;
use phy_math::{RealField, Vec3};

use crate::grid_geometry::GridGeometry;
use crate::grid::{Bc, ScalarField};

/// 电磁场子系统接口(供 `World` 动态查找 + 刚体↔EM 双向耦合)。
pub trait EmFieldLike<T: RealField + Copy>: GridGeometry<T> + Any {
    /// 三线性插值电场矢量,给定最近格点索引 (cx,cy,cz) 和 [0,1] 单元内偏移 (tx,ty,tz)。
    fn sample_e_field(
        &self,
        cx: usize,
        cy: usize,
        cz: usize,
        tx: f64,
        ty: f64,
        tz: f64,
    ) -> Vec3<T>
    where
        T: num_traits::ToPrimitive;
    /// 向网格单元 (cx,cy,cz) 累加一个电荷项(运动带电体沉积/电流注入)。
    fn add_charge(&mut self, cx: usize, cy: usize, cz: usize, q: T);
    /// 外加均匀磁场 `B`(洛伦兹力 `v×B` 项)。
    fn b_ext(&self) -> Vec3<T>;
}

/// 电磁场子系统(M12)。
pub struct EmField<T: RealField + Copy> {
    /// 电荷密度场(单位网格电荷)。
    pub rho: ScalarField<T>,
    /// 电势工作场(泊松松弛的工作变量)。
    pub phi: ScalarField<T>,
    /// 电场矢量(每格一个),由 `-∇φ` 得到。
    pub e: Vec<Vec3<T>>,
    /// 外加均匀磁场 `B`(洛伦兹力 `v×B` 项)。默认零。
    pub b_ext: Vec3<T>,
    /// 泊松松弛的 Jacobi 迭代次数(每步)。
    pub jacobi_iters: usize,
    /// 介电常数 ε。
    pub epsilon: T,
}

impl<T: RealField + Copy> EmField<T> {
    /// 构造并初始化与 `rho` 同几何的电势/电场缓冲。
    pub fn build(rho: ScalarField<T>, epsilon: T) -> Self {
        let nx = rho.nx;
        let ny = rho.ny;
        let nz = rho.nz;
        let dx = rho.dx;
        let origin = rho.origin;
        let phi = ScalarField::new(nx, ny, nz, dx, T::zero(), Bc::Neumann).with_origin(origin);
        let n = nx * ny * nz;
        Self {
            rho,
            phi,
            e: vec![Vec3::zeros(); n],
            b_ext: Vec3::zeros(),
            jacobi_iters: 30,
            epsilon,
        }
    }

    #[inline]
    pub fn idx(&self, x: usize, y: usize, z: usize) -> usize {
        x + self.rho.nx * (y + self.rho.ny * z)
    }
}

impl<T: RealField + Copy> GridGeometry<T> for EmField<T> {
    fn dims(&self) -> (usize, usize, usize) {
        (self.rho.nx, self.rho.ny, self.rho.nz)
    }
    fn origin(&self) -> Vec3<T> {
        self.rho.origin
    }
    fn cell_size(&self) -> T {
        self.rho.dx
    }
}

impl<T: RealField + Copy> EmFieldLike<T> for EmField<T> {
    fn sample_e_field(
        &self,
        cx: usize,
        cy: usize,
        cz: usize,
        tx: f64,
        ty: f64,
        tz: f64,
    ) -> Vec3<T>
    where
        T: num_traits::ToPrimitive,
    {
        let nx = self.rho.nx;
        let ny = self.rho.ny;
        let nz = self.rho.nz;
        let fx = T::from_f64(tx).unwrap_or(T::zero());
        let fy = T::from_f64(ty).unwrap_or(T::zero());
        let fz = T::from_f64(tz).unwrap_or(T::zero());
        let cl = |v: usize, n: usize| v.min(n - 1);
        let c000 = self.e[self.idx(cl(cx, nx), cl(cy, ny), cl(cz, nz))];
        let c100 = self.e[self.idx(cl(cx + 1, nx), cl(cy, ny), cl(cz, nz))];
        let c010 = self.e[self.idx(cl(cx, nx), cl(cy + 1, ny), cl(cz, nz))];
        let c110 = self.e[self.idx(cl(cx + 1, nx), cl(cy + 1, ny), cl(cz, nz))];
        let c001 = self.e[self.idx(cl(cx, nx), cl(cy, ny), cl(cz + 1, nz))];
        let c101 = self.e[self.idx(cl(cx + 1, nx), cl(cy, ny), cl(cz + 1, nz))];
        let c011 = self.e[self.idx(cl(cx, nx), cl(cy + 1, ny), cl(cz + 1, nz))];
        let c111 = self.e[self.idx(cl(cx + 1, nx), cl(cy + 1, ny), cl(cz + 1, nz))];
        let x00 = c000 + (c100 - c000) * fx;
        let x10 = c010 + (c110 - c010) * fx;
        let x01 = c001 + (c101 - c001) * fx;
        let x11 = c011 + (c111 - c011) * fx;
        let y0 = x00 + (x10 - x00) * fy;
        let y1 = x01 + (x11 - x01) * fy;
        y0 + (y1 - y0) * fz
    }

    fn add_charge(&mut self, cx: usize, cy: usize, cz: usize, q: T) {
        self.rho.add_source(cx, cy, cz, q);
    }

    fn b_ext(&self) -> Vec3<T> {
        self.b_ext
    }
}

impl<T: RealField + Copy> Subsystem<T> for EmField<T> {
    fn as_any(&self) -> &dyn Any {
        self
    }

    fn as_any_mut(&mut self) -> &mut dyn Any {
        self
    }

    fn step(&mut self, _dt: &T) {
        // 泊松松弛 ∇²φ = -ρ/ε (Jacobi)。Neumann 边界(镜像自身=零通量)。
        // 离散:φ = (Σ邻居 - dx²·(-ρ/ε)) / 6 = (Σ邻居 + dx²·ρ/ε) / 6。
        let dx2 = self.rho.dx * self.rho.dx;
        let fac = dx2 / self.epsilon;
        let (nx, ny, nz) = (self.rho.nx, self.rho.ny, self.rho.nz);
        // 初始化 phi 为上一帧值(热启动),首帧为零。
        for _ in 0..self.jacobi_iters {
            let mut pnew = vec![T::zero(); self.phi.u.len()];
            for z in 0..nz {
                for y in 0..ny {
                    for x in 0..nx {
                        let i = self.idx(x, y, z);
                        let val = |x: isize, y: isize, z: isize| -> T {
                            let (x, y, z) = (x as usize, y as usize, z as usize);
                            if x < nx && y < ny && z < nz {
                                self.phi.u[self.idx(x, y, z)]
                            } else {
                                self.phi.u[i] // Neumann:镜像自身
                            }
                        };
                        let sum = val(x as isize - 1, y as isize, z as isize)
                            + val(x as isize + 1, y as isize, z as isize)
                            + val(x as isize, y as isize - 1, z as isize)
                            + val(x as isize, y as isize + 1, z as isize)
                            + val(x as isize, y as isize, z as isize - 1)
                            + val(x as isize, y as isize, z as isize + 1);
                        pnew[i] = (sum + fac * self.rho.u[i]) / T::from_f64(6.0).unwrap();
                    }
                }
            }
            self.phi.u = pnew;
        }
        // E = -∇φ(中心差分)。
        let two_dx = T::from_f64(2.0).unwrap() * self.rho.dx;
        for z in 0..nz {
            for y in 0..ny {
                for x in 0..nx {
                    let i = self.idx(x, y, z);
                    let val = |x: isize, y: isize, z: isize| -> T {
                        let (x, y, z) = (x as usize, y as usize, z as usize);
                        if x < nx && y < ny && z < nz {
                            self.phi.u[self.idx(x, y, z)]
                        } else {
                            self.phi.u[i]
                        }
                    };
                    let ex = -(val(x as isize + 1, y as isize, z as isize)
                        - val(x as isize - 1, y as isize, z as isize))
                        / two_dx;
                    let ey = -(val(x as isize, y as isize + 1, z as isize)
                        - val(x as isize, y as isize - 1, z as isize))
                        / two_dx;
                    let ez = -(val(x as isize, y as isize, z as isize + 1)
                        - val(x as isize, y as isize, z as isize - 1))
                        / two_dx;
                    self.e[i] = Vec3::new(ex, ey, ez);
                }
            }
        }
        self.rho.t += *_dt; // 仅维持时间推进语义
    }
    fn name(&self) -> &'static str {
        "em"
    }
}
