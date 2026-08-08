//! 引力场子系统(M13)。
//!
//! `GravField` 持有质量密度场 `rho` 与引力势工作场 `phi`,通过 Jacobi 松弛求解
//! 泊松方程 ∇²Φ = 4πG·ρ,再由 `g = -∇Φ` 中心差分得到引力加速度矢量 `g`。
//! 用于表示空间变化的质量分布产生的局部引力井(叠加在 `RigidWorld` 的均匀重力之上)。
//! 支持刚体↔引力双向耦合:物体受局部引力偏转加速,同时把质量沉积进网格反向塑造引力场。

use std::any::Any;

use phy_core::Subsystem;
use phy_math::{RealField, Vec3};

use crate::grid_geometry::GridGeometry;
use crate::grid::{Bc, ScalarField};

/// 引力场子系统接口(供 `World` 动态查找 + 刚体↔引力双向耦合)。
pub trait GravFieldLike<T: RealField + Copy>: GridGeometry<T> + Any {
    /// 三线性插值引力加速度矢量,给定最近格点索引 (cx,cy,cz) 与 [0,1] 单元内偏移 (tx,ty,tz)。
    fn sample_g_field(
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
    /// 向网格单元 (cx,cy,cz) 沉积一个质量项(运动物体贡献 / 静态质量注入)。
    fn add_mass(&mut self, cx: usize, cy: usize, cz: usize, m: T);
    /// 引力常数 G。
    fn g_const(&self) -> T;
}

/// 引力场子系统(M13)。
pub struct GravField<T: RealField + Copy> {
    /// 质量密度场(单位网格质量)。
    pub rho: ScalarField<T>,
    /// 引力势工作场(泊松松弛的工作变量)。
    pub phi: ScalarField<T>,
    /// 引力加速度矢量(每格一个),由 `g = -∇Φ` 得到。
    pub g: Vec<Vec3<T>>,
    /// 引力常数 G。
    pub g_const: T,
    /// 泊松松弛的 Jacobi 迭代次数(每步)。
    pub jacobi_iters: usize,
}

impl<T: RealField + Copy> GravField<T> {
    /// 构造并初始化与 `rho` 同几何的势/引力缓冲。
    pub fn build(rho: ScalarField<T>, g_const: T) -> Self {
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
            g: vec![Vec3::zeros(); n],
            g_const,
            jacobi_iters: 30,
        }
    }

    #[inline]
    pub fn idx(&self, x: usize, y: usize, z: usize) -> usize {
        x + self.rho.nx * (y + self.rho.ny * z)
    }
}

impl<T: RealField + Copy> GridGeometry<T> for GravField<T> {
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

impl<T: RealField + Copy> GravFieldLike<T> for GravField<T> {
    fn sample_g_field(
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
        let c000 = self.g[self.idx(cl(cx, nx), cl(cy, ny), cl(cz, nz))];
        let c100 = self.g[self.idx(cl(cx + 1, nx), cl(cy, ny), cl(cz, nz))];
        let c010 = self.g[self.idx(cl(cx, nx), cl(cy + 1, ny), cl(cz, nz))];
        let c110 = self.g[self.idx(cl(cx + 1, nx), cl(cy + 1, ny), cl(cz, nz))];
        let c001 = self.g[self.idx(cl(cx, nx), cl(cy, ny), cl(cz + 1, nz))];
        let c101 = self.g[self.idx(cl(cx + 1, nx), cl(cy, ny), cl(cz + 1, nz))];
        let c011 = self.g[self.idx(cl(cx, nx), cl(cy + 1, ny), cl(cz + 1, nz))];
        let c111 = self.g[self.idx(cl(cx + 1, nx), cl(cy + 1, ny), cl(cz + 1, nz))];
        let x00 = c000 + (c100 - c000) * fx;
        let x10 = c010 + (c110 - c010) * fx;
        let x01 = c001 + (c101 - c001) * fx;
        let x11 = c011 + (c111 - c011) * fx;
        let y0 = x00 + (x10 - x00) * fy;
        let y1 = x01 + (x11 - x01) * fy;
        y0 + (y1 - y0) * fz
    }

    fn add_mass(&mut self, cx: usize, cy: usize, cz: usize, m: T) {
        self.rho.add_source(cx, cy, cz, m);
    }

    fn g_const(&self) -> T {
        self.g_const
    }
}

impl<T: RealField + Copy> Subsystem<T> for GravField<T> {
    fn as_any(&self) -> &dyn Any {
        self
    }

    fn as_any_mut(&mut self) -> &mut dyn Any {
        self
    }

    fn step(&mut self, _dt: &T) {
        // 先把源缓冲 src 并入质量密度 u(运动物体/天体注入),再清零。
        let n = self.rho.u.len();
        for i in 0..n {
            self.rho.u[i] += self.rho.src[i];
            self.rho.src[i] = T::zero();
        }
        // 泊松松弛 ∇²Φ = 4πG·ρ (Jacobi)。Neumann 边界(镜像自身=零通量)。
        // 离散:Φ = (Σ邻居 - dx²·4πG·ρ) / 6 —— 源项取负使质量处 Φ 为负阱,
        // 则 g = -∇Φ 在质量上方 ∂Φ/∂y>0 ⇒ g.y<0(向下指向质量)= 吸引。
        let dx2 = self.rho.dx * self.rho.dx;
        let four_pi_g = T::from_f64(4.0 * std::f64::consts::PI).unwrap() * self.g_const;
        let fac = -dx2 * four_pi_g;
        let (nx, ny, nz) = (self.rho.nx, self.rho.ny, self.rho.nz);
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
        // g = -∇Φ(中心差分)。质量为正 → Φ 在质量处取负 → ∇Φ 指向质量 → -∇Φ 指向质量 = 吸引。
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
                    let gx = -(val(x as isize + 1, y as isize, z as isize)
                        - val(x as isize - 1, y as isize, z as isize))
                        / two_dx;
                    let gy = -(val(x as isize, y as isize + 1, z as isize)
                        - val(x as isize, y as isize - 1, z as isize))
                        / two_dx;
                    let gz = -(val(x as isize, y as isize, z as isize + 1)
                        - val(x as isize, y as isize, z as isize - 1))
                        / two_dx;
                    self.g[i] = Vec3::new(gx, gy, gz);
                }
            }
        }
        self.rho.t += *_dt;
    }
    fn name(&self) -> &'static str {
        "grav"
    }
}
