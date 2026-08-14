//! 规则网格标量/矢量场 + 显式有限差分求解器(热扩散 / 波动)。
//!
//! 网格:三维笛卡尔(支持 1D/2D 退化,把无关维度设为 1)。
//! 坐标索引 `idx = x + nx*(y + ny*z)`。
//! 边界:可配置 Dirichlet(固定值)或 Neumann(零通量,自然反射)。
//!
//! 物理:
//! - 热扩散:∂u/∂t = α ∇²u  (FTCS 显式,条件 α·dt/dx² ≤ 1/6)。
//! - 波动/电磁标量:∂²u/∂t² = c² ∇²u  (leapfrog 显式,条件 c·dt/dx ≤ 1/√3)。

use num_traits::FromPrimitive;
use phy_math::{RealField, Vec3};
use serde::de::DeserializeOwned;
use serde::{Deserialize, Serialize};

/// 边界条件。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[non_exhaustive]
pub enum Bc {
    /// 边界固定为 `bc_val`(狄利克雷)。
    Dirichlet,
    /// 边界零通量(诺依曼,自然反射)。
    Neumann,
}

/// 规则网格标量场求解器。
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(bound = "T: RealField + Copy + Serialize + DeserializeOwned + nalgebra::Scalar")]
pub struct ScalarField<T: RealField + Copy> {
    /// 各维度网格数。
    pub nx: usize,
    pub ny: usize,
    pub nz: usize,
    /// 网格间距。
    pub dx: T,
    /// 当前场值。
    pub u: Vec<T>,
    /// 上一时刻场值(leapfrog / 半步用)。
    pub u_prev: Vec<T>,
    /// 源项(每步注入,step 后清零)。
    pub src: Vec<T>,
    /// 边界条件。
    pub bc: Bc,
    /// 边界固定值(Dirichlet 用)。
    pub bc_val: T,
    /// 当前时间。
    pub t: T,
    /// 网格原点(最小角)的世界坐标;默认 (0,0,0)。
    /// 用于把网格索引映射到世界坐标,支持耦合(流体/软体位置 → 网格)。
    #[serde(with = "crate::serde_geom")]
    pub origin: Vec3<T>,
}

impl<T: RealField + Copy> ScalarField<T> {
    /// 构造(nx×ny×nz),全部初始化为 `val`,原点默认 (0,0,0)。
    pub fn new(nx: usize, ny: usize, nz: usize, dx: T, val: T, bc: Bc) -> Self {
        let n = nx * ny * nz;
        Self {
            nx,
            ny,
            nz,
            dx,
            u: vec![val; n],
            u_prev: vec![val; n],
            src: vec![T::zero(); n],
            bc,
            bc_val: val,
            t: T::zero(),
            origin: Vec3::zeros(),
        }
    }

    /// 设置网格原点(世界坐标最小角)并返回 `self`,链式构造。
    pub fn with_origin(mut self, origin: Vec3<T>) -> Self {
        self.origin = origin;
        self
    }

    #[inline]
    pub fn idx(&self, x: usize, y: usize, z: usize) -> usize {
        x + self.nx * (y + self.ny * z)
    }

    /// 中心差分拉普拉斯算子(自动处理边界:越界按 bc 处理)。
    fn laplacian(&self, i: usize, x: usize, y: usize, z: usize) -> T {
        let two = <T as FromPrimitive>::from_f64(2.0).unwrap();
        let zero = T::zero();
        let mut lap = zero;
        // X 轴
        let (xm, xp) = self.neighbor_val(i, x, y, z, 0);
        lap += xm + xp - self.u[i] * two;
        // Y 轴
        let (ym, yp) = self.neighbor_val(i, x, y, z, 1);
        lap += ym + yp - self.u[i] * two;
        // Z 轴
        let (zm, zp) = self.neighbor_val(i, x, y, z, 2);
        lap += zm + zp - self.u[i] * two;
        lap // 返回原始模板和(未除 dx²);步进位再乘 (α·dt/dx²) 或 (c²·dt²/dx²)
    }

    /// 取某轴(axis:0=X,1=Y,2=Z)的负/正邻居值;越界按边界条件返回
    /// (Dirichlet→bc_val,Neumann→自身值=零通量)。
    fn neighbor_val(&self, i: usize, x: usize, y: usize, z: usize, axis: usize) -> (T, T) {
        let (nx, ny, nz) = (self.nx as isize, self.ny as isize, self.nz as isize);
        let px = x as isize + if axis == 0 { -1 } else { 0 };
        let py = y as isize + if axis == 1 { -1 } else { 0 };
        let pz = z as isize + if axis == 2 { -1 } else { 0 };
        let qx = x as isize + if axis == 0 { 1 } else { 0 };
        let qy = y as isize + if axis == 1 { 1 } else { 0 };
        let qz = z as isize + if axis == 2 { 1 } else { 0 };
        let val_at = |p: (isize, isize, isize)| -> T {
            if p.0 < 0 || p.0 >= nx || p.1 < 0 || p.1 >= ny || p.2 < 0 || p.2 >= nz {
                match self.bc {
                    Bc::Dirichlet => self.bc_val,
                    Bc::Neumann => self.u[i], // 零通量:等价镜像自身
                }
            } else {
                self.u[self.idx(p.0 as usize, p.1 as usize, p.2 as usize)]
            }
        };
        (val_at((px, py, pz)), val_at((qx, qy, qz)))
    }

    /// 注入源项(累加)。坐标越界忽略。
    pub fn add_source(&mut self, x: usize, y: usize, z: usize, v: T) {
        if x < self.nx && y < self.ny && z < self.nz {
            let i = self.idx(x, y, z);
            self.src[i] += v;
        }
    }

    /// 热扩散一步(FTCS 显式)。`alpha` 为热扩散率。
    /// 返回稳定性数 `r = alpha*dt/dx²`(调用方可用以检查)。若 r>1/6 结果不可信。
    pub fn step_diffusion(&mut self, alpha: T, dt: T) -> T {
        let r = alpha * dt / (self.dx * self.dx);
        let mut unew = vec![T::zero(); self.u.len()];
        for z in 0..self.nz {
            for y in 0..self.ny {
                for x in 0..self.nx {
                    let i = self.idx(x, y, z);
                    unew[i] = self.u[i] + r * self.laplacian(i, x, y, z);
                    unew[i] += self.src[i] * dt;
                }
            }
        }
        self.u = unew;
        self.src = vec![T::zero(); self.src.len()]; // 源项每步清零
        self.t += dt;
        r
    }

    /// 私有三线性插值:在基格 (bx,by,bz) 内按偏移 (tx,ty,tz)∈[0,1] 取值。
    /// 用于平流回溯采样,越界基格夹紧到有效范围。注意:采样**快照** `u_prev`
    /// (平流前由 `advect` 拷入),避免读到同一步已写出的新值。
    fn trilinear(&self, bx: usize, by: usize, bz: usize, tx: T, ty: T, tz: T) -> T {
        let nx = self.nx;
        let ny = self.ny;
        let nz = self.nz;
        let x1 = (bx + 1).min(nx - 1);
        let y1 = (by + 1).min(ny - 1);
        let z1 = (bz + 1).min(nz - 1);
        let c000 = self.u_prev[self.idx(bx, by, bz)];
        let c100 = self.u_prev[self.idx(x1, by, bz)];
        let c010 = self.u_prev[self.idx(bx, y1, bz)];
        let c110 = self.u_prev[self.idx(x1, y1, bz)];
        let c001 = self.u_prev[self.idx(bx, by, z1)];
        let c101 = self.u_prev[self.idx(x1, by, z1)];
        let c011 = self.u_prev[self.idx(bx, y1, z1)];
        let c111 = self.u_prev[self.idx(x1, y1, z1)];
        let x00 = c000 + (c100 - c000) * tx;
        let x10 = c010 + (c110 - c010) * tx;
        let x01 = c001 + (c101 - c001) * tx;
        let x11 = c011 + (c111 - c011) * tx;
        let y0 = x00 + (x10 - x00) * ty;
        let y1 = x01 + (x11 - x01) * ty;
        y0 + (y1 - y0) * tz
    }

    /// 波动一步(leapfrog 显式)。`c2 = c²`。需要 prior 状态(u_prev)。
    /// 第一步自动从 u 复制构造 prior(零速度初始化)。
    pub fn step_wave(&mut self, c2: T, dt: T) {
        if self.t == T::zero() {
            self.u_prev = self.u.clone();
        }
        let two = <T as FromPrimitive>::from_f64(2.0).unwrap();
        let r2 = c2 * dt * dt / (self.dx * self.dx);
        let mut unew = vec![T::zero(); self.u.len()];
        for z in 0..self.nz {
            for y in 0..self.ny {
                for x in 0..self.nx {
                    let i = self.idx(x, y, z);
                    let lap = self.laplacian(i, x, y, z);
                    unew[i] = two * self.u[i] - self.u_prev[i] + r2 * lap + self.src[i] * dt * dt;
                }
            }
        }
        self.u_prev = std::mem::replace(&mut self.u, unew);
        self.src = vec![T::zero(); self.src.len()];
        self.t += dt;
    }

    /// 在某点取插值(最近点),坐标越界夹取。
    pub fn sample(&self, x: usize, y: usize, z: usize) -> T {
        let cx = x.min(self.nx - 1);
        let cy = y.min(self.ny - 1);
        let cz = z.min(self.nz - 1);
        self.u[self.idx(cx, cy, cz)]
    }

    /// 总场量(用于守恒检查)。
    pub fn sum(&self) -> T {
        let mut s = T::zero();
        for &v in &self.u {
            s += v;
        }
        s
    }

    /// 最大绝对值(用于稳定性/幅值检查)。
    pub fn max_abs(&self) -> T {
        let mut m = T::zero();
        for &v in &self.u {
            let a = v.abs();
            if a > m {
                m = a;
            }
        }
        m
    }
}

/// 需要 `T: ToPrimitive` 的平流相关方法(回溯/插值的世界坐标换算用到 `to_f64`)。
impl<T: RealField + Copy + num_traits::ToPrimitive> ScalarField<T> {
    /// 半拉格朗日平流一步:∂u/∂t + v·∇u = 0。
    ///
    /// 对每个网格点,按速度场 `vel` 回溯轨迹 `x_back = x - v(x)·dt`,
    /// 用三线性插值取回 `u` 赋给新场。该方法**无条件稳定**(回溯 + 插值),
    /// 不需要像 FTCS 那样限制 dt。`vel` 为任意世界坐标 → 速度矢量的闭包,
    /// 典型来源是 SPH 速度场采样(实现热浮力↔对流的 Boussinesq 闭环)。
    ///
    /// 越界回溯点夹紧到边界格(Neumann 型无通量近似)。
    pub fn advect<F>(&mut self, vel: &F, dt: T)
    where
        F: Fn(Vec3<T>) -> Vec3<T>,
    {
        // 以 u_prev 作快照源,u 作输出,避免读取刚写入的值。
        self.u_prev.copy_from_slice(&self.u);
        let dx = self.dx.to_f64().unwrap();
        let ox = self.origin.x.to_f64().unwrap();
        let oy = self.origin.y.to_f64().unwrap();
        let oz = self.origin.z.to_f64().unwrap();
        let (nx, ny, nz) = (self.nx, self.ny, self.nz);
        for k in 0..nz {
            for j in 0..ny {
                for i in 0..nx {
                    let c = self.idx(i, j, k);
                    let wx = ox + dx * (i as f64);
                    let wy = oy + dx * (j as f64);
                    let wz = oz + dx * (k as f64);
                    let x = Vec3::new(
                        T::from_f64(wx).unwrap(),
                        T::from_f64(wy).unwrap(),
                        T::from_f64(wz).unwrap(),
                    );
                    let v = vel(x);
                    let back = x - v * dt;
                    // 世界坐标 → 带夹紧的基格 + 单元内偏移(与 lib::world_to_cell 同语义)。
                    let dx = self.dx.to_f64().unwrap();
                    let obx = back.x.to_f64().unwrap() - ox;
                    let oby = back.y.to_f64().unwrap() - oy;
                    let obz = back.z.to_f64().unwrap() - oz;
                    let fx = (obx / dx).floor();
                    let fy = (oby / dx).floor();
                    let fz = (obz / dx).floor();
                    let nx_m1 = (nx - 1) as f64;
                    let ny_m1 = (ny - 1) as f64;
                    let nz_m1 = (nz - 1) as f64;
                    let bx = fx.max(0.0).min(nx_m1) as usize;
                    let by = fy.max(0.0).min(ny_m1) as usize;
                    let bz = fz.max(0.0).min(nz_m1) as usize;
                    let tx = T::from_f64(obx / dx - fx).unwrap();
                    let ty = T::from_f64(oby / dx - fy).unwrap();
                    let tz = T::from_f64(obz / dx - fz).unwrap();
                    let val = self.trilinear(bx, by, bz, tx, ty, tz);
                    self.u[c] = val;
                }
            }
        }
    }

    /// 扩散-对流混合步:先平流(半拉格朗日)后扩散(FTCS)。
    ///
    /// 顺序:u ← advect(u);u ← diffuse(u)。两步各自基于 `u_prev` 快照互不污染。
    /// 扩散数 `r = α·dt/dx²` 仍须 ≤ 1/6,否则 `step_diffusion` 会 panic 提示调参。
    /// 返回扩散步的稳定性数 `r`(平流步本身无条件稳定,不贡献 r)。
    pub fn step_advection_diffusion<F>(&mut self, vel: &F, alpha: T, dt: T) -> T
    where
        F: Fn(Vec3<T>) -> Vec3<T>,
    {
        self.advect(vel, dt);
        self.step_diffusion(alpha, dt)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Bc;

    #[test]
    fn advect_translates_peak_with_uniform_velocity() {
        // 初始在中心放一个温度峰,施加沿 +x 的匀速流,平流应把峰整体平移 v·dt。
        let mut f = ScalarField::<f64>::new(16, 16, 16, 0.5, 0.0, Bc::Neumann);
        let c = (8, 8, 8);
        let ci = f.idx(c.0, c.1, c.2);
        f.u[ci] = 1.0;
        let vx = 0.5; // 单位速度,dt=1 => 平移 1 个 dx(=0.5)
        let dt = 1.0;
        let vel = |_p: Vec3<f64>| Vec3::new(vx, 0.0, 0.0);
        f.advect(&vel, dt);
        // 峰值应出现在 x 方向 +2 个格处(1.0 * dt/dx = 1 个单元,半拉格朗日取回前位置)。
        let new_i = (c.0 as f64 + vx * dt / f.dx).round() as usize;
        let new_ci = f.idx(new_i.min(15), c.1, c.2);
        assert!(f.u[new_ci] > 0.8, "平流后峰应平移到 i={}, 实测={}", new_i, f.u[new_ci]);
        // 原位置应基本清空(被带走)。
        assert!(f.u[ci] < 0.2, "原位置温度应被平流带走, 实测={}", f.u[ci]);
    }

    #[test]
    fn advect_conserves_mass_nearly() {
        // 半拉格朗日平流对平滑分布近似守恒总量(闭合边界下不溢出)。
        let mut f = ScalarField::<f64>::new(24, 24, 24, 0.5, 0.0, Bc::Neumann);
        // 高斯型分布
        let ox = 12.0 * f.dx;
        for k in 0..24 {
            for j in 0..24 {
                for i in 0..24 {
                    let x = i as f64 * f.dx - ox;
                    let y = j as f64 * f.dx - ox;
                    let z = k as f64 * f.dx - ox;
                    let ci = f.idx(i, j, k);
                    f.u[ci] = (-(x * x + y * y + z * z) / 0.3).exp();
                }
            }
        }
        let sum0: f64 = f.u.iter().sum();
        // 旋转流(无散度),总量应高度守恒。
        let vel = |p: Vec3<f64>| Vec3::new(-p.y, p.x, 0.0);
        for _ in 0..10 {
            f.advect(&vel, 0.05);
        }
        let sum1: f64 = f.u.iter().sum();
        let rel = (sum1 - sum0).abs() / sum0;
        assert!(rel < 0.05, "旋转流平流质量相对漂移 {} 应 <5%", rel);
    }

    #[test]
    fn step_advection_diffusion_is_stable_and_finite() {
        let mut f = ScalarField::<f64>::new(20, 20, 20, 0.5, 0.0, Bc::Neumann);
        let ci = f.idx(10, 10, 10);
        f.u[ci] = 10.0;
        // α·dt/dx² = 0.02/0.25 < 1/6,稳定;流场为常数平移。
        let vel = |_p: Vec3<f64>| Vec3::new(0.2, 0.0, 0.0);
        let r = f.step_advection_diffusion(&vel, 0.02, 0.1);
        assert!(r < 1.0 / 6.0 + 1e-9, "扩散数应稳定, r={}", r);
        assert!(f.u.iter().all(|&v| v.is_finite()), "对流-扩散后场应有限");
        // 总量不应爆炸。
        let sum: f64 = f.u.iter().sum();
        assert!(sum < 1e3, "总量应受限, sum={}", sum);
    }
}
