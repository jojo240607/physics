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
use phy_math::RealField;

/// 边界条件。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Bc {
    /// 边界固定为 `bc_val`(狄利克雷)。
    Dirichlet,
    /// 边界零通量(诺依曼,自然反射)。
    Neumann,
}

/// 规则网格标量场求解器。
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
}

impl<T: RealField + Copy> ScalarField<T> {
    /// 构造(nx×ny×nz),全部初始化为 `val`。
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
        }
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
