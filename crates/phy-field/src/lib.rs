//! # phy-field
//!
//! 连续场规则(热/电磁/波),M7 实现。基于规则网格显式有限差分:
//!
//! - `grid::ScalarField`:三维规则网格标量场 + 边界条件(Dirichlet/Neumann)。
//!   - `step_diffusion(alpha, dt)`:热扩散 ∂u/∂t = α∇²u(FTCS 显式)。
//!   - `step_wave(c2, dt)`:波动/电磁标量 ∂²u/∂t² = c²∇²u(leapfrog 显式)。
//! - `HeatField` / `WaveField`:`ScalarField` 的 `phy_core::Subsystem` 适配。

mod grid;

pub use grid::{Bc, ScalarField};

use phy_core::Subsystem;
use phy_math::RealField;
use phy_math::Vec3;

/// 规则网格的几何参数(维度/原点/单元尺寸)。
///
/// 作为 `HeatFieldLike`/`EmFieldLike` 的 supertrait,使 `world_to_cell`、
/// `sample_world` 等采样辅助函数可统一复用,不依赖具体场类型。
pub trait GridGeometry<T: RealField + Copy> {
    /// 网格维度 (nx, ny, nz)。
    fn dims(&self) -> (usize, usize, usize);
    /// 网格原点(最小角)世界坐标。
    fn origin(&self) -> Vec3<T>;
    /// 单元尺寸 dx(立方网格)。
    fn cell_size(&self) -> T;
}

/// 可被流体/软体耦合访问的“温度场”抽象。
///
/// 由 `HeatField` 实现;`phy-fluid` 仅依赖此 trait(而非 `phy-field` 的具体类型)
/// 以避免循环依赖。暴露耦合所需的最小接口:网格几何、三线性温度采样、热源注入。
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
    /// 向网格单元 (cx,cy,cz) 累加一个热源项。用于对流换热(流动区域升温)。
    fn add_source(&mut self, cx: usize, cy: usize, cz: usize, q: T);
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
        // 累加到同格 u(隐式 Neumann 边界会保持守恒)。
        self.field.add_source(cx, cy, cz, q);
    }
}

/// 把世界坐标 `p` 转换为场网格最近格点索引 (cx,cy,cz) 与单元内 [0,1) 偏移 (tx,ty,tz)。
///
/// 超出网格时夹紧到边界格(视为最近格点采样,不报错),供刚体/软体/流体统一复用。
/// 对任一实现了 `GridGeometry` 的场(热场/电磁场)通用。
pub fn world_to_cell<T: RealField + Copy, G: GridGeometry<T> + ?Sized>(
    field: &G,
    p: Vec3<T>,
) -> (usize, usize, usize, f64, f64, f64)
where
    T: num_traits::ToPrimitive,
{
    let o = field.origin();
    let dx = field.cell_size();
    let (nx, ny, nz) = field.dims();
    let fx = ((p.x - o.x) / dx).to_f64().unwrap_or(0.0);
    let fy = ((p.y - o.y) / dx).to_f64().unwrap_or(0.0);
    let fz = ((p.z - o.z) / dx).to_f64().unwrap_or(0.0);
    let clamp_idx = |f: f64, n: usize| -> (usize, f64) {
        if n <= 1 {
            return (0, 0.0);
        }
        if f <= 0.0 {
            (0, 0.0)
        } else if f >= (n - 1) as f64 {
            (n - 2, 1.0)
        } else {
            let fl = f.floor();
            (fl as usize, f - fl)
        }
    };
    let (cx, tx) = clamp_idx(fx, nx);
    let (cy, ty) = clamp_idx(fy, ny);
    let (cz, tz) = clamp_idx(fz, nz);
    (cx, cy, cz, tx, ty, tz)
}

/// 在世界坐标 `p` 处三线性采样温度(越界夹紧到边界格)。供刚体/软体耦合复用。
pub fn sample_world<T: RealField + Copy>(
    heat: &dyn HeatFieldLike<T>,
    p: Vec3<T>,
) -> T
where
    T: num_traits::ToPrimitive,
{
    let (cx, cy, cz, tx, ty, tz) = world_to_cell(heat, p);
    heat.sample_trilinear(cx, cy, cz, tx, ty, tz)
}

/// 在世界坐标 `p` 处三线性采样电场矢量(越界夹紧到边界格)。供刚体耦合复用。
pub fn sample_e_field<T: RealField + Copy>(
    em: &dyn EmFieldLike<T>,
    p: Vec3<T>,
) -> Vec3<T>
where
    T: num_traits::ToPrimitive,
{
    let (cx, cy, cz, tx, ty, tz) = world_to_cell(em, p);
    em.sample_e_field(cx, cy, cz, tx, ty, tz)
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
    pub fn new(field: ScalarField<T>, alpha: T) -> Self {
        Self {
            field,
            alpha,
            last_r: T::zero(),
        }
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

/// 波动/电磁标量子系统。
pub struct WaveField<T: RealField + Copy> {
    /// 底层标量场(电势/位移)。
    pub field: ScalarField<T>,
    /// 波速平方 c²。
    pub c2: T,
}

impl<T: RealField + Copy> WaveField<T> {
    pub fn new(field: ScalarField<T>, c2: T) -> Self {
        Self { field, c2 }
    }
}

impl<T: RealField + Copy> Subsystem<T> for WaveField<T> {
    fn as_any(&self) -> &dyn std::any::Any {
        self
    }

    fn as_any_mut(&mut self) -> &mut dyn std::any::Any {
        self
    }

    fn step(&mut self, dt: &T) {
        self.field.step_wave(self.c2, *dt);
    }
    fn name(&self) -> &'static str {
        "wave"
    }
}

/// 可被刚体/软体耦合访问的“电磁场”抽象。
///
/// 由 `EmField` 实现;`phy-rigid` 仅依赖此 trait 以避免循环依赖。暴露耦合所需的最小
/// 接口:网格几何、三线性电场采样、电荷注入。电场由电荷密度经泊松松弛(`step`)求解。
pub trait EmFieldLike<T: RealField + Copy>: GridGeometry<T> + std::any::Any {
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
///
/// 模型:电荷密度 `rho`(标量网格)经 Jacobi 松弛求解泊松方程 ∇²φ = -ρ/ε,
/// 再由 `E = -∇φ` 中心差分得到电场矢量 `e`(每格一个 `Vec3`)。外加均匀磁场 `b_ext`
/// 用于洛伦兹力的 `v×B` 项(可选,默认零)。每步 `step` 重新由当前 `rho` 求解 `E`,
/// 因此电场随电荷分布实时演化,支持刚体↔电磁的双向耦合。
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
    fn idx(&self, x: usize, y: usize, z: usize) -> usize {
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
    fn as_any(&self) -> &dyn std::any::Any {
        self
    }

    fn as_any_mut(&mut self) -> &mut dyn std::any::Any {
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn diffusion_spreads_and_conserves_total() {
        // 1D 热扩散:中心脉冲随时间展宽;Neumann 边界下总热量近似守恒。
        let n = 81usize;
        let dx = 1.0_f64;
        let mut f = ScalarField::<f64>::new(n, 1, 1, dx, 0.0, Bc::Neumann);
        let c = n / 2;
        f.u[c] = 1.0;
        f.u_prev[c] = 1.0;
        let total0 = f.sum();
        let alpha = 0.1; // r = 0.1 < 1/6,稳定
        let dt = 1.0;
        for _ in 0..200 {
            f.step_diffusion(alpha, dt);
        }
        // 总热量基本守恒(Neumann 无通量泄漏)。
        let total1 = f.sum();
        assert!(
            (total1 - total0).abs() < 1e-6,
            "Neumann 下总热量应守恒: {} vs {}",
            total1,
            total0
        );
        // 峰值下降、分布变宽(中心不再是 1,但仍是局部最大)。
        assert!(f.sample(c, 0, 0) < 1.0, "中心脉冲应被摊平: {}", f.sample(c, 0, 0));
        // 对称性:中心两侧对称。
        let left = f.sample(c - 10, 0, 0);
        let right = f.sample(c + 10, 0, 0);
        assert!((left - right).abs() < 1e-12, "扩散应左右对称");
    }

    #[test]
    fn diffusion_matches_analytic_gaussian() {
        // 自由空间(大网格 Neumann)下,热核解形状 u(x)/u(0) = exp(-x²/(4αt))。
        // 用比值比较,规避网格离散化/质量归一因子的绝对误差。
        let n = 401usize;
        let dx = 0.5_f64;
        let mut f = ScalarField::<f64>::new(n, 1, 1, dx, 0.0, Bc::Neumann);
        let c = n / 2;
        f.u[c] = 1.0;
        f.u_prev[c] = 1.0;
        let alpha = 0.1; // r = 0.1·0.25/0.25 = 0.1 < 0.5,稳定
        let dt = 0.25;
        let tt = 40.0;
        let steps = (tt / dt) as usize;
        for _ in 0..steps {
            f.step_diffusion(alpha, dt);
        }
        let u0 = f.sample(c, 0, 0);
        let d = 20usize; // 偏移 10 网格
        let ud = f.sample(c + d, 0, 0);
        let x = (d as f64) * dx;
        let ratio_analytic = (-(x * x) / (4.0 * alpha * tt)).exp();
        let ratio_num = ud / u0;
        assert!(
            (ratio_num - ratio_analytic).abs() < 0.05,
            "热核形状比值应吻合: num={}, ana={}",
            ratio_num,
            ratio_analytic
        );
    }

    #[test]
    fn wave_preserves_shape_and_speed() {
        // 1D 波动:高斯脉冲以速度 c 平移;leapfrog 下应向右移动且保持有限。
        // 高分辨率减少色散,使数值波速接近 c。
        let n = 1601usize;
        let dx = 0.1_f64;
        let mut f = ScalarField::<f64>::new(n, 1, 1, dx, 0.0, Bc::Neumann);
        let c0 = n / 2;
        let width = 6.0_f64;
        let c = 2.0_f64;
        let dt = 0.02;
        let c2 = c * c;
        // 初始化为右行行波:u(x,0)=g(x), u(x,-dt)=g(x - c·dt),
        // 使脉冲以速度 c 整体右移而非分裂/色散。
        let g = |x: f64| (-(x * x) / (2.0 * width * width)).exp();
        for k in 0..n {
            let x = (k as f64 - c0 as f64) * dx;
            f.u[k] = g(x);
            f.u_prev[k] = g(x + c * dt); // 右行波:u(x,-dt)=g(x+c·dt)
        }
        // CFL: c·dt/dx = 0.4 < 1,稳定。
        let travel = 20.0; // 期望位移
        let steps = (travel / (c * dt)) as usize;
        for _ in 0..steps {
            f.step_wave(c2, dt);
        }
        // 峰值应向右移动(向右传播),且位移量为正、量级合理。
        let mut peak_idx = 0usize;
        let mut peak = -1.0_f64;
        for k in 0..n {
            if f.u[k] > peak {
                peak = f.u[k];
                peak_idx = k;
            }
        }
        let actual_travel = (peak_idx as f64 - c0 as f64) * dx;
        // 行波应传播约 c·t 的距离(数值波速≈c,允许色散/方向偏差)。
        assert!(
            actual_travel.abs() > travel * 0.5,
            "脉冲应传播约 {},实际位移 {}",
            travel,
            actual_travel
        );
        assert!(
            actual_travel.abs() < travel * 1.5,
            "数值波速不应偏离过多: 实际位移 {} 期望 {}",
            actual_travel,
            travel
        );
        // 峰值幅度不应爆炸(稳定)。
        assert!(peak.is_finite() && peak > 0.0, "波峰应有限且为正: {}", peak);
    }

    #[test]
    fn wave_is_lossless_amplitude() {
        // 无源自由波动:零初速度高斯包在足够大网格下幅度不应增长(无数值爆炸)。
        let n = 201usize;
        let dx = 1.0_f64;
        let mut f = ScalarField::<f64>::new(n, 1, 1, dx, 0.0, Bc::Neumann);
        let c0 = n / 2;
        let width = 6.0_f64;
        for k in 0..n {
            let x = (k as f64 - c0 as f64) * dx;
            f.u[k] = (-(x * x) / (2.0 * width * width)).exp();
            f.u_prev[k] = f.u[k];
        }
        let c2 = 1.0_f64;
        let dt = 0.1; // CFL 0.1<0.577
        let peak0 = f.max_abs();
        for _ in 0..500 {
            f.step_wave(c2, dt);
        }
        let peak1 = f.max_abs();
        assert!(
            peak1 <= peak0 * 1.5,
            "无源波动幅度不应增长>50%(无爆炸): {} vs {}",
            peak1,
            peak0
        );
    }

    #[test]
    fn subsystem_adapter_runs() {
        let f = ScalarField::<f64>::new(21, 1, 1, 1.0, 0.0, Bc::Neumann);
        let mut heat = HeatField::new(f, 0.1);
        heat.step(&1.0_f64);
        assert!(heat.last_r < 1.0 / 6.0 + 1e-9, "r 应在稳定区间");
        let f2 = ScalarField::<f64>::new(21, 1, 1, 1.0, 0.0, Bc::Neumann);
        let mut wave = WaveField::new(f2, 1.0);
        wave.step(&0.1_f64);
        // 步进后应无 NaN。
        assert!(wave.field.max_abs().is_finite());
    }

    #[test]
    fn em_poisson_yields_outward_e_from_positive_charge() {
        // 中心放正电荷 → 泊松松弛出电势 φ,电场 E=-∇φ 应从电荷向外(中心处 E≈0,
        // 右侧 E.x>0、左侧 E.x<0)。
        let n = 11usize;
        let dx = 1.0_f64;
        let mut rho = ScalarField::<f64>::new(n, 1, 1, dx, 0.0, Bc::Neumann);
        rho.u[n / 2] = 1.0;
        let mut em = EmField::build(rho, 1.0);
        em.step(&0.1_f64);
        let c = n / 2;
        let e_right = em.e[em.idx(c + 1, 0, 0)].x;
        let e_left = em.e[em.idx(c - 1, 0, 0)].x;
        assert!(e_right > 0.0, "正电荷右侧 E.x 应指向外(>0): {}", e_right);
        assert!(e_left < 0.0, "正电荷左侧 E.x 应指向外(<0): {}", e_left);
        assert!(em.e[em.idx(c, 0, 0)].x.abs() < 1e-9, "对称中心 E.x≈0");
    }

    #[test]
    fn em_sample_e_field_trilinear_finite() {
        let n = 5usize;
        let mut rho = ScalarField::<f64>::new(n, n, n, 1.0, 0.0, Bc::Neumann);
        let c = rho.idx(2, 2, 2);
        rho.u[c] = 1.0;
        let mut em = EmField::build(rho, 1.0);
        em.step(&0.1_f64);
        let e = em.sample_e_field(2, 2, 2, 0.5, 0.5, 0.5);
        assert!(e.x.is_finite() && e.y.is_finite() && e.z.is_finite());
    }

    #[test]
    fn em_add_charge_accumulates_into_rho() {
        let mut rho = ScalarField::<f64>::new(3, 3, 3, 1.0, 0.0, Bc::Neumann);
        let mut em = EmField::build(rho, 1.0);
        em.add_charge(1, 1, 1, 2.0);
        // add_charge 累加到 rho.src(源缓冲),由 step 注入 u。
        assert!((em.rho.src[em.rho.idx(1, 1, 1)] - 2.0).abs() < 1e-12);
    }
}
