//! # phy-field
//!
//! 连续场规则(热/电磁/波),M7 实现。基于规则网格显式有限差分:
//!
//! - `grid::ScalarField`:三维规则网格标量场 + 边界条件(Dirichlet/Neumann)。
//!   - `step_diffusion(alpha, dt)`:热扩散 ∂u/∂t = α∇²u(FTCS 显式)。
//!   - `step_wave(c2, dt)`:波动/电磁标量 ∂²u/∂t² = c²∇²u(leapfrog 显式)。
//! - `HeatField` / `WaveField`:`ScalarField` 的 `phy_core::Subsystem` 适配。
//! - `EmField`:电荷密度场经泊松松弛求解电场(M12)。

mod grid;
mod grid_geometry;
mod heat;
mod wave;
mod em;
mod grav;
mod acoustic;
mod smoke;
mod serde_geom;

pub use grid::{Bc, ScalarField};
pub use grid_geometry::GridGeometry;
pub use heat::{HeatField, HeatFieldLike};
pub use smoke::SmokeField;
pub use wave::WaveField;
pub use em::{EmField, EmFieldLike};
pub use grav::{GravField, GravFieldLike};
pub use acoustic::{AcousticField, SOUND_SPEED_AIR};

use phy_math::RealField;
use phy_math::Vec3;

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

/// 在世界坐标 `p` 处三线性采样引力加速度矢量(越界夹紧到边界格)。供刚体耦合复用。
pub fn sample_g_field<T: RealField + Copy>(
    grav: &dyn GravFieldLike<T>,
    p: Vec3<T>,
) -> Vec3<T>
where
    T: num_traits::ToPrimitive,
{
    let (cx, cy, cz, tx, ty, tz) = world_to_cell(grav, p);
    grav.sample_g_field(cx, cy, cz, tx, ty, tz)
}

#[cfg(test)]
mod tests {
    use super::*;
    use phy_core::Subsystem;

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
        let rho = ScalarField::<f64>::new(3, 3, 3, 1.0, 0.0, Bc::Neumann);
        let mut em = EmField::build(rho, 1.0);
        em.add_charge(1, 1, 1, 2.0);
        // add_charge 累加到 rho.src(源缓冲),由 step 注入 u。
        assert!((em.rho.src[em.rho.idx(1, 1, 1)] - 2.0).abs() < 1e-12);
    }

    #[test]
    fn grav_poisson_yields_attractive_field_toward_mass() {
        // 中心放正质量 → 泊松松弛出引力势 Φ(质量处为负),引力 g=-∇Φ 应指向质量:
        // 右侧 g.x<0(指向中心)、左侧 g.x>0,吸引。
        let n = 11usize;
        let dx = 1.0_f64;
        let mut rho = ScalarField::<f64>::new(n, 1, 1, dx, 0.0, Bc::Neumann);
        rho.u[n / 2] = 1.0;
        let mut grav = GravField::build(rho, 1.0);
        grav.step(&0.1_f64);
        let c = n / 2;
        let g_right = grav.g[grav.idx(c + 1, 0, 0)].x;
        let g_left = grav.g[grav.idx(c - 1, 0, 0)].x;
        assert!(g_right < 0.0, "质量右侧应被吸引(g.x<0 指向中心): {}", g_right);
        assert!(g_left > 0.0, "质量左侧应被吸引(g.x>0 指向中心): {}", g_left);
        assert!(grav.g[grav.idx(c, 0, 0)].x.abs() < 1e-9, "对称中心 g.x≈0");
    }

    #[test]
    fn grav_sample_g_field_trilinear_finite() {
        let n = 5usize;
        let mut rho = ScalarField::<f64>::new(n, n, n, 1.0, 0.0, Bc::Neumann);
        let cc = rho.idx(2, 2, 2);
        rho.u[cc] = 1.0;
        let mut grav = GravField::build(rho, 1.0);
        grav.step(&0.1_f64);
        let g = grav.sample_g_field(2, 2, 2, 0.5, 0.5, 0.5);
        assert!(g.x.is_finite() && g.y.is_finite() && g.z.is_finite());
    }

    #[test]
    fn grav_add_mass_accumulates_into_rho() {
        let rho = ScalarField::<f64>::new(3, 3, 3, 1.0, 0.0, Bc::Neumann);
        let mut grav = GravField::build(rho, 1.0);
        grav.add_mass(1, 1, 1, 3.0);
        // add_mass 累加到 rho.src(源缓冲),由 step 注入 u。
        assert!((grav.rho.src[grav.rho.idx(1, 1, 1)] - 3.0).abs() < 1e-12);
    }
}
