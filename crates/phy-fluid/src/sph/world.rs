//! SPH 流体世界:求解器主体与刚体 / 热场 / 软体质点的双向耦合。

use std::fmt;

use nalgebra::Scalar;
use num_traits::ToPrimitive;
use phy_field::HeatFieldLike;
use phy_math::{RealField, Vec3};
use phy_rigid::shape::{Body, Shape};
use serde::de::DeserializeOwned;
use serde::{Deserialize, Serialize};

use crate::kernels::Kernels;
use crate::particle::Particle;

use super::couple::CouplePoint;
use super::grid::Grid;
use super::params::SphParams;

/// SPH 流体世界。
pub struct FluidWorld<T: RealField + Copy + ToPrimitive> {
    /// 求解参数。
    pub params: SphParams<T>,
    /// 所有粒子。
    pub particles: Vec<Particle<T>>,
    kernels: Kernels<T>,
    grid: Grid<T>,
}

/// 序列化辅助结构:仅存档公开状态(params + particles),核/网格缓存重建。
#[derive(Serialize, Deserialize)]
#[serde(bound = "T: RealField + Copy + Serialize + DeserializeOwned + Default + Scalar + ToPrimitive")]
struct FluidWorldData<T: RealField + Copy + ToPrimitive> {
    params: SphParams<T>,
    particles: Vec<Particle<T>>,
}

impl<T: RealField + Copy + ToPrimitive> Serialize for FluidWorld<T>
where
    T: Serialize + DeserializeOwned + Default + Scalar + ToPrimitive,
{
    fn serialize<S: serde::Serializer>(&self, s: S) -> Result<S::Ok, S::Error> {
        FluidWorldData {
            params: self.params.clone(),
            particles: self.particles.clone(),
        }
        .serialize(s)
    }
}

impl<'de, T: RealField + Copy + ToPrimitive> Deserialize<'de> for FluidWorld<T>
where
    T: Serialize + DeserializeOwned + Default + Scalar + ToPrimitive,
{
    fn deserialize<D: serde::Deserializer<'de>>(d: D) -> Result<Self, D::Error> {
        let data = FluidWorldData::<T>::deserialize(d)?;
        Ok(FluidWorld::new(data.params).with_particles(data.particles))
    }
}

impl<T: RealField + Copy + ToPrimitive + fmt::Debug> fmt::Debug for FluidWorld<T> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("FluidWorld")
            .field("params", &self.params)
            .field("particles", &self.particles)
            .finish()
    }
}

impl<T: RealField + Copy + ToPrimitive> Clone for FluidWorld<T> {
    fn clone(&self) -> Self {
        FluidWorld::new(self.params.clone()).with_particles(self.particles.clone())
    }
}

impl<T: RealField + Copy + ToPrimitive> FluidWorld<T> {
    /// 以给定参数创建空世界(含核与网格)。
    ///
    /// # 示例
    ///
    /// 造一个含 100+ 粒子的初始长方体流体,推进若干步,断言全程无 NaN:
    ///
    /// ```
    /// use phy_fluid::{FluidWorld, SphParams};
    /// use phy_math::Vec3 as V3;
    ///
    /// let mut w = FluidWorld::<f64>::new(SphParams::<f64>::defaults());
    /// w.fill_box(V3::new(-1.0, 1.0, -1.0), V3::new(1.0, 3.0, 1.0), 0.3, 0.1);
    /// for _ in 0..30 { w.step(0.01); }
    /// for p in w.particles.iter() {
    ///     assert!(p.pos.x.is_finite() && p.pos.y.is_finite() && p.pos.z.is_finite());
    /// }
    /// ```
    pub fn new(params: SphParams<T>) -> Self {
        let h = params.h;
        Self {
            params,
            particles: Vec::new(),
            kernels: Kernels::new(h),
            grid: Grid::new(h),
        }
    }

    /// 当前粒子数。
    pub fn len(&self) -> usize {
        self.particles.len()
    }

    /// 用已有粒子列表构造(反序列化后重建,跳过核/网格重新计算)。
    pub fn with_particles(mut self, particles: Vec<Particle<T>>) -> Self {
        self.particles = particles;
        self
    }

    /// 是否为空。
    pub fn is_empty(&self) -> bool {
        self.particles.is_empty()
    }

    /// 以规则网格填充一个长方体区域内(初速度为零),用于 dam-break 等场景。
    ///
    /// `spacing` 为粒子初始间距;`margin` 为距盒内壁的内缩。
    /// 为保证 SPH 静止密度收敛,此处按 `ρ0·spacing³` 反算单粒子质量(标准初始化)。
    pub fn fill_box(&mut self, min: Vec3<T>, max: Vec3<T>, spacing: T, margin: T) {
        let lo = min + Vec3::new(margin, margin, margin);
        let hi = max - Vec3::new(margin, margin, margin);
        let d = spacing;
        // 由当前 h 下的规则晶格核求和反算质量,使静止密度精确等于 ρ0。
        let mass = self.lattice_mass(d);
        let mut x = lo.x;
        while x <= hi.x {
            let mut y = lo.y;
            while y <= hi.y {
                let mut z = lo.z;
                while z <= hi.z {
                    self.particles
                        .push(Particle::new(Vec3::new(x, y, z), mass));
                    z += d;
                }
                y += d;
            }
            x += d;
        }
    }

    /// 由规则晶格(间距 `s`)在光滑长度 `h` 下的 Poly6 核求和,反算单粒子质量:
    /// `mass = ρ0 / Σ_{晶格点} W(r)`,使静止晶格密度精确为 ρ0。
    fn lattice_mass(&self, s: T) -> T {
        let h = self.params.h;
        let k = &self.kernels;
        let n_shell = ((h / s).to_f64().unwrap_or(1.0).ceil() as i32) + 1;
        let mut sum_w = T::zero();
        for i in -n_shell..=n_shell {
            for j in -n_shell..=n_shell {
                for l in -n_shell..=n_shell {
                    let rr = ((i * i + j * j + l * l) as f64).sqrt();
                    let r = s * T::from_f64(rr).unwrap();
                    if r < h {
                        sum_w += k.poly6(r);
                    }
                }
            }
        }
        if sum_w > T::zero() {
            self.params.rest_density / sum_w
        } else {
            self.params.mass
        }
    }

    /// 推入单个粒子。
    pub fn add_particle(&mut self, p: Particle<T>) {
        self.particles.push(p);
    }

    /// 查询粒子 `i` 上一 SPH 步生效的非牛顿有效粘度(用于自检/可视化;
    /// 需先调用 `step` 使 `compute_forces` 写入 `mu_eff`)。
    pub fn effective_viscosity(&self, i: usize) -> T {
        self.particles[i].mu_eff
    }

    /// 推进一个 SPH 时间步 `dt`(不含刚体耦合;耦合见 `couple_bodies`)。
    pub fn step(&mut self, dt: T) {
        if self.particles.is_empty() {
            return;
        }
        self.build_grid();
        self.compute_density_pressure();
        self.compute_forces();
        self.integrate(dt);
        self.enforce_bounds();
    }

    /// 重建邻居网格(每步调用;也供 W4 GPU 扁平化前显式调用)。
    pub fn build_grid(&mut self) {
        self.grid.build(&self.particles);
    }

    /// 密度与压力:ρ_i = Σ m_j W(i,j); p_i = max(0, k (ρ_i - ρ0))。
    ///
    /// S7 并行后端:每粒子只读全局 `pos`/`mass`(经网格邻居查询),把 `rho`/`p`
    /// 累加到独立并行缓冲,最后一次性写回。计算与遍历顺序无关,故 rayon 并行
    /// 结果与串行逐一对应、完全确定。
    fn compute_density_pressure(&mut self) {
        let h = self.params.h;
        let n = self.particles.len();
        if n == 0 {
            return;
        }
        let k = &self.kernels;
        let rest = self.params.rest_density;
        let stiff = self.params.stiffness;

        // 快照只读量(位置/质量),供并行闭包借用,避免写回/读取竞争。
        let pos: Vec<Vec3<T>> = self.particles.iter().map(|p| p.pos).collect();
        let mass: Vec<T> = self.particles.iter().map(|p| p.mass).collect();
        let grid = &self.grid;

        use rayon::prelude::*;
        let (rho_out, p_out): (Vec<T>, Vec<T>) = (0..n)
            .into_par_iter()
            .map(|i| {
                let p_i = pos[i];
                let mut rho = T::zero();
                grid.for_each_neighbor(&p_i, |j| {
                    let r = na_distance(&p_i, &pos[j]);
                    if r < h {
                        rho += mass[j] * k.poly6(r);
                    }
                });
                // 状态方程(理想气体型),压力非负以防聚团吸力。
                let over = rho - rest;
                let p = if over > T::zero() {
                    stiff * over
                } else {
                    T::zero()
                };
                (rho, p)
            })
            .unzip();

        for (i, part) in self.particles.iter_mut().enumerate() {
            part.rho = rho_out[i];
            part.p = p_out[i];
        }
    }

    /// 受力:对称压力力 + 粘性力(非牛顿幂律有效粘度) + 重力,汇聚成加速度。
    ///
    /// S7 并行后端:每粒子只读本步 `pos`/`vel`/`rho`/`p`/`mass`/`material`,把
    /// `acc`/`mu_eff` 累加到独立并行缓冲后写回。各粒子结果互不影响,并行安全。
    fn compute_forces(&mut self) {
        let n = self.particles.len();
        if n == 0 {
            return;
        }
        let k = &self.kernels;
        let h = self.params.h;
        let r_eps = T::from_f64(1e-4).unwrap();
        let visc_k = self.params.visc_k.clone();
        let visc_n = self.params.visc_n.clone();
        let shear_min = self.params.shear_min;

        // 快照只读量(位置/速度/密度/压力/质量/材质),供并行闭包借用。
        let pos: Vec<Vec3<T>> = self.particles.iter().map(|p| p.pos).collect();
        let vel: Vec<Vec3<T>> = self.particles.iter().map(|p| p.vel).collect();
        let rho: Vec<T> = self.particles.iter().map(|p| p.rho).collect();
        let pr: Vec<T> = self.particles.iter().map(|p| p.p).collect();
        let mass: Vec<T> = self.particles.iter().map(|p| p.mass).collect();
        let mat: Vec<usize> = self.particles.iter().map(|p| p.material).collect();
        let grid = &self.grid;

        use rayon::prelude::*;
        let results: Vec<(Vec3<T>, T, Vec3<T>)> = (0..n)
            .into_par_iter()
            .map(|i| {
                let p_i = pos[i];
                let v_i = vel[i];
                let rho_i = rho[i];
                let p_i_p = pr[i];
                let m_i = mass[i];

                let mut f_press = Vec3::zeros();
                // 粘性累加(尚未乘有效粘度 μ)
                let mut f_visc_raw = Vec3::zeros();
                // XSPH 速度修正累加: Σ (m_j/ρ_j)(v_j - v_i) W_poly6(r)
                let mut xsph = Vec3::zeros();
                // 局部应变率代理 = Σ |v_j - v_i| / (r+ε) · (m_j/ρ_j)
                let mut shear = T::zero();

                grid.for_each_neighbor(&p_i, |j| {
                    if j == i {
                        return;
                    }
                    let d = pos[j] - p_i;
                    let r = d.norm();
                    if r >= h || r <= T::zero() {
                        return;
                    }
                    let rho_j = rho[j];
                    let p_j = pr[j];
                    // 对称压力力: -m_i m_j (p_i/ρ_i² + p_j/ρ_j²) ∇W
                    let grad = k.spiky_grad_mag(r); // 含负系数
                    let dir = d / r; // 由 j 指向 i
                    let coef = m_i * mass[j] * (p_i_p / (rho_i * rho_i) + p_j / (rho_j * rho_j));
                    f_press += dir * (coef * grad);
                    // 粘性力(原始项,μ 在外层乘): μ m_i m_j (v_j - v_i)/ρ_j ∇²W
                    let lap = k.visc_lap(r);
                    f_visc_raw += (vel[j] - v_i) * (m_i * mass[j] / rho_j * lap);
                    // XSPH 速度修正(用 Poly6 核作权重,平滑粒子间相对速度)。
                    let w = k.poly6(r);
                    xsph += (vel[j] - v_i) * (mass[j] / rho_j * w);
                    // 应变率代理累加
                    let dv = (vel[j] - v_i).norm();
                    shear += dv / (r + r_eps) * (mass[j] / rho_j);
                });

                // 非牛顿幂律有效粘度:μ_eff = k·max(剪切率, ε)^(n-1)
                let m_idx = if mat[i] < visc_k.len() {
                    mat[i]
                } else {
                    0
                };
                let kc = visc_k[m_idx];
                let nn = visc_n[m_idx];
                let sreg = if shear > shear_min { shear } else { shear_min };
                let mu_eff = kc * sreg.powf(nn - T::one());

                let f_visc = f_visc_raw * mu_eff;

                // SPH 近邻加速度(压力+粘性),不含体力;重力/浮力在 integrate 经 body_acc 叠加。
                let acc = if rho_i > T::zero() {
                    (f_press + f_visc) / rho_i
                } else {
                    Vec3::zeros()
                };
                (acc, mu_eff, xsph)
            })
            .collect::<Vec<(Vec3<T>, T, Vec3<T>)>>();

        for (i, part) in self.particles.iter_mut().enumerate() {
            let (acc, mu_eff, xsph) = results[i];
            part.acc = acc;
            part.mu_eff = mu_eff;
            part.xsph = xsph;
        }
    }

    /// 半隐式欧拉:v += a dt; x += v dt。
    /// 总加速度 = SPH 近邻加速度 `acc` + 体力 `body_acc`(重力 + 浮力等耦合体力);
    /// 结算后立即清零 `body_acc`,使浮力每帧由 `couple` 重新写入(对齐 World 的 step→couple 约定)。
    fn integrate(&mut self, dt: T) {
        let g = self.params.gravity;
        let xsph_eps = self.params.xsph_eps;
        for pt in self.particles.iter_mut() {
            let a = pt.acc + g + pt.body_acc;
            pt.vel += a * dt;
            // XSPH 速度修正:平滑粒子间相对速度,抑制闭合系统动能无序增长。
            pt.vel += pt.xsph * xsph_eps;
            pt.pos += pt.vel * dt;
            pt.body_acc = Vec3::zeros();
        }
    }

    /// 盒边界:越界则夹回,并把该轴速度按阻尼反弹。
    fn enforce_bounds(&mut self) {
        let lo = self.params.bounds_min;
        let hi = self.params.bounds_max;
        let damp = self.params.boundary_damp;
        for pt in self.particles.iter_mut() {
            let mut p = pt.pos;
            let mut v = pt.vel;
            if p.x < lo.x {
                p.x = lo.x;
                v.x = -v.x * damp;
            } else if p.x > hi.x {
                p.x = hi.x;
                v.x = -v.x * damp;
            }
            if p.y < lo.y {
                p.y = lo.y;
                v.y = -v.y * damp;
            } else if p.y > hi.y {
                p.y = hi.y;
                v.y = -v.y * damp;
            }
            if p.z < lo.z {
                p.z = lo.z;
                v.z = -v.z * damp;
            } else if p.z > hi.z {
                p.z = hi.z;
                v.z = -v.z * damp;
            }
            pt.pos = p;
            pt.vel = v;
        }
    }

    /// 与刚体双向耦合:浮力 + 阻力 + 动量交换。
    ///
    /// 对每个位于刚体内部的粒子:
    /// 1. 位置推回表面(防穿透);
    /// 2. 速度吸附到刚体速度(切向保留 `friction` 比例),由此产生对刚体的反作用冲量;
    /// 3. 累计排开体积,按阿基米德浮力 `(ρ_f - ρ_b) V g` 对刚体施加上举力(反作用分配到内部粒子加速度)。
    ///
    /// `bodies` 中 `inv_mass = 0` 的静态体只当作不可穿透边界,不受浮力影响。
    pub fn couple_bodies(&mut self, bodies: &mut [Body<T>], dt: T, friction: T) {
        if self.particles.is_empty() || bodies.is_empty() {
            return;
        }
        let rho_f = self.params.rest_density;
        let g = self.params.gravity;
        let particle_vol = self.params.mass / rho_f; // 单粒子代表体积

        // 每刚体累计排开体积(用于浮力)。
        let mut displaced: Vec<T> = vec![T::zero(); bodies.len()];
        // 每刚体累计来自粒子的反作用冲量(线性)。
        let mut body_imp: Vec<Vec3<T>> = vec![Vec3::zeros(); bodies.len()];

        for pi in 0..self.particles.len() {
            let p_world = self.particles[pi].pos;
            let v_p = self.particles[pi].vel;
            for bi in 0..bodies.len() {
                let body = &bodies[bi];
                if body.inv_mass <= T::zero() {
                    // 静态体:仅做不可穿透边界。
                    let local = body.to_local(&p_world);
                    if body.shape.contains_local(&local) {
                        Self::push_out(&mut self.particles[pi], body, friction);
                    }
                    continue;
                }
                let local = body.to_local(&p_world);
                if body.shape.contains_local(&local) {
                    let v_b = body.vel;
                    // 反作用冲量 = 粒子动量变化(从旧速度到吸附速度),施加到刚体(反向)。
                    let v_new = v_b + (v_p - v_b) * friction; // 近似无滑边界
                    let dp = (v_new - v_p) * self.particles[pi].mass;
                    body_imp[bi] -= dp; // 刚体获得 -粒子动量变化
                    self.particles[pi].vel = v_new;
                    displaced[bi] += particle_vol;
                }
            }
        }

        // 对刚体施加流体作用力:完整阿基米德上举力 = -ρf·V_sub·g(向上);
        // 刚体自身重力(ρb·V·g)由刚体求解器另行施加,二者合成净力 (ρb-ρf)V g。
        for bi in 0..bodies.len() {
            let body = &mut bodies[bi];
            if body.inv_mass <= T::zero() {
                continue;
            }
            // 排开体积不得超过刚体自身体积(避免淹没粒子过量计数导致数值爆炸)。
            let vol_b = Self::body_volume(&body.shape);
            let v_sub = if vol_b > T::zero() {
                if displaced[bi] > vol_b {
                    vol_b
                } else {
                    displaced[bi]
                }
            } else {
                displaced[bi]
            };
            let buoy = -g * (rho_f * v_sub);
            let j_body = body_imp[bi] + buoy * dt; // 总冲量 = 反作用 + 浮力*dt
            body.apply_impulse(j_body);
        }
    }

    /// 估算刚体形状体积(球/盒闭式;凸多面体退化为 0,由调用方回退到静止密度)。
    fn body_volume(s: &Shape<T>) -> T {
        match s {
            Shape::Sphere { r } => {
                T::from_f64(4.0).unwrap() / T::from_f64(3.0).unwrap()
                    * T::from_f64(std::f64::consts::PI).unwrap()
                    * (*r)
                    * (*r)
                    * (*r)
            }
            Shape::Box { half } => {
                half.x * half.y * half.z * T::from_f64(8.0).unwrap()
            }
            Shape::Capsule { half_height, r } => {
                // 圆柱(π r² · 2·half) + 两半球(4/3 π r³)
                let pi = T::from_f64(std::f64::consts::PI).unwrap();
                pi * (*r) * (*r) * T::from_f64(2.0).unwrap() * (*half_height)
                    + T::from_f64(4.0).unwrap() / T::from_f64(3.0).unwrap()
                        * pi
                        * (*r)
                        * (*r)
                        * (*r)
            }
            Shape::Convex { .. } => T::zero(),
        }
    }

    /// 流体↔热场(温度场)双向耦合。
    ///
    /// 两个物理效应:
    /// 1. **热浮力(thermal buoyancy)**:流体密度随温度升高而降低
    ///    (ρ(T) = ρ0 / (1 + β·(T - T_ref))),于是浮力修正为
    ///    `f_b = -g · ρ(T) · vol`(比常温处更轻、上浮更强)。
    ///    温度 `T` 由热场在粒子位置三线性插值得到。
    /// 2. **对流热源(advective heating)**:流体粒子的动能耗散/速度被注入为热源,
    ///    即把每个粒子的速度幅值(运动强度)累加到热场同格 `src` 上,
    ///    使流动区域升温(对流换热近似)。`heat_gain` 控制注入强度。
    ///
    /// `heat` 为热场的可变引用(由调用方从 `World` 取出);`t_ref` 为参考温度
    /// (通常取热场初值,对应无浮力修正)。`dt` 仅用于对流热源分配(若 heat_gain=0 可忽略)。
    pub fn couple_heat(
        &mut self,
        heat: &mut dyn HeatFieldLike<T>,
        dt: T,
        t_ref: T,
        beta: T,
        heat_gain: T,
    ) where
        T: ToPrimitive,
    {
        if self.particles.is_empty() {
            return;
        }
        let g = self.params.gravity;
        let lo = self.params.bounds_min;
        let hi = self.params.bounds_max;
        let dims = heat.dims();
        let hmin = heat.origin();
        let dx = heat.cell_size();
        let nx_m1 = T::from_usize(dims.0.saturating_sub(1)).unwrap_or(T::zero());
        let ny_m1 = T::from_usize(dims.1.saturating_sub(1)).unwrap_or(T::zero());
        let nz_m1 = T::from_usize(dims.2.saturating_sub(1)).unwrap_or(T::zero());
        let to_idx = |x: T, y: T, z: T| -> Option<(usize, usize, usize, T, T, T)> {
            if x < hmin.x
                || x > hmin.x + dx * nx_m1
                || y < hmin.y
                || y > hmin.y + dx * ny_m1
                || z < hmin.z
                || z > hmin.z + dx * nz_m1
            {
                return None;
            }
            let fx = ((x - hmin.x) / dx).floor();
            let fy = ((y - hmin.y) / dx).floor();
            let fz = ((z - hmin.z) / dx).floor();
            let cx = fx.to_usize().unwrap_or(0);
            let cy = fy.to_usize().unwrap_or(0);
            let cz = fz.to_usize().unwrap_or(0);
            let tx = (x - hmin.x) / dx - fx;
            let ty = (y - hmin.y) / dx - fy;
            let tz = (z - hmin.z) / dx - fz;
            Some((cx, cy, cz, tx, ty, tz))
        };
        // 安装对流速度场采样器:把当前 SPH 粒子速度快照按最近邻插值给热场,
        // 使热场在下一步做扩散-对流求解(Boussinesq 闭环:热羽流被自身流速带走)。
        // 闭包捕获快照(可 'static),按网格查询点数线性扫描,规模可控。
        let samp_pos: Vec<(f64, f64, f64)> = self
            .particles
            .iter()
            .map(|p| {
                (
                    p.pos.x.to_f64().unwrap_or(0.0),
                    p.pos.y.to_f64().unwrap_or(0.0),
                    p.pos.z.to_f64().unwrap_or(0.0),
                )
            })
            .collect();
        let samp_vel: Vec<Vec3<T>> = self.particles.iter().map(|p| p.vel).collect();
        let vel_fn: Box<dyn Fn(Vec3<T>) -> Vec3<T>> = Box::new(move |q: Vec3<T>| {
            let qx = q.x.to_f64().unwrap_or(0.0);
            let qy = q.y.to_f64().unwrap_or(0.0);
            let qz = q.z.to_f64().unwrap_or(0.0);
            let mut bi = 0usize;
            let mut bd = f64::INFINITY;
            for (i, s) in samp_pos.iter().enumerate() {
                let dxq = qx - s.0;
                let dyq = qy - s.1;
                let dzq = qz - s.2;
                let d = dxq * dxq + dyq * dyq + dzq * dzq;
                if d < bd {
                    bd = d;
                    bi = i;
                }
            }
            if samp_vel.is_empty() {
                Vec3::new(T::zero(), T::zero(), T::zero())
            } else {
                samp_vel[bi]
            }
        });
        heat.set_vel_sampler(Some(vel_fn));
        for pt in self.particles.iter_mut() {
            // 仅对流体盒内的粒子做热浮力(盒外无温度场)。
            if pt.pos.x < lo.x
                || pt.pos.x > hi.x
                || pt.pos.y < lo.y
                || pt.pos.y > hi.y
                || pt.pos.z < lo.z
                || pt.pos.z > hi.z
            {
                continue;
            }
            // 三线性插值温度。
            let temp = if let Some((cx, cy, cz, tx, ty, tz)) = to_idx(pt.pos.x, pt.pos.y, pt.pos.z)
            {
                heat.sample_trilinear(
                    cx,
                    cy,
                    cz,
                    tx.to_f64().unwrap_or(0.0),
                    ty.to_f64().unwrap_or(0.0),
                    tz.to_f64().unwrap_or(0.0),
                )
            } else {
                t_ref
            };
            // Boussinesq 浮力(单位质量体力):
            // 暖流体(ρ_t<ρ_f)应上浮。对单个流体微元,重力作用于其真实质量
            // (ρ_t·g,向下),而排开环境流体(密度 ρ_f)获得全浮力 ρ_f·g(向上),
            // 合力 ÷ 微元质量 ρ_t ⇒ 加速度 = -g·(ρ_f/ρ_t - 1)。
            // 代入 ρ_t = ρ0/(1+β·ΔT),ρ_f/ρ_t = 1+β·ΔT,得 加速度 = -g·β·ΔT。
            // 当 ΔT>0(暖)时该修正沿 -g 的反方向(向上),使暖羽流真正上举;
            // 这与 step() 中施加的均匀重力 g 叠加后,暖粒子净加速度向上。
            let buoy_corr = -g * beta * (temp - t_ref);
            pt.body_acc += buoy_corr;
            // 对流热源:把速度幅值注入热场(运动区域升温)。
            if heat_gain > T::zero() {
                let speed = pt.vel.norm();
                if let Some((cx, cy, cz, _tx, _ty, _tz)) = to_idx(pt.pos.x, pt.pos.y, pt.pos.z) {
                    heat.add_source(cx, cy, cz, heat_gain * speed * dt);
                }
            }
        }
    }

    /// 软体质点(或任意点质量)与流体的双向耦合:浮力 + 阻力 + 动量交换。
    ///
    /// 对每个位于流体盒内的点:
    /// 1. 用均匀网格采样邻域平均流体速度 `v_f`;
    /// 2. 浮力 `f_b = -ρf·vol·g`(阿基米德,`vol = mass / soft_density`);
    /// 3. 线性阻力 `f_d = drag·mass·(v_f - v)`(趋向局部流速);
    /// 合力写入 `pt.force`。同时把等大反向冲量 `-(f_b+f_d)·dt` 分配到邻域流体粒子
    /// (按质量比例),实现 soft→fluid 的动量交换(流体被软体推开)。
    ///
    /// 点位于盒外时不耦合(`force` 保持零)。`soft_density` 为软体质点的等效密度,
    /// 用于把质量换算成排开体积;取 0 时退化为无浮力(仅阻力)。
    pub fn couple_points(&mut self, pts: &mut [CouplePoint<T>], dt: T, drag: T, soft_density: T) {
        if pts.is_empty() || self.particles.is_empty() {
            return;
        }
        let rho_f = self.params.rest_density;
        let g = self.params.gravity;
        let lo = self.params.bounds_min;
        let hi = self.params.bounds_max;
        // 重建邻居网格(耦合在 step 之后调用,粒子位置已变,需刷新)。
        self.grid.build(&self.particles);

        for pt in pts.iter_mut() {
            // 仅在流体盒内耦合。
            if pt.pos.x < lo.x
                || pt.pos.x > hi.x
                || pt.pos.y < lo.y
                || pt.pos.y > hi.y
                || pt.pos.z < lo.z
                || pt.pos.z > hi.z
            {
                continue;
            }
            // 采样邻域平均流体速度(含自身单元 3x3x3)。
            let mut v_sum = Vec3::zeros();
            let mut w_sum = T::zero();
            self.grid.for_each_neighbor(&pt.pos, |j| {
                let w = self.particles[j].mass;
                v_sum += self.particles[j].vel * w;
                w_sum += w;
            });
            let v_f = if w_sum > T::zero() {
                v_sum / w_sum
            } else {
                Vec3::zeros()
            };
            // 浮力:排开体积 = 质量 / 软体密度。
            let vol = if soft_density > T::zero() {
                pt.mass / soft_density
            } else {
                T::zero()
            };
            let f_b = -g * (rho_f * vol);
            // 阻力:趋向局部流速。
            let f_d = (v_f - pt.vel) * (drag * pt.mass);
            let f = f_b + f_d;
            pt.force = f;
            // soft→fluid 反向动量:把 -f·dt 按质量比例分配到邻域流体粒子。
            let mut m_sum = T::zero();
            let mut idxs: Vec<usize> = Vec::new();
            self.grid.for_each_neighbor(&pt.pos, |j| {
                m_sum += self.particles[j].mass;
                idxs.push(j);
            });
            if m_sum > T::zero() {
                let reaction = -f * dt;
                for &j in idxs.iter() {
                    let frac = self.particles[j].mass / m_sum;
                    self.particles[j].vel += reaction * frac;
                }
            }
        }
    }

    /// 把穿透粒子的位置沿法线推回刚体表面,并阻尼其法向相对速度。
    fn push_out(pt: &mut Particle<T>, body: &Body<T>, _friction: T) {
        let local = body.to_local(&pt.pos);
        // 用"指向表面点的方向"推回;若恰好在中心则取 +Y。
        let dir = if local.norm() > T::from_f64(1e-9).unwrap() {
            local.normalize()
        } else {
            Vec3::new(T::zero(), T::one(), T::zero())
        };
        let support_dir = body.shape.support_local(&dir);
        let target = body.to_world(&support_dir);
        pt.pos = target;
        // 去除进入刚体的法向速度分量(相对刚体速度)。
        let v_rel = pt.vel - body.vel;
        let n = (body.to_world(&(support_dir.normalize())) - body.pos).normalize();
        let vn = v_rel.dot(&n);
        if vn < T::zero() {
            pt.vel -= n * vn; // 去掉穿透分量
        }
    }
}

/// 距离(避免与 kernels 的泛型 dist 重名)。
pub(crate) fn na_distance<T: RealField + Copy>(a: &Vec3<T>, b: &Vec3<T>) -> T {
    (a - b).norm()
}

// W4: 导出 GPU 扁平数据(同模块可访问私有字段 grid/params/particles)。
impl FluidWorld<f32> {
    /// 把当前粒子 + 网格状态导出为 GPU 友好扁平数据(W4 前置)。
    ///
    /// 要求 `grid` 已 `build`(即 `step` 已调用过,或外部显式 `build_grid`)。
    /// GPU 端(Web Demo)用 `SphFlatData` 把字段上传到 WebGPU buffer 并跑 wgsl 内核
    /// (复刻 `compute_density_pressure` + `compute_forces`)。CPU 路径完全不变。
    pub fn to_gpu_flat(&self) -> super::gpu_flat::SphFlatData {
        let n = self.particles.len();
        let mut pos = Vec::with_capacity(n);
        let mut vel = Vec::with_capacity(n);
        let mut scalar = Vec::with_capacity(n);
        for p in &self.particles {
            pos.push([p.pos.x, p.pos.y, p.pos.z, 0.0]);
            vel.push([p.vel.x, p.vel.y, p.vel.z, 0.0]);
            // scalar = (rho, p, mass, material)
            scalar.push([p.rho, p.p, p.mass, p.material as f32]);
        }
        let flat: super::grid::FlatGrid<f32> = self.grid.to_flat();
        super::gpu_flat::SphFlatData {
            n,
            pos,
            vel,
            scalar,
            cell_start: flat.cell_start,
            sorted: flat.sorted,
            grid_min: [flat.min_i, flat.min_j, flat.min_k],
            nc: [flat.ncx, flat.ncy, flat.ncz],
            h: self.params.h,
            rest_density: self.params.rest_density,
            stiffness: self.params.stiffness,
            visc_k: self.params.visc_k.clone(),
            visc_n: self.params.visc_n.clone(),
            shear_min: self.params.shear_min,
            gravity: [
                self.params.gravity.x,
                self.params.gravity.y,
                self.params.gravity.z,
            ],
        }
    }
}

// 运行时切换:f64 世界导出 GPU 扁平数据(Web Demo 把 f64 世界副本送 GPU compute 写回 World<f64>)。
impl FluidWorld<f64> {
    /// f64 世界导出 GPU 友好扁平数据(等价为 f32 版,字段经 `as f32` 缩小精度)。
    ///
    /// 调用前需 `build_grid`(或先 `step` 过),否则邻居网格为空、GPU 内核找不到邻居。
    pub fn to_gpu_flat(&self) -> super::gpu_flat::SphFlatData {
        let f = |x: f64| -> f32 { x as f32 };
        let n = self.particles.len();
        let mut pos = Vec::with_capacity(n);
        let mut vel = Vec::with_capacity(n);
        let mut scalar = Vec::with_capacity(n);
        for p in &self.particles {
            pos.push([f(p.pos.x), f(p.pos.y), f(p.pos.z), 0.0]);
            vel.push([f(p.vel.x), f(p.vel.y), f(p.vel.z), 0.0]);
            scalar.push([f(p.rho), f(p.p), f(p.mass), p.material as f32]);
        }
        let flat: super::grid::FlatGrid<f64> = self.grid.to_flat();
        super::gpu_flat::SphFlatData {
            n,
            pos,
            vel,
            scalar,
            cell_start: flat.cell_start,
            sorted: flat.sorted,
            grid_min: [flat.min_i, flat.min_j, flat.min_k],
            nc: [flat.ncx, flat.ncy, flat.ncz],
            h: f(self.params.h),
            rest_density: f(self.params.rest_density),
            stiffness: f(self.params.stiffness),
            visc_k: self.params.visc_k.iter().map(|x| *x as f32).collect(),
            visc_n: self.params.visc_n.iter().map(|x| *x as f32).collect(),
            shear_min: f(self.params.shear_min),
            gravity: [f(self.params.gravity.x), f(self.params.gravity.y), f(self.params.gravity.z)],
        }
    }
}
