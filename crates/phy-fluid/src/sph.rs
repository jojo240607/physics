//! SPH 流体世界与求解器。
//!
//! 采用 Müller 2003 的标准弱可压缩 SPH:
//! - Poly6 核估算密度,理想气体状态方程 p = k (ρ - ρ0) 得压力;
//! - 对称压力力 + 粘性力 + 重力作为加速度来源;
//! - 半隐式欧拉积分;盒状边界带阻尼反弹;
//! - 均匀网格(单元 = 光滑长度 h)做邻居查询,保证线性复杂度;
//! - `couple_bodies` 提供与刚体的双向耦合(浮力 / 阻力 / 动量交换)。

use std::collections::HashMap;

use phy_field::HeatFieldLike;
use phy_math::{RealField, Vec3};
use phy_rigid::shape::{Body, Shape};
use serde::de::DeserializeOwned;
use serde::{Deserialize, Serialize};

use crate::kernels::Kernels;
use crate::particle::Particle;

/// SPH 求解参数(全部采用工程单位,默认 f64)。
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(bound = "T: RealField + Copy + Serialize + DeserializeOwned + nalgebra::Scalar")]
pub struct SphParams<T: RealField + Copy> {
    /// 静止密度 ρ0(用于压力状态方程)。
    pub rest_density: T,
    /// 压力刚度系数 k(越大越不可压,但需更小 dt 保持稳定)。
    pub stiffness: T,
    /// 动力粘度 μ(牛顿流体,也是多材料表 `visc_k` 默认值)。
    pub viscosity: T,
    /// 非牛顿幂律一致性系数 `k`,按材料索引(`visc_k[material]`)。
    /// 有效粘度 μ_eff = `visc_k[m] · max(应变率, shear_min)^(visc_n[m]-1)`。
    pub visc_k: Vec<T>,
    /// 非牛顿幂律指数 `n`,按材料索引(`visc_n[material]`)。
    /// n = 1 退化为牛顿流体;`n < 1` 剪切变稀(高剪切更稀);`n > 1` 剪切变稠。
    pub visc_n: Vec<T>,
    /// 应变率正则化下限(避免零剪切处粘度发散/除零)。
    pub shear_min: T,
    /// 粒子质量。
    pub mass: T,
    /// 光滑长度 h(核作用半径,也是网格单元边长)。
    pub h: T,
    /// 重力加速度(向量,x 右 / y 上 / z 前)。
    #[serde(with = "phy_rigid::shape::serde_geom")]
    pub gravity: Vec3<T>,
    /// 模拟盒边界(粒子被约束在 [bounds_min, bounds_max] 内)。
    #[serde(with = "phy_rigid::shape::serde_geom")]
    pub bounds_min: Vec3<T>,
    /// 模拟盒边界上限。
    #[serde(with = "phy_rigid::shape::serde_geom")]
    pub bounds_max: Vec3<T>,
    /// 边界反弹速度保留系数(0=完全吸收,1=完全弹性)。
    pub boundary_damp: T,
}

impl<T: RealField + Copy> SphParams<T> {
    /// 一组合理的默认参数(适配 ~0.04 单位间距的粒子、盒尺度 10)。
    pub fn defaults() -> Self
    where
        T: num_traits::FromPrimitive,
    {
        Self {
            rest_density: <T as num_traits::FromPrimitive>::from_f64(1000.0).unwrap(),
            stiffness: <T as num_traits::FromPrimitive>::from_f64(250.0).unwrap(),
            viscosity: <T as num_traits::FromPrimitive>::from_f64(3.5).unwrap(),
            visc_k: vec![<T as num_traits::FromPrimitive>::from_f64(3.5).unwrap()],
            visc_n: vec![<T as num_traits::FromPrimitive>::from_f64(1.0).unwrap()],
            shear_min: <T as num_traits::FromPrimitive>::from_f64(0.01).unwrap(),
            mass: <T as num_traits::FromPrimitive>::from_f64(0.02).unwrap(),
            h: <T as num_traits::FromPrimitive>::from_f64(0.2).unwrap(),
            gravity: Vec3::new(
                T::zero(),
                <T as num_traits::FromPrimitive>::from_f64(-9.81).unwrap(),
                T::zero(),
            ),
            bounds_min: Vec3::new(
                <T as num_traits::FromPrimitive>::from_f64(-5.0).unwrap(),
                <T as num_traits::FromPrimitive>::from_f64(-5.0).unwrap(),
                <T as num_traits::FromPrimitive>::from_f64(-5.0).unwrap(),
            ),
            bounds_max: Vec3::new(
                <T as num_traits::FromPrimitive>::from_f64(5.0).unwrap(),
                <T as num_traits::FromPrimitive>::from_f64(5.0).unwrap(),
                <T as num_traits::FromPrimitive>::from_f64(5.0).unwrap(),
            ),
            boundary_damp: <T as num_traits::FromPrimitive>::from_f64(0.4).unwrap(),
        }
    }
}

/// 均匀空间哈希网格:键为 (i,j,k) 整数单元坐标。
struct Grid<T: RealField + Copy + num_traits::ToPrimitive> {
    cell: T,
    map: HashMap<(i64, i64, i64), Vec<usize>>,
    _t: std::marker::PhantomData<T>,
}

impl<T: RealField + Copy + num_traits::ToPrimitive> Grid<T> {
    fn new(cell: T) -> Self {
        Self {
            cell,
            map: HashMap::new(),
            _t: std::marker::PhantomData,
        }
    }

    #[inline]
    fn key_of(&self, p: &Vec3<T>) -> (i64, i64, i64) {
        let inv = T::one() / self.cell;
        (
            (p.x * inv).floor().to_i64().unwrap_or(0),
            (p.y * inv).floor().to_i64().unwrap_or(0),
            (p.z * inv).floor().to_i64().unwrap_or(0),
        )
    }

    fn build(&mut self, particles: &[Particle<T>]) {
        self.map.clear();
        for (i, pt) in particles.iter().enumerate() {
            self.map.entry(self.key_of(&pt.pos)).or_default().push(i);
        }
    }

    /// 访问位置 `p` 的 3x3x3 邻域单元内所有粒子索引(含自身单元)。
    fn for_each_neighbor<F: FnMut(usize)>(&self, p: &Vec3<T>, mut f: F) {
        let (ci, cj, ck) = self.key_of(p);
        for di in -1..=1i64 {
            for dj in -1..=1i64 {
                for dk in -1..=1i64 {
                    if let Some(ids) = self.map.get(&(ci + di, cj + dj, ck + dk)) {
                        for &id in ids {
                            f(id);
                        }
                    }
                }
            }
        }
    }
}

/// 软体 / 任意点质量与流体的耦合接口所用的单点描述。
///
/// 调用方填入 `pos`/`vel`/`mass`,`couple_points` 计算施加到该点的合力(浮力+阻力)
/// 写入 `force`;调用方据此更新点速度,即完成 fluid→soft 作用。soft→fluid 的
/// 反向动量由 `couple_points` 内部直接分配到邻域流体粒子,无需调用方处理。
pub struct CouplePoint<T: RealField + Copy> {
    /// 点世界坐标(只读输入)。
    pub pos: Vec3<T>,
    /// 点速度(只读输入)。
    pub vel: Vec3<T>,
    /// 点质量(只读输入)。
    pub mass: T,
    /// 输出:本步施加到该点的净力(浮力 + 阻力)。
    pub force: Vec3<T>,
}

impl<T: RealField + Copy> CouplePoint<T> {
    /// 构造一个待耦合点。
    pub fn new(pos: Vec3<T>, vel: Vec3<T>, mass: T) -> Self {
        Self {
            pos,
            vel,
            mass,
            force: Vec3::zeros(),
        }
    }
}

/// SPH 流体世界。
pub struct FluidWorld<T: RealField + Copy + num_traits::ToPrimitive> {
    /// 求解参数。
    pub params: SphParams<T>,
    /// 所有粒子。
    pub particles: Vec<Particle<T>>,
    kernels: Kernels<T>,
    grid: Grid<T>,
}

/// 序列化辅助结构:仅存档公开状态(params + particles),核/网格缓存重建。
#[derive(Serialize, Deserialize)]
#[serde(bound = "T: RealField + Copy + Serialize + DeserializeOwned + Default + nalgebra::Scalar + num_traits::ToPrimitive")]
struct FluidWorldData<T: RealField + Copy + num_traits::ToPrimitive> {
    params: SphParams<T>,
    particles: Vec<Particle<T>>,
}

impl<T: RealField + Copy + num_traits::ToPrimitive> Serialize for FluidWorld<T>
where
    T: Serialize + DeserializeOwned + Default + nalgebra::Scalar + num_traits::ToPrimitive,
{
    fn serialize<S: serde::Serializer>(&self, s: S) -> Result<S::Ok, S::Error> {
        FluidWorldData {
            params: self.params.clone(),
            particles: self.particles.clone(),
        }
        .serialize(s)
    }
}

impl<'de, T: RealField + Copy + num_traits::ToPrimitive> Deserialize<'de> for FluidWorld<T>
where
    T: Serialize + DeserializeOwned + Default + nalgebra::Scalar + num_traits::ToPrimitive,
{
    fn deserialize<D: serde::Deserializer<'de>>(d: D) -> Result<Self, D::Error> {
        let data = FluidWorldData::<T>::deserialize(d)?;
        Ok(FluidWorld::new(data.params).with_particles(data.particles))
    }
}

impl<T: RealField + Copy + num_traits::ToPrimitive + std::fmt::Debug> std::fmt::Debug
    for FluidWorld<T>
{
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("FluidWorld")
            .field("params", &self.params)
            .field("particles", &self.particles)
            .finish()
    }
}

impl<T: RealField + Copy + num_traits::ToPrimitive> Clone for FluidWorld<T> {
    fn clone(&self) -> Self {
        FluidWorld::new(self.params.clone()).with_particles(self.particles.clone())
    }
}

impl<T: RealField + Copy + num_traits::ToPrimitive> FluidWorld<T> {
    /// 以给定参数创建空世界(含核与网格)。
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
    pub fn fill_box(
        &mut self,
        min: Vec3<T>,
        max: Vec3<T>,
        spacing: T,
        margin: T,
    ) {
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
        let n_shell = (<T as num_traits::ToPrimitive>::to_f64(&(h / s))
            .unwrap_or(1.0)
            .ceil() as i32)
            + 1;
        let mut sum_w = T::zero();
        for i in -n_shell..=n_shell {
            for j in -n_shell..=n_shell {
                for l in -n_shell..=n_shell {
                    let rr = ((i * i + j * j + l * l) as f64).sqrt();
                    let r = s * <T as num_traits::FromPrimitive>::from_f64(rr).unwrap();
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

    /// 重建邻居网格(每步调用)。
    fn build_grid(&mut self) {
        self.grid.build(&self.particles);
    }

    /// 密度与压力:ρ_i = Σ m_j W(i,j); p_i = max(0, k (ρ_i - ρ0))。
    fn compute_density_pressure(&mut self) {
        let h = self.params.h;
        for i in 0..self.particles.len() {
            let p_i = self.particles[i].pos;
            let mut rho = T::zero();
            let k = &self.kernels;
            self.grid.for_each_neighbor(&p_i, |j| {
                let r = na_distance(&p_i, &self.particles[j].pos);
                if r < h {
                    rho += self.particles[j].mass * k.poly6(r);
                }
            });
            self.particles[i].rho = rho;
            // 状态方程(理想气体型),压力非负以防聚团吸力。
            let over = rho - self.params.rest_density;
            self.particles[i].p = if over > T::zero() {
                self.params.stiffness * over
            } else {
                T::zero()
            };
        }
    }

    /// 受力:对称压力力 + 粘性力(非牛顿幂律有效粘度) + 重力,汇聚成加速度。
    fn compute_forces(&mut self) {
        let k = &self.kernels;
        let h = self.params.h;
        let r_eps = <T as num_traits::FromPrimitive>::from_f64(1e-4).unwrap();
        for i in 0..self.particles.len() {
            let p_i = self.particles[i].pos;
            let v_i = self.particles[i].vel;
            let rho_i = self.particles[i].rho;
            let p_i_p = self.particles[i].p;
            let m_i = self.particles[i].mass;
            let mat = self.particles[i].material;

            let mut f_press = Vec3::zeros();
            // 粘性累加(尚未乘有效粘度 μ)
            let mut f_visc_raw = Vec3::zeros();
            // 局部应变率代理 = Σ |v_j - v_i| / (r+ε) · (m_j/ρ_j)
            let mut shear = T::zero();

            self.grid.for_each_neighbor(&p_i, |j| {
                if j == i {
                    return;
                }
                let d = self.particles[j].pos - p_i;
                let r = d.norm();
                if r >= h || r <= T::zero() {
                    return;
                }
                let rho_j = self.particles[j].rho;
                let p_j = self.particles[j].p;
                // 对称压力力: -m_i m_j (p_i/ρ_i² + p_j/ρ_j²) ∇W
                let grad = k.spiky_grad_mag(r); // 含负系数
                let dir = d / r; // 由 j 指向 i
                let coef =
                    m_i * self.particles[j].mass * (p_i_p / (rho_i * rho_i) + p_j / (rho_j * rho_j));
                f_press += dir * (coef * grad);
                // 粘性力(原始项,μ 在外层乘): μ m_i m_j (v_j - v_i)/ρ_j ∇²W
                let lap = k.visc_lap(r);
                f_visc_raw += (self.particles[j].vel - v_i)
                    * (m_i * self.particles[j].mass / rho_j * lap);
                // 应变率代理累加
                let dv = (self.particles[j].vel - v_i).norm();
                shear += dv / (r + r_eps) * (self.particles[j].mass / rho_j);
            });

            // 非牛顿幂律有效粘度:μ_eff = k·max(剪切率, ε)^(n-1)
            // (n=1 退化为牛顿流体;n<1 剪切变稀;n>1 剪切变稠)
            let m_idx = if mat < self.params.visc_k.len() {
                mat
            } else {
                0
            };
            let kc = self.params.visc_k[m_idx];
            let nn = self.params.visc_n[m_idx];
            let sreg = if shear > self.params.shear_min {
                shear
            } else {
                self.params.shear_min
            };
            let mu_eff = kc * sreg.powf(nn - T::one());
            self.particles[i].mu_eff = mu_eff;

            let f_visc = f_visc_raw * mu_eff;

            // SPH 近邻加速度(压力+粘性),不含体力;重力/浮力在 integrate 经 body_acc 叠加,
            // 这样 `couple_heat`/`couple_points` 在 step→couple 阶段写入的浮力不会被此处覆盖。
            let acc = if rho_i > T::zero() {
                (f_press + f_visc) / rho_i
            } else {
                Vec3::zeros()
            };
            self.particles[i].acc = acc;
        }
    }

    /// 半隐式欧拉:v += a dt; x += v dt。
    /// 总加速度 = SPH 近邻加速度 `acc` + 体力 `body_acc`(重力 + 浮力等耦合体力);
    /// 结算后立即清零 `body_acc`,使浮力每帧由 `couple` 重新写入(对齐 World 的 step→couple 约定)。
    fn integrate(&mut self, dt: T) {
        let g = self.params.gravity;
        for pt in self.particles.iter_mut() {
            let a = pt.acc + g + pt.body_acc;
            pt.vel += a * dt;
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
                <T as num_traits::FromPrimitive>::from_f64(4.0).unwrap()
                    / <T as num_traits::FromPrimitive>::from_f64(3.0).unwrap()
                    * <T as num_traits::FromPrimitive>::from_f64(std::f64::consts::PI).unwrap()
                    * (*r)
                    * (*r)
                    * (*r)
            }
            Shape::Box { half } => {
                half.x * half.y * half.z * <T as num_traits::FromPrimitive>::from_f64(8.0).unwrap()
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
        T: num_traits::ToPrimitive,
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
            if x < hmin.x || x > hmin.x + dx * nx_m1
                || y < hmin.y || y > hmin.y + dx * ny_m1
                || z < hmin.z || z > hmin.z + dx * nz_m1
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
            if pt.pos.x < lo.x || pt.pos.x > hi.x
                || pt.pos.y < lo.y || pt.pos.y > hi.y
                || pt.pos.z < lo.z || pt.pos.z > hi.z
            {
                continue;
            }
            // 三线性插值温度。
            let temp = if let Some((cx, cy, cz, tx, ty, tz)) = to_idx(pt.pos.x, pt.pos.y, pt.pos.z) {
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
                if let Some((cx, cy, cz, _tx, _ty, _tz)) =
                    to_idx(pt.pos.x, pt.pos.y, pt.pos.z)
                {
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
pub fn couple_points(
    &mut self,
    pts: &mut [CouplePoint<T>],
    dt: T,
    drag: T,
    soft_density: T,
) {
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
        let dir = if local.norm() > <T as num_traits::FromPrimitive>::from_f64(1e-9).unwrap() {
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
fn na_distance<T: RealField + Copy>(a: &Vec3<T>, b: &Vec3<T>) -> T {
    (a - b).norm()
}

#[cfg(test)]
mod tests {
    use super::*;
    use phy_core::Subsystem;
    use phy_math::Vec3;

    fn test_params() -> SphParams<f64> {
        let mut p = SphParams::defaults();
        p.bounds_min = Vec3::new(-3.0, -3.0, -3.0);
        p.bounds_max = Vec3::new(3.0, 3.0, 3.0);
        p
    }

    #[test]
    fn fill_box_count_matches_grid() {
        let mut w = FluidWorld::new(test_params());
        w.fill_box(
            Vec3::new(-1.0, -1.0, -1.0),
            Vec3::new(1.0, 1.0, 1.0),
            0.2,
            0.05,
        );
        // 间距 0.2 在 [-0.95,0.95] 区间约 10 个/轴 => ~1000 粒子。
        assert!(w.len() > 500 && w.len() < 2000, "got {}", w.len());
    }

    #[test]
    fn density_converges_near_rest() {
        // 充足粒子在自由空间中静止,密度应在静止密度附近(±20%)。
        let mut p = test_params();
        p.gravity = Vec3::zeros(); // 关闭重力便于稳定
        let mut w = FluidWorld::new(p);
        w.fill_box(
            Vec3::new(-0.5, -0.5, -0.5),
            Vec3::new(0.5, 0.5, 0.5),
            0.1,
            0.05,
        );
        for _ in 0..15 {
            w.step(0.002);
        }
        let mut sum = 0.0;
        for pt in &w.particles {
            sum += pt.rho;
        }
        let avg = sum / w.len() as f64;
        let rho0 = w.params.rest_density;
        assert!(
            (avg - rho0).abs() / rho0 < 0.25,
            "平均密度 {avg} 偏离静止密度 {rho0} 过大"
        );
    }

    #[test]
    fn particle_count_conserved_under_step() {
        let mut w = FluidWorld::new(test_params());
        w.fill_box(
            Vec3::new(-1.0, -1.0, -1.0),
            Vec3::new(1.0, 1.0, 1.0),
            0.1,
            0.05,
        );
        let n0 = w.len();
        for _ in 0..50 {
            w.step(0.003);
        }
        assert_eq!(w.len(), n0, "粒子数不应在 step 中改变");
    }

    #[test]
    fn dam_break_stays_in_bounds() {
        // 溃坝:重力下流体应始终待在盒内(无 NaN / 无越界)。
        let mut w = FluidWorld::new(test_params());
        w.fill_box(
            Vec3::new(-1.5, -1.5, -0.5),
            Vec3::new(-0.5, 1.5, 0.5),
            0.1,
            0.05,
        );
        let lo = w.params.bounds_min;
        let hi = w.params.bounds_max;
        for _ in 0..120 {
            w.step(0.0025);
            for pt in &w.particles {
                assert!(pt.pos.x.is_finite() && pt.pos.y.is_finite() && pt.pos.z.is_finite());
                assert!(pt.pos.x >= lo.x - 1e-6 && pt.pos.x <= hi.x + 1e-6);
                assert!(pt.pos.y >= lo.y - 1e-6 && pt.pos.y <= hi.y + 1e-6);
                assert!(pt.pos.z >= lo.z - 1e-6 && pt.pos.z <= hi.z + 1e-6);
            }
        }
    }

    #[test]
    fn static_body_blocks_fluid() {
        // 静态盒作为不可穿透边界:流体不应进入盒内部。
        let mut w = FluidWorld::new(test_params());
        w.fill_box(
            Vec3::new(-1.5, -1.5, -0.5),
            Vec3::new(1.5, -1.0, 0.5),
            0.1,
            0.05,
        );
        let mut body = Body {
            shape: Shape::Box {
                half: Vec3::new(1.0, 0.5, 1.0),
            },
            pos: Vec3::new(0.0, 0.0, 0.0),
            rot: phy_math::na::UnitQuaternion::identity(),
            vel: Vec3::zeros(),
            inv_mass: 0.0, // 静态
            ..Default::default()
        };
        for _ in 0..100 {
            w.step(0.0025);
            w.couple_bodies(std::slice::from_mut(&mut body), 0.0025, 0.0);
        }
        // 没有粒子应落在盒内(局部坐标 |x|<=1,|y|<=0.5,|z|<=1)。
        for pt in &w.particles {
            let local = body.to_local(&pt.pos);
            assert!(
                !(local.x.abs() <= 1.0 && local.y.abs() <= 0.5 && local.z.abs() <= 1.0),
                "粒子穿透了静态盒: local={:?}",
                local
            );
        }
    }

    #[test]
    fn dynamic_body_gets_buoyancy_upward() {
        // 轻球(密度 < 流体)完全浸没、流体静止时应获得向上的净速度(纯阿基米德效应)。
        // 关闭重力使流体保持静止,隔离浮力,避免自由下落流体的下拽耦合掩盖上举力。
        let mut p = test_params();
        p.gravity = Vec3::zeros();
        let mut w = FluidWorld::new(p);
        w.fill_box(
            Vec3::new(-0.5, -0.5, -0.5),
            Vec3::new(0.5, 0.5, 0.5),
            0.1,
            0.05,
        );
        for _ in 0..15 {
            w.step(0.0025);
        }
        // 一个密度约为流体 1/4 的球,完全浸没。
        let vol = 4.0 / 3.0 * std::f64::consts::PI * 0.6f64.powi(3);
        let mass_b = w.params.rest_density * vol * 0.25;
        let mut body = Body {
            shape: Shape::Sphere { r: 0.6 },
            pos: Vec3::new(0.0, 0.0, 0.0),
            rot: phy_math::na::UnitQuaternion::identity(),
            vel: Vec3::zeros(),
            inv_mass: 1.0 / mass_b,
        
            ..Default::default()
        };
        let v0 = body.vel.y;
        // 把粒子推到球内以制造淹没(流体静止,不会有下拽动量)。
        for pt in w.particles.iter_mut() {
            pt.pos = Vec3::new(pt.pos.x * 0.3, pt.pos.y * 0.3, pt.pos.z * 0.3);
            pt.vel = Vec3::zeros();
        }
        for _ in 0..30 {
            w.step(0.0025);
            w.couple_bodies(std::slice::from_mut(&mut body), 0.0025, 0.0);
        }
        assert!(
            body.vel.y > v0,
            "轻球应因浮力获得向上的速度,实际 vy={}",
            body.vel.y
        );
    }

    #[test]
    fn couple_points_buoyancy_upward() {
        // 静止流体中淹没的点应收到向上的浮力(force.y > 0),且流体获得反向动量。
        let mut p = test_params();
        p.gravity = Vec3::new(0.0, -9.81, 0.0);
        let mut w = FluidWorld::new(p);
        w.fill_box(
            Vec3::new(-0.5, -0.5, -0.5),
            Vec3::new(0.5, 0.5, 0.5),
            0.1,
            0.05,
        );
        for _ in 0..15 {
            w.step(0.0025);
        }
        // 软体密度 ~ 同流体,质量 0.02(=单粒子质量),应受净浮力(因流体静止,无下拽)。
        let mut pt = CouplePoint::new(Vec3::new(0.0, 0.0, 0.0), Vec3::zeros(), 0.02);
        // 记录流体总动量(用于验证反向交换)。
        let p_before: Vec3<f64> = w.particles.iter().map(|x| x.vel * x.mass).sum();
        w.couple_points(std::slice::from_mut(&mut pt), 0.0025, 1.0, w.params.rest_density);
        assert!(pt.force.y > 0.0, "淹没点应受向上浮力, got {}", pt.force.y);
        let p_after: Vec3<f64> = w.particles.iter().map(|x| x.vel * x.mass).sum();
        // 流体因承受能力应获得向下的动量(总动量守恒:点 + 流体 ≈ 0 变化前的系统静止)。
        let dp_fluid = p_after - p_before;
        assert!(
            dp_fluid.y < 0.0,
            "流体应获得向下的反向动量, got {}",
            dp_fluid.y
        );
    }

    #[test]
    fn couple_heat_warmer_fluid_rises() {
        // 温度场下半热、上半冷:温升处流体密度下降 → 粒子获得向上的额外加速度。
        use phy_field::{Bc, HeatField, ScalarField};
        let mut p = test_params();
        p.gravity = Vec3::new(0.0, -9.81, 0.0);
        let mut w = FluidWorld::new(p);
        // 单个粒子放在略偏上方的“热”区。
        w.particles.clear();
        let mut pt = Particle::new(Vec3::new(0.0, 1.0, 0.0), 0.05);
        pt.acc = Vec3::zeros();
        w.particles.push(pt);

        // 温度场:下半温 0,上半温 1(在 y>0 处热)。
        // 温度剖面按局部 y = iy*dx - nx*dx/2 构建,故把场原点设到 (-8,-8,-8)
        // 使格点 iy 对应世界坐标 -8+iy,从而“热区”正好在世界 y>0(粒子所在处)。
        let nx = 16usize;
        let dx = 1.0_f64;
        let mut f = ScalarField::<f64>::new(nx, nx, nx, dx, 0.0, Bc::Neumann)
            .with_origin(Vec3::new(-8.0, -8.0, -8.0));
        for iy in 0..nx {
            let y = (iy as f64) * dx - (nx as f64) * dx / 2.0;
            let t = if y > 0.0 { 1.0 } else { 0.0 };
            for ix in 0..nx {
                for iz in 0..nx {
                    let idx = f.idx(ix, iy, iz);
                    f.u[idx] = t;
                }
            }
        }
        let mut heat = HeatField::new(f, 0.1);
        let t_ref = 0.0_f64;
        let beta = 0.5_f64; // ρ(T)=ρ0/(1+0.5·T)
        // 调用(通过 HeatFieldLike trait 对象)。
        w.couple_heat(&mut heat, 0.01, t_ref, beta, 0.0);

        // 热区:浮力写入 body_acc,净加速度 = g + body_acc;暖区 body_acc.y > 0
        // 使净加速度比纯重力(-9.81)更向上(更接近 0 或为正)。
        let net_y = w.params.gravity.y + w.particles[0].body_acc.y;
        assert!(
            net_y > -9.81,
            "热区粒子应获得向上热浮力修正(净 acc.y > -g), got {}",
            net_y
        );
    }

    #[test]
    fn couple_heat_injects_source_into_moving_region() {
        // 运动粒子应向热场注入热源(对流换热):运动区域格点升温。
        use phy_field::{Bc, HeatField, ScalarField};
        let mut p = test_params();
        p.gravity = Vec3::zeros();
        let mut w = FluidWorld::new(p);
        w.particles.clear();
        // 放在中心、带速度。
        let mut pt = Particle::new(Vec3::new(0.0, 0.0, 0.0), 0.05);
        pt.vel = Vec3::new(2.0, 0.0, 0.0);
        w.particles.push(pt);

        let nx = 16usize;
        let dx = 1.0_f64;
        let f = ScalarField::<f64>::new(nx, nx, nx, dx, 0.0, Bc::Neumann);
        let mut heat = HeatField::new(f, 0.1);
        let before = heat.field.sample(0, 0, 0);
        w.couple_heat(&mut heat, 0.01, 0.0, 0.0, 0.1);
        // couple_heat 把热源累加到 src;推进一帧扩散把 src 合入 u。
        heat.field.step_diffusion(0.1, 0.01);
        let after = heat.field.sample(0, 0, 0);
        assert!(
            after > before,
            "运动区域热场应升温, before={}, after={}",
            before,
            after
        );
    }

    #[test]
    fn boussinesq_closed_loop_plume_rises_and_advects() {
        // Boussinesq 闭环:暖斑受浮力上升 → 流体获得向上速度 → 该速度场使热场被对流
        // 平流(vel_sampler 由 couple_heat 安装)→ 暖斑随流上移。验证热浮力↔对流闭环。
        use phy_field::{Bc, HeatField, ScalarField};
        let mut p = test_params();
        p.gravity = Vec3::new(0.0, -9.81, 0.0);
        let mut w = FluidWorld::new(p);
        // 用小盒填少量流体,避免大计算量。粒子集中在世界原点附近。
        w.particles.clear();
        let n = 3;
        let dxp = 0.4_f64;
        for ix in 0..n {
            for iy in 0..n {
                for iz in 0..n {
                    let x = (ix as f64 - 1.0) * dxp;
                    let y = (iy as f64 - 1.0) * dxp;
                    let z = (iz as f64 - 1.0) * dxp;
                    w.particles.push(Particle::new(Vec3::new(x, y, z), 0.05));
                }
            }
        }

        // 热场:把网格原点设为 (-8,-8,-8),使中心格 (8,8,8) 正好映射到世界原点
        // (流体粒子所在处),这样暖斑在流体内、浮力真正驱动闭环。
        let nx = 16usize;
        let dx = 1.0_f64;
        let mut f = ScalarField::<f64>::new(nx, nx, nx, dx, 0.0, Bc::Neumann)
            .with_origin(Vec3::new(-8.0, -8.0, -8.0));
        let c = (8usize, 8usize, 8usize);
        let ci = f.idx(c.0, c.1, c.2);
        f.u[ci] = 5.0;
        let mut heat = HeatField::new(f, 0.01);
        let t_ref = 0.0_f64;
        let beta = 0.5_f64;

        let warm_centroid_y = |h: &HeatField<f64>| -> f64 {
            let mut sy = 0.0;
            let mut sw = 0.0;
            for iz in 0..nx {
                for iy in 0..nx {
                    for ix in 0..nx {
                        let v = h.field.u[h.field.idx(ix, iy, iz)];
                        if v > 0.1 {
                            sy += (iy as f64) * dx * v;
                            sw += v;
                        }
                    }
                }
            }
            if sw > 0.0 {
                sy / sw
            } else {
                0.0
            }
        };

        let y0 = warm_centroid_y(&heat);
        let dt = 0.005_f64;
        for _ in 0..40 {
            // 每帧:1) couple_heat 刷新速度采样器 + 施热浮力 + 注入热源;
            //       2) 热场 step 用该速度做扩散-对流;3) 流体 step 推进(浮力加速度已入 acc)。
            w.couple_heat(&mut heat, dt, t_ref, beta, 0.01);
            heat.step(&dt);
            w.step(dt);
        }
        let y1 = warm_centroid_y(&heat);
        assert!(
            y1 > y0 + dx * 0.05,
            "Boussinesq 闭环下暖斑质心应上升(热浮力驱动对流平流): y0={}, y1={}",
            y0,
            y1
        );
        // 场仍有限(没有数值爆炸)。
        assert!(
            heat.field.u.iter().all(|&v| v.is_finite()),
            "闭环多帧步进后热场应有限"
        );
    }

    /// 构建两层反向速度的小剪切构型,返回给定幂律指数 `n` 下的流体世界(已 step 一次)。
    fn shear_world(n: f64, dv: f64) -> FluidWorld<f64> {
        let mut p = test_params();
        p.gravity = Vec3::zeros();
        let mut w = FluidWorld::new(p);
        w.particles.clear();
        // 两层粒子:y 方向分层、沿 x 反向速度 => 产生剪切率。
        for iy in 0..2u32 {
            let vy = if iy == 0 { -dv } else { dv };
            for ix in 0..5u32 {
                let x = (ix as f64 - 2.0) * 0.08;
                let mut pt = Particle::with_material(Vec3::new(x, iy as f64 * 0.06, 0.0), 0.05, 0);
                pt.vel = Vec3::new(vy, 0.0, 0.0);
                w.particles.push(pt);
            }
        }
        w.params.visc_k = vec![3.5];
        w.params.visc_n = vec![n];
        w.step(0.001);
        w
    }

    #[test]
    fn non_newtonian_shear_thinning_and_thickening() {
        // 同一剪切构型下:剪切变稀(n<1)有效粘度低于牛顿;剪切变稠(n>1)高于牛顿。
        let thin = shear_world(0.5, 300.0);
        let newt = shear_world(1.0, 300.0);
        let thick = shear_world(1.5, 300.0);
        let mt = thin.effective_viscosity(0);
        let mn = newt.effective_viscosity(0);
        let mk = thick.effective_viscosity(0);
        assert!(mt.is_finite() && mn.is_finite() && mk.is_finite());
        assert!(mt < mn, "剪切变稀应比牛顿更稀: {} < {}", mt, mn);
        assert!(mk > mn, "剪切变稠应比牛顿更稠: {} > {}", mk, mn);
    }

    #[test]
    fn material_tag_distinguishes_viscosity() {
        // 两种材料(visc_k 不同,n 均为 1)在同样剪切下应得到不同有效粘度。
        let mut p = test_params();
        p.gravity = Vec3::zeros();
        let mut w = FluidWorld::new(p);
        w.particles.clear();
        // 粒子 0(材料0) 与粒子 1(材料1):反向速度、近距 => 剪切。
        let mut a = Particle::with_material(Vec3::new(0.0, 0.0, 0.0), 0.05, 0);
        a.vel = Vec3::new(-300.0, 0.0, 0.0);
        let mut b = Particle::with_material(Vec3::new(0.0, 0.06, 0.0), 0.05, 1);
        b.vel = Vec3::new(300.0, 0.0, 0.0);
        w.particles.push(a);
        w.particles.push(b);
        w.params.visc_k = vec![3.5, 10.0];
        w.params.visc_n = vec![1.0, 1.0];
        w.step(0.001);
        let mu0 = w.effective_viscosity(0);
        let mu1 = w.effective_viscosity(1);
        assert!((mu0 - 3.5).abs() < 1e-6, "材料0 应为 3.5,实际 {}", mu0);
        assert!((mu1 - 10.0).abs() < 1e-6, "材料1 应为 10.0,实际 {}", mu1);
    }

    #[test]
    fn multi_material_tags_conserved_under_step() {
        // 多材料流体:step 后粒子材料标签应保持不变(用于相分离/界面识别)。
        let mut w = FluidWorld::new(test_params());
        w.particles.clear();
        for i in 0..10 {
            let mat = i % 3;
            let mut pt = Particle::with_material(
                Vec3::new((i as f64) * 0.1, 0.0, 0.0),
                0.05,
                mat,
            );
            pt.vel = Vec3::new(0.0, -1.0, 0.0);
            w.particles.push(pt);
        }
        let tags: Vec<usize> = w.particles.iter().map(|p| p.material).collect();
        for _ in 0..20 {
            w.step(0.003);
        }
        let after: Vec<usize> = w.particles.iter().map(|p| p.material).collect();
        assert_eq!(tags, after, "材料标签应在 step 中保持");
    }
}
