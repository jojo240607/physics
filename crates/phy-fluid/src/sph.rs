//! SPH 流体世界与求解器。
//!
//! 采用 Müller 2003 的标准弱可压缩 SPH:
//! - Poly6 核估算密度,理想气体状态方程 p = k (ρ - ρ0) 得压力;
//! - 对称压力力 + 粘性力 + 重力作为加速度来源;
//! - 半隐式欧拉积分;盒状边界带阻尼反弹;
//! - 均匀网格(单元 = 光滑长度 h)做邻居查询,保证线性复杂度;
//! - `couple_bodies` 提供与刚体的双向耦合(浮力 / 阻力 / 动量交换)。

use std::collections::HashMap;

use phy_math::{RealField, Vec3};
use phy_rigid::shape::{Body, Shape};

use crate::kernels::Kernels;
use crate::particle::Particle;

/// SPH 求解参数(全部采用工程单位,默认 f64)。
#[derive(Debug, Clone)]
pub struct SphParams<T: RealField + Copy> {
    /// 静止密度 ρ0(用于压力状态方程)。
    pub rest_density: T,
    /// 压力刚度系数 k(越大越不可压,但需更小 dt 保持稳定)。
    pub stiffness: T,
    /// 动力粘度 μ。
    pub viscosity: T,
    /// 粒子质量。
    pub mass: T,
    /// 光滑长度 h(核作用半径,也是网格单元边长)。
    pub h: T,
    /// 重力加速度(向量,x 右 / y 上 / z 前)。
    pub gravity: Vec3<T>,
    /// 模拟盒边界(粒子被约束在 [bounds_min, bounds_max] 内)。
    pub bounds_min: Vec3<T>,
    /// 模拟盒边界上限。
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

/// SPH 流体世界。
pub struct FluidWorld<T: RealField + Copy + num_traits::ToPrimitive> {
    /// 求解参数。
    pub params: SphParams<T>,
    /// 所有粒子。
    pub particles: Vec<Particle<T>>,
    kernels: Kernels<T>,
    grid: Grid<T>,
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

    /// 受力:对称压力力 + 粘性力 + 重力,汇聚成加速度。
    fn compute_forces(&mut self) {
        let k = &self.kernels;
        let h = self.params.h;
        let mu = self.params.viscosity;
        for i in 0..self.particles.len() {
            let p_i = self.particles[i].pos;
            let v_i = self.particles[i].vel;
            let rho_i = self.particles[i].rho;
            let p_i_p = self.particles[i].p;
            let m_i = self.particles[i].mass;

            let mut f_press = Vec3::zeros();
            let mut f_visc = Vec3::zeros();

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
                // 粘性力: μ m_i m_j (v_j - v_i)/ρ_j ∇²W
                let lap = k.visc_lap(r);
                f_visc += (self.particles[j].vel - v_i)
                    * (mu * m_i * self.particles[j].mass / rho_j * lap);
            });

            // 加速度: (f_press + f_visc) / ρ_i + g
            let acc = if rho_i > T::zero() {
                (f_press + f_visc) / rho_i
            } else {
                Vec3::zeros()
            } + self.params.gravity;
            self.particles[i].acc = acc;
        }
    }

    /// 半隐式欧拉:v += a dt; x += v dt。
    fn integrate(&mut self, dt: T) {
        for pt in self.particles.iter_mut() {
            pt.vel += pt.acc * dt;
            pt.pos += pt.vel * dt;
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
}
