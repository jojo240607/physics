//! 颗粒世界核心:球面颗粒 + PBD 时间步进 + 接触/边界约束。
//!
//! PBD 一步(`step`):
//! 1. **预测**:可动颗粒按惯性 + 外力(重力)预测下一位置 `p_pred = p + v·dt + g·dt²`。
//! 2. **约束投影**(Gauss-Seidel 迭代 `iterations` 次):
//!    - 球-球非穿透:对每对间隙 `gap = d - (r_i+r_j) < 0` 的接触,沿连线按反质量
//!      加权把两球推开到恰好相切(单边约束,分离不再作用)。
//!    - 盒边界:颗粒中心越出 `[lo,hi]` 夹回,并消去对应法向速度分量。
//! 3. **速度回写**:`v = (p_new - p_old)/dt · vel_damp`(PBD 速度由位置差定义)。
//! 4. **切向摩擦**(近似):接触后按 `friction` 衰减切向相对速度,维持堆积角。
//!
//! 宽相位邻近检测用 `phy_core::SpatialGrid` 均匀网格哈希(见 `step`),将朴素
//! O(n²) 邻域搜索降为近似 O(n),支撑数万级颗粒规模。

use phy_core::SpatialGrid;
use phy_math::{gravity, RealField, Vec3};
use serde::de::DeserializeOwned;
use crate::gpu_flat::GranularFlatData;
use serde::{Deserialize, Serialize};
use rayon::prelude::*;

/// 单个球面颗粒。
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(bound = "T: RealField + Copy + Serialize + DeserializeOwned + nalgebra::Scalar")]
pub struct Grain<T: RealField + Copy> {
    /// 当前世界位置(球心)。
    #[serde(with = "phy_rigid::shape::serde_geom")]
    pub pos: Vec3<T>,
    /// 速度(世界)。
    #[serde(with = "phy_rigid::shape::serde_geom")]
    pub vel: Vec3<T>,
    /// 半径。
    pub radius: T,
    /// 反质量(0 = 固定/墙体绑定,常量重力下不坠)。
    pub inv_mass: T,
}

impl<T: RealField + Copy> Grain<T> {
    /// 构造可动颗粒(默认 inv_mass = 1/mass)。
    pub fn new(pos: Vec3<T>, radius: T, mass: T) -> Self {
        let inv = if mass > T::zero() {
            T::one() / mass
        } else {
            T::zero()
        };
        Self {
            pos,
            vel: Vec3::zeros(),
            radius,
            inv_mass: inv,
        }
    }

    /// 构造固定颗粒(如锚定大石,inv_mass=0)。
    pub fn fixed(pos: Vec3<T>, radius: T) -> Self {
        Self {
            pos,
            vel: Vec3::zeros(),
            radius,
            inv_mass: T::zero(),
        }
    }
}

/// 颗粒世界。
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(bound = "T: RealField + Copy + Serialize + DeserializeOwned + nalgebra::Scalar")]
pub struct GranularWorld<T: RealField + Copy> {
    /// 颗粒集合。
    pub grains: Vec<Grain<T>>,
    /// 重力(默认 -Y 9.81)。
    #[serde(with = "phy_rigid::shape::serde_geom")]
    pub gravity: Vec3<T>,
    /// 容器盒下界(颗粒中心约束在 `[bounds_lo, bounds_hi]`,含半径余量由约束内部处理)。
    #[serde(with = "phy_rigid::shape::serde_geom")]
    pub bounds_lo: Vec3<T>,
    /// 容器盒上界。
    #[serde(with = "phy_rigid::shape::serde_geom")]
    pub bounds_hi: Vec3<T>,
    /// PBD 约束投影迭代次数(越多越硬/越精确)。
    pub iterations: usize,
    /// 速度阻尼(<1 衰减,1 无)。
    pub vel_damp: T,
    /// 切向摩擦系数(0 = 无摩擦,1 = 强摩擦)。
    pub friction: T,
    /// 当前仿真时间。
    pub t: T,
}

impl<T: RealField + Copy> GranularWorld<T> {
    /// 构造空世界,容器默认 `(-10,-10,-10)..(10,10,10)`,重力默认 -Y。
    pub fn new() -> Self {
        let lo = Vec3::new(
            T::from_f64(-10.0).unwrap(),
            T::from_f64(-10.0).unwrap(),
            T::from_f64(-10.0).unwrap(),
        );
        let hi = Vec3::new(
            T::from_f64(10.0).unwrap(),
            T::from_f64(10.0).unwrap(),
            T::from_f64(10.0).unwrap(),
        );
        Self {
            grains: Vec::new(),
            gravity: gravity::<T>(),
            bounds_lo: lo,
            bounds_hi: hi,
            iterations: 4,
            vel_damp: T::from_f64(0.99).unwrap(),
            friction: T::from_f64(0.3).unwrap(),
            t: T::zero(),
        }
    }

    /// 设置容器盒。
    pub fn set_bounds(&mut self, lo: Vec3<T>, hi: Vec3<T>) {
        self.bounds_lo = lo;
        self.bounds_hi = hi;
    }

    /// 添加颗粒。
    pub fn add(&mut self, g: Grain<T>) {
        self.grains.push(g);
    }

    /// 在盒内随机/网格撒布 `count` 个等半径颗粒(网格填充,避免初始重叠)。
    ///
    /// `radius` 为颗粒半径,`mass` 为每个颗粒质量。`seed`-less 确定性网格布局:
    /// 沿 XYZ 按间距 `2·radius·pack` 铺排,从盒底向上堆叠,直到放满 `count` 个
    /// 或到达盒顶。`pack > 1` 留初始间隙避免接触约束首步过度修正。
    pub fn fill_grid(&mut self, count: usize, radius: T, mass: T, pack: T) {
        let lo = self.bounds_lo;
        let hi = self.bounds_hi;
        let step = radius * T::from_f64(2.0).unwrap() * pack;
        let mut placed = 0usize;
        let mut y = lo.y + radius;
        while y <= hi.y - radius && placed < count {
            let mut x = lo.x + radius;
            while x <= hi.x - radius && placed < count {
                let mut z = lo.z + radius;
                while z <= hi.z - radius && placed < count {
                    self.add(Grain::new(Vec3::new(x, y, z), radius, mass));
                    z += step;
                    placed += 1;
                }
                x += step;
            }
            y += step;
        }
    }

    /// PBD 推进一步。
    pub fn step(&mut self, dt: T)
    where
        T: num_traits::NumCast,
    {
        let n = self.grains.len();
        if n == 0 {
            return;
        }
        let g = self.gravity;
        let dt2 = dt * dt;

        // 1. 预测位置(保存旧位置)。
        let mut predicted: Vec<Vec3<T>> = Vec::with_capacity(n);
        for gr in &self.grains {
            let pred = if gr.inv_mass > T::zero() {
                gr.pos + gr.vel * dt + g * dt2
            } else {
                gr.pos
            };
            predicted.push(pred);
        }
        let old: Vec<Vec3<T>> = self.grains.iter().map(|gr| gr.pos).collect();

        // 2. 约束投影(Jacobi 式并行;S7 并行后端)。
        //
        // 原实现为 Gauss-Seidel(每对即时改写 `predicted`,顺序敏感、不可并行)。
        // 现改为 Jacobi:每对只读本迭代起点的 `predicted` 快照,把位移修正累加到
        // 独立的 per-body delta 缓冲(`deltas`),迭代末统一施加。这样可在 rayon
        // 下并行遍历所有 (i<j) 对;reduce 以固定顺序合并,结果与串行逐对相加
        // 完全一致(同索引对的修正大小相同,仅求和顺序固定),保持确定性(利于 S8)。
        //
        // 宽相位邻域对由 `_contact_pairs` 经 `SpatialGrid` 空间哈希生成(见该函数),
        // 把朴素 O(n²) 降为近似 O(n)。
        // M2-fix: 用 Arc 共享只读接触对,避免每次迭代 clone 大向量。
        let pairs = std::sync::Arc::new(self._contact_pairs(&predicted));
        let nthreads = rayon::current_num_threads().max(1);
        // 分块大小:让并行任务数约等于线程数(而非 pairs 数),把 fold 的
        // 全量 delta 缓冲分配次数从 O(pairs) 降到 O(threads),并让 reduce
        // 的合并步数降到 O(log threads)。原实现 `pairs.par_iter().fold()`
        // 把 pairs 切成成百上千个细粒度任务,每个任务都分配一个 `vec![zero; n]`
        // 全量缓冲,且 reduce 的 `|a,b| ...collect()` 每次都 clone 一个 n 维向量,
        // 总开销 O(pairs × n) 内存分配+复制 —— 这就是 n 较大时性能悬崖(5000≈1167ms、
        // 10000≈4430ms/帧)的根因。
        let chunk_size = (pairs.len() / nthreads).max(1) + 1;
        for _ in 0..self.iterations {
            // 2a. 球-球非穿透(单边约束),Jacobi 并行累积。
            let zero = Vec3::zeros();
            let deltas: Vec<Vec3<T>> = pairs
                .par_chunks(chunk_size)
                .fold(
                    || vec![zero; n],
                    |mut local, chunk| {
                        for &(i, j) in chunk {
                            let ri = self.grains[i].radius;
                            let rj = self.grains[j].radius;
                            let pi = predicted[i];
                            let pj = predicted[j];
                            let d = pj - pi;
                            let dist = d.norm().max(T::from_f64(1e-9).unwrap());
                            let min_dist = ri + rj;
                            if dist < min_dist {
                                let wi = self.grains[i].inv_mass;
                                let wj = self.grains[j].inv_mass;
                                let wsum = wi + wj;
                                if wsum > T::zero() {
                                    let corr = (min_dist - dist) / dist;
                                    let dir = d * corr;
                                    local[i] -= dir * (wi / wsum);
                                    local[j] += dir * (wj / wsum);
                                }
                            }
                        }
                        local
                    },
                )
                .reduce(|| vec![zero; n], |mut a, b| {
                    for k in 0..n {
                        a[k] += b[k];
                    }
                    a
                });

            for k in 0..n {
                predicted[k] += deltas[k];
            }
            // 2b. 盒边界约束(夹回中心,留半径余量)。
            let lo = self.bounds_lo;
            let hi = self.bounds_hi;
            for k in 0..n {
                if self.grains[k].inv_mass <= T::zero() {
                    continue;
                }
                let r = self.grains[k].radius;
                let p = &mut predicted[k];
                if p.x < lo.x + r {
                    p.x = lo.x + r;
                }
                if p.x > hi.x - r {
                    p.x = hi.x - r;
                }
                if p.y < lo.y + r {
                    p.y = lo.y + r;
                }
                if p.y > hi.y - r {
                    p.y = hi.y - r;
                }
                if p.z < lo.z + r {
                    p.z = lo.z + r;
                }
                if p.z > hi.z - r {
                    p.z = hi.z - r;
                }
            }
        }

        // 3. 速度回写 + 4. 摩擦/边界法向消去。
        let damp = self.vel_damp;
        let fr = self.friction;
        for k in 0..n {
            if self.grains[k].inv_mass <= T::zero() {
                continue;
            }
            let np = predicted[k];
            let mut new_vel = (np - old[k]) / dt;

            // 边界法向速度消去(撞墙不动)。
            let lo = self.bounds_lo;
            let hi = self.bounds_hi;
            let r = self.grains[k].radius;
            if np.x <= lo.x + r + T::from_f64(1e-6).unwrap() {
                if new_vel.x < T::zero() {
                    new_vel.x = T::zero();
                }
            }
            if np.x >= hi.x - r - T::from_f64(1e-6).unwrap() {
                if new_vel.x > T::zero() {
                    new_vel.x = T::zero();
                }
            }
            if np.y <= lo.y + r + T::from_f64(1e-6).unwrap() {
                if new_vel.y < T::zero() {
                    new_vel.y = T::zero();
                }
            }
            if np.y >= hi.y - r - T::from_f64(1e-6).unwrap() {
                if new_vel.y > T::zero() {
                    new_vel.y = T::zero();
                }
            }
            if np.z <= lo.z + r + T::from_f64(1e-6).unwrap() {
                if new_vel.z < T::zero() {
                    new_vel.z = T::zero();
                }
            }
            if np.z >= hi.z - r - T::from_f64(1e-6).unwrap() {
                if new_vel.z > T::zero() {
                    new_vel.z = T::zero();
                }
            }

            // 近似切向摩擦:对速度施加轻微衰减(全局,简易堆积角维持)。
            if fr > T::zero() {
                new_vel *= T::one() - fr * (T::one() - damp);
            } else {
                new_vel *= damp;
            }

            self.grains[k].vel = new_vel;
            self.grains[k].pos = np;
        }
        self.t += dt;
    }

    /// 宽相位接触对生成(空间哈希)。
    ///
    /// 给定快照位置 `pts`(长度须等于 `grains.len()`),返回所有满足
    /// `|pts[i]-pts[j]| < r_i+r_j` 且 `i<j` 的接触对,按 `(i,j)` 升序排列。
    /// 内部用 `SpatialGrid`(cell_size = 2·max_radius)只扫描邻近 27 桶,把朴素
    /// O(n²) 降为近似 O(n)。结果与暴力全配对**集合等价**(仅顺序固定为升序),
    /// 保证确定性且不会漏掉任何真实接触。
    fn _contact_pairs(&self, pts: &[Vec3<T>]) -> Vec<(usize, usize)>
    where
        T: num_traits::NumCast,
    {
        let n = pts.len();
        let cast = |x: T| -> f64 { num_traits::cast::<T, f64>(x).unwrap() };
        let max_r = self
            .grains
            .iter()
            .map(|gr| gr.radius)
            .fold(T::zero(), |a, b| if a > b { a } else { b });
        if max_r <= T::zero() {
            // 退化:全配对 O(n²)。
            let mut v = Vec::with_capacity(n * (n.saturating_sub(1)) / 2);
            for i in 0..n {
                for j in (i + 1)..n {
                    v.push((i, j));
                }
            }
            return v;
        }
        // 内部用 f64 网格(避免向 GranularWorld 主 impl 扩散 ToPrimitive bound)。
        let cell_f = cast(max_r) * 2.0;
        let pts_f: Vec<Vec3<f64>> = pts
            .iter()
            .map(|p| Vec3::new(cast(p.x), cast(p.y), cast(p.z)))
            .collect();
        let mut grid = SpatialGrid::with_cell_size(cell_f);
        grid.build(&pts_f);
        let mut seen = std::collections::HashSet::new();
        let mut v = Vec::new();
        for i in 0..n {
            let center = pts_f[i];
            let ri = cast(self.grains[i].radius);
            // 查询半径覆盖最坏情况:两球切于相邻桶边界时圆心距 = ri+rj <= 2·max_r。
            let cand = grid.neighbors(center, cell_f);
            for &j in &cand {
                if j <= i {
                    continue; // 只保留 i<j。
                }
                let rj = cast(self.grains[j].radius);
                let min_dist = ri + rj;
                let d = pts_f[j] - pts_f[i];
                if d.dot(&d) <= min_dist * min_dist {
                    let key = (i, j);
                    if seen.insert(key) {
                        v.push((i, j));
                    }
                }
            }
        }
        v.sort_unstable();
        v
    }

    /// W5 前置:导出 GPU 友好扁平数据。
    ///
    /// 在调用方预测位置(`step` 的 1. 阶段)后调用,上传当前 `grains` 位置作为
    /// 预测快照,并预生成全部 `(i<j)` 接触对(空间哈希宽相位,与 CPU 端 `pairs`
    /// 一致),供 `par_pairs_reduce` 内核逐对累加 per-body 位移修正。
    pub fn to_gpu_flat(&self) -> GranularFlatData
    where
        T: num_traits::NumCast,
    {
        let f = |x: T| -> f32 { num_traits::cast::<T, f32>(x).unwrap() };
        let n = self.grains.len();
        let mut pos = Vec::with_capacity(n);
        let mut old = Vec::with_capacity(n);
        let mut vel = Vec::with_capacity(n);
        let mut inv_mass = Vec::with_capacity(n);
        for gr in &self.grains {
            pos.push([f(gr.pos.x), f(gr.pos.y), f(gr.pos.z), f(gr.radius)]);
            old.push([f(gr.pos.x), f(gr.pos.y), f(gr.pos.z), 0.0]);
            vel.push([f(gr.vel.x), f(gr.vel.y), f(gr.vel.z), 0.0]);
            inv_mass.push(f(gr.inv_mass));
        }
        // 宽相位接触对生成(复用 `_contact_pairs`,与 CPU `step` 一致)。
        // 用当前 `grains` 位置(= pos 缓冲)作为快照,展开为 flat `u32` 对。
        let positions_t: Vec<Vec3<T>> = self
            .grains
            .iter()
            .map(|gr| Vec3::new(gr.pos.x, gr.pos.y, gr.pos.z))
            .collect();
        let contact = self._contact_pairs(&positions_t);
        let mut pairs = Vec::with_capacity(contact.len() * 2);
        for (i, j) in &contact {
            pairs.push(*i as u32);
            pairs.push(*j as u32);
        }
        let npairs = contact.len();
        GranularFlatData {
            n,
            pos,
            old,
            vel,
            inv_mass,
            pairs,
            npairs,
            gravity: [f(self.gravity.x), f(self.gravity.y), f(self.gravity.z)],
            bounds_lo: [f(self.bounds_lo.x), f(self.bounds_lo.y), f(self.bounds_lo.z)],
            bounds_hi: [f(self.bounds_hi.x), f(self.bounds_hi.y), f(self.bounds_hi.z)],
            iterations: self.iterations as u32,
            vel_damp: f(self.vel_damp),
            friction: f(self.friction),
            dt: 1.0f32 / 60.0,
        }
    }

    /// 统计当前颗粒总体积(球体积之和),用于密度/堆积比测试。
    pub fn total_volume(&self) -> T {
        let mut v = T::zero();
        let four_thirds_pi = T::from_f64(4.0 / 3.0 * std::f64::consts::PI).unwrap();
        for gr in &self.grains {
            v += four_thirds_pi * gr.radius * gr.radius * gr.radius;
        }
        v
    }

    /// 查询当前 `grains` 位置的活跃接触对(球-球非穿透约束的候选集)。
    ///
    /// 返回所有满足 `|pos[i]-pos[j]| < r_i+r_j` 且 `i<j` 的对,按 `(i,j)` 升序。
    /// 内部经 `SpatialGrid` 空间哈希宽相位生成,与 `step` / `to_gpu_flat` 用的
    /// 接触集完全一致。可用于调试、接触网络可视化或耦合其他子系统。
    pub fn contact_pairs(&self) -> Vec<(usize, usize)>
    where
        T: num_traits::NumCast,
    {
        let positions: Vec<Vec3<T>> = self
            .grains
            .iter()
            .map(|gr| Vec3::new(gr.pos.x, gr.pos.y, gr.pos.z))
            .collect();
        self._contact_pairs(&positions)
    }
}

impl<T: RealField + Copy> Default for GranularWorld<T> {
    fn default() -> Self {
        Self::new()
    }
}
