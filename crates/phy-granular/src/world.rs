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
//! 邻近检测用朴素 O(n²)(颗粒数几千内足够;海量规模应换空间哈希,留待增强)。

use phy_math::{gravity, RealField, Vec3};
use serde::de::DeserializeOwned;
use serde::{Deserialize, Serialize};

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
    pub fn step(&mut self, dt: T) {
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

        // 2. 约束投影(Gauss-Seidel)。
        for _ in 0..self.iterations {
            // 2a. 球-球非穿透(单边约束)。
            for i in 0..n {
                for j in (i + 1)..n {
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
                        if wsum <= T::zero() {
                            continue;
                        }
                        let corr = (min_dist - dist) / dist;
                        let dir = d * corr;
                        predicted[i] -= dir * (wi / wsum);
                        predicted[j] += dir * (wj / wsum);
                    }
                }
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

    /// 统计当前颗粒总体积(球体积之和),用于密度/堆积比测试。
    pub fn total_volume(&self) -> T {
        let mut v = T::zero();
        let four_thirds_pi = T::from_f64(4.0 / 3.0 * std::f64::consts::PI).unwrap();
        for gr in &self.grains {
            v += four_thirds_pi * gr.radius * gr.radius * gr.radius;
        }
        v
    }
}

impl<T: RealField + Copy> Default for GranularWorld<T> {
    fn default() -> Self {
        Self::new()
    }
}
