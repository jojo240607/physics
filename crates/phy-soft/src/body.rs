//! 软体质点-弹簧模型核心。

use phy_math::{gravity, RealField, Vec3};

use crate::{DEFAULT_DAMPING, DEFAULT_STIFFNESS, DEFAULT_VERLET_DAMP};

/// 单个质点(velocity-Verlet 积分,显式存速度)。
#[derive(Debug, Clone)]
pub struct Particle<T: RealField + Copy> {
    /// 当前世界位置。
    pub pos: Vec3<T>,
    /// 速度(世界)。
    pub vel: Vec3<T>,
    /// 反质量(0 = 固定点,如布料钉扎)。
    pub inv_mass: T,
    /// 当前帧累加力(每步清零)。
    pub force: Vec3<T>,
}

impl<T: RealField + Copy> Particle<T> {
    /// 构造(初速度为零)。
    pub fn new(pos: Vec3<T>, inv_mass: T) -> Self {
        Self {
            pos,
            vel: Vec3::zeros(),
            inv_mass,
            force: Vec3::zeros(),
        }
    }
}

/// 两质点间弹簧(Hooke + 阻尼)。
#[derive(Debug, Clone)]
pub struct Spring<T: RealField + Copy> {
    /// 质点 a 索引。
    pub a: usize,
    /// 质点 b 索引。
    pub b: usize,
    /// 自然长度。
    pub rest: T,
    /// 刚度系数 k。
    pub k: T,
    /// 阻尼系数(相对速度衰减)。
    pub damp: T,
}

/// 软体:质点网格 + 弹簧集合。
#[derive(Debug, Clone)]
pub struct SoftBody<T: RealField + Copy> {
    /// 质点。
    pub particles: Vec<Particle<T>>,
    /// 弹簧。
    pub springs: Vec<Spring<T>>,
    /// 重力(默认 -Y 9.81)。
    pub gravity: Vec3<T>,
    /// 地面高度(质点 y 不可低于此值)。
    pub ground_y: T,
    /// 速度阻尼(1 = 无阻尼,<1 衰减)。
    pub vel_damp: T,
    /// 地面恢复系数(碰撞后法向速度保留比例)。
    pub restitution: T,
}

impl<T: RealField + Copy> SoftBody<T> {
    /// 空软体(给定地面高度)。
    pub fn new(ground_y: T) -> Self {
        Self {
            particles: Vec::new(),
            springs: Vec::new(),
            gravity: gravity::<T>(),
            ground_y,
            vel_damp: T::from_f64(DEFAULT_VERLET_DAMP).unwrap(),
            restitution: T::from_f64(0.2).unwrap(),
        }
    }

    /// 规则 3D 晶格软体:生成 nx*ny*nz 质点 + 结构/剪切/弯曲弹簧。
    ///
    /// 质点间距 `spacing`,整体平移到 `origin`。顶部一层(ny-1)钉扎为固定点,
    /// 模拟悬挂布料/果冻。
    pub fn from_lattice(
        nx: usize,
        ny: usize,
        nz: usize,
        spacing: T,
        origin: Vec3<T>,
    ) -> Self {
        let mut body = Self::new(T::zero());
        let fixed = T::zero();
        let movable = T::from_f64(1.0).unwrap();

        for iz in 0..nz {
            for iy in 0..ny {
                for ix in 0..nx {
                    let fx = T::from_usize(ix).unwrap();
                    let fy = T::from_usize(iy).unwrap();
                    let fz = T::from_usize(iz).unwrap();
                    let x = origin.x + fx * spacing;
                    let y = origin.y + fy * spacing;
                    let z = origin.z + fz * spacing;
                    // 顶部层钉扎(模拟悬挂)。
                    let inv_mass = if iy + 1 == ny { fixed } else { movable };
                    let p = Vec3::new(x, y, z);
                    body.particles.push(Particle::new(p, inv_mass));
                }
            }
        }

        // 弹簧:结构(邻轴) + 面对角(剪切) + 体对角(弯曲)。
        let idx = |ix: usize, iy: usize, iz: usize| (iz * ny + iy) * nx + ix;
        let k = T::from_f64(DEFAULT_STIFFNESS).unwrap();
        let d = T::from_f64(DEFAULT_DAMPING).unwrap();
        let sp = spacing;
        let sp2 = sp * T::from_f64(2.0).unwrap();
        let sp_sqrt2 = sp * T::from_f64(2.0_f64.sqrt()).unwrap();
        let sp_sqrt3 = sp * T::from_f64(3.0_f64.sqrt()).unwrap();

        for iz in 0..nz {
            for iy in 0..ny {
                for ix in 0..nx {
                    let a = idx(ix, iy, iz);
                    if ix + 1 < nx {
                        body.add_spring(a, idx(ix + 1, iy, iz), k, d);
                    }
                    if iy + 1 < ny {
                        body.add_spring(a, idx(ix, iy + 1, iz), k, d);
                    }
                    if iz + 1 < nz {
                        body.add_spring(a, idx(ix, iy, iz + 1), k, d);
                    }
                    if ix + 1 < nx && iy + 1 < ny {
                        body.add_spring_len(a, idx(ix + 1, iy + 1, iz), sp_sqrt2, k, d);
                    }
                    if ix + 1 < nx && iz + 1 < nz {
                        body.add_spring_len(a, idx(ix + 1, iy, iz + 1), sp_sqrt2, k, d);
                    }
                    if iy + 1 < ny && iz + 1 < nz {
                        body.add_spring_len(a, idx(ix, iy + 1, iz + 1), sp_sqrt2, k, d);
                    }
                    if ix + 1 < nx && iy + 1 < ny && iz + 1 < nz {
                        body.add_spring_len(a, idx(ix + 1, iy + 1, iz + 1), sp_sqrt3, k, d);
                    }
                    if ix + 2 < nx {
                        body.add_spring_len(a, idx(ix + 2, iy, iz), sp2, k, d);
                    }
                }
            }
        }
        body
    }

    /// 添加质点,返回索引。
    pub fn add_particle(&mut self, pos: Vec3<T>, inv_mass: T) -> usize {
        self.particles.push(Particle::new(pos, inv_mass));
        self.particles.len() - 1
    }

    /// 添加弹簧(自动用当前距离作为 rest)。
    pub fn add_spring(&mut self, a: usize, b: usize, k: T, damp: T) {
        let rest = (self.particles[b].pos - self.particles[a].pos).norm();
        self.springs.push(Spring { a, b, rest, k, damp });
    }

    /// 添加弹簧(显式 rest 长度)。
    pub fn add_spring_len(&mut self, a: usize, b: usize, rest: T, k: T, damp: T) {
        self.springs.push(Spring { a, b, rest, k, damp });
    }

    /// 累加所有力到 `particles[i].force`(重力 + 弹簧 Hooke + 阻尼)。
    fn accumulate_forces(&mut self) {
        // 重力。
        for p in self.particles.iter_mut() {
            p.force = if p.inv_mass > T::zero() {
                self.gravity / p.inv_mass
            } else {
                Vec3::zeros()
            };
        }
        // 弹簧。
        for s in &self.springs {
            let (pa, pb) = (self.particles[s.a].pos, self.particles[s.b].pos);
            let d = pb - pa;
            let len = d.norm().max(T::from_f64(1e-9).unwrap());
            let dir = d / len;
            let rel_vel = (self.particles[s.b].vel - self.particles[s.a].vel).dot(&dir);
            let f_mag = (len - s.rest) * s.k + rel_vel * s.damp;
            let f = dir * f_mag;
            if self.particles[s.a].inv_mass > T::zero() {
                self.particles[s.a].force += f;
            }
            if self.particles[s.b].inv_mass > T::zero() {
                self.particles[s.b].force -= f;
            }
        }
    }

    /// 推进一步:velocity-Verlet 积分(对常加速度精确) + 地面碰撞。
    pub fn step(&mut self, dt: T) {
        let dt2 = dt * dt;
        // 1. 当前加速度。
        self.accumulate_forces();
        let acc_old: Vec<Vec3<T>> = self
            .particles
            .iter()
            .map(|p| {
                if p.inv_mass > T::zero() {
                    p.force * p.inv_mass
                } else {
                    Vec3::zeros()
                }
            })
            .collect();

        // 2. 更新位置(用旧速度 + 旧加速度半步)。
        for (p, a) in self.particles.iter_mut().zip(acc_old.iter()) {
            if p.inv_mass <= T::zero() {
                continue;
            }
            p.pos += p.vel * dt + *a * (dt2 * T::from_f64(0.5).unwrap());
        }

        // 3. 新位置下重新算力 -> 新加速度。
        self.accumulate_forces();
        let acc_new: Vec<Vec3<T>> = self
            .particles
            .iter()
            .map(|p| {
                if p.inv_mass > T::zero() {
                    p.force * p.inv_mass
                } else {
                    Vec3::zeros()
                }
            })
            .collect();

        // 4. 更新速度(平均加速度)。
        let damp = self.vel_damp;
        for (p, (ao, an)) in self
            .particles
            .iter_mut()
            .zip(acc_old.iter().zip(acc_new.iter()))
        {
            if p.inv_mass <= T::zero() {
                continue;
            }
            p.vel += (*ao + *an) * (dt * T::from_f64(0.5).unwrap());
            p.vel *= damp;
        }

        // 5. 地面/边界碰撞(单向推出 + 法向反弹)。
        let gy = self.ground_y;
        let rest = self.restitution;
        for p in self.particles.iter_mut() {
            if p.inv_mass <= T::zero() {
                continue;
            }
            if p.pos.y < gy {
                p.pos.y = gy;
                if p.vel.y < T::zero() {
                    p.vel.y = -p.vel.y * rest;
                }
            }
        }
    }
}
