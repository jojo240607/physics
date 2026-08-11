//! 布料模拟:位置动力学(PBD / Position-Based Dynamics)(M19 / 路线图 #3)。
//!
//! 与 `body.rs` 的质点-弹簧(显式受力 + velocity-Verlet)不同,PBD 走
//! "预测-约束投影-速度回写" 路线,对大刚度布料无条件稳定:
//!
//! 1. **预测**:每个可动质点按惯性 + 外力(重力)预测下一位置
//!    `p_pred = p + v·dt + a_ext·dt²`。
//! 2. **约束投影**(Gauss-Seidel 迭代 `iterations` 次):对每个距离约束,
//!    把两端质点沿连线方向拉回 `rest`,按反质量加权(钉扎点 inv_mass=0 不动)。
//! 3. **速度回写**:`v = (p_new - p_old)/dt`(PBD 速度由位置差定义,天然阻尼)。
//! 4. **地面碰撞**:投影后若 `y<ground_y` 直接夹到地面(位置约束)。
//!
//! 约束类型:
//! - 结构边(相邻经纬质点,rest = 间距)—— 维持布料骨架。
//! - 剪切对角(相邻格对角,rest = √2·间距)—— 抗剪切。
//! - 弯曲边(隔一个的经纬质点,rest = 2·间距)—— 抗折叠(可选)。
//!
//! 钉扎:通过 `pin(i)` 把质点设为 inv_mass=0(固定),模拟悬挂/系绳。

use phy_core::Subsystem;
use phy_math::{gravity, RealField, Vec3};
use serde::de::DeserializeOwned;
use serde::{Deserialize, Serialize};

use crate::body::Particle;

/// 距离约束(两点保持固定间距)。
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(bound = "T: RealField + Copy + Serialize + DeserializeOwned")]
pub struct DistanceConstraint<T: RealField + Copy> {
    pub a: usize,
    pub b: usize,
    pub rest: T,
}

/// 布料(PBD)。
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(bound = "T: RealField + Copy + Serialize + DeserializeOwned")]
pub struct Cloth<T: RealField + Copy> {
    /// 质点网格(nx × ny,平整铺在 XY 平面,z=0)。
    pub particles: Vec<Particle<T>>,
    /// 网格分辨率(列/行)。
    pub nx: usize,
    pub ny: usize,
    /// 距离约束(结构 + 剪切 + 可选弯曲)。
    pub constraints: Vec<DistanceConstraint<T>>,
    /// 网格间距。
    pub spacing: T,
    /// 重力(默认 -Y)。
    #[serde(with = "phy_rigid::shape::serde_geom")]
    pub gravity: Vec3<T>,
    /// 地面高度(质点 y 不可低于此值)。
    pub ground_y: T,
    /// PBD 约束投影迭代次数(越多越硬)。
    pub iterations: usize,
    /// 速度阻尼(<1 衰减,1 无)。
    pub vel_damp: T,
}

impl<T: RealField + Copy> Cloth<T> {
    /// 构造 nx×ny 平整布料,锚点在 `origin`,整体位于 XY 平面(z=0)。
    /// 默认无钉扎;调用 [`Cloth::pin`] 设置固定点。
    pub fn new(nx: usize, ny: usize, spacing: T, origin: Vec3<T>) -> Self {
        let mut particles = Vec::with_capacity(nx * ny);
        let movable = T::from_f64(1.0).unwrap();
        for iy in 0..ny {
            for ix in 0..nx {
                let fx = T::from_usize(ix).unwrap();
                let fy = T::from_usize(iy).unwrap();
                let pos = Vec3::new(
                    origin.x + fx * spacing,
                    origin.y - fy * spacing,
                    origin.z,
                );
                particles.push(Particle::new(pos, movable));
            }
        }
        let mut cloth = Self {
            particles,
            nx,
            ny,
            constraints: Vec::new(),
            spacing,
            gravity: gravity::<T>(),
            ground_y: T::from_f64(-1000.0).unwrap(),
            iterations: 5,
            vel_damp: T::from_f64(0.99).unwrap(),
        };
        cloth.build_constraints(true, true);
        cloth
    }

    #[inline]
    fn idx(&self, ix: usize, iy: usize) -> usize {
        iy * self.nx + ix
    }

    /// 构建约束:结构边 + 剪切对角 + (可选)弯曲边。
    pub fn build_constraints(&mut self, shear: bool, bend: bool) {
        let sp = self.spacing;
        let sp2 = sp * T::from_f64(2.0).unwrap();
        let sp_d = sp * T::from_f64(2.0_f64.sqrt()).unwrap();
        let add = |a: usize, b: usize, rest: T, out: &mut Vec<DistanceConstraint<T>>| {
            out.push(DistanceConstraint { a, b, rest });
        };
        for iy in 0..self.ny {
            for ix in 0..self.nx {
                let a = self.idx(ix, iy);
                if ix + 1 < self.nx {
                    add(a, self.idx(ix + 1, iy), sp, &mut self.constraints);
                }
                if iy + 1 < self.ny {
                    add(a, self.idx(ix, iy + 1), sp, &mut self.constraints);
                }
                if shear {
                    if ix + 1 < self.nx && iy + 1 < self.ny {
                        add(a, self.idx(ix + 1, iy + 1), sp_d, &mut self.constraints);
                    }
                    if ix + 1 < self.nx && iy > 0 {
                        add(a, self.idx(ix + 1, iy - 1), sp_d, &mut self.constraints);
                    }
                }
                if bend {
                    if ix + 2 < self.nx {
                        add(a, self.idx(ix + 2, iy), sp2, &mut self.constraints);
                    }
                    if iy + 2 < self.ny {
                        add(a, self.idx(ix, iy + 2), sp2, &mut self.constraints);
                    }
                }
            }
        }
    }

    /// 钉扎质点(设为固定点,inv_mass=0)。常用于悬挂布料四角/上边。
    pub fn pin(&mut self, ix: usize, iy: usize) {
        let i = self.idx(ix, iy);
        self.particles[i].inv_mass = T::zero();
        self.particles[i].vel = Vec3::zeros();
    }

    /// PBD 推进一步。
    pub fn step(&mut self, dt: T) {
        let g = self.gravity;
        let dt2 = dt * dt;
        let n = self.particles.len();

        // 1. 预测位置(惯性 + 重力),保存旧位置。
        let mut predicted: Vec<Vec3<T>> = Vec::with_capacity(n);
        for p in &self.particles {
            let pred = if p.inv_mass > T::zero() {
                p.pos + p.vel * dt + g * dt2
            } else {
                p.pos
            };
            predicted.push(pred);
        }

        // 2. 约束投影(Gauss-Seidel)。
        for _ in 0..self.iterations {
            for c in &self.constraints {
                let pa = predicted[c.a];
                let pb = predicted[c.b];
                let w_a = self.particles[c.a].inv_mass;
                let w_b = self.particles[c.b].inv_mass;
                let w_sum = w_a + w_b;
                if w_sum <= T::zero() {
                    continue; // 两端皆固定。
                }
                let d = pb - pa;
                let len = d.norm().max(T::from_f64(1e-9).unwrap());
                let corr = (len - c.rest) / len;
                let dir = d * corr;
                predicted[c.a] += dir * (w_a / w_sum);
                predicted[c.b] -= dir * (w_b / w_sum);
            }
        }

        // 3. 地面碰撞(位置约束) + 4. 回写位置/速度。
        let gy = self.ground_y;
        let damp = self.vel_damp;
        for (p, pred) in self.particles.iter_mut().zip(predicted.into_iter()) {
            if p.inv_mass <= T::zero() {
                continue;
            }
            let mut np = pred;
            if np.y < gy {
                np.y = gy;
            }
            p.vel = (np - p.pos) / dt * damp;
            p.pos = np;
        }
    }
}

impl<T: RealField + Copy> Subsystem<T> for Cloth<T> {
    fn as_any(&self) -> &dyn std::any::Any {
        self
    }
    fn as_any_mut(&mut self) -> &mut dyn std::any::Any {
        self
    }
    fn step(&mut self, dt: &T) {
        self.step(*dt);
    }
    fn name(&self) -> &'static str {
        "cloth"
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use phy_core::world::World;
    use phy_math::Vec3;

    #[test]
    fn pinned_cloth_sags_but_stays_connected() {
        // 上边两点钉扎,自由下垂;重力下应下沉,但布料不应撕裂(相邻质点
        // 距离应接近 rest,且不应出现 NaN/爆炸)。
        let mut cloth = Cloth::<f64>::new(6, 6, 0.5, Vec3::new(0.0, 5.0, 0.0));
        // 钉扎上边两角。
        cloth.pin(0, 0);
        cloth.pin(5, 0);
        cloth.ground_y = -100.0;
        let dt = 1.0 / 60.0;
        for _ in 0..200 {
            cloth.step(dt);
        }
        // 钉扎点应保持原位。
        assert!((cloth.particles[cloth.idx(0, 0)].pos - Vec3::new(0.0, 5.0, 0.0)).norm() < 1e-9);
        assert!((cloth.particles[cloth.idx(5, 0)].pos - Vec3::new(2.5, 5.0, 0.0)).norm() < 1e-9);
        // 自由下垂:底部中心质点 y 应明显低于钉扎高度。
        let bottom = cloth.particles[cloth.idx(2, 5)].pos.y;
        assert!(bottom < 5.0, "cloth should sag below pinned top row, got {}", bottom);
        // 结构约束维持:相邻质点距离应在 rest 附近(容许 PBD 松弛余量)。
        for c in &cloth.constraints {
            let d = (cloth.particles[c.b].pos - cloth.particles[c.a].pos).norm();
            assert!((d - c.rest).abs() < 0.15, "constraint stretch {:.3} vs rest {:.3}", d, c.rest);
            assert!(cloth.particles[c.a].pos.x.is_finite());
        }
    }

    #[test]
    fn cloth_runs_as_subsystem_in_world() {
        let mut cloth = Cloth::<f64>::new(4, 4, 0.4, Vec3::new(0.0, 3.0, 0.0));
        cloth.pin(0, 0);
        cloth.pin(3, 0);
        let mut w: World<f64> = World::default();
        w.add_subsystem(Box::new(cloth));
        for _ in 0..30 {
            w.step(1.0 / 60.0);
        }
        let c = w.get(0).unwrap().as_any().downcast_ref::<Cloth<f64>>().unwrap();
        assert!(c.particles[c.idx(0, 0)].pos.y == 3.0);
    }
}
