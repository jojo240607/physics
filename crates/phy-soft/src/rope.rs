//! 绳索 / 链 (S3):基于 XPBD 距离约束的一维软体。
//!
//! 拓扑:一条粒子链 `p[0]..p[n-1]`,相邻粒子间用 `DistanceConstraint` 连接
//! (`p[i]`-`p[i+1]`,共 n-1 条)。与 `Cloth`(二维网格)共用同一套 PBD 投影,
//! 故表现一致、可无缝加入同一 `World`。
//!
//! 两种典型做法:
//! - 绳索(rope):一端或两端钉死(`pinned[0]` / `pinned[n-1]`),其余悬挂。
//! - 链(chain):完全无钉死,整体自由下落/被抛甩。

use std::any::Any;

use crate::body::Particle;
use crate::cloth::DistanceConstraint;
use phy_core::Subsystem;
use phy_math::{gravity, RealField, Vec3};

/// 一维 PBD 绳索/链。
///
/// `particles[i]` 与 `particles[i+1]` 之间用 `constraints[i]` 约束。
pub struct Rope<T: RealField + Copy> {
    /// 粒子链(位置/速度/反质量)。
    pub particles: Vec<Particle<T>>,
    /// 相邻距离约束:`constraints[i]` 连接 `particles[i]` 与 `particles[i+1]`。
    pub constraints: Vec<DistanceConstraint<T>>,
    /// `pinned[i]==true` 的粒子在求解前后都被强制还原到 `rest[i]`(钉死)。
    pub pinned: Vec<bool>,
    /// 钉死粒子的目标位置(仅在 `pinned[i]` 时有效)。
    pub rest: Vec<Vec3<T>>,
    /// 投影迭代次数(越大越接近硬约束,默认 20)。
    pub iterations: usize,
    /// 速度阻尼(1 = 无阻尼)。
    pub vel_damp: T,
    /// 重力(默认向下 -9.81)。
    pub gravity: Vec3<T>,
}

impl<T: RealField + Copy> Rope<T> {
    /// 沿一条线段构造 `n` 个粒子的均匀绳索,端点 0 钉死。
    ///
    /// - `start` / `end`:绳索两端世界坐标。
    /// - `n`:粒子数(≥2)。
    /// - `mass`:每个粒子质量(>0)。
    pub fn line(start: Vec3<T>, end: Vec3<T>, n: usize, mass: T) -> Self {
        assert!(n >= 2, "Rope 至少需要 2 个粒子");
        let n_minus_1 = T::from_usize(n - 1).expect("n 超出可表示范围");
        let inv_mass = T::one() / mass;
        let mut particles = Vec::with_capacity(n);
        let mut rest = Vec::with_capacity(n);
        for i in 0..n {
            let fi = T::from_usize(i).expect("i 超出可表示范围");
            let t = fi / n_minus_1;
            let pos = start + (end - start) * t;
            particles.push(Particle::new(pos, inv_mass));
            rest.push(pos);
        }

        let mut pinned = vec![false; n];
        pinned[0] = true; // 默认悬吊:顶端钉死

        let mut constraints = Vec::with_capacity(n - 1);
        for i in 0..(n - 1) {
            let rest_len = (particles[i + 1].pos - particles[i].pos).norm();
            constraints.push(DistanceConstraint {
                a: i,
                b: i + 1,
                rest: rest_len,
            });
        }

        Self {
            particles,
            constraints,
            pinned,
            rest,
            iterations: 20,
            vel_damp: T::one(),
            gravity: gravity::<T>(),
        }
    }

    /// 钉死指定端点:0 = 起点, n-1 = 终点。
    pub fn pin(&mut self, idx: usize) {
        assert!(idx < self.particles.len(), "pin 索引越界");
        self.pinned[idx] = true;
    }

    /// 取消钉死指定端点(变为自由链节)。
    pub fn unpin(&mut self, idx: usize) {
        assert!(idx < self.particles.len(), "unpin 索引越界");
        self.pinned[idx] = false;
    }

    /// 单步推进 `dt`:钉死还原 -> 预测 -> 约束投影 -> 回写速度/位置。
    ///
    /// 采用与 `Cloth` 一致的 position-based dynamics(Gauss-Seidel 投影)。
    pub fn step(&mut self, dt: &T) {
        let n = self.particles.len();
        if n == 0 {
            return;
        }
        let mut pinned_count = 0usize;
        for i in 0..n {
            if self.pinned[i] {
                self.particles[i].pos = self.rest[i];
                self.particles[i].vel = Vec3::new(T::zero(), T::zero(), T::zero());
                pinned_count += 1;
            }
        }
        if pinned_count == n {
            return; // 全钉死,无需仿真
        }

        // 1. 预测位置(惯性 + 重力),保存旧位置。
        let dt2 = *dt * *dt;
        let g = self.gravity;
        let mut predicted: Vec<Vec3<T>> = Vec::with_capacity(n);
        for (i, p) in self.particles.iter().enumerate() {
            let pred = if self.pinned[i] {
                self.rest[i]
            } else {
                p.pos + p.vel * *dt + g * dt2
            };
            predicted.push(pred);
        }

        // 2. 约束投影(Gauss-Seidel,多迭代收敛到硬约束)。
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

        // 3. 回写位置/速度(钉死粒子保持目标位置)。
        for i in 0..n {
            if self.pinned[i] {
                self.particles[i].pos = self.rest[i];
                continue;
            }
            let np = predicted[i];
            self.particles[i].vel = (np - self.particles[i].pos) / *dt * self.vel_damp;
            self.particles[i].pos = np;
        }
    }

    /// 返回绳索总长度(各段静止长度之和),用于测试与调试。
    pub fn rest_length(&self) -> T {
        let mut s = T::zero();
        for c in &self.constraints {
            s = s + c.rest;
        }
        s
    }
}

impl<T: RealField + Copy> Subsystem<T> for Rope<T> {
    fn as_any(&self) -> &dyn Any {
        self
    }
    fn as_any_mut(&mut self) -> &mut dyn Any {
        self
    }
    fn step(&mut self, dt: &T) {
        Rope::step(self, dt);
    }
    fn name(&self) -> &'static str {
        "rope"
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use phy_math::{Vec3};
    type F32 = f32;

    /// 单端钉死的悬垂线应被重力向下拉伸,且不可伸长(总长不增)。
    #[test]
    fn hanging_rope_stretches_down_not_longer() {
        let start = Vec3::<f32>::new(0.0, 2.0, 0.0);
        let end = Vec3::<f32>::new(0.0, 0.0, 0.0);
        let mut rope = Rope::<f32>::line(start, end, 8, 1.0);
        let rest = rope.rest_length();
        let dt = 0.01f32;
        for _ in 0..120 {
            rope.step(&dt);
        }
        // 末端(自由端)应向下移动(垂荡)。
        assert!(rope.particles[7].pos.y < end.y);
        // 总长度不得因重力而显著增加(不可伸长约束)。
        let actual: f32 = rope
            .particles
            .windows(2)
            .map(|w| (w[1].pos - w[0].pos).norm())
            .fold(0.0, |a, b| a + b);
        assert!(actual <= rest * 1.05, "绳索被拉长了: {} > {}", actual, rest);
    }

    /// 完全自由(无钉死)的链应整体下落,但不解算崩溃、长度守恒。
    #[test]
    fn free_chain_falls_but_keeps_length() {
        let start = Vec3::<f32>::new(0.0, 2.0, 0.0);
        let end = Vec3::<f32>::new(1.0, 2.0, 0.0);
        let mut rope = Rope::<f32>::line(start, end, 6, 1.0);
        // 全部解除钉死 -> 自由链。
        for i in 0..rope.particles.len() {
            rope.unpin(i);
        }
        let y0: f32 = rope.particles.iter().map(|p| p.pos.y).sum::<f32>()
            / rope.particles.len() as F32;
        let dt = 0.01f32;
        for _ in 0..30 {
            rope.step(&dt);
        }
        let y1: f32 = rope.particles.iter().map(|p| p.pos.y).sum::<f32>()
            / rope.particles.len() as F32;
        assert!(y1 < y0, "自由链应整体下落");

        let actual: f32 = rope
            .particles
            .windows(2)
            .map(|w| (w[1].pos - w[0].pos).norm())
            .fold(0.0, |a, b| a + b);
        assert!(
            actual <= rope.rest_length() * 1.05,
            "自由链段被拉长了: {} > {}",
            actual,
            rope.rest_length()
        );
    }

    /// 双端钉死的绳索为张紧弦,自由节应在两端之间,且整体不下落崩溃。
    #[test]
    fn pinned_both_ends_stays_between() {
        let start = Vec3::<f32>::new(-1.0, 0.0, 0.0);
        let end = Vec3::<f32>::new(1.0, 0.0, 0.0);
        let mut rope = Rope::<f32>::line(start, end, 6, 1.0);
        rope.pin(0);
        rope.pin(5);
        let dt = 0.01f32;
        for _ in 0..80 {
            rope.step(&dt);
        }
        // 中间节 x 应在 [-1,1] 内、y 不应大幅下垂(张紧)。
        for i in 1..5 {
            assert!(rope.particles[i].pos.x.abs() <= 1.001);
            assert!(rope.particles[i].pos.y.abs() < 0.3, "中间节下垂过多");
        }
        // 两端保持钉死位置。
        assert!((rope.particles[0].pos - start).norm() < 1e-5);
        assert!((rope.particles[5].pos - end).norm() < 1e-5);
    }
}
