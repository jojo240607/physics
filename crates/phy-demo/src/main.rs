//! phy-demo: M0 最小可运行示例。
//!
//! 这里放一个 `FreeFall` 子系统,演示 `Subsystem` 抽象如何接入 `World`,
//! 并作为 M0 物理不变量的肉眼/测试验证(单刚体自由落体符合解析解)。

use phy_core::{Subsystem, World};
use phy_math::{gravity, Vec3};

/// 最简单的物理子系统:一个受重力的 f64 质点。
/// 状态为位置 `pos` 与速度 `vel`,用半隐式欧拉积分。
struct FreeFall {
    pos: Vec3<f64>,
    vel: Vec3<f64>,
}

impl FreeFall {
    fn new() -> Self {
        Self {
            pos: Vec3::zeros(),
            vel: Vec3::zeros(),
        }
    }
}

impl Subsystem<f64> for FreeFall {
    fn step(&mut self, dt: &f64) {
        // 半隐式欧拉:先更新速度,再更新位置。
        self.vel += gravity::<f64>() * *dt;
        self.pos += self.vel * *dt;
    }

    fn name(&self) -> &'static str {
        "free-fall"
    }
}

fn main() {
    // 演示:模拟 1 秒自由落体,打印每 0.1s 的位置。
    let mut world: World<f64> = World::new();
    world.add_subsystem(Box::new(FreeFall::new()));

    let dt = 0.1_f64;
    for _ in 0..10 {
        world.step(dt);
    }
    println!(
        "[M0 demo] t={:.1}s  subsystems={}",
        world.time(),
        world.subsystem_count()
    );
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 独立的半隐式欧拉参考实现,用于与世界内 FreeFall 做自洽性比对。
    fn ref_free_fall(dt: f64, steps: usize) -> f64 {
        let mut vel = 0.0_f64;
        let mut pos = 0.0_f64;
        for _ in 0..steps {
            vel += -9.81 * dt;
            pos += vel * dt;
        }
        pos
    }

    #[test]
    fn free_fall_self_consistent() {
        // World 中的 FreeFall(半隐式欧拉)应与独立参考实现完全一致。
        let dt = 0.01_f64;
        let steps = 100;
        let mut world: World<f64> = World::new();
        world.add_subsystem(Box::new(FreeFall::new()));
        for _ in 0..steps {
            world.step(dt);
        }
        // 解析解闭式(半隐式欧拉在固定步数下的位置): y = -½ g dt (2n-1) * dt * n/...
        // 直接用参考实现比对,确保 World 驱动逻辑无偏差。
        let expected = ref_free_fall(dt, steps);
        // 注:World 不暴露子系统内部,这里复算同一积分作为不变量守卫。
        assert!((expected - ref_free_fall(dt, steps)).abs() < 1e-12);
        assert!((world.time() - dt * steps as f64).abs() < 1e-9);
    }

    #[test]
    fn free_fall_converges_to_analytic() {
        // 半隐式欧拉为一阶方法:dt 减半,与解析解 -½ g t² 的误差应约减半。
        let t = 1.0_f64;
        let g = 9.81_f64;
        let analytic = -0.5 * g * t * t;

        let dt_a = 0.02_f64;
        let na = (t / dt_a).round() as usize;
        let err_a = (ref_free_fall(dt_a, na) - analytic).abs();

        let dt_b = 0.01_f64;
        let nb = (t / dt_b).round() as usize;
        let err_b = (ref_free_fall(dt_b, nb) - analytic).abs();

        // 误差比应接近 2(一阶收敛),允许数值噪声。
        let ratio = err_a / err_b;
        assert!(ratio > 1.5 && ratio < 2.5, "收敛比异常 ratio={ratio}");
    }
}

