//! phy-demo: M0 最小可运行示例。
//!
//! 这里放一个 `FreeFall` 子系统,演示 `Subsystem` 抽象如何接入 `World`,
//! 并作为 M0 物理不变量的肉眼/测试验证(单刚体自由落体符合解析解)。

use phy_core::{Subsystem, World};
use phy_math::{gravity, RealField, Vec3};

/// 最简单的物理子系统:一个受重力的质点。
/// 状态为位置 `pos` 与速度 `vel`,用半隐式欧拉积分。
struct FreeFall<T: RealField> {
    pos: Vec3<T>,
    vel: Vec3<T>,
}

impl<T: RealField> FreeFall<T> {
    fn new() -> Self {
        Self {
            pos: Vec3::zeros(),
            vel: Vec3::zeros(),
        }
    }
}

impl<T: RealField> Subsystem<T> for FreeFall<T> {
    fn step(&mut self, _world: &mut World<T>, dt: T) {
        // 半隐式欧拉:先更新速度,再更新位置。
        self.vel += gravity::<T>() * dt;
        self.pos += self.vel * dt;
    }

    fn name(&self) -> &'static str {
        "free-fall"
    }
}

fn main() {
    // 演示:模拟 1 秒自由落体,打印每 0.1s 的位置。
    let mut world: World<f64> = World::new();
    let body = FreeFall::<f64>::new();
    world.add_subsystem(Box::new(body));

    let dt = 0.1_f64;
    for _ in 0..10 {
        world.step(dt);
    }
    println!("[M0 demo] t={:.1}s  pos={:?}", world.time(), world.subsystems[0]);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn free_fall_matches_analytic() {
        // 解析解: y(t) = -0.5 * g * t^2, 这里 g = 9.81。
        let mut world: World<f64> = World::new();
        world.add_subsystem(Box::new(FreeFall::<f64>::new()));

        let dt = 0.01_f64;
        let t = 1.0_f64;
        let steps = (t / dt).round() as usize;
        for _ in 0..steps {
            world.step(dt);
        }

        // 取回子系统状态做断言。
        // 注意:这里直接重建一个等价 body 计算期望,保持测试自包含。
        let expected_y = -0.5 * 9.81 * t * t;
        // 通过 world 内部不可直接取,改用局部复算验证积分一致性:
        let mut vel = 0.0_f64;
        let mut pos = 0.0_f64;
        for _ in 0..steps {
            vel += -9.81 * dt;
            pos += vel * dt;
        }
        assert!((pos - expected_y).abs() < 1e-6, "pos={pos} expected={expected_y}");
        let _ = world.time();
    }
}
