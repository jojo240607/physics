//! 颗粒摩擦堆积验证。
//!
//! 引擎 `phy-granular` 采用 **接触级切向摩擦**(速度级动摩擦 + 位置级 PBD 静
//! 摩擦,见 `world.rs` 步骤 2 的 `pos_friction` 与步骤 4 的 `friction`)。
//! 位置级摩擦使接触颗粒"咬合"、抵抗切向滑动,维持堆积角(安息角)。
//! 本测试验证颗粒在摩擦下能堆积出**显著高度**并最终静止。

#[cfg(test)]
mod tests {
    use crate::world::{Grain, GranularWorld};
    use phy_math::Vec3;

    /// 一批颗粒从中心撒下,位置级摩擦使其堆积出高度并静止。
    #[test]
    fn grains_pile_up_with_friction() {
        let mut w: GranularWorld<f64> = GranularWorld::new();
        w.set_bounds(Vec3::new(-2.0, -0.2, -2.0), Vec3::new(2.0, 4.0, 2.0));
        w.gravity = Vec3::new(0.0, -9.81, 0.0);
        w.friction = 0.8;     // 速度级动摩擦。
        w.pos_friction = 0.8; // 位置级静摩擦(咬合,维持安息角)。
        w.vel_damp = 0.99;
        w.iterations = 6;

        let r = 0.15;
        let spacing = r * 2.2;
        let n = 200usize;
        let per_row = 7;
        let base = -(per_row as f64 - 1.0) / 2.0 * spacing;
        for i in 0..n {
            let layer = i / (per_row * per_row);
            let idx = i % (per_row * per_row);
            let row = (idx % per_row) as f64;
            let col = (idx / per_row) as f64;
            let x = base + row * spacing;
            let z = base + col * spacing;
            let y = 1.5 - layer as f64 * spacing;
            w.add(Grain::new(Vec3::new(x, y, z), r, 0.4));
        }

        let dt = 1.0 / 120.0;
        for _ in 0..1500 {
            w.step(dt);
        }

        // 统计:最高颗粒 y、最大速度、水平展宽。
        let mut max_y = 0.0f64;
        let mut max_speed = 0.0f64;
        let mut xz_span = 0.0f64;
        for g in &w.grains {
            max_y = max_y.max(g.pos.y);
            max_speed = max_speed.max(g.vel.norm());
            xz_span = xz_span.max((g.pos.x.abs() + g.pos.z.abs()).sqrt());
        }
        let height = max_y - (-0.2 + r); // 堆顶 - 堆底。
        eprintln!(
            "[pile] grains={} max_y={:.3} height={:.3} max_speed={:.4}",
            w.grains.len(),
            max_y,
            height,
            max_speed
        );

        // 1) 堆积出显著高度(非平铺):高于颗粒半径的 4 倍。
        assert!(max_y > 4.0 * r, "grains should pile up, max_y={max_y}");
        // 2) 颗粒基本静止。
        assert!(max_speed < 0.5, "settled grains should be near-static, max_speed={max_speed}");
    }

    /// 位置级摩擦(安息角)验证:在等效斜面上(重力带水平分量),颗粒下滑时应被
    /// "咬住"停在更陡处。`pos_friction` 越高,颗粒停止时的水平展宽越小(更收拢、
    /// 安息角更陡),而非一路滑到盒壁摊平。
    #[test]
    fn pos_friction_steeper_repose_angle() {
        fn spread_on_slope(pos_friction: f64) -> f64 {
            let mut w: GranularWorld<f64> = GranularWorld::new();
            // 宽而浅的盒,重力带 +x 分量(等效斜面),颗粒会被推向右壁并堆积。
            w.set_bounds(Vec3::new(-6.0, -0.2, -6.0), Vec3::new(6.0, 3.0, 6.0));
            // 重力斜向右下:等效在斜面上,颗粒有下滑(向 +x)趋势。
            let g = 9.81;
            w.gravity = Vec3::new(g * 0.35, -g, 0.0);
            w.friction = 0.6;
            w.pos_friction = pos_friction;
            w.vel_damp = 0.99;
            w.iterations = 10;
            let r = 0.15;
            let n = 200usize;
            let dt = 1.0 / 120.0;
            // 从左侧高处撒下,观察其向右滑落并被摩擦咬住的展宽。
            let mut rng = 1234u64;
            for _ in 0..n {
                rng = rng.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
                let jx = ((rng >> 33) as f64 / (1u64 << 31) as f64) - 0.5;
                rng = rng.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
                let jz = ((rng >> 33) as f64 / (1u64 << 31) as f64) - 0.5;
                w.add(Grain::new(
                    Vec3::new(-4.0 + jx * 0.5, 2.5, jz * 5.0),
                    r,
                    0.4,
                ));
            }
            for _ in 0..2000 {
                w.step(dt);
            }
            // 颗粒停止后的 x 方向展宽(从最左到最右)。
            let xs: Vec<f64> = w.grains.iter().map(|g| g.pos.x).collect();
            let minx = xs.iter().cloned().fold(f64::INFINITY, f64::min);
            let maxx = xs.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
            maxx - minx
        }

        let spread_no = spread_on_slope(0.0);
        let spread_yes = spread_on_slope(0.8);
        eprintln!(
            "[repose] spread x: no-fric={:.3} pos-fric=0.8={:.3}",
            spread_no, spread_yes
        );
        // 位置级摩擦应让颗粒在更靠近起点处被咬住(展宽更小),而非一路滑到 +x 壁。
        assert!(
            spread_yes < spread_no - 0.5,
            "position-level friction should hold grains on a steeper repose (narrower spread): no={spread_no:.3} yes={spread_yes:.3}"
        );
    }
}
