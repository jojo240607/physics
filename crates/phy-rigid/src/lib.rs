//! phy-rigid: 刚体动力学核心(M1: 碰撞检测)。
//!
//! 提供:
//! - `Shape` / `Body`:几何与带位姿的碰撞体
//! - `broadphase`: SAP 粗筛潜在相交对
//! - `collide`: Narrow-phase 精确碰撞(球/盒 SAT / 通用 GJK+EPA)
//! - `Contact`: 接触点/法线/穿透深度
//!
//! M2 将在此之上加积分器与约束求解器。

pub mod shape;
pub mod contact;
pub mod broadphase;
pub mod narrowphase;
pub mod solver;
pub mod joint;
pub mod raycast;
pub mod subsystem;
pub mod vehicle;
pub mod world;

pub use shape::{Body, Shape};
pub use contact::Contact;
pub use broadphase::broadphase;
pub use narrowphase::collide;
pub use solver::{SolverParams, solve_position, solve_velocity};
pub use joint::{Joint, JointConstraint};
pub use raycast::{ray_cast, RayHit};
pub use subsystem::RigidSubsystem;
pub use vehicle::{Vehicle, Wheel};
pub use world::RigidWorld;

#[cfg(test)]
mod tests {
    use super::*;
    use phy_math::{na, Vec3};

    fn body(shape: Shape<f64>, pos: Vec3<f64>) -> Body<f64> {
        Body {
            shape,
            pos,
            rot: na::UnitQuaternion::identity(),
            vel: Vec3::zeros(),
            inv_mass: 1.0,
        }
    }

    #[test]
    fn sphere_overlap_gives_contact() {
        let a = body(Shape::Sphere { r: 1.0 }, Vec3::new(0.0, 0.0, 0.0));
        let b = body(Shape::Sphere { r: 1.0 }, Vec3::new(1.0, 0.0, 0.0));
        let c = collide(&a, &b).expect("应相交");
        assert!((c.depth - 1.0).abs() < 1e-9, "depth={}", c.depth);
        // 法线由 a 指向 b => +X
        assert!(c.normal.x > 0.99);
    }

    #[test]
    fn sphere_separated_no_contact() {
        let a = body(Shape::Sphere { r: 1.0 }, Vec3::new(0.0, 0.0, 0.0));
        let b = body(Shape::Sphere { r: 1.0 }, Vec3::new(3.0, 0.0, 0.0));
        assert!(collide(&a, &b).is_none());
    }

    #[test]
    fn box_overlap_gives_contact() {
        let a = body(
            Shape::Box {
                half: Vec3::new(1.0, 1.0, 1.0),
            },
            Vec3::new(0.0, 0.0, 0.0),
        );
        let b = body(
            Shape::Box {
                half: Vec3::new(1.0, 1.0, 1.0),
            },
            Vec3::new(1.5, 0.0, 0.0),
        );
        let c = collide(&a, &b).expect("盒应相交");
        assert!(c.depth > 0.0 && c.depth < 1.0, "depth={}", c.depth);
        assert!(c.normal.x > 0.9);
    }

    #[test]
    fn box_separated_no_contact() {
        let a = body(
            Shape::Box {
                half: Vec3::new(1.0, 1.0, 1.0),
            },
            Vec3::new(0.0, 0.0, 0.0),
        );
        let b = body(
            Shape::Box {
                half: Vec3::new(1.0, 1.0, 1.0),
            },
            Vec3::new(4.0, 0.0, 0.0),
        );
        assert!(collide(&a, &b).is_none());
    }

    #[test]
    fn rotated_box_still_detects() {
        // 把一个盒绕 Z 转 45°,与另一个重叠
        let rot = na::UnitQuaternion::from_axis_angle(&na::Vector3::z_axis(), std::f64::consts::FRAC_PI_4);
        let a = Body {
            shape: Shape::Box {
                half: Vec3::new(1.0, 1.0, 1.0),
            },
            pos: Vec3::new(0.0, 0.0, 0.0),
            rot: na::UnitQuaternion::identity(),
            vel: Vec3::zeros(),
            inv_mass: 1.0,
        };
        let b = Body {
            shape: Shape::Box {
                half: Vec3::new(1.0, 1.0, 1.0),
            },
            pos: Vec3::new(1.2, 0.0, 0.0),
            rot,
            vel: Vec3::zeros(),
            inv_mass: 1.0,
        };
        assert!(collide(&a, &b).is_some());
    }

    #[test]
    fn broadphase_then_narrowphase_pipeline() {
        let bodies = vec![
            body(Shape::Sphere { r: 1.0 }, Vec3::new(0.0, 0.0, 0.0)),
            body(Shape::Sphere { r: 1.0 }, Vec3::new(1.0, 0.0, 0.0)),
            body(Shape::Sphere { r: 1.0 }, Vec3::new(50.0, 0.0, 0.0)),
        ];
        let pairs = broadphase(&bodies);
        assert_eq!(pairs, vec![(0, 1)]);
        // 对 broad-phase 产出的对做 narrow-phase
        for (i, j) in pairs {
            assert!(collide(&bodies[i], &bodies[j]).is_some());
        }
    }

    // ---- M2 刚体动力学 ----

    /// 静态地面的构造 helper(反质量 0)。
    fn ground() -> Body<f64> {
        Body {
            shape: Shape::Box {
                half: Vec3::new(50.0, 0.5, 50.0),
            },
            pos: Vec3::new(0.0, -0.5, 0.0),
            rot: na::UnitQuaternion::identity(),
            vel: Vec3::zeros(),
            inv_mass: 0.0, // 静态
        }
    }

    #[test]
    fn box_lands_on_ground_no_penetration() {
        // 一个盒从 y=5 自由下落,应停在地面上方(不穿透)。
        let mut w = RigidWorld::<f64>::new();
        w.add_body(ground());
        let box_id = w.add_body(body(
            Shape::Box {
                half: Vec3::new(0.5, 0.5, 0.5),
            },
            Vec3::new(0.0, 5.0, 0.0),
        ));

        let dt = 1.0 / 120.0;
        let mut max_penetration = 0.0_f64;
        for step in 0..1000 {
            w.step(dt);
            let bottom = w.bodies[box_id].pos.y - 0.5;
            // 冲击瞬间允许较大穿透(由后续位置投影恢复);统计最大穿透深度
            max_penetration = max_penetration.max(-bottom);
            // 仅稳态阶段(最后 200 步)要求紧约束
            if step >= 800 {
                assert!(bottom > -0.02, "稳态穿透! bottom={}", bottom);
            }
        }

        // 不变量:冲击穿透应在合理范围(物理引擎允许冲击穿透 < 盒高 20%)
        assert!(
            max_penetration < 0.2,
            "冲击穿透过大: {}",
            max_penetration
        );
        // 静止后盒应停在地面之上
        let rest_bottom = w.bodies[box_id].pos.y - 0.5;
        assert!(
            rest_bottom > -0.02 && rest_bottom < 0.05,
            "未正确停在地面, bottom={}",
            rest_bottom
        );
        // 速度应衰减到接近 0
        assert!(w.bodies[box_id].vel.norm() < 0.5, "速度未收敛");
    }

    #[test]
    fn two_boxes_stack_stably() {
        // 两个盒竖直堆叠在地面上,上盒不得穿透下盒,且整体静止稳定。
        let mut w = RigidWorld::<f64>::new();
        // 静态地面
        let _ground = w.add_body(ground());
        // 下盒(贴近地面, 动态)
        let lower = w.add_body(body(
            Shape::Box {
                half: Vec3::new(0.5, 0.5, 0.5),
            },
            Vec3::new(0.0, 0.5, 0.0),
        ));
        let upper = w.add_body(body(
            Shape::Box {
                half: Vec3::new(0.5, 0.5, 0.5),
            },
            Vec3::new(0.0, 1.6, 0.0), // 略高于接触,应下落贴合
        ));

        let dt = 1.0 / 120.0;
        for step in 0..1500 {
            w.step(dt);
            // 仅稳态阶段(最后 300 步)要求紧约束
            if step >= 1200 {
                let upper_bottom = w.bodies[upper].pos.y - 0.5;
                let lower_top = w.bodies[lower].pos.y + 0.5;
                assert!(
                    upper_bottom > lower_top - 0.05,
                    "上盒穿透下盒: ub={} lt={}",
                    upper_bottom,
                    lower_top
                );
                assert!(w.bodies[upper].vel.norm() < 0.5, "堆叠不稳定");
            }
        }

        // 最终不变量:上盒停在正确高度(下盒顶 1.0 之上,差 < 0.05)
        let upper_bottom = w.bodies[upper].pos.y - 0.5;
        assert!(
            upper_bottom > 0.95,
            "上盒未正确堆叠, bottom={}",
            upper_bottom
        );
        // 下盒也应停在地面之上
        let lower_bottom = w.bodies[lower].pos.y - 0.5;
        assert!(lower_bottom > -0.05, "下盒穿透地面, bottom={}", lower_bottom);
    }
}
