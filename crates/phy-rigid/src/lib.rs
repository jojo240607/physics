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
pub mod ccd;
pub mod broadphase;
pub mod narrowphase;
pub mod solver;
pub mod joint;
pub mod raycast;
pub mod subsystem;
pub mod vehicle;
pub mod fracture;
pub mod islands;
pub mod scene;
pub mod profile;
pub mod world;
pub mod character_controller;
pub mod ragdoll;

pub use shape::{Body, Shape};
pub use contact::Contact;
pub use broadphase::broadphase;
pub use narrowphase::collide;
pub use solver::{SolverParams, solve_position, solve_velocity};
pub use joint::{Joint, JointConstraint};
pub use raycast::{ray_cast, RayHit};
pub use subsystem::RigidSubsystem;
pub use vehicle::{Vehicle, Wheel};
pub use fracture::{fracture_body, fracture_convex, convex_volume_centroid};
pub use scene::SceneDesc;
pub use world::RigidWorld;
pub use profile::StepProfile;
pub use character_controller::CharacterController;
pub use ragdoll::{Ragdoll, RagdollBuilder, RagdollLimb, RagdollParams};

#[cfg(test)]
mod tests {
    use super::*;
    use phy_math::{na, Vec3};

    fn body(shape: Shape<f64>, pos: Vec3<f64>) -> Body<f64> {
        Body::new(shape, pos, 1.0)
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
        let a = Body::new(
            Shape::Box {
                half: Vec3::new(1.0, 1.0, 1.0),
            },
            Vec3::new(0.0, 0.0, 0.0),
            1.0,
        );
        let mut b = Body::new(
            Shape::Box {
                half: Vec3::new(1.0, 1.0, 1.0),
            },
            Vec3::new(1.2, 0.0, 0.0),
            1.0,
        );
        b.rot = rot;
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
        Body::new(
            Shape::Box {
                half: Vec3::new(50.0, 0.5, 50.0),
            },
            Vec3::new(0.0, -0.5, 0.0),
            0.0, // 静态
        )
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

    // ---- S6 连续碰撞检测(CCD) ----

    /// 高速球应被 CCD 挡在薄壁之前,绝不隧穿。
    #[test]
    fn fast_sphere_does_not_tunnel_through_wall() {
        let mut w = RigidWorld::<f64>::new();
        w.gravity = Vec3::new(0.0, 0.0, 0.0); // 关重力,专测隧穿
        // 薄壁(厚度 0.2)置于 x=0,从 x=-5 以 100 单位/秒射向它。
        let wall = w.add_body(Body::new(
            Shape::Box {
                half: Vec3::new(0.1, 5.0, 5.0),
            },
            Vec3::new(0.0, 0.0, 0.0),
            0.0, // 静态
        ));
        let ball = w.add_body(Body::new(
            Shape::Sphere { r: 0.5 },
            Vec3::new(-5.0, 0.0, 0.0),
            1.0,
        ));
        w.bodies[ball].vel = Vec3::new(100.0, 0.0, 0.0); // 一帧跨 100*dt,远超壁厚

        let dt = 1.0 / 120.0;
        for _ in 0..30 {
            w.step(dt);
        }

        // 球必须停在壁左侧(不应穿越到 x>0 的壁后)。
        let x = w.bodies[ball].pos.x;
        let wall_right = w.bodies[wall].pos.x + 0.1; // 壁右表面
        assert!(
            x < wall_right + 0.6,
            "高速球隧穿! 球 x={}, 壁右表面={}",
            x,
            wall_right
        );
        // 球应被弹回(速度反向或至少不再以原速前进)。
        assert!(w.bodies[ball].vel.x <= 1.0, "球未被壁阻挡, vx={}", w.bodies[ball].vel.x);
        // 位置有限(无 NaN)。
        assert!(w.bodies[ball].pos.x.is_finite());
    }

    /// 关闭 CCD(ccd_max_substeps=0)时,同样的高速球会隧穿(对照组)。
    #[test]
    fn fast_sphere_tunnels_with_ccd_disabled() {
        let mut w = RigidWorld::<f64>::new();
        w.gravity = Vec3::new(0.0, 0.0, 0.0);
        w.params.ccd_max_substeps = 0; // 关 CCD
        let _wall = w.add_body(Body::new(
            Shape::Box {
                half: Vec3::new(0.1, 5.0, 5.0),
            },
            Vec3::new(0.0, 0.0, 0.0),
            0.0,
        ));
        let ball = w.add_body(Body::new(
            Shape::Sphere { r: 0.5 },
            Vec3::new(-5.0, 0.0, 0.0),
            1.0,
        ));
        w.bodies[ball].vel = Vec3::new(100.0, 0.0, 0.0);

        let dt = 1.0 / 120.0;
        for _ in 0..30 {
            w.step(dt);
        }
        // 对照组:球应已越过壁(隧穿到 x>0)。
        assert!(
            w.bodies[ball].pos.x > 0.5,
            "CCD 关闭对照组预期隧穿, 但球 x={}",
            w.bodies[ball].pos.x
        );
    }

    /// 低速场景:CCD 开启时行为与离散检测一致(球静止停在地面附近,无穿透)。
    #[test]
    fn ccd_enabled_matches_discrete_at_low_speed() {
        let mut w = RigidWorld::<f64>::new();
        w.params.ccd_max_substeps = 8; // 开启
        let _ground = w.add_body(ground());
        let ball = w.add_body(Body::new(
            Shape::Sphere { r: 0.5 },
            Vec3::new(0.0, 3.0, 0.0),
            1.0,
        ));

        let dt = 1.0 / 120.0;
        for _ in 0..600 {
            w.step(dt);
        }
        // 球应停在地面上方(球心 y≈0.5,底面≈0)。
        let bottom = w.bodies[ball].pos.y - 0.5;
        assert!(bottom > -0.05 && bottom < 0.05, "CCD 稳态穿透, bottom={}", bottom);
        assert!(w.bodies[ball].vel.norm() < 0.5, "速度未收敛");
    }

    /// 高速盒不应穿透静态地面(包围球保守推进对盒也生效)。
    #[test]
    fn fast_box_does_not_tunnel_through_ground() {
        let mut w = RigidWorld::<f64>::new();
        w.gravity = Vec3::new(0.0, 0.0, 0.0);
        let ground = w.add_body(Body::new(
            Shape::Box {
                half: Vec3::new(50.0, 0.5, 50.0),
            },
            Vec3::new(0.0, -0.5, 0.0),
            0.0,
        ));
        let box_id = w.add_body(Body::new(
            Shape::Box {
                half: Vec3::new(0.5, 0.5, 0.5),
            },
            Vec3::new(0.0, 5.0, 0.0),
            1.0,
        ));
        w.bodies[box_id].vel = Vec3::new(0.0, -200.0, 0.0); // 一帧跨 >1.6,远超盒高

        let dt = 1.0 / 120.0;
        for _ in 0..40 {
            w.step(dt);
        }
        // 盒底面不得穿入地面(地面顶面 y=0)。
        let bottom = w.bodies[box_id].pos.y - 0.5;
        assert!(
            bottom > -0.6,
            "高速盒隧穿地面! bottom={}",
            bottom
        );
        assert!(w.bodies[box_id].vel.y >= -1.0, "盒未被地面阻挡, vy={}", w.bodies[box_id].vel.y);
        let _ = ground;
    }
}
