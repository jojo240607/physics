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

pub use shape::{Body, Shape};
pub use contact::Contact;
pub use broadphase::broadphase;
pub use narrowphase::collide;

#[cfg(test)]
mod tests {
    use super::*;
    use phy_math::{na, Vec3};

    fn body(shape: Shape<f64>, pos: Vec3<f64>) -> Body<f64> {
        Body {
            shape,
            pos,
            rot: na::UnitQuaternion::identity(),
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
            inv_mass: 1.0,
        };
        let b = Body {
            shape: Shape::Box {
                half: Vec3::new(1.0, 1.0, 1.0),
            },
            pos: Vec3::new(1.2, 0.0, 0.0),
            rot,
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
}
