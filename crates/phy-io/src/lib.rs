//! `phy-io`:物理仿真结果导出(轨迹 / 场切片 CSV)。
//!
//! 提供两类导出:
//! - [`trajectory`]:刚体位姿/速度时间序列,供回放与绘图。
//! - [`field_slice`]:三维标量场二维切片,供热场/势场可视化。
//!
//! 均为纯文本 CSV,无额外运行时依赖,可被 gnuplot / Python / ParaView 直接消费。

pub mod csv;
pub mod field_slice;
pub mod trajectory;

pub use csv::CsvWriter;
pub use field_slice::{write_center_slice, write_slice, SliceAxis};
pub use trajectory::{write_trajectory, BodySample, COLUMNS};

#[cfg(test)]
mod tests {
    use super::*;
    use phy_field::{Bc, ScalarField};
    use phy_math::{na, Vec3};
    use phy_rigid::{Body, RigidWorld, Shape};

    #[test]
    fn trajectory_csv_has_header_and_one_row_per_body() {
        let mut world = RigidWorld::<f64>::new();
        world.add_body(Body {
            shape: Shape::Sphere { r: 0.5 },
            pos: Vec3::new(1.0, 2.0, 3.0),
            rot: na::one(),
            vel: Vec3::new(0.0, 0.0, 0.0),
            inv_mass: 1.0,
        });
        let frames = vec![BodySample::snapshot(&world, 0.0)];
        let path = std::env::temp_dir().join("phy_io_test_traj.csv");
        {
            let mut w = CsvWriter::to_path(&path, COLUMNS).unwrap();
            for s in &frames[0] {
                w.write_row(s.as_row()).unwrap();
            }
            w.flush().unwrap();
        }
        let text = std::fs::read_to_string(&path).unwrap();
        let lines: Vec<&str> = text.lines().collect();
        assert_eq!(lines[0], "id,t,px,py,pz,vx,vy,vz,qw,qx,qy,qz,inv_mass");
        assert_eq!(lines.len(), 2, "表头 + 1 个刚体");
        assert!(lines[1].starts_with("0,0,1,2,3"), "首行数据坐标正确");
        let _ = std::fs::remove_file(&path);
    }

    #[test]
    fn field_slice_csv_has_grid_rows() {
        let f = ScalarField::<f64>::new(4, 4, 4, 1.0, 0.0, Bc::Neumann);
        let path = std::env::temp_dir().join("phy_io_test_slice.csv");
        write_slice(&path, &f, SliceAxis::XY, 2).unwrap();
        let text = std::fs::read_to_string(&path).unwrap();
        let lines: Vec<&str> = text.lines().collect();
        // 表头 + 4 行(y 方向)。
        assert_eq!(lines.len(), 5);
        assert!(lines[0].starts_with("i,v0,v1,v2,v3,world_x,world_y,world_z"));
        let _ = std::fs::remove_file(&path);
    }
}

