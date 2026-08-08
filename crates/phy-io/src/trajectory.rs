//! 刚体轨迹导出:把 `RigidWorld` 每个时间步的刚体位姿/速度写成 CSV。
//!
//! 用途:把动力学仿真结果导出给外部绘图/回放工具(gnuplot、Python、ParaView 等)。

use num_traits::FromPrimitive;
use phy_math::RealField;
use phy_rigid::RigidWorld;

use crate::csv::CsvWriter;

/// 单个刚体在一个采样时刻的快照。
#[derive(Debug, Clone)]
pub struct BodySample<T> {
    pub id: usize,
    pub t: T,
    pub px: T,
    pub py: T,
    pub pz: T,
    pub vx: T,
    pub vy: T,
    pub vz: T,
    /// 旋转四元数 (w,x,y,z)。
    pub qw: T,
    pub qx: T,
    pub qy: T,
    pub qz: T,
    /// 反质量(0 = 静态)。
    pub inv_mass: T,
}

impl<T: RealField + Copy + FromPrimitive + std::fmt::Display> BodySample<T> {
    /// 从 `RigidWorld` 在时刻 `t` 抓取所有刚体快照。
    pub fn snapshot(world: &RigidWorld<T>, t: T) -> Vec<BodySample<T>> {
        let mut out = Vec::with_capacity(world.bodies.len());
        for (id, b) in world.bodies.iter().enumerate() {
            let q = b.rot.quaternion();
            out.push(BodySample {
                id,
                t,
                px: b.pos.x,
                py: b.pos.y,
                pz: b.pos.z,
                vx: b.vel.x,
                vy: b.vel.y,
                vz: b.vel.z,
                qw: q.w,
                qx: q.i,
                qy: q.j,
                qz: q.k,
                inv_mass: b.inv_mass,
            });
        }
        out
    }

    /// 该快照的列顺序(与 [`COLUMNS`] 对应)。
    pub fn as_row(&self) -> Vec<T> {
        vec![
            T::from_usize(self.id).unwrap(),
            self.t,
            self.px,
            self.py,
            self.pz,
            self.vx,
            self.vy,
            self.vz,
            self.qw,
            self.qx,
            self.qy,
            self.qz,
            self.inv_mass,
        ]
    }
}

/// 轨迹 CSV 的列名(顺序与 [`BodySample::as_row`] 严格对应)。
pub const COLUMNS: &[&str] = &[
    "id", "t", "px", "py", "pz", "vx", "vy", "vz", "qw", "qx", "qy", "qz", "inv_mass",
];

/// 把依次采集的快照写到 CSV 文件。
pub fn write_trajectory<P, T>(
    path: P,
    frames: &[Vec<BodySample<T>>],
) -> std::io::Result<()>
where
    P: AsRef<std::path::Path>,
    T: RealField + Copy + FromPrimitive + std::fmt::Display,
{
    let mut w = CsvWriter::to_path(path, COLUMNS)?;
    for frame in frames {
        for s in frame {
            w.write_row(s.as_row())?;
        }
    }
    w.flush()
}
