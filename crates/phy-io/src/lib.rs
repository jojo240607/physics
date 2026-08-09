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
pub mod archive;

pub use archive::{archive_world, load_world, load_world_json, save_world, save_world_json, unarchive_world, SubArchive, WorldArchive};
pub use csv::CsvWriter;
pub use field_slice::{write_center_slice, write_slice, SliceAxis};
pub use trajectory::{write_trajectory, BodySample, COLUMNS};

#[cfg(test)]
mod tests;

#[cfg(test)]
mod archive_test;

