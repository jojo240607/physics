//! 标量场切片导出:把三维标量场切成二维平面(沿 X/Y/Z 轴),写成 CSV 标量网格。
//!
//! 用途:热场 / 电势 / 引力势等的可视化。每一片是一张数值表,
//! 配合世界坐标原点可定位。

use num_traits::FromPrimitive;
use phy_field::ScalarField;
use phy_math::RealField;

use crate::csv::CsvWriter;

/// 切片平面方向。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum SliceAxis {
    /// 固定 z = k,导出 (x,y) 平面。
    XY,
    /// 固定 y = k,导出 (x,z) 平面。
    XZ,
    /// 固定 x = k,导出 (y,z) 平面。
    YZ,
}

impl SliceAxis {
    fn dims<T: RealField + Copy>(&self, f: &ScalarField<T>) -> (usize, usize) {
        match self {
            SliceAxis::XY => (f.nx, f.ny),
            SliceAxis::XZ => (f.nx, f.nz),
            SliceAxis::YZ => (f.ny, f.nz),
        }
    }
}

/// 把切片写成 CSV:第一列是行索引 `i`,随后是该行每个单元的数值列 `v0..v{cols-1}`,
/// 最后三列是该行起点对应的世界坐标(便于定位)。
pub fn write_slice<T, P>(path: P, field: &ScalarField<T>, axis: SliceAxis, k: usize) -> std::io::Result<()>
where
    T: RealField + Copy + FromPrimitive + std::fmt::Display,
    P: AsRef<std::path::Path>,
{
    let (cols, rows) = axis.dims(field);
    let (kx, ky, kz) = match axis {
        SliceAxis::XY => (0usize, 0usize, k),
        SliceAxis::XZ => (0usize, k, 0usize),
        SliceAxis::YZ => (k, 0usize, 0usize),
    };
    // 构造列名(泄漏到 'static 仅用于本次写出的表头字符串)。
    let mut headers: Vec<&str> = Vec::with_capacity(cols + 4);
    headers.push("i");
    for c in 0..cols {
        headers.push(Box::leak(format!("v{c}").into_boxed_str()) as &str);
    }
    headers.push("world_x");
    headers.push("world_y");
    headers.push("world_z");

    let mut w = CsvWriter::to_path(path, &headers)?;
    for r in 0..rows {
        let (x0, y0, z0) = match axis {
            SliceAxis::XY => (0, r, kz),
            SliceAxis::XZ => (0, ky, r),
            SliceAxis::YZ => (kx, r, 0),
        };
        let mut row: Vec<T> = Vec::with_capacity(cols + 4);
        row.push(T::from_usize(r).unwrap());
        for c in 0..cols {
            let (x, y, z) = match axis {
                SliceAxis::XY => (c, y0, z0),
                SliceAxis::XZ => (c, y0, z0),
                SliceAxis::YZ => (y0, c, z0),
            };
            row.push(field.sample(x, y, z));
        }
        // 该行起点的世界坐标(x0,y0,z0 为网格索引,乘间距加原点)。
        row.push(field.origin.x + T::from_usize(x0).unwrap() * field.dx);
        row.push(field.origin.y + T::from_usize(y0).unwrap() * field.dx);
        row.push(field.origin.z + T::from_usize(z0).unwrap() * field.dx);
        w.write_row(row)?;
    }
    w.flush()
}

/// 便捷:把标量场中心切片(XY 平面,z = nz/2)导出。
pub fn write_center_slice<T, P>(path: P, field: &ScalarField<T>) -> std::io::Result<()>
where
    T: RealField + Copy + FromPrimitive + std::fmt::Display,
    P: AsRef<std::path::Path>,
{
    let k = field.nz / 2;
    write_slice(path, field, SliceAxis::XY, k)
}
