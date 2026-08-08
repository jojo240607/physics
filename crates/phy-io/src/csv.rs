//! 极简 CSV 写出工具(无外部依赖)。
//!
//! 提供一个轻量 [`CsvWriter`],把行缓冲成文本并一次性写出。
//! 数值用 `Display`;向量分量拆成多列。

use std::fmt::Display;
use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::Path;

/// 轻量 CSV 写出器:列名一次写入,之后逐行追加。
pub struct CsvWriter<W: Write> {
    w: BufWriter<W>,
}

impl<W: Write> CsvWriter<W> {
    /// 绑定一个写出流并写入表头。
    pub fn new(w: W, headers: &[&str]) -> std::io::Result<Self> {
        let mut bw = BufWriter::new(w);
        let head = headers
            .iter()
            .map(|s| s.to_string())
            .collect::<Vec<_>>()
            .join(",");
        bw.write_all(head.as_bytes())?;
        bw.write_all(b"\n")?;
        Ok(Self { w: bw })
    }

    /// 写入一行(任意可 `Display` 的标量)。自动用逗号分隔。
    pub fn write_row<I, D>(&mut self, cols: I) -> std::io::Result<()>
    where
        I: IntoIterator<Item = D>,
        D: Display,
    {
        let mut first = true;
        for c in cols {
            if !first {
                self.w.write_all(b",")?;
            }
            self.w.write_all(c.to_string().as_bytes())?;
            first = false;
        }
        self.w.write_all(b"\n")?;
        Ok(())
    }

    /// 冲刷底层缓冲。
    pub fn flush(&mut self) -> std::io::Result<()> {
        self.w.flush()
    }
}

impl CsvWriter<File> {
    /// 便捷构造:直接打开一个文件写出。
    pub fn to_path<P: AsRef<Path>>(path: P, headers: &[&str]) -> std::io::Result<Self> {
        let f = File::create(path)?;
        Self::new(f, headers)
    }
}

/// 把任意可 `Display` 的切片铺平成 CSV 行所需的列向量。
pub fn flatten_display<D: Display>(items: &[D]) -> Vec<String> {
    items.iter().map(|x| x.to_string()).collect()
}
