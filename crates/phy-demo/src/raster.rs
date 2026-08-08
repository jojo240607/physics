//! 极简软件光栅化器:把一个网格(三角形列表)按给定模型矩阵+视图投影画到帧缓冲。
//!
//! 功能:
//! - 透视正确的三角形插值(用 1/w)。
//! - 每像素 Z 缓冲。
//! - 对实例颜色做朗伯光照(世界空间方向光)。
//! 不依赖任何 GPU 后端,仅用 nalgebra 做矩阵运算,保证在纯 MinGW 工具链下可编译运行。

use phy_math::na::{Matrix4, Vector3, Vector4};

/// 帧缓冲:ARGB 像素 + 深度缓冲。
pub struct Framebuffer {
    pub width: u32,
    pub height: u32,
    pub pixels: Vec<u32>,
    zbuf: Vec<f32>,
}

impl Framebuffer {
    pub fn new(width: u32, height: u32) -> Self {
        let n = (width * height) as usize;
        Self {
            width,
            height,
            pixels: vec![0; n],
            zbuf: vec![f32::INFINITY; n],
        }
    }

    /// 清空为深蓝背景。
    pub fn clear(&mut self) {
        let bg = pack(0.05, 0.07, 0.10);
        for p in self.pixels.iter_mut() {
            *p = bg;
        }
        for z in self.zbuf.iter_mut() {
            *z = f32::INFINITY;
        }
    }

    fn put(&mut self, x: i32, y: i32, depth: f32, color: u32) {
        if x < 0 || y < 0 || x >= self.width as i32 || y >= self.height as i32 {
            return;
        }
        let idx = (y as u32 * self.width + x as u32) as usize;
        if depth < self.zbuf[idx] {
            self.zbuf[idx] = depth;
            self.pixels[idx] = color;
        }
    }

    /// 填充一个实心圆(屏幕坐标,带深度测试)。
    pub fn fill_circle(&mut self, cx: i32, cy: i32, r: i32, depth: f32, color: [u8; 3]) {
        if r <= 0 {
            return;
        }
        let c = pack(
            color[0] as f32 / 255.0,
            color[1] as f32 / 255.0,
            color[2] as f32 / 255.0,
        );
        let r2 = r * r;
        for y in cy - r..=cy + r {
            for x in cx - r..=cx + r {
                let dx = x - cx;
                let dy = y - cy;
                if dx * dx + dy * dy <= r2 {
                    self.put(x, y, depth, c);
                }
            }
        }
    }

    /// 直接按深度写一个像素(盒体填充用)。
    pub fn set_depth(&mut self, x: i32, y: i32, depth: f32, color: [u8; 3]) {
        let c = pack(
            color[0] as f32 / 255.0,
            color[1] as f32 / 255.0,
            color[2] as f32 / 255.0,
        );
        self.put(x, y, depth, c);
    }

    /// 画一条线段(屏幕坐标,带深度测试,深度取两端平均)。
    pub fn draw_line(
        &mut self,
        mut x0: i32,
        mut y0: i32,
        mut x1: i32,
        mut y1: i32,
        depth: f32,
        color: [u8; 3],
    ) {
        let c = pack(
            color[0] as f32 / 255.0,
            color[1] as f32 / 255.0,
            color[2] as f32 / 255.0,
        );
        let dx = (x1 - x0).abs();
        let dy = -(y1 - y0).abs();
        let sx = if x0 < x1 { 1 } else { -1 };
        let sy = if y0 < y1 { 1 } else { -1 };
        let mut err = dx + dy;
        loop {
            self.put(x0, y0, depth, c);
            if x0 == x1 && y0 == y1 {
                break;
            }
            let e2 = 2 * err;
            if e2 >= dy {
                err += dy;
                x0 += sx;
            }
            if e2 <= dx {
                err += dx;
                y0 += sy;
            }
        }
    }
}

/// 打包 f32 RGB(0..1) 为 0xAARRGGBB。
pub fn pack(r: f32, g: f32, b: f32) -> u32 {
    let c = |v: f32| -> u32 {
        let v = (v.clamp(0.0, 1.0) * 255.0).round() as u32;
        v & 0xFF
    };
    (0xFF << 24) | (c(r) << 16) | (c(g) << 8) | c(b)
}

/// 一个三角形:三个顶点(位置 + 法线,模型空间)。
pub struct Tri {
    pub p: [[f32; 3]; 3],
    pub n: [[f32; 3]; 3],
}

/// 把网格的所有三角形画到帧缓冲。
///
/// `view_proj` 为列主序 mat4(f32);`model` 为实例模型矩阵(f64,内部转 f32)。
/// `light` 为世界空间方向光方向(已归一化,指向光源)。
pub fn draw_mesh(
    fb: &mut Framebuffer,
    view_proj: &Matrix4<f32>,
    model: &Matrix4<f64>,
    tris: &[Tri],
    color: [f32; 4],
    light: &Vector3<f32>,
) {
    let m = model.cast::<f32>();
    // 取模型矩阵的上 3x3 作为法线变换(忽略平移/尺度对法线方向的影响,
    // 因均匀/非均匀缩放对法线方向在小角度下近似可接受;本 demo 仅视觉用途)。
    let rot = Matrix4::from_columns(&[
        m.column(0),
        m.column(1),
        m.column(2),
        Matrix4::<f32>::identity().column(3),
    ]);

    for tri in tris {
        // 裁剪空间顶点 = view_proj * model * p(齐次,保留 w 做透视除法)
        let mut clip = [Vector4::zeros(); 3];
        let mut world_n = [[0.0f32; 3]; 3];
        for i in 0..3 {
            let model_p = m * Vector4::new(tri.p[i][0], tri.p[i][1], tri.p[i][2], 1.0);
            clip[i] = *view_proj * model_p;
            let wn = rot.transform_vector(&Vector3::new(tri.n[i][0], tri.n[i][1], tri.n[i][2]));
            world_n[i] = [wn.x, wn.y, wn.z];
        }

        // 近裁剪:任一顶点在相机之后则跳过。
        if clip[0].w <= 1e-5 || clip[1].w <= 1e-5 || clip[2].w <= 1e-5 {
            continue;
        }

        // 透视除法 -> NDC -> 屏幕像素(y 翻转)
        let mut s = [[0.0f32; 2]; 3];
        let mut invw = [0.0f32; 3];
        for i in 0..3 {
            let inv = 1.0 / clip[i].w;
            invw[i] = inv;
            let ndc_x = clip[i].x * inv;
            let ndc_y = clip[i].y * inv;
            s[i][0] = (ndc_x * 0.5 + 0.5) * fb.width as f32;
            s[i][1] = (1.0 - (ndc_y * 0.5 + 0.5)) * fb.height as f32;
        }

        raster_triangle(fb, &s, &invw, &world_n, color, light);
    }
}

/// 透视正确的三角形光栅化(屏幕空间包围盒 + 边函数 + 1/w 插值)。
fn raster_triangle(
    fb: &mut Framebuffer,
    s: &[[f32; 2]; 3],
    invw: &[f32; 3],
    n: &[[f32; 3]; 3],
    color: [f32; 4],
    light: &Vector3<f32>,
) {
    let min_x = s[0][0].min(s[1][0]).min(s[2][0]).floor() as i32;
    let max_x = s[0][0].max(s[1][0]).max(s[2][0]).ceil() as i32;
    let min_y = s[0][1].min(s[1][1]).min(s[2][1]).floor() as i32;
    let max_y = s[0][1].max(s[1][1]).max(s[2][1]).ceil() as i32;

    let area = edge(s[0], s[1], s[2]);
    if area.abs() < 1e-6 {
        return;
    }

    for y in min_y..=max_y {
        for x in min_x..=max_x {
            let px = x as f32 + 0.5;
            let py = y as f32 + 0.5;
            let w0 = edge(s[1], s[2], [px, py]);
            let w1 = edge(s[2], s[0], [px, py]);
            let w2 = edge(s[0], s[1], [px, py]);
            let same = (w0 >= 0.0 && w1 >= 0.0 && w2 >= 0.0)
                || (w0 <= 0.0 && w1 <= 0.0 && w2 <= 0.0);
            if !same {
                continue;
            }
            let b0 = w0 / area;
            let b1 = w1 / area;
            let b2 = w2 / area;

            // 透视正确插值 1/w 与法线
            let iw = b0 * invw[0] + b1 * invw[1] + b2 * invw[2];
            let w = 1.0 / iw;
            let nx = (b0 * n[0][0] * invw[0] + b1 * n[1][0] * invw[1] + b2 * n[2][0] * invw[2]) * w;
            let ny = (b0 * n[0][1] * invw[0] + b1 * n[1][1] * invw[1] + b2 * n[2][1] * invw[2]) * w;
            let nz = (b0 * n[0][2] * invw[0] + b1 * n[1][2] * invw[1] + b2 * n[2][2] * invw[2]) * w;
            let nl = (nx * nx + ny * ny + nz * nz).sqrt().max(1e-6);
            let nx = nx / nl;
            let ny = ny / nl;
            let nz = nz / nl;

            // 深度用插值后的 1/w(越小越近),便于 Z 缓冲比较。
            let depth = iw;

            let diff = (nx * light.x + ny * light.y + nz * light.z).max(0.0);
            let ambient = 0.25;
            let r = color[0] * (ambient + 0.75 * diff);
            let g = color[1] * (ambient + 0.75 * diff);
            let b = color[2] * (ambient + 0.75 * diff);
            fb.put(x, y, depth, pack(r, g, b));
        }
    }
}

/// 2D 边函数(叉积符号 = 半平面)。
fn edge(a: [f32; 2], b: [f32; 2], c: [f32; 2]) -> f32 {
    (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
}
