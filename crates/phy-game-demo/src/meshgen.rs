//! 游戏体网格生成:把物理 shape 转为光栅化三角网格。
//!
//! 仅用于可视化(软件光栅化),与物理求解无关。复用 `phy_demo::raster::Tri` 原语。

use phy_demo::raster::Tri;
use phy_math::na::{Matrix4, Vector3};

/// 单位立方体(半边长 1)的 12 三角形(每面 2 个),带面法线。
pub fn box_mesh(hx: f32, hy: f32, hz: f32) -> Vec<Tri> {
    let c = [
        [-hx, -hy, -hz],
        [hx, -hy, -hz],
        [hx, hy, -hz],
        [-hx, hy, -hz],
        [-hx, -hy, hz],
        [hx, -hy, hz],
        [hx, hy, hz],
        [-hx, hy, hz],
    ];
    // 每面 (a,b,c,d) 逆时针(视外侧),拆成 (a,b,c)+(a,c,d)。
    let faces: [(usize, usize, usize, usize, [f32; 3]); 6] = [
        (0, 1, 2, 3, [0.0, 0.0, -1.0]), // 后
        (4, 5, 6, 7, [0.0, 0.0, 1.0]),  // 前
        (0, 4, 7, 3, [-1.0, 0.0, 0.0]), // 左
        (1, 5, 6, 2, [1.0, 0.0, 0.0]),  // 右
        (3, 2, 6, 7, [0.0, 1.0, 0.0]),  // 上
        (0, 1, 5, 4, [0.0, -1.0, 0.0]), // 下
    ];
    let mut tris = Vec::with_capacity(12);
    for (a, b, c2, d, n) in faces {
        let na = n;
        let nb = n;
        let nc = n;
        let nd = n;
        tris.push(Tri {
            p: [c[a], c[b], c[c2]],
            n: [na, nb, nc],
        });
        tris.push(Tri {
            p: [c[a], c[c2], c[d]],
            n: [na, nc, nd],
        });
    }
    tris
}

/// UV 球(半径 r,经度/纬度细分)网格。
pub fn sphere_mesh(r: f32, lat: usize, lon: usize) -> Vec<Tri> {
    let mut verts = Vec::new();
    for i in 0..=lat {
        let theta = std::f32::consts::PI * (i as f32 / lat as f32);
        let st = theta.sin();
        let ct = theta.cos();
        for j in 0..=lon {
            let phi = 2.0 * std::f32::consts::PI * (j as f32 / lon as f32);
            let x = r * st * phi.cos();
            let y = r * ct;
            let z = r * st * phi.sin();
            verts.push([x, y, z]);
        }
    }
    let idx = |i: usize, j: usize| -> usize { i * (lon + 1) + j };
    let mut tris = Vec::new();
    for i in 0..lat {
        for j in 0..lon {
            let a = verts[idx(i, j)];
            let b = verts[idx(i + 1, j)];
            let c = verts[idx(i + 1, j + 1)];
            let d = verts[idx(i, j + 1)];
            let na = norm(a);
            let nb = norm(b);
            let nc = norm(c);
            let nd = norm(d);
            tris.push(Tri {
                p: [a, b, c],
                n: [na, nb, nc],
            });
            tris.push(Tri {
                p: [a, c, d],
                n: [na, nc, nd],
            });
        }
    }
    tris
}

/// 胶囊近似网格:圆柱(中间)+ 上下两个半球帽。
/// 物理上胶囊 = 线段(±half_height 沿 y)两端接半径 r 的球;这里用细分球帽 + 圆柱近似。
pub fn capsule_mesh(half_height: f32, r: f32) -> Vec<Tri> {
    let mut tris = Vec::new();
    let lon = 16;
    let lat_cap = 8;
    // 上半球帽(中心在 +half_height)
    for i in 0..lat_cap {
        let t0 = std::f32::consts::FRAC_PI_2 * (i as f32 / lat_cap as f32);
        let t1 = std::f32::consts::FRAC_PI_2 * ((i + 1) as f32 / lat_cap as f32);
        for j in 0..lon {
            let p0 = j as f32 / lon as f32 * 2.0 * std::f32::consts::PI;
            let p1 = (j + 1) as f32 / lon as f32 * 2.0 * std::f32::consts::PI;
            let v = |t: f32, p: f32| -> [f32; 3] {
                [
                    r * t.sin() * p.cos(),
                    half_height + r * t.cos(),
                    r * t.sin() * p.sin(),
                ]
            };
            let a = v(t0, p0);
            let b = v(t1, p0);
            let c = v(t1, p1);
            let d = v(t0, p1);
            let na = norm(a);
            let nb = norm(b);
            let nc = norm(c);
            let nd = norm(d);
            tris.push(Tri { p: [a, b, c], n: [na, nb, nc] });
            tris.push(Tri { p: [a, c, d], n: [na, nc, nd] });
        }
    }
    // 下半球帽:纬度角 s 从 0(赤道)到 PI/2(南极),y 从 -half_height 降到 -half_height-r。
    for i in 0..lat_cap {
        let s0 = std::f32::consts::FRAC_PI_2 * (i as f32 / lat_cap as f32);
        let s1 = std::f32::consts::FRAC_PI_2 * ((i + 1) as f32 / lat_cap as f32);
        for j in 0..lon {
            let p0 = j as f32 / lon as f32 * 2.0 * std::f32::consts::PI;
            let p1 = (j + 1) as f32 / lon as f32 * 2.0 * std::f32::consts::PI;
            let v = |s: f32, p: f32| -> [f32; 3] {
                [
                    r * s.sin() * p.cos(),
                    -half_height - r * s.cos(),
                    r * s.sin() * p.sin(),
                ]
            };
            let a = v(s0, p0);
            let b = v(s1, p0);
            let c = v(s1, p1);
            let d = v(s0, p1);
            let na = norm(a);
            let nb = norm(b);
            let nc = norm(c);
            let nd = norm(d);
            tris.push(Tri { p: [a, b, c], n: [na, nb, nc] });
            tris.push(Tri { p: [a, c, d], n: [na, nc, nd] });
        }
    }
    // 圆柱侧壁
    for j in 0..lon {
        let p0 = j as f32 / lon as f32 * 2.0 * std::f32::consts::PI;
        let p1 = (j + 1) as f32 / lon as f32 * 2.0 * std::f32::consts::PI;
        let side = |y: f32, p: f32| -> [f32; 3] { [r * p.cos(), y, r * p.sin()] };
        let nrm = |p: f32| -> [f32; 3] { [p.cos(), 0.0, p.sin()] };
        let a = side(half_height, p0);
        let b = side(half_height, p1);
        let c = side(-half_height, p1);
        let d = side(-half_height, p0);
        let na = nrm(p0);
        let nb = nrm(p1);
        let nc = nrm(p1);
        let nd = nrm(p0);
        tris.push(Tri { p: [a, b, c], n: [na, nb, nc] });
        tris.push(Tri { p: [a, c, d], n: [na, nc, nd] });
    }
    tris
}

/// 构造平移+旋转(四元数)的模型矩阵(f64,供 draw_mesh 内部转 f32)。
pub fn model_matrix(pos: &Vector3<f64>, quat: &phy_math::na::UnitQuaternion<f64>) -> Matrix4<f64> {
    let rot = quat.to_homogeneous();
    let mut m = rot;
    m[(0, 3)] = pos.x;
    m[(1, 3)] = pos.y;
    m[(2, 3)] = pos.z;
    m
}

fn norm(v: [f32; 3]) -> [f32; 3] {
    let l = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt().max(1e-8);
    [v[0] / l, v[1] / l, v[2] / l]
}
