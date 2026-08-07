//! 单位网格(立方体 / 球)的三角形列表,供软件光栅化器使用。

use crate::raster::Tri;

/// 单位立方体(顶点范围 [-0.5,0.5])的 12 个三角形。法线朝外。
pub fn cube_tris() -> Vec<Tri> {
    let h = 0.5_f32;
    // 8 个角
    let c = [
        [-h, -h, -h],
        [h, -h, -h],
        [h, h, -h],
        [-h, h, -h],
        [-h, -h, h],
        [h, -h, h],
        [h, h, h],
        [-h, h, h],
    ];
    // 6 面,每面两个三角形,法线为面朝向
    let faces: [(usize, usize, usize, usize, [f32; 3]); 6] = [
        (0, 1, 2, 3, [0.0, 0.0, -1.0]), // -Z
        (4, 5, 6, 7, [0.0, 0.0, 1.0]),  // +Z
        (1, 5, 6, 2, [1.0, 0.0, 0.0]),  // +X
        (0, 4, 7, 3, [-1.0, 0.0, 0.0]), // -X
        (3, 2, 6, 7, [0.0, 1.0, 0.0]),  // +Y
        (0, 1, 5, 4, [0.0, -1.0, 0.0]), // -Y
    ];
    let mut tris = Vec::with_capacity(12);
    for (a, b, c2, d, n) in faces {
        tris.push(Tri {
            p: [c[a], c[b], c[c2]],
            n: [n, n, n],
        });
        tris.push(Tri {
            p: [c[a], c[c2], c[d]],
            n: [n, n, n],
        });
    }
    tris
}

/// 单位球(半径 1)的三角形列表(UV 球)。法线 = 顶点方向。
pub fn sphere_tris(stacks: u32, slices: u32) -> Vec<Tri> {
    let mut verts = Vec::new();
    for i in 0..=stacks {
        let phi = std::f32::consts::PI * (i as f32 / stacks as f32);
        for j in 0..=slices {
            let theta = 2.0 * std::f32::consts::PI * (j as f32 / slices as f32);
            let x = phi.sin() * theta.cos();
            let y = phi.cos();
            let z = phi.sin() * theta.sin();
            verts.push(([x, y, z], [x, y, z]));
        }
    }
    let row = slices + 1;
    let idx = |i: u32, j: u32| -> usize { (i * row + j) as usize };
    let mut tris = Vec::new();
    for i in 0..stacks {
        for j in 0..slices {
            let a = idx(i, j);
            let b = idx(i + 1, j);
            let c = idx(i, j + 1);
            let d = idx(i + 1, j + 1);
            tris.push(Tri {
                p: [verts[a].0, verts[b].0, verts[c].0],
                n: [verts[a].1, verts[b].1, verts[c].1],
            });
            tris.push(Tri {
                p: [verts[c].0, verts[b].0, verts[d].0],
                n: [verts[c].1, verts[b].1, verts[d].1],
            });
        }
    }
    tris
}
