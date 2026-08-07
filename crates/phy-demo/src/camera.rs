//! 相机:轨道视角(azimuth/pitch/distance)+ 透视投影。
//!
//! 提供 `view_proj()` 输出可直接上传给 wgpu 的列主序 `mat4`
//! (nalgebra 的 `Matrix4` 已是列主序,且这里把 OpenGL 风格裁剪空间
//!  z∈[-1,1] 校正到 wgpu/D3D 的 z∈[0,1])。

use phy_math::na::{Matrix4, Point3, Vector3};

/// 轨道相机。
pub struct Camera {
    /// 注视点(世界)。
    pub target: Point3<f32>,
    /// 相机到注视点的距离。
    pub distance: f32,
    /// 方位角(yaw,绕 Y 轴)。
    pub yaw: f32,
    /// 俯仰角(pitch,相对水平面)。
    pub pitch: f32,
    /// 垂直视场角(弧度)。
    pub fov: f32,
}

impl Default for Camera {
    fn default() -> Self {
        Self {
            target: Point3::new(0.0, 1.0, 0.0),
            distance: 28.0,
            yaw: 0.6,
            pitch: 0.45,
            fov: std::f32::consts::FRAC_PI_4,
        }
    }
}

impl Camera {
    /// 视图矩阵(右手坐标系,看向 -Z)。
    pub fn view(&self) -> Matrix4<f32> {
        let cp = self.pitch.cos();
        let dir = Vector3::new(
            self.yaw.sin() * cp,
            self.pitch.sin(),
            self.yaw.cos() * cp,
        );
        let eye = self.target + dir * self.distance;
        Matrix4::look_at_rh(&eye, &self.target, &Vector3::y())
    }

    /// 透视投影矩阵(OpenGL 风格 z∈[-1,1])。
    pub fn proj(&self, aspect: f32) -> Matrix4<f32> {
        let p = Matrix4::new_perspective(aspect, self.fov, 0.1, 2000.0);
        // OpenGL [-1,1] -> wgpu / D3D [0,1]
        let correction = Matrix4::new(
            1.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 0.5, 0.5,
            0.0, 0.0, 0.0, 1.0,
        );
        correction * p
    }

    /// 视图-投影组合矩阵(列主序,可直接上传)。
    pub fn view_proj(&self, aspect: f32) -> Matrix4<f32> {
        self.proj(aspect) * self.view()
    }
}
