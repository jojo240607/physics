//! phy-demo 库目标:导出场景/光栅化/相机模块,供桌面二进制、Web 版本与测试复用。

pub mod camera;
pub mod raster;
pub mod scene;

pub use camera::Camera;
pub use raster::Framebuffer;
pub use scene::{DemoMode, Scene};
