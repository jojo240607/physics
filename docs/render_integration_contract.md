# 物理 ↔ 渲染集成契约

> 姊妹文档：[public_module_roadmap.md](./public_module_roadmap.md) → P1-a。
> 目标：固化物理侧与渲染侧之间的**坐标、矩阵内存布局、背面剔除、单位**约定，避免每个游戏/可视化接入方重复踩坑。
> 本文档的约定基于 `phy-game-demo`（wgpu 渲染路径）在 `wfix` 分支修复的两类真实 bug：① 模型矩阵被隐式行/列转置导致物体塌缩到原点呈放射状；② 背面剔除在相机转 180° 时错误剔除正面造成"穿模"假象。

---

## 1. 坐标系约定（强制）

| 项 | 约定 | 备注 |
|---|---|---|
| 轴向 | **右手系，Y 轴向上** | 物理世界、meshgen、角色控制器均一致（up = +y） |
| 单位 | **米 (m) / 秒 (s) / 千克 (kg)** | 不在物理层缩放；渲染侧缩放因子由业务方自行处理 |
| 朝向 | 四元数 `quat` 表示 body→world 旋转 | 经 `quat.to_homogeneous()` 得到旋转部分 |

> ⚠️ 渲染侧若使用 Z-up（如某些 DCC 工具）或左手系（如 Unity/Unreal 默认），**必须在业务侧做一次显式轴向转换**，物理层不提供转换。

---

## 2. 变换矩阵内存布局（强制）

### 2.1 列主序是唯一的真相

- 物理侧 `phy_math::na::Matrix4`（nalgebra）**列主序存储**：`m[(row, col)]`，底层按列连续排列 16 个标量。
- 渲染侧 WGSL `mat4x4<f32>` / `mat4x4<f64>` **同样列主序**。
- 因此：**实例缓冲里的模型矩阵必须按"列连续"铺开 16 个 f32**，即第 0..4 个是第 0 列（含平移 x 在索引 3），第 4..8 个是第 1 列（含平移 y 在索引 7），以此类推。

> ⚠️ **历史 bug**：`phy-game-demo` 曾按"行连续"铺开模型矩阵，导致 GPU 拿到的矩阵是物理矩阵的**转置**——物体绕原点旋转放大呈放射状塌缩。只可按列主序铺开。

### 2.2 模型矩阵构造

meshgen 提供 `model_matrix(pos, quat)` 返回 nalgebra `Matrix4<f64>`（列主序，平移写入 `m[(0,3)]/m[(1,3)]/m[(2,3)]`）。渲染侧取其 `.as_slice()` 或逐列展开即可，**不要**对结果做 `transpose()`。

```rust
// 物理侧（正确）
let m = model_matrix(&pos, &quat);          // 列主序 Matrix4
instance.model = m;                          // 直接上传，勿转置
```

### 2.3 每 body 独立缓冲（避免覆盖）

多 body 渲染时，**每个 body 使用独立的顶点/实例缓冲**，或在统一缓冲里按 `body_index` 分片写入后一次性 `write_buffer`。切勿在共用缓冲上每帧覆盖同一段——`phy-game-demo` 早期曾因共用缓冲被后续 body 覆盖而只渲染最后一个物体。

---

## 3. 背面剔除约定（强制）

> ⚠️ **历史 bug**：`phy-game-demo` 开启 `cull_mode: Some(Face::Back)` + `front_face: Ccw` 时，相机转到 180° 出现"穿模"——盒子被看穿、露出后方物体。

### 3.1 根因

meshgen 生成的封闭凸体绕序**并非全部严格外 CCW**：
- 立方体 6 面中，部分面（如"前"面顶点序 `4→5→6` 从外侧 +z 看为顺时针）绕序与注释声称的"逆时针"相反；
- UV 球 / 胶囊的经纬度网格上下半球绕序也可能不一致。

开启 Back 剔除后，当这些绕序相反的面正对相机时，被误判为"背面"剔除 → 看穿。

### 3.2 强制约定（二选一，推荐前者）

**方案 A（推荐，已落地 `render_wgpu.rs`）**：关闭背面剔除。

```rust
primitive: wgpu::PrimitiveState {
    topology: wgpu::PrimitiveTopology::TriangleList,
    cull_mode: None,                 // ← 封闭凸体配合深度写入即可正确遮挡
    front_face: wgpu::FrontFace::Ccw,
    ..Default::default()
}
```

理由：盒/球/胶囊均为封闭凸体，开启深度写入（`depth_write_enabled: true` + `Less` 比较）后，关剔除的遮挡结果完全正确，多画的背面三角形代价可忽略。

**方案 B（更高像素效率，需先修正绕序）**：保留 `cull_mode: Back`，但必须先把 meshgen 全部面的绕序修正为严格外 CCW（逐面核对 6 个面 + 球/胶囊经纬网格上下半球）。风险较高，当前版本不推荐。

---

## 4. 投影与深度（约定）

| 项 | 约定 |
|---|---|
| 近/远平面 | `(0.1, 2000.0)`（wgpu demo 默认） |
| 投影矩阵 z 映射 | OpenGL 风格 `[-1, 1]`，**需经校正矩阵映射到 WebGPU/D3D 的 `[0, 1]`**（见 `phy-demo/src/camera.rs` 的 `correction`） |
| 深度比较 | `Less`（近处覆盖远处） |
| 深度写入 | 不透明物体 `true`；透明物体另议 |

> 渲染侧若使用 OpenGL（而非 WebGPU/D3D），**不要**叠加 `[0,1]` 校正矩阵，否则深度错误。

---

## 5. 接入检查清单（渲染侧）

- [ ] 模型矩阵按**列主序**上传到实例缓冲，未做 transpose
- [ ] 多 body 使用独立缓冲或按索引分片写入，无覆盖
- [ ] 背面剔除设为 `None`（或已修正全部绕序为外 CCW）
- [ ] 投影矩阵 z 映射按目标后端（WebGPU/D3D 需 `[0,1]` 校正，OpenGL 不需要）正确配置
- [ ] 物理世界 Y-up，渲染侧若 Z-up/左手系已做轴向转换
- [ ] 单位统一为米，渲染侧缩放不在物理层进行

---

## 6. 参考实现

| 文件 | 角色 |
|---|---|
| `crates/phy-game-demo/src/meshgen.rs` | `box_mesh` / `sphere_mesh` / `capsule_mesh` / `model_matrix` |
| `crates/phy-game-demo/src/render_wgpu.rs` | wgpu 管线（`cull_mode: None` + 列主序实例缓冲） |
| `crates/phy-demo/src/camera.rs` | `proj()` / `view_proj()` / z 校正矩阵 |
