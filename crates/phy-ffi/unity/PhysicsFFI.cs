// PhysicsFFI.cs — Unity (C#) P/Invoke 绑定 for phy-ffi (ABI v2).
//
// 用法:
//   1. 把 phy_ffi.dll 放到 Unity 工程的 Assets/Plugins/x86_64/ 下。
//   2. 把 phy_ffi.lib 用于 MSVC 链接(或 libphy_ffi.dll.a 用于 MinGW);Unity 用 MSVC,
//      直接 [DllImport("phy_ffi")] 即可,Unity 会在 Plugins 目录找到 dll。
//   3. 挂载本脚本到一个 GameObject,在 Start() 里 phy_ffi_abi_version() 核对契约,
//      然后 phy_world_create_rigid() 建世界、在 Update() 里 phy_world_step(),
//      用 phy_world_get_rigid_transforms 把位姿刷到 Mesh/Transform。
//
// 注意:
//   - 所有 `double`(f64) 通道与 Rust 侧一致;游戏侧若用 f32,改用 _f32 系列函数。
//   - 句柄是裸指针,务必在 OnDestroy 里 phy_world_destroy 释放,避免泄漏。
//   - ABI 版本不符(fail-fast)时拒绝加载,避免调用到签名已变更的 phy_* 符号。

using System;
using System.Runtime.InteropServices;

namespace PhysicsEngine
{
    public static class PhysicsFFI
    {
        // —— 加载契约 ——
        [DllImport("phy_ffi")] public static extern uint phy_ffi_abi_version();

        // —— 场景工厂 ——
        [DllImport("phy_ffi")] public static extern IntPtr phy_world_create_fluid();
        [DllImport("phy_ffi")] public static extern IntPtr phy_world_create_rigid();
        [DllImport("phy_ffi")] public static extern IntPtr phy_world_create_rigid_empty();
        [DllImport("phy_ffi")] public static extern IntPtr phy_world_create_granular();
        [DllImport("phy_ffi")] public static extern IntPtr phy_world_create_coupled();

        // —— 步进 ——
        [DllImport("phy_ffi")] public static extern int phy_world_step(IntPtr w, double dt);
        [DllImport("phy_ffi")] public static extern int phy_world_step_checked(IntPtr w, double dt);
        [DllImport("phy_ffi")] public static extern int phy_world_step_f32(IntPtr w, float dt);

        // —— 查询 ——
        [DllImport("phy_ffi")] public static extern double phy_world_time(IntPtr w);
        [DllImport("phy_ffi")] public static extern UIntPtr phy_world_sub_count(IntPtr w);
        [DllImport("phy_ffi")] public static extern UIntPtr phy_world_fluid_count(IntPtr w);
        [DllImport("phy_ffi")] public static extern UIntPtr phy_world_rigid_count(IntPtr w);

        // —— 读回(f64 通道;buf 由调用方分配,长度 = count * stride)——
        [DllImport("phy_ffi")]
        public static extern UIntPtr phy_world_get_fluid_positions(IntPtr w, double[] buf, UIntPtr len);
        [DllImport("phy_ffi")]
        public static extern UIntPtr phy_world_get_fluid_velocities(IntPtr w, double[] buf, UIntPtr len);
        [DllImport("phy_ffi")]
        public static extern UIntPtr phy_world_get_rigid_transforms(IntPtr w, double[] buf, UIntPtr len);
        [DllImport("phy_ffi")]
        public static extern UIntPtr phy_world_get_fluid_positions_f32(IntPtr w, float[] buf, UIntPtr len);
        [DllImport("phy_ffi")]
        public static extern UIntPtr phy_world_get_rigid_transforms_f32(IntPtr w, float[] buf, UIntPtr len);

        // —— 刚体增删 / 受力 ——
        // shape_kind: 0=Sphere(半径取 inertia3[0]),1=Box(半长取 inertia3[0..2])
        // mass: kg;0 表示静态。pos7 = [px,py,pz, qw,qi,qj,qk];inertia3 = [Ixx,Iyy,Izz]
        [DllImport("phy_ffi")]
        public static extern long phy_world_rigid_add_body(
            IntPtr w, int shape_kind, double mass, double[] pos7, double[] inertia3);
        [DllImport("phy_ffi")]
        public static extern int phy_world_rigid_apply_force(
            IntPtr w, long id, double[] f3, double dt, int mode);
        [DllImport("phy_ffi")]
        public static extern int phy_world_rigid_apply_torque(
            IntPtr w, long id, double[] t3, double dt, int mode);
        [DllImport("phy_ffi")]
        public static extern int phy_world_rigid_get_velocity(IntPtr w, long id, double[] out3);
        [DllImport("phy_ffi")]
        public static extern int phy_world_rigid_get_angular_velocity(IntPtr w, long id, double[] out3);

        // —— 序列化 ——
        [DllImport("phy_ffi")]
        public static extern int phy_world_save(IntPtr w,
            [MarshalAs(UnmanagedType.LPStr)] string path);
        [DllImport("phy_ffi")]
        public static extern IntPtr phy_world_load(
            [MarshalAs(UnmanagedType.LPStr)] string path);

        // —— 释放 ——
        [DllImport("phy_ffi")] public static extern void phy_world_destroy(IntPtr w);
    }

    // 刚体位姿:pos.xyz + quat.wijk(7 个 double 交错)。
    public struct RigidTransform
    {
        public double px, py, pz;
        public double qw, qi, qj, qk;

        public static RigidTransform FromBuf(double[] buf, int body)
        {
            int o = body * 7;
            return new RigidTransform
            {
                px = buf[o], py = buf[o + 1], pz = buf[o + 2],
                qw = buf[o + 3], qi = buf[o + 4], qj = buf[o + 5], qk = buf[o + 6],
            };
        }
    }
}
