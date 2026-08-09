#!/usr/bin/env python3
"""用 --features gpu 重建 Web Demo 的 wasm 包(含 W1-W5 GPU 自测)。

普通 `wasm-pack build` 不含 gpu feature(自测方法不会出现在 pkg 里)。
本脚本产出带 WebGPU 后端的 pkg/,配合 index.html 的 GPU 自测面板使用。

前置:
    - wasm-pack (cargo install wasm-pack)
    - 目标 wasm32-unknown-unknown 已加 (`rustup target add wasm32-unknown-unknown`)
    - 本机有支持 WebGPU 的浏览器(Chrome/Edge 113+)才能实际跑自测;
      无 GPU 时 `GpuContext::init` 会返回错误,面板会打印失败信息(不崩)。

用法:
    python build_gpu.py            # 重建 pkg/(--features gpu --target web)
    python build_gpu.py --release  # release 构建
"""
import subprocess
import sys
import os

ROOT = os.path.dirname(os.path.abspath(__file__))


def main():
    release = "--release" in sys.argv
    cmd = [
        "wasm-pack",
        "build",
        "--target",
        "web",
        "--features",
        "gpu",
    ]
    if release:
        cmd.append("--release")
    print(f"[build_gpu] {' '.join(cmd)}")
    # wasm-pack 在 crate 目录内执行。
    rc = subprocess.call(cmd, cwd=ROOT)
    if rc != 0:
        print("[build_gpu] 构建失败")
        sys.exit(rc)
    print("[build_gpu] 完成 -> pkg/ (含 gpu_self_test/optic_self_test/"
          "caustics_self_test/sph_self_test/granular_self_test)")


if __name__ == "__main__":
    main()
