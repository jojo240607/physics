#!/usr/bin/env python3
"""Cross-compile the Web-relevant crates to wasm32-unknown-unknown.

This guards the claims made in PLAN.md:
  * S8 / L2: `phy-core::replay` is wasm32-compatible (no `std`-only deps).
  * W7   : `phy-demo-web` builds for wasm32 (default + `gpu` feature).

`phy-ffi` is a `cdylib` and intentionally excluded (desktop-only C ABI, L3).

Usage:
    python3 wasm-cross-check.py            # default-feature builds
    python3 wasm-cross-check.py --gpu      # also build phy-demo-web --features gpu
    python3 wasm-cross-check.py --all      # both of the above

Requires: `rustup target add wasm32-unknown-unknown` (already installed in CI/dev).
"""
import subprocess
import sys
import shutil


TARGET = "wasm32-unknown-unknown"


def run(cmd):
    print("+ " + " ".join(cmd), flush=True)
    rc = subprocess.call(cmd)
    if rc != 0:
        print(f"FAILED (exit {rc}): {' '.join(cmd)}", file=sys.stderr, flush=True)
        sys.exit(rc)


def main():
    include_gpu = "--gpu" in sys.argv or "--all" in sys.argv

    if shutil.which("cargo") is None:
        print("cargo not found on PATH", file=sys.stderr)
        sys.exit(1)

    # Ensure the target is installed.
    installed = subprocess.run(
        ["rustup", "target", "list", "--installed"],
        capture_output=True, text=True,
    ).stdout
    if TARGET not in installed:
        run(["rustup", "target", "add", TARGET])

    # 1) phy-core: the deterministic-replay / World core must be wasm32-clean.
    run(["cargo", "build", "-p", "phy-core", "--target", TARGET])

    # 2) phy-demo-web: the Web consumer (default features).
    run(["cargo", "build", "-p", "phy-demo-web", "--target", TARGET])

    # 3) phy-demo-web with the gpu feature (W1-W7 WebGPU backend).
    if include_gpu:
        run(["cargo", "build", "-p", "phy-demo-web",
             "--target", TARGET, "--features", "gpu"])

    print("\nwasm32 cross-compile OK: phy-core, phy-demo-web"
          + (" (+gpu)" if include_gpu else ""))


if __name__ == "__main__":
    main()
