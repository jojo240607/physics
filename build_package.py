#!/usr/bin/env python3
"""Build & package the physics engine for downstream consumers.

Produces a redistributable under `pkg/<profile>/` containing:
  * C dynamic library (cdylib):  phy_ffi.dll / libphy_ffi.so / libphy_ffi.dylib
  * Import library (Windows):    libphy_ffi.dll.a  (MinGW) / phy_ffi.lib (MSVC)
  * C header (cbindgen-generated): phy_ffi.h
  * Rust rlib (for Rust consumers): libphy_ffi.rlib
  * A short USAGE note.

Subcommands:
    python3 build_package.py              # release build + package (default)
    python3 build_package.py --debug       # debug profile
    python3 build_package.py --check       # also run workspace tests before packaging
    python3 build_package.py --no-header   # skip cbindgen header (re)generation
    python3 build_package.py --target x86_64-pc-windows-msvc   # cross/alternate target

C header generation requires cbindgen (already a build-dep of phy-ffi); it is
regenerated via `PHY_FFI_GEN_HEADER=1 cargo build -p phy-ffi` so the header
always matches the current FFI surface.

This script is host-only (desktop C ABI). For the Web/WASM build, see
wasm-cross-check.py (phy-ffi is intentionally excluded there).
"""
import os
import shutil
import subprocess
import sys

ROOT = os.path.dirname(os.path.abspath(__file__))
PKG_ROOT = os.path.join(ROOT, "pkg")

PROFILE = "release"
CHECK = False
REGEN_HEADER = True
TARGET = None  # None => host default triple


def run(cmd, env=None):
    print("+ " + " ".join(cmd), flush=True)
    rc = subprocess.call(cmd, env=env, cwd=ROOT)
    if rc != 0:
        print(f"FAILED (exit {rc}): {' '.join(cmd)}", file=sys.stderr, flush=True)
        sys.exit(rc)


def detect_lib_ext(target):
    """Return (cdylib_ext, importlib_name_or_None) for the given target."""
    if target is None:
        # Best-effort host detection.
        if sys.platform == "win32":
            return ".dll", "libphy_ffi.dll.a"
        if sys.platform == "darwin":
            return ".dylib", None
        return ".so", None
    if "windows" in target:
        return ".dll", "libphy_ffi.dll.a"
    if "apple" in target or "darwin" in target:
        return ".dylib", None
    return ".so", None


def main():
    global PROFILE, CHECK, REGEN_HEADER, TARGET
    for a in sys.argv[1:]:
        if a == "--debug":
            PROFILE = "debug"
        elif a == "--check":
            CHECK = True
        elif a == "--no-header":
            REGEN_HEADER = False
        elif a.startswith("--target="):
            TARGET = a.split("=", 1)[1]
        else:
            print(f"unknown arg: {a}", file=sys.stderr)
            sys.exit(2)

    if shutil.which("cargo") is None:
        print("cargo not found on PATH", file=sys.stderr)
        sys.exit(1)

    if TARGET:
        installed = subprocess.run(
            ["rustup", "target", "list", "--installed"],
            capture_output=True, text=True,
        ).stdout
        if TARGET not in installed:
            run(["rustup", "target", "add", TARGET])

    target_args = ["--target", TARGET] if TARGET else []
    profile_args = ["--profile", PROFILE] if PROFILE == "release" else []

    # 0) Optional pre-flight test gate.
    if CHECK:
        run(["cargo", "test", "--workspace", *target_args])
        run(["cargo", "test", "--release", "-p", "phy-demo", "--", "--ignored"])

    # 1) Regenerate the C header so it matches the current FFI surface.
    if REGEN_HEADER:
        env = dict(os.environ)
        env["PHY_FFI_GEN_HEADER"] = "1"
        run(["cargo", "build", "-p", "phy-ffi", *profile_args, *target_args], env=env)

    # 2) Build the full FFI crate (header step above already built it; this is a
    #    no-op rebuild unless artifacts were cleaned, but keeps the chain explicit).
    run(["cargo", "build", "-p", "phy-ffi", *profile_args, *target_args])

    # 3) Collect artifacts into pkg/<profile>/.
    out_dir = os.path.join(PKG_ROOT, PROFILE)
    os.makedirs(out_dir, exist_ok=True)

    cdylib_ext, importlib_name = detect_lib_ext(TARGET)

    # Locate the built cdylib + rlib in the cargo target dir.
    target_dir = os.path.join(ROOT, "target")
    if TARGET:
        artifact_dir = os.path.join(target_dir, TARGET, PROFILE)
    else:
        artifact_dir = os.path.join(target_dir, PROFILE)

    cdylib_name = "phy_ffi" + cdylib_ext
    rlib_name = "libphy_ffi.rlib"
    src_cdylib = os.path.join(artifact_dir, cdylib_name)
    src_rlib = os.path.join(artifact_dir, rlib_name)
    src_header = os.path.join(ROOT, "crates", "phy-ffi", "phy_ffi.h")

    copied = []
    if os.path.exists(src_cdylib):
        shutil.copy2(src_cdylib, os.path.join(out_dir, cdylib_name))
        copied.append(cdylib_name)
    if importlib_name and os.path.exists(os.path.join(artifact_dir, importlib_name)):
        shutil.copy2(os.path.join(artifact_dir, importlib_name),
                     os.path.join(out_dir, importlib_name))
        copied.append(importlib_name)
    if os.path.exists(src_rlib):
        shutil.copy2(src_rlib, os.path.join(out_dir, rlib_name))
        copied.append(rlib_name)
    if os.path.exists(src_header):
        shutil.copy2(src_header, os.path.join(out_dir, "phy_ffi.h"))
        copied.append("phy_ffi.h")

    # 4) Write a short USAGE note next to the artifacts.
    usage = f"""# Physics Engine — redistributable package ({PROFILE})

Built: {PROFILE} profile{(' for target ' + TARGET) if TARGET else ''}.

## Contents
  phy_ffi.h            C/C++/Unity/Unreal header (cbindgen-generated, do not edit by hand)
  phy_ffi{ cdylib_ext }            Dynamic library (link at runtime / load via dlopen)
  libphy_ffi.dll.a     MinGW import library (Windows, if present)
  libphy_ffi.rlib      Rust static lib (for Rust consumers: `phy-ffi` as a normal crate dep)
  phy_ffi.lib          MSVC import library (Windows, if present)

## C/C++/Unity/Unreal
  1. Include `phy_ffi.h`.
  2. Link against the dynamic library (add the .dll to your runtime PATH / bundle it).
  3. Call `phy_world_create_*`, step with `phy_world_step` / `phy_world_step_checked`,
     pull state with `phy_world_get_fluid_positions` / `phy_world_get_rigid_transforms`,
     and free with `phy_world_destroy`.

## Rust
  Add `phy-ffi` (or any sub-crate like `phy-core`, `phy-rigid`) as a normal dependency;
  the rlib is provided for offline / vendored builds.

Regenerate the header after any FFI change:
  PHY_FFI_GEN_HEADER=1 cargo build -p phy-ffi
"""
    with open(os.path.join(out_dir, "USAGE.md"), "w", encoding="utf-8") as f:
        f.write(usage)
    copied.append("USAGE.md")

    print(f"\nPackage ready in: {out_dir}")
    for c in copied:
        print(f"  + {c}")
    if not copied:
        print("  (warning) no artifacts copied — check the build output above.")
        sys.exit(1)


if __name__ == "__main__":
    main()
