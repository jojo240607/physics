//! build.rs — 仅在 `PHY_FFI_GEN_HEADER=1` 时调用 cbindgen 重新生成 `phy_ffi.h`。
//! 常规 `cargo build` 不触发,避免把 cbindgen 编译开销加进每次构建。

fn main() {
    // 声明环境变量依赖:设置/清除 PHY_FFI_GEN_HEADER 时必须重跑 build script,
    // 否则 cargo 会缓存旧产物导致头文件陈旧(之前曾因此让打包脚本读到缺符号的旧头)。
    println!("cargo:rerun-if-env-changed=PHY_FFI_GEN_HEADER");

    // 默认始终生成 C 头文件,保证头与源码同步(避免陈旧头陷阱)。仅当显式
    // `PHY_FFI_GEN_HEADER=0` 时跳过(极 rare 用例,例如想冻结头)。
    let gen = std::env::var("PHY_FFI_GEN_HEADER").map(|v| v != "0").unwrap_or(true);
    if gen {
        let crate_dir = std::env::var("CARGO_MANIFEST_DIR").unwrap();
        let config = cbindgen::Config::from_file(format!("{}/cbindgen.toml", crate_dir))
            .expect("读取 cbindgen.toml 失败");
        cbindgen::Builder::new()
            .with_crate(&crate_dir)
            .with_config(config)
            .generate()
            .expect("cbindgen 生成失败")
            .write_to_file(format!("{}/phy_ffi.h", crate_dir));
    }
}
