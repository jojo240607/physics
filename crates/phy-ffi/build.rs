//! build.rs — 仅在 `PHY_FFI_GEN_HEADER=1` 时调用 cbindgen 重新生成 `phy_ffi.h`。
//! 常规 `cargo build` 不触发,避免把 cbindgen 编译开销加进每次构建。

fn main() {
    if std::env::var("PHY_FFI_GEN_HEADER").is_ok() {
        let crate_dir = std::env::var("CARGO_MANIFEST_DIR").unwrap();
        let config = cbindgen::Config::from_file(format!("{}/cbindgen.toml", crate_dir))
            .expect("读取 cbindgen.toml 失败");
        cbindgen::Builder::new()
            .with_crate(&crate_dir)
            .with_config(config)
            .generate()
            .expect("cbindgen 生成失败")
            .write_to_file(format!("{}/phy_ffi.h", crate_dir));
        println!("cargo:warning=phy_ffi.h 已重新生成");
    }
}
