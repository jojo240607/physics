#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
M1 真实 WebGPU adapter 验收脚本(Playwright 驱动真实浏览器)。

为什么需要这个:
  M1 的目标是"真实 WebGPU adapter 验收"——即 demo 的 WebGPU compute 路径必须在
  一个真实的 WebGPU adapter 上跑通(而非占位/模拟)。本机开发沙箱(Windows/MinGW)
  无法链接原生 wgpu(需要 MSVC 的 D3D12)且自带的 Chromium 未编译 WebGPU,因此
  "真实 adapter 运行"这一步必须在**带 WebGPU 的浏览器**里执行。本脚本即该验收。

验收内容:
  1) adapter_info():向浏览器请求真实 WebGPU adapter(不强加 fallback),并读取其
     vendor / device / architecture / description —— 证明"真实 adapter 被接受"。
  2) W1..W5 自测:分别调用 wasm 暴露的 gpu_self_test / optic_self_test /
     caustics_self_test / sph_self_test / granular_self_test,这些函数内部会
     用 demo 实际发布的 WGSL 在真实 adapter 上创建 ShaderModule / ComputePipeline
     并执行,返回结果字符串。任一失败即验收失败。

用法:
  # 先构建带 gpu feature 的 wasm pkg(已生成 pkg/)
  python build_gpu.py
  # 在带 WebGPU 的浏览器环境执行(Chrome/Edge 113+,或开启 WebGPU 的 Chromium)
  python m1_webgpu_accept.py
  # 可指定浏览器: --browser chromium (默认) | msedge
  # 可指定可执行路径: --executable "C:/path/to/chrome.exe"

退出码:0=验收通过, 1=失败。
"""
import argparse
import json
import sys
import threading
import http.server
import socketserver
import os

from playwright.sync_api import sync_playwright

HERE = os.path.dirname(os.path.abspath(__file__))
PKG_DIR = os.path.join(HERE, "pkg")


class QuietHandler(http.server.SimpleHTTPRequestHandler):
    def log_message(self, *args, **kwargs):  # 静默访问日志
        pass


def serve():
    os.chdir(HERE)
    handler = QuietHandler
    httpd = socketserver.TCPServer(("127.0.0.1", 0), handler)
    port = httpd.server_address[1]
    t = threading.Thread(target=httpd.serve_forever, daemon=True)
    t.start()
    return httpd, port


def launch_browser(p, browser, executable):
    args = [
        "--enable-unsafe-webgpu",
        "--enable-features=Vulkan,WebGPU",
        "--ignore-gpu-blocklist",
    ]
    if browser == "msedge":
        return p.chromium.launch(channel="msedge", headless=False, args=args)
    if executable:
        return p.chromium.launch(executable_path=executable, headless=False, args=args)
    return p.chromium.launch(headless=False, args=args)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--browser", default="chromium", choices=["chromium", "msedge"])
    ap.add_argument("--executable", default=None, help="显式指定浏览器可执行路径")
    args = ap.parse_args()

    if not os.path.isdir(PKG_DIR):
        print("ERROR: pkg/ 不存在,请先运行 `python build_gpu.py` 生成带 gpu 的 wasm 包。")
        sys.exit(1)

    httpd, port = serve()
    base = f"http://127.0.0.1:{port}/index.html"

    results = []
    passed = True

    with sync_playwright() as p:
        browser = launch_browser(p, args.browser, args.executable)
        page = browser.new_page()
        page.on("console", lambda m: results.append(f"[console.{m.type}] {m.text}"))
        page.on("pageerror", lambda e: results.append(f"[pageerror] {e}"))

        page.goto(base, wait_until="load", timeout=30000)
        # 等待 wasm 初始化完成(window.__app 出现)
        try:
            page.wait_for_function("window.__app !== undefined", timeout=20000)
        except Exception as e:
            print("WASM 初始化超时(未出现 window.__app):", e)
            print("\n".join(results))
            browser.close(); httpd.shutdown(); sys.exit(1)

        # 1) 真实 adapter 验收
        try:
            info = page.evaluate("async () => { return await window.__app.adapter_info(); }")
            # adapter_info 返回 JSON 字符串
            info_obj = json.loads(info) if isinstance(info, str) else info
            if not info_obj.get("ok"):
                results.append(f"[M1] adapter_info 返回 ok=false: {info}")
                passed = False
            else:
                results.append(f"[M1] 真实 adapter 接受: {info}")
        except Exception as e:
            results.append(f"[M1] adapter_info 调用失败(可能无 WebGPU adapter): {e}")
            passed = False

        # 2) W1..W5 自测(真实 adapter 上跑 demo 发布的 WGSL compute)
        tests = [
            ("W1", "gpu_self_test"),
            ("W2", "optic_self_test"),
            ("W3", "caustics_self_test"),
            ("W4", "sph_self_test"),
            ("W5", "granular_self_test"),
        ]
        for label, fn in tests:
            try:
                res = page.evaluate(
                    f"async () => {{ return await window.__app.{fn}(); }}")
                res_str = res if isinstance(res, str) else json.dumps(res)
                # 失败约定:返回字符串含 "失败"/"Err"/"error" 或抛出异常
                if any(k in res_str.lower() for k in ["err", "失败", "error", "none"]):
                    results.append(f"[{label}] 自测疑似失败: {res_str}")
                    passed = False
                else:
                    results.append(f"[{label}] 自测通过: {res_str}")
            except Exception as e:
                results.append(f"[{label}] 自测异常: {e}")
                passed = False

        browser.close()
    httpd.shutdown()

    print("=" * 64)
    print("\n".join(results))
    print("=" * 64)
    if passed:
        print("M1 验收结果: PASS —— 真实 WebGPU adapter 已被接受,W1..W5 GPU 自测全过。")
        sys.exit(0)
    else:
        print("M1 验收结果: FAIL —— 见上。")
        sys.exit(1)


if __name__ == "__main__":
    main()
