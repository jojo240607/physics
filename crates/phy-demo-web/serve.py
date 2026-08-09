#!/usr/bin/env python3
"""本地静态服务器:启动后用浏览器打开 http://localhost:8000 即可预览 Web 版 demo。
wasm 不能用 file:// 直接打开,必须经 http。零依赖(仅标准库)。

用法:
    python serve.py            # 默认 8000 端口,服务本目录(index.html + pkg/)
    python serve.py 8080       # 指定端口
"""
import http.server
import socketserver
import sys
import os

PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 8000
ROOT = os.path.dirname(os.path.abspath(__file__))


class Handler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=ROOT, **kwargs)

    def end_headers(self):
        # 允许 cross-origin(本目录内访问)
        self.send_header("Cross-Origin-Opener-Policy", "same-origin")
        self.send_header("Cross-Origin-Embedder-Policy", "require-corp")
        self.send_header("Cache-Control", "no-store")
        super().end_headers()

    def log_message(self, fmt, *args):
        print("[serve] " + (fmt % args))


if __name__ == "__main__":
    os.chdir(ROOT)
    with socketserver.TCPServer(("", PORT), Handler) as httpd:
        print(f"[serve] phy-demo-web 预览: http://localhost:{PORT}/")
        print("[serve] 按 Ctrl+C 停止")
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            print("\n[serve] 已停止")
