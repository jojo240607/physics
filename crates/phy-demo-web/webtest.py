#!/usr/bin/env python3
"""真实浏览器自测:启动本地服务器,用无头 chromium 打开 Web demo,
读取 <canvas> 真实像素,统计非背景像素,并对每个模式截图。
这能验证"页面真的画出了内容",而不是只跑逻辑层。
"""
import subprocess, threading, time, sys, os
from playwright.sync_api import sync_playwright

ROOT = os.path.dirname(os.path.abspath(__file__))
PORT = 8123
URL = f"http://localhost:{PORT}/"


def serve():
    import http.server, socketserver

    os.chdir(ROOT)
    h = http.server.SimpleHTTPRequestHandler
    with socketserver.TCPServer(("", PORT), h) as s:
        s.serve_forever()


def main():
    t = threading.Thread(target=serve, daemon=True)
    t.start()
    time.sleep(1.0)

    with sync_playwright() as p:
        browser = p.chromium.launch()
        page = browser.new_page(viewport={"width": 1000, "height": 760})
        errors = []
        page.on("console", lambda m: errors.append(f"{m.type}: {m.text}"))
        page.on("pageerror", lambda e: errors.append(f"pageerror: {e}"))
        page.goto(URL, wait_until="networkidle")
        time.sleep(2.0)

        # 检查 wasm 是否初始化、canvas 是否存在
        has_canvas = page.evaluate("!!document.getElementById('screen')")
        status = page.evaluate("document.getElementById('status')?.textContent || ''")
        print(f"[check] canvas存在={has_canvas} status='{status}'")

        # 先调 debug_fill 验证 present 管线:暂停 raf 后把 canvas 填红
        page.evaluate("if (window.__rafId) cancelAnimationFrame(window.__rafId); window.__rafId=null;")
        time.sleep(0.1)
        page.evaluate("window.__app.debug_fill()")
        time.sleep(0.3)
        red_stats = page.evaluate(
            """() => {
                const c = document.getElementById('screen');
                const ctx = c.getContext('2d');
                const d = ctx.getImageData(0,0,c.width,c.height).data;
                let red=0, other=0;
                for (let i=0;i<d.length;i+=4){
                    if (d[i]===255 && d[i+1]===0 && d[i+2]===0) red++; else other++;
                }
                return {red,other};
            }"""
        )
        print(f"[check] debug_fill red={red_stats['red']} other={red_stats['other']}")
        page.screenshot(path=os.path.join(ROOT, "shot_debug_fill.png"))

        # 用 app.frame() 手动推进,不依赖 raf
        def render_one():
            page.evaluate("window.__app.frame()")
            time.sleep(0.05)

        modes = {
            "1": "Rigid", "2": "Fluid", "3": "Heat", "4": "Soft", "5": "Optics",
            "6": "FluidHeat", "7": "Em", "8": "Grav", "9": "Wave", "0": "Acoustic",
            "a": "All",
        }
        all_ok = True
        for key, name in modes.items():
            page.keyboard.press(key)
            for _ in range(10):
                render_one()
            # 读 canvas 像素:统计 RGB 分布。
            # 背景色 = pack(0.05,0.07,0.10) = (13,18,26)。
            stats = page.evaluate(
                """() => {
                    const c = document.getElementById('screen');
                    const ctx = c.getContext('2d');
                    const w = c.width, h = c.height;
                    const d = ctx.getImageData(0,0,w,h).data;
                    let red=0, green=0, blue=0, bg=0, other=0, black=0, total=w*h;
                    for (let i=0;i<d.length;i+=4){
                        const r=d[i],g=d[i+1],b=d[i+2],a=d[i+3];
                        if (r===255 && g===0 && b===0) red++;
                        else if (r===0 && g===255 && b===0) green++;
                        else if (r===0 && g===0 && b===255) blue++;
                        else if (r===0 && g===0 && b===0) black++;
                        else if (r===13 && g===18 && b===26) bg++;
                        else other++;
                    }
                    return {w,h,red,green,blue,bg,other,black,total,first:[d[0],d[1],d[2],d[3]]};
                }"""
            )
            # 真实“内容像素”:既不是背景也不是纯黑。
            content = stats["red"] + stats["green"] + stats["blue"] + stats["other"]
            print(
                f"    dist red={stats['red']} green={stats['green']} blue={stats['blue']} "
                f"bg={stats['bg']} black={stats['black']} other={stats['other']} "
                f"first={stats['first']}"
            )
            pct = 100.0 * content / max(1, stats["total"])
            # 大部分模式应画出可见内容;Acoustic 等弱信号模式允许较少内容但应>0。
            ok = content > 0
            print(
                f"[{'PASS' if ok else 'FAIL'}] mode={name:9s}({key}) "
                f"canvas={stats['w']}x{stats['h']} content={content} ({pct:.1f}%)"
            )
            if not ok:
                all_ok = False
                print(f"    !!! {name} 渲染为空")
            # 截图
            page.screenshot(path=os.path.join(ROOT, f"shot_{name}.png"))

        print("\n[console 日志]")
        for e in errors:
            print("  ", e)

        page.screenshot(path=os.path.join(ROOT, "shot_final.png"))
        browser.close()
        if not all_ok:
            sys.exit(1)


if __name__ == "__main__":
    main()
