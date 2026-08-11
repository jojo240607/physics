import sys, tempfile, subprocess, time, json, urllib.request, http.server, socketserver, threading
from playwright.sync_api import sync_playwright

EXE = r"C:\Program Files\Google\Chrome\Application\chrome.exe"
ROOT = r"d:\project\game\physics\crates\phy-demo-web"
UDD = tempfile.mkdtemp(prefix="chrome_m1_")
PORT = 9333
HTTP_PORT = 8123
out = []
_log = open("m1_result.txt", "w")

def log(s):
    out.append(s)
    _log.write(s + "\n")
    _log.flush()

handler = http.server.SimpleHTTPRequestHandler
httpd = socketserver.TCPServer(("127.0.0.1", HTTP_PORT), handler)
httpd.allow_reuse_address = True
import os as _os
_orig = _os.getcwd()
_os.chdir(ROOT)
threading.Thread(target=httpd.serve_forever, daemon=True).start()
log("HTTP_OK:" + str(HTTP_PORT))

args = [
    EXE,
    "--user-data-dir=" + UDD,
    "--remote-debugging-port=" + str(PORT),
    "--no-first-run",
    "--no-default-browser-check",
    "--enable-unsafe-webgpu",
    "--enable-features=Vulkan,WebGPU,Dawn",
    "--ignore-gpu-blocklist",
    "--enable-gpu",
    "about:blank",
]
proc = subprocess.Popen(args, stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL)
log("PID:" + str(proc.pid))

up = False
for _ in range(30):
    try:
        with urllib.request.urlopen("http://127.0.0.1:%d/json/version" % PORT, timeout=2) as resp:
            ver = json.loads(resp.read().decode())
        up = True
        break
    except Exception:
        time.sleep(1)
if not up:
    log("CDP_FAIL")
    proc.terminate()
    httpd.shutdown()
    sys.exit(0)

log("CDP_OK:" + ver.get("Browser", "?"))

def run_check(pg, name):
    try:
        s = pg.evaluate("async () => String(await window.__app.%s())" % name)
        return s
    except Exception as e:
        return "EXC:" + str(e)

try:
    with sync_playwright() as p:
        b = p.chromium.connect_over_cdp(ver["webSocketDebuggerUrl"])
        ctx = b.contexts[0]
        pg = ctx.new_page()
        pg.set_default_timeout(90000)
        pg.on("console", lambda m: log("C:" + m.text))
        pg.on("pageerror", lambda e: log("E:" + str(e)))
        time.sleep(3)
        pg.goto("http://127.0.0.1:%d/index.html" % HTTP_PORT, timeout=30000)
        log("PAGE_LOADED")
        # wait for wasm + __app ready (poll)
        ready = False
        for _ in range(40):
            try:
                r = pg.evaluate("typeof window.__app !== 'undefined'")
                if r:
                    ready = True
                    break
            except Exception:
                pass
            time.sleep(1)
        log("APP_READY:" + str(ready))
        if not ready:
            log("APP_NOT_READY_ABORT")
            b.close()
            proc.terminate(); httpd.shutdown(); _os.chdir(_orig); sys.exit(0)

        checks = ["adapter_info", "gpu_self_test", "optic_self_test",
                  "caustics_self_test", "sph_self_test", "granular_self_test"]
        results = {}
        for name in checks:
            log("RUN_" + name)
            s = run_check(pg, name)
            results[name] = s
            log("CHECK_%s: %s" % (name, s[:500]))

        all_ok = True
        for name in checks:
            s = results[name]
            if s.startswith("EXC:") or s.startswith("E:"):
                all_ok = False
                log("FAIL_REASON:%s -> %s" % (name, s[:200]))
                continue
            try:
                j = json.loads(s)
                ok = bool(j.get("ok"))
            except Exception:
                # 识别各类 ok 字面量。W1/gpu_self_test 用 "ok=true" 或 JSON "ok":true；
                # W2-W5 用 "Wx ... ok: ..." 形式（ok: 后跟非 false 即成功）。
                ok = False
                if ("ok=true" in s) or ('"ok": true' in s) or ('"ok":true' in s):
                    ok = True
                # 形如 "ok: true" / "ok:true" / " ok: ..."：提取 ok: 后的首个词
                import re as _re
                m = _re.search(r"ok:\s*([a-z]+)", s)
                if m:
                    tok = m.group(1)
                    if tok == "true":
                        ok = True
                    elif tok == "false":
                        ok = False
                    else:
                        # 其它值（如 "ok: center=..." 表示正常完成，无失败标记）
                        ok = True
                # 显式失败标记
                if ("ok=false" in s) or ('"ok": false' in s) or ("ok:false" in s) \
                        or ("ok: false" in s) or ("EXC:" in s):
                    ok = False
            if not ok:
                all_ok = False
                log("FAIL_REASON:%s -> %s" % (name, s[:200]))
        log("VERDICT:" + ("PASS" if all_ok else "FAIL"))
        b.close()
except Exception as e:
    log("OUTER_EXC:" + str(e))
finally:
    try:
        proc.terminate()
        proc.wait(timeout=5)
    except Exception:
        try: proc.kill()
        except Exception: pass
    httpd.shutdown()
    _os.chdir(_orig)
    _log.close()
