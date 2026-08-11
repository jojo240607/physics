// M1 真实 WebGPU adapter 验收 —— 浏览器控制台一键脚本
// 用法:
//   1) 在支持 WebGPU 的 Chrome/Edge(桌面版,有 GPU)打开
//      file:///d:/project/game/physics/crates/phy-demo-web/index.html
//      (或 http://localhost:8000/index.html,需先 `python -m http.server 8000`)
//   2) F12 打开 DevTools Console,把本文件内容整段粘贴运行。
//   3) 观察输出: 每个自测打印一行; 最后打印 "M1 验收结果: PASS" 或 "FAIL"。
//
// 注意: 必须用 `wasm32 + --features gpu` 构建的 pkg/(build_gpu.py 已生成)。
// 若页面是用默认(无 gpu)构建打开的,adapter_info / *_self_test 会是 undefined。

(async () => {
  const app = window.__app;
  if (!app) {
    console.error("[M1] window.__app 未定义。请确认 index.html 已加载且 pkg/ 带 gpu feature 构建。");
    return;
  }

  const checks = [
    { name: "adapter_info",      fn: () => app.adapter_info() },
    { name: "W1 gpu",            fn: () => app.gpu_self_test() },
    { name: "W2 optic",          fn: () => app.optic_self_test() },
    { name: "W3 caustics",       fn: () => app.caustics_self_test() },
    { name: "W4 sph",            fn: () => app.sph_self_test() },
    { name: "W5 granular",       fn: () => app.granular_self_test() },
  ];

  let allOk = true;

  for (const c of checks) {
    try {
      if (typeof c.fn() === "undefined") {
        console.error(`[M1] ${c.name}: 方法未暴露(undefined) —— 可能 pkg 未用 --features gpu 构建。`);
        allOk = false;
        continue;
      }
      const s = await c.fn();           // js_sys::Promise<string>
      const str = (typeof s === "string") ? s : String(s);
      // 判定: 返回串含 "ok=true" 或 "ok":true 即视为通过; adapter_info 需含 "ok":true。
      const ok = /ok\s*[:=]\s*true/.test(str) || /"ok"\s*:\s*true/.test(str);
      if (ok) {
        console.log(`[M1] ${c.name}: PASS  ->  ${str}`);
      } else {
        console.error(`[M1] ${c.name}: FAIL  ->  ${str}`);
        allOk = false;
      }
    } catch (e) {
      console.error(`[M1] ${c.name}: ERROR ->  ${e && e.message ? e.message : e}`);
      allOk = false;
    }
  }

  // adapter_info 额外校验: 必须是真实 adapter(name/backend 非空),而非回退。
  try {
    const info = await app.adapter_info();
    const infoStr = (typeof info === "string") ? info : String(info);
    const j = JSON.parse(infoStr);
    if (j.ok && j.backend && j.name && j.name !== "unknown") {
      console.log(`[M1] 真实 adapter 确认: name=${j.name} backend=${j.backend} vendor=${j.vendor} device=${j.device}`);
    } else {
      console.error(`[M1] adapter 非真实/回退: ${infoStr}`);
      allOk = false;
    }
  } catch (e) {
    console.error(`[M1] adapter_info 解析失败: ${e}`);
    allOk = false;
  }

  console.log(allOk
    ? "\n==== M1 验收结果: PASS —— 真实 WebGPU adapter 已被接受, W1..W5 GPU 自测全过。===="
    : "\n==== M1 验收结果: FAIL —— 见上方各检查项。====");
})();
