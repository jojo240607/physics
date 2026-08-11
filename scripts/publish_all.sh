#!/usr/bin/env bash
# C1: 按依赖拓扑顺序发布 phy-* 所有 crate 到 crates.io。
# 前置: `cargo login <token>`(crates.io API token)。
# 用法: bash scripts/publish_all.sh [--dry-run]
set -euo pipefail

DRY=""
if [[ "${1:-}" == "--dry-run" ]]; then
  DRY="--dry-run"
  echo "[dry-run] 仅预检,不真正发布"
fi

# 发布顺序(依赖拓扑,从底到顶):
# 1. phy-math(无内部依赖)
# 2. phy-core -> phy-math
# 3. phy-field -> phy-core, phy-math
# 4. phy-rigid -> phy-core, phy-math, phy-field
# 5. phy-fluid / phy-solid / phy-granular / phy-optics -> phy-rigid
# 6. phy-soft -> phy-fluid, phy-field
# 7. phy-io -> 上述全部
# 8. phy-ffi / phy-sdk -> phy-io(依赖全部)
ORDER=(
  phy-math
  phy-core
  phy-field
  phy-rigid
  phy-fluid
  phy-solid
  phy-granular
  phy-optics
  phy-soft
  phy-io
  phy-ffi
  phy-sdk
)

for crate in "${ORDER[@]}"; do
  echo "=== cargo publish -p $crate $DRY ==="
  cargo publish -p "$crate" --allow-dirty $DRY
  echo "--- $crate 发布成功 ---"
done

echo "全部 12 个 crate 发布完成(或 dry-run 预检通过)。"
echo "注: phy-demo / phy-demo-web 为 publish=false,不发布(Web demo 依赖 wasm-only 库)。"
