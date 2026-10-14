#!/usr/bin/env bash
set -euo pipefail
HOST_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PYTHON="${OPENARM_PYTHON:-$HOME/miniconda3/envs/panthera/bin/python}"
if [[ ! -x "$PYTHON" ]]; then
  echo "未找到 panthera Python，请设置 OPENARM_PYTHON 为可用解释器路径。" >&2
  exit 1
fi
# 无参数直接启动真机网页；连接硬件仍需在页面点击 Connect。
if [[ $# -eq 0 ]]; then
  set -- --live --port 5002
fi
cd "$HOST_DIR"
exec env -u LD_PRELOAD "$PYTHON" "$HOST_DIR/backend/app.py" "$@"
