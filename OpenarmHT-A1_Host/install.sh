#!/usr/bin/env bash
set -euo pipefail
HOST_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PYTHON="${OPENARM_PYTHON:-$HOME/miniconda3/envs/panthera/bin/python}"
if [[ ! -x "$PYTHON" ]]; then
  echo "请先准备 panthera 环境，或设置 OPENARM_PYTHON。" >&2
  exit 1
fi
env -u LD_PRELOAD "$PYTHON" -m pip install -r "$HOST_DIR/backend/requirements.txt"
cd "$HOST_DIR/frontend"
if [[ -f package-lock.json ]]; then npm ci; else npm install; fi
npm run build
printf '安装完成。执行 ./start.sh，然后访问 http://localhost:5001\n'
