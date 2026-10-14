#!/usr/bin/env bash
set -euo pipefail
HOST_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$HOST_DIR/frontend"
if [[ ! -d node_modules ]]; then
  echo "前端依赖未安装，请先运行 ./install.sh" >&2
  exit 1
fi
exec npm run dev -- "$@"
