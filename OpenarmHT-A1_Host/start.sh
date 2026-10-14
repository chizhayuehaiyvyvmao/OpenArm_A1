#!/usr/bin/env bash
# One-process Demo launcher. Build is local; no robot connection occurs.
set -euo pipefail
HOST_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$HOST_DIR/frontend"
npm run build
cd "$HOST_DIR"
exec ./backend.sh --demo
