#!/usr/bin/env bash
set -Eeuo pipefail

PYTHON_BIN="/home/nvidia/miniconda3/envs/panthera/bin/python"
TUI_SCRIPT="/home/nvidia/OpenarmHT-A1_SDK/Quest_controller_stream/quest_stream_tui.py"

if [[ ! -x "$PYTHON_BIN" ]]; then
    echo "Missing panthera Python: $PYTHON_BIN" >&2
    exit 1
fi
if [[ ! -f "$TUI_SCRIPT" ]]; then
    echo "Missing Quest terminal UI: $TUI_SCRIPT" >&2
    exit 1
fi

exec "$PYTHON_BIN" -u "$TUI_SCRIPT"
