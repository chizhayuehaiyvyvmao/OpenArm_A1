#!/usr/bin/env bash
set -euo pipefail

tool_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
viewer_bin="${tool_dir}/d435i_web_viewer"
port="8765"
pid_file="/tmp/d435i_web_viewer.pid"
log_file="/home/nvidia/d435i_web_viewer.log"

if [[ -r "${pid_file}" ]]; then
    old_pid="$(<"${pid_file}")"
    if kill -0 "${old_pid}" 2>/dev/null; then
        echo "D435i web viewer is already running (PID ${old_pid})."
        echo "Open http://127.0.0.1:${port} through VS Code port forwarding."
        exit 0
    fi
fi

nohup "${viewer_bin}" "${port}" >"${log_file}" 2>&1 </dev/null &
viewer_pid="$!"
printf '%s\n' "${viewer_pid}" >"${pid_file}"

for _ in {1..40}; do
    if curl -fsS "http://127.0.0.1:${port}/" >/dev/null 2>&1; then
        echo "D435i web viewer started (PID ${viewer_pid})."
        echo "Open http://127.0.0.1:${port} through VS Code port forwarding."
        exit 0
    fi
    if ! kill -0 "${viewer_pid}" 2>/dev/null; then
        break
    fi
    sleep 0.25
done

echo "D435i web viewer failed to start. Last log lines:" >&2
tail -40 "${log_file}" >&2 || true
exit 1
