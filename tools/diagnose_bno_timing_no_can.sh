#!/usr/bin/env bash
set -euo pipefail
cd /home/tony/open_sprite_runtime
export PYTHONPATH=src
socket="/tmp/bno_timing_diag_$$.sock"
report="reports/bno_timing_ipc_$(date +%Y%m%d_%H%M%S).json"
build/imu_native_stage1/sprite_imu_bno_diagnostic --record 180 "$socket" "$report" &
pid=$!
trap 'kill "$pid" 2>/dev/null || true; wait "$pid" 2>/dev/null || true' EXIT
sleep 1
.venv/bin/python tools/check_native_imu_no_can.py --socket "$socket" --seconds 170
wait "$pid"
trap - EXIT
echo "REPORT $report"
