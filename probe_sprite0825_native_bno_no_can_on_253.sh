#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"
echo "SENSOR/INFERENCE DIAGNOSTIC ONLY: no CAN, no enable, no IMU configuration writes"
echo "Existing IMU readers must be stopped. Existing walking entrypoints are unchanged."
run_dir=$(mktemp -d reports/native_bno_stage1_XXXXXXXX)
socket_path="/tmp/sprite-imu-$$.sock"
producer_pid=""
cleanup() {
  if [[ -n "$producer_pid" ]] && kill -0 "$producer_pid" 2>/dev/null; then
    kill -TERM "$producer_pid"
    wait "$producer_pid" || true
  fi
}
trap cleanup EXIT INT TERM
build/imu_native_stage1/sprite_imu_bno --record 130 "$socket_path" "$run_dir/producer.json" >"$run_dir/producer.log" 2>&1 &
producer_pid=$!
sleep 1
if ! kill -0 "$producer_pid" 2>/dev/null; then
  cat "$run_dir/producer.log"
  exit 1
fi
PYTHONPATH=src .venv/bin/python tools/check_native_imu_no_can.py \
  --socket "$socket_path" --seconds 120 \
  --model artifacts/g60_model3450/deploy/policy.onnx \
  --trace reports/native_protected_policy_trace_20261013_130716.npz \
  | tee "$run_dir/consumer.log"
wait "$producer_pid"
producer_pid=""
cat "$run_dir/producer.json"
echo "REPORT DIRECTORY: $PWD/$run_dir"
