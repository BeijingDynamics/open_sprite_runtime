#!/usr/bin/env bash
set -euo pipefail

ROOT=/home/tony/open_sprite_runtime
CANDIDATE=/home/tony/sprite_runtime/sprite0825_stage2_g74_model3000_sim2real_candidate
DURATION="${1:-10}"
STAMP="$(date +%Y%m%d_%H%M%S)"
SOCKET="/tmp/open_sprite_policy_${$}.sock"
NATIVE_REPORT="$ROOT/reports/native_policy_ipc_transport_${STAMP}.json"
POLICY_REPORT="$ROOT/reports/native_policy_ipc_actor_${STAMP}.json"
NATIVE_LOG="$ROOT/reports/native_policy_ipc_transport_${STAMP}.log"

mkdir -p "$ROOT/reports"
PYTHONPATH="$ROOT/src" "$ROOT/.venv/bin/python" \
  "$ROOT/tools/export_native_motor_config.py" \
  --hardware "$ROOT/config/hardware.sprite0825.measurement.json" \
  --output "$ROOT/build/native/motors.tsv"

JOINT_HASH="$(PYTHONPATH="$ROOT/src" "$ROOT/.venv/bin/python" -c \
  'import json,sys; from open_sprite_runtime.native_ipc import ordered_name_hash; d=json.load(open(sys.argv[1])); print(hex(ordered_name_hash(d["joint_names"])))' \
  "$CANDIDATE/deploy/contract.json")"

echo "NATIVE POLICY IPC SHADOW: C++ owns four CAN buses; Python owns IMU + 50Hz ONNX"
echo "ACTIVE CAN TX remains position echo with velocity/Kp/Kd/torque all zero"
echo "Policy targets cross IPC for validation only and cannot reach CAN frames"
echo "The robot must remain mechanically supported and every motor disabled"
echo "DURATION ${DURATION}s; NATIVE_REPORT $NATIVE_REPORT; POLICY_REPORT $POLICY_REPORT"

"$ROOT/build/native/sprite_can_shadow" \
  --config "$ROOT/build/native/motors.tsv" \
  --duration "$DURATION" \
  --cpu 5 \
  --realtime-priority 50 \
  --output "$NATIVE_REPORT" \
  --ipc-socket "$SOCKET" \
  --policy-joint-hash "$JOINT_HASH" \
  --acknowledge-hardware-tx ZERO_GAIN_NATIVE_SHADOW \
  --all-motors-disabled-confirmed >"$NATIVE_LOG" 2>&1 &
NATIVE_PID=$!

cleanup() {
  if kill -0 "$NATIVE_PID" 2>/dev/null; then
    kill "$NATIVE_PID" 2>/dev/null || true
    wait "$NATIVE_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT

for _ in $(seq 1 100); do
  [[ -S "$SOCKET" ]] && break
  kill -0 "$NATIVE_PID" 2>/dev/null || {
    cat "$NATIVE_LOG"
    exit 1
  }
  sleep 0.05
done
[[ -S "$SOCKET" ]] || {
  echo "Native IPC socket did not appear" >&2
  cat "$NATIVE_LOG"
  exit 1
}

set +e
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 taskset -c 4 env PYTHONPATH="$ROOT/src" \
  "$ROOT/.venv/bin/python" \
  -m open_sprite_runtime.native_policy_client \
  --socket "$SOCKET" \
  --hardware-config "$ROOT/config/hardware.sprite0825.measurement.json" \
  --contract "$CANDIDATE/deploy/contract.json" \
  --imu-device /dev/ttyCH341USB0 \
  --imu-baud 115200 \
  --vx 0 --vy 0 --yaw-rate 0 \
  --output "$POLICY_REPORT"
POLICY_STATUS=$?

wait "$NATIVE_PID"
NATIVE_STATUS=$?
set -e
trap - EXIT
cat "$NATIVE_LOG"
if (( POLICY_STATUS != 0 || NATIVE_STATUS != 0 )); then
  echo "IPC shadow failed: policy_status=$POLICY_STATUS native_status=$NATIVE_STATUS" >&2
  exit 1
fi
