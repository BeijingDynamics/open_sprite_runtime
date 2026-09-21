#!/usr/bin/env bash
set -euo pipefail

ROOT=/home/tony/open_sprite_runtime
CANDIDATE=/home/tony/sprite_runtime/sprite0825_stage2_g74_model3000_sim2real_candidate
DURATION="${1:-10}"
GAIN_SCALE="${2:-}"
STAMP="$(date +%Y%m%d_%H%M%S)"
SOCKET="/tmp/open_sprite_policy_${$}.sock"
NATIVE_REPORT="$ROOT/reports/native_policy_ipc_transport_${STAMP}.json"
POLICY_REPORT="$ROOT/reports/native_policy_ipc_actor_${STAMP}.json"
POLICY_TRACE="$ROOT/reports/native_policy_ipc_trace_${STAMP}.npz"
NATIVE_LOG="$ROOT/reports/native_policy_ipc_transport_${STAMP}.log"
JOINT_LIMITS="$ROOT/artifacts/g60_model3450/reports/sprite0825_urdf_limit_candidates.json"
[[ -f "$JOINT_LIMITS" ]] || JOINT_LIMITS="$ROOT/reports/sprite0825_urdf_limit_candidates.json"

mkdir -p "$ROOT/reports"
PYTHONPATH="$ROOT/src" "$ROOT/.venv/bin/python" \
  "$ROOT/tools/export_native_motor_config.py" \
  --hardware "$ROOT/config/hardware.sprite0825.measurement.json" \
  --output "$ROOT/build/native/motors.tsv"

JOINT_HASH="$(PYTHONPATH="$ROOT/src" "$ROOT/.venv/bin/python" -c \
  'import json,sys; from open_sprite_runtime.native_ipc import ordered_name_hash; d=json.load(open(sys.argv[1])); print(hex(ordered_name_hash(d["joint_names"])))' \
  "$CANDIDATE/deploy/contract.json")"

NATIVE_SAFETY_ARGS=()
POLICY_SAFETY_ARGS=()
if [[ -n "$GAIN_SCALE" ]]; then
  [[ -f "$JOINT_LIMITS" ]] || {
    echo "Joint limit report not found: $JOINT_LIMITS" >&2
    exit 1
  }
  PYTHONPATH="$ROOT/src" "$ROOT/.venv/bin/python" \
    "$ROOT/tools/export_native_joint_safety_config.py" \
    --contract "$CANDIDATE/deploy/contract.json" \
    --joint-limit-candidates "$JOINT_LIMITS" \
    --gain-scale "$GAIN_SCALE" \
    --maximum-embedded-kd 3.0 \
    --output "$ROOT/build/native/joint_safety.tsv"
  NATIVE_SAFETY_ARGS=(--joint-safety-config "$ROOT/build/native/joint_safety.tsv")
  POLICY_SAFETY_ARGS=(--joint-limit-candidates "$JOINT_LIMITS" --gain-scale "$GAIN_SCALE")
fi

echo "NATIVE POLICY IPC SHADOW: C++ owns four CAN buses; Python owns IMU + 50Hz ONNX"
echo "ACTIVE CAN TX remains position echo with velocity/Kp/Kd/torque all zero"
echo "Policy targets cross IPC for validation only and cannot reach CAN frames"
echo "The robot must remain mechanically supported and every motor disabled"
echo "DURATION ${DURATION}s; NATIVE_REPORT $NATIVE_REPORT; POLICY_REPORT $POLICY_REPORT"
echo "REPLAYABLE_TRACE $POLICY_TRACE"
echo "PROTECTED_TARGET_GAIN_SCALE ${GAIN_SCALE:-disabled}"

"$ROOT/build/native/sprite_can_shadow" \
  --config "$ROOT/build/native/motors.tsv" \
  --duration "$DURATION" \
  --cpu 5 \
  --realtime-priority 50 \
  --output "$NATIVE_REPORT" \
  --ipc-socket "$SOCKET" \
  --policy-joint-hash "$JOINT_HASH" \
  "${NATIVE_SAFETY_ARGS[@]}" \
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
  --imu-device /dev/sprite0825-imu \
  --imu-baud 115200 \
  --vx 0 --vy 0 --yaw-rate 0 \
  "${POLICY_SAFETY_ARGS[@]}" \
  --output "$POLICY_REPORT" \
  --trace-output "$POLICY_TRACE"
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
