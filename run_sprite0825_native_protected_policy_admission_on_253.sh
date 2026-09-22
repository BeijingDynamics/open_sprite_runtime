#!/usr/bin/env bash
set -euo pipefail

ACK="${1:-}"
[[ "$ACK" == "ENABLE_NATIVE_PROTECTED_POLICY_ACTUATION" ]] || {
  echo "Exact acknowledgement required: ENABLE_NATIVE_PROTECTED_POLICY_ACTUATION" >&2
  exit 2
}

ROOT=/home/tony/open_sprite_runtime
CANDIDATE=/home/tony/sprite_runtime/sprite0825_stage2_g74_model3000_sim2real_candidate
DURATION=2.0
GAIN_SCALE=0.02
STARTUP_HOLD_SECONDS=1.0
STARTUP_RAMP_SECONDS=4.0
STAMP="$(date +%Y%m%d_%H%M%S)"
SOCKET="/tmp/open_sprite_policy_active_${$}.sock"
NATIVE_REPORT="$ROOT/reports/native_protected_policy_admission_${STAMP}.json"
POLICY_REPORT="$ROOT/reports/native_protected_policy_actor_${STAMP}.json"
POLICY_TRACE="$ROOT/reports/native_protected_policy_trace_${STAMP}.npz"
NATIVE_LOG="$ROOT/reports/native_protected_policy_admission_${STAMP}.log"
JOINT_LIMITS="$ROOT/artifacts/g60_model3450/reports/sprite0825_urdf_limit_candidates.json"
[[ -f "$JOINT_LIMITS" ]] || JOINT_LIMITS="$ROOT/reports/sprite0825_urdf_limit_candidates.json"

mkdir -p "$ROOT/build/native" "$ROOT/reports"
getcap "$ROOT/build/native/sprite_can_shadow" | grep -q 'cap_sys_nice' || {
  echo "Native runtime lacks CAP_SYS_NICE; refuse active policy control" >&2
  echo "Reinstall the reviewed capability after every rebuild" >&2
  exit 1
}
[[ -f "$JOINT_LIMITS" ]] || {
  echo "Joint limit report not found: $JOINT_LIMITS" >&2
  exit 1
}
PYTHONPATH="$ROOT/src" "$ROOT/.venv/bin/python" \
  "$ROOT/tools/export_native_motor_config.py" \
  --hardware "$ROOT/config/hardware.sprite0825.measurement.json" \
  --output "$ROOT/build/native/motors.tsv"
PYTHONPATH="$ROOT/src" "$ROOT/.venv/bin/python" \
  "$ROOT/tools/export_native_kinematics_config.py" \
  --hardware "$ROOT/config/hardware.sprite0825.measurement.json" \
  --contract "$CANDIDATE/deploy/contract.json" \
  --output "$ROOT/build/native/kinematics.tsv"
PYTHONPATH="$ROOT/src" "$ROOT/.venv/bin/python" \
  "$ROOT/tools/export_native_joint_safety_config.py" \
  --contract "$CANDIDATE/deploy/contract.json" \
  --joint-limit-candidates "$JOINT_LIMITS" \
  --gain-scale "$GAIN_SCALE" \
  --maximum-embedded-kd 3.0 \
  --output "$ROOT/build/native/joint_safety.tsv"

JOINT_HASH="$(PYTHONPATH="$ROOT/src" "$ROOT/.venv/bin/python" -c \
  'import json,sys; from open_sprite_runtime.native_ipc import ordered_name_hash; d=json.load(open(sys.argv[1])); print(hex(ordered_name_hash(d["joint_names"])))' \
  "$CANDIDATE/deploy/contract.json")"

echo "ACTIVE HARDWARE CONTROL: first suspended protected-policy admission"
echo "Fixed tier: duration=${DURATION}s gain_scale=${GAIN_SCALE} hold=${STARTUP_HOLD_SECONDS}s ramp=${STARTUP_RAMP_SECONDS}s"
echo "Per-motor command cap: 10% of rated torque, checked after MIT quantization"
echo "Native watchdogs cover target age, status, hard position, speed, torque, temperature, and timing"
echo "Any fault or SIGINT/SIGTERM performs whole-body disable and verifies all 31 disabled"
echo "No mode switch and no zero-position reset are implemented in this path"
echo "Robot must remain suspended; independent power safety operator must be ready"
echo "NATIVE_REPORT $NATIVE_REPORT"
echo "POLICY_REPORT $POLICY_REPORT"

"$ROOT/build/native/sprite_can_shadow" \
  --config "$ROOT/build/native/motors.tsv" \
  --duration "$DURATION" \
  --cpu 5 \
  --realtime-priority 50 \
  --output "$NATIVE_REPORT" \
  --ipc-socket "$SOCKET" \
  --policy-joint-hash "$JOINT_HASH" \
  --kinematics-config "$ROOT/build/native/kinematics.tsv" \
  --joint-safety-config "$ROOT/build/native/joint_safety.tsv" \
  --policy-actuation \
  --acknowledge-hardware-tx ENABLE_NATIVE_PROTECTED_POLICY_ACTUATION \
  --all-motors-disabled-confirmed >"$NATIVE_LOG" 2>&1 &
NATIVE_PID=$!

shutdown_native() {
  if kill -0 "$NATIVE_PID" 2>/dev/null; then
    kill -TERM "$NATIVE_PID" 2>/dev/null || true
    wait "$NATIVE_PID" 2>/dev/null || true
  fi
}
trap shutdown_native INT TERM EXIT

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
  "$ROOT/.venv/bin/python" -m open_sprite_runtime.native_policy_client \
  --socket "$SOCKET" \
  --hardware-config "$ROOT/config/hardware.sprite0825.measurement.json" \
  --contract "$CANDIDATE/deploy/contract.json" \
  --imu-device /dev/sprite0825-imu \
  --imu-baud 115200 \
  --vx 0 --vy 0 --yaw-rate 0 \
  --joint-limit-candidates "$JOINT_LIMITS" \
  --gain-scale "$GAIN_SCALE" \
  --physical-startup-hold-seconds "$STARTUP_HOLD_SECONDS" \
  --physical-startup-ramp-seconds "$STARTUP_RAMP_SECONDS" \
  --output "$POLICY_REPORT" \
  --trace-output "$POLICY_TRACE"
POLICY_STATUS=$?
wait "$NATIVE_PID"
NATIVE_STATUS=$?
set -e
trap - INT TERM EXIT
cat "$NATIVE_LOG"
if (( POLICY_STATUS != 0 || NATIVE_STATUS != 0 )); then
  echo "Protected-policy admission failed: policy_status=$POLICY_STATUS native_status=$NATIVE_STATUS" >&2
  exit 1
fi
