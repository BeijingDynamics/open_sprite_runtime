#!/usr/bin/env bash
set -euo pipefail

ACK="${1:-}"
TIER="${2:-first_admission}"

case "$TIER" in
  first_admission)
    EXPECTED_ACK=ENABLE_NATIVE_PROTECTED_POLICY_ACTUATION
    DURATION=2.0
    GAIN_SCALE=0.02
    DM3507_GAIN_MULTIPLIER=1.0
    MAXIMUM_COMMAND_TORQUE_NM=1.4
    ;;
  full_ramp_dm3507_tier)
    EXPECTED_ACK=ENABLE_NATIVE_PROTECTED_POLICY_FULL_RAMP
    DURATION=6.0
    GAIN_SCALE=0.015
    DM3507_GAIN_MULTIPLIER=0.1
    MAXIMUM_COMMAND_TORQUE_NM=1.4
    ;;
  full_ramp_02nm_tier)
    EXPECTED_ACK=ENABLE_NATIVE_PROTECTED_POLICY_02NM_FULL_RAMP
    DURATION=6.0
    GAIN_SCALE=0.006
    DM3507_GAIN_MULTIPLIER=0.1
    MAXIMUM_COMMAND_TORQUE_NM=0.2
    ;;
  *)
    echo "Unknown protected-policy tier: $TIER" >&2
    exit 2
    ;;
esac
[[ "$ACK" == "$EXPECTED_ACK" ]] || {
  echo "Exact acknowledgement required: $EXPECTED_ACK" >&2
  exit 2
}

ROOT=/home/tony/open_sprite_runtime
CANDIDATE=/home/tony/sprite_runtime/sprite0825_stage2_g74_model3000_sim2real_candidate
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
PREFLIGHT_LOG="$ROOT/reports/native_protected_policy_preflight_${STAMP}.log"
PREFLIGHT_REPORT="$ROOT/reports/native_protected_policy_preflight_${STAMP}.json"

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

echo "ZERO-GAIN STARTUP READINESS PREFLIGHT: 6.0s"
echo "Requires ankle excursions <=0.01rad, <=1% ticks, <=2 consecutive ticks; horizontal projected gravity <=0.10"
"$ROOT/probe_sprite0825_native_policy_ipc_shadow_on_253.sh" \
  6.0 "$GAIN_SCALE" 0 0 "$DM3507_GAIN_MULTIPLIER" | tee "$PREFLIGHT_LOG"
PREFLIGHT_TRACE="$(awk '/^REPLAYABLE_TRACE / {print $2}' "$PREFLIGHT_LOG" | tail -1)"
PREFLIGHT_NATIVE_REPORT="$(awk '/^DURATION / {for (i=1; i<=NF; ++i) if ($i == "NATIVE_REPORT") {gsub(/;/, "", $(i+1)); print $(i+1)}}' "$PREFLIGHT_LOG" | tail -1)"
[[ -n "$PREFLIGHT_TRACE" && -f "$PREFLIGHT_TRACE" ]] || {
  echo "Startup readiness preflight did not produce a trace" >&2
  exit 1
}
[[ -n "$PREFLIGHT_NATIVE_REPORT" && -f "$PREFLIGHT_NATIVE_REPORT" ]] || {
  echo "Startup readiness preflight did not produce a native report" >&2
  exit 1
}
"$ROOT/.venv/bin/python" - "$PREFLIGHT_NATIVE_REPORT" "$MAXIMUM_COMMAND_TORQUE_NM" <<'PY'
import json
import sys

report = json.load(open(sys.argv[1], encoding="utf-8"))
observed = float(report["preview_maximum_abs_estimated_torque_nm"])
limit = float(sys.argv[2])
if observed > limit:
    raise SystemExit(
        f"startup command torque preview {observed:.6f} Nm exceeds {limit:.6f} Nm"
    )
print(f"STARTUP_COMMAND_TORQUE_PASSED observed={observed:.6f}Nm limit={limit:.6f}Nm")
PY
PYTHONPATH="$ROOT/src" "$ROOT/.venv/bin/python" \
  "$ROOT/tools/analyze_policy_joint_limit_clamps.py" \
  --trace "$PREFLIGHT_TRACE" \
  --joint-limit-candidates "$JOINT_LIMITS" \
  --contract "$CANDIDATE/deploy/contract.json" \
  --fail-on-violation-joint left_ankle_pitch_joint \
  --fail-on-violation-joint right_ankle_pitch_joint \
  --fail-on-violation-joint left_ankle_roll_joint \
  --fail-on-violation-joint right_ankle_roll_joint \
  --maximum-gated-overshoot-rad 0.01 \
  --maximum-gated-violation-fraction 0.01 \
  --maximum-gated-consecutive-violation-ticks 2 \
  --maximum-horizontal-gravity-norm 0.10 \
  --output "$PREFLIGHT_REPORT" >/dev/null
echo "STARTUP_READINESS_PASSED report=$PREFLIGHT_REPORT"

JOINT_GAIN_ARGS=()
if [[ "$DM3507_GAIN_MULTIPLIER" != "1.0" ]]; then
  for joint in \
    head_pitch_joint head_roll_joint head_yaw_joint \
    left_wrist_pitch_joint left_wrist_roll_joint \
    right_wrist_pitch_joint right_wrist_roll_joint; do
    JOINT_GAIN_ARGS+=(--joint-gain-multiplier "$joint=$DM3507_GAIN_MULTIPLIER")
  done
fi
PYTHONPATH="$ROOT/src" "$ROOT/.venv/bin/python" \
  "$ROOT/tools/export_native_motor_config.py" \
  --hardware "$ROOT/config/hardware.sprite0825.measurement.json" \
  --maximum-commissioning-torque-nm "$MAXIMUM_COMMAND_TORQUE_NM" \
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
  "${JOINT_GAIN_ARGS[@]}" \
  --output "$ROOT/build/native/joint_safety.tsv"

JOINT_HASH="$(PYTHONPATH="$ROOT/src" "$ROOT/.venv/bin/python" -c \
  'import json,sys; from open_sprite_runtime.native_ipc import ordered_name_hash; d=json.load(open(sys.argv[1])); print(hex(ordered_name_hash(d["joint_names"])))' \
  "$CANDIDATE/deploy/contract.json")"

echo "ACTIVE HARDWARE CONTROL: suspended protected-policy admission"
echo "Fixed tier: name=${TIER} duration=${DURATION}s gain_scale=${GAIN_SCALE} DM3507_multiplier=${DM3507_GAIN_MULTIPLIER} hold=${STARTUP_HOLD_SECONDS}s ramp=${STARTUP_RAMP_SECONDS}s"
echo "Per-motor command cap: min(10% of rated torque, ${MAXIMUM_COMMAND_TORQUE_NM} Nm), checked after MIT quantization"
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
  "${JOINT_GAIN_ARGS[@]}" \
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
