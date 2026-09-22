#!/usr/bin/env bash
set -euo pipefail

DURATION="${1:-0.5}"
ROOT=/home/tony/open_sprite_runtime
CONTRACT="$ROOT/artifacts/g60_model3450/deploy/contract.json"
STAMP="$(date +%Y%m%d_%H%M%S)"
MOTOR_CONFIG="$ROOT/build/native/motors.tsv"
KINEMATICS_CONFIG="$ROOT/build/native/kinematics.tsv"
OUTPUT="$ROOT/reports/native_full_body_measured_pose_hold_${STAMP}.json"

case "$DURATION" in
  0.5|1.0|2.0) ;;
  *)
    echo "DURATION must be exactly 0.5, 1.0, or 2.0 seconds" >&2
    exit 2
    ;;
esac

mkdir -p "$ROOT/build/native" "$ROOT/reports"
PYTHONPATH="$ROOT/src" "$ROOT/.venv/bin/python" \
  "$ROOT/tools/export_native_motor_config.py" \
  --hardware "$ROOT/config/hardware.sprite0825.measurement.json" \
  --output "$MOTOR_CONFIG"
PYTHONPATH="$ROOT/src" "$ROOT/.venv/bin/python" \
  "$ROOT/tools/export_native_kinematics_config.py" \
  --hardware "$ROOT/config/hardware.sprite0825.measurement.json" \
  --contract "$CONTRACT" \
  --output "$KINEMATICS_CONFIG"

echo "ACTIVE HARDWARE CONTROL: suspended 31-motor measured-pose hold"
echo "Direct joints: embedded MIT PD at 50Hz, Kp=0.2 Kd=0.05"
echo "Ankle differential pairs: host joint PD at 500Hz, Kp=0.5 Kd=0.05"
echo "Head pitch/roll pair: host joint PD at 50Hz, Kp=0.2 Kd=0.03"
echo "All targets are captured measured positions; feedforward torque is tightly capped"
echo "Any fault disables all 31 motors; no mode switch and no zero reset"
echo "Robot must remain suspended and the independent power safety operator must be ready"
echo "SCHED_OTHER is guarded by a zero-miss and <2ms measured deadline gate"
echo "DURATION ${DURATION}s; CPU 5; OUTPUT $OUTPUT"

exec "$ROOT/build/native/sprite_can_shadow" \
  --config "$MOTOR_CONFIG" \
  --kinematics-config "$KINEMATICS_CONFIG" \
  --duration "$DURATION" \
  --cpu 5 \
  --realtime-priority 0 \
  --output "$OUTPUT" \
  --measured-pose-hold \
  --acknowledge-hardware-tx ENABLE_NATIVE_FULL_BODY_MEASURED_POSE_HOLD \
  --all-motors-disabled-confirmed
