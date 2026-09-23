#!/usr/bin/env bash
set -euo pipefail

ACK=ENABLE_RIGHT_HIP_YAW_5DEG_MEDIUM_GAIN_MOTION
if [[ "${1:-}" != "$ACK" ]]; then
  echo "ACTIVE SINGLE-JOINT TEST: suspended right hip yaw only" >&2
  echo "Robot suspended, right leg clear, independent power cut-off ready." >&2
  echo "Path: current -> +5deg -> -5deg -> current; 15s at 50Hz." >&2
  echo "Kp=20 Kd=0.5 tau=0; speed<=0.8rad/s; torque<=2.5Nm." >&2
  echo "Run only after explicit approval: $0 $ACK" >&2
  exit 2
fi

ROOT=/home/tony/open_sprite_runtime
STAMP="$(date +%Y%m%d_%H%M%S)"
SNAPSHOT="$ROOT/reports/kcan2_right_hip_yaw_5deg_medium_${STAMP}.json"
OUTPUT="$ROOT/reports/right_hip_yaw_5deg_medium_${STAMP}.json"
mkdir -p "$ROOT/reports"
ip -j -d link show type can > "$SNAPSHOT"

echo "ACTIVE HARDWARE MOTION: exactly one suspended right hip-yaw motor"
echo "PATH current -> +5deg -> -5deg -> current; 15s; 50Hz"
echo "Kp=20 Kd=0.5 tau=0; speed<=0.8rad/s; torque<=2.5Nm"
echo "DIRECTION GATE both directions>=2.5deg; return error<=2.5deg"
echo "NO mode switch; NO set-zero; NO other motor command"
echo "OUTPUT $OUTPUT"

cd "$ROOT"
exec env PYTHONPATH="$ROOT/src" "$ROOT/.venv/bin/python" \
  "$ROOT/tools/move_right_hip_yaw_5deg.py" \
  --hardware "$ROOT/config/hardware.sprite0825.measurement.json" \
  --socketcan-snapshot "$SNAPSHOT" \
  --output "$OUTPUT" \
  --robot-suspended \
  --safety-operator-ready \
  --right-leg-clear \
  --medium-gain \
  --confirm-hardware-tx "$ACK"
