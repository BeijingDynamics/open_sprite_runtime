#!/usr/bin/env bash
set -euo pipefail

ACK=ENABLE_RIGHT_HIP_YAW_5DEG_MOTION
if [[ "${1:-}" != "$ACK" ]]; then
  echo "ACTIVE SINGLE-JOINT TEST: suspended right hip yaw only" >&2
  echo "The robot must remain suspended, the right leg must be clear," >&2
  echo "and the safety operator must control the independent power cut-off." >&2
  echo "Fixed path: current -> +5deg -> -5deg -> current; 15s at 50Hz." >&2
  echo "Kp=8.0 Kd=0.3 tau=0; speed<=0.6rad/s; torque<=1.0Nm." >&2
  echo "Run only after explicit approval: $0 $ACK" >&2
  exit 2
fi

ROOT=/home/tony/open_sprite_runtime
STAMP="$(date +%Y%m%d_%H%M%S)"
SNAPSHOT="$ROOT/reports/kcan2_right_hip_yaw_5deg_${STAMP}.json"
OUTPUT="$ROOT/reports/right_hip_yaw_5deg_${STAMP}.json"

mkdir -p "$ROOT/reports"
ip -j -d link show type can > "$SNAPSHOT"

echo "ACTIVE HARDWARE MOTION: exactly one suspended right hip-yaw motor"
echo "MOTOR right_hip_yaw_motor; kcan2 ID 0x03; feedback 0x13; DM-J4340P"
echo "TRAJECTORY current -> +5deg -> -5deg -> current; quintic smoothstep"
echo "DURATION 15s; RATE 50Hz; Kp=8.0 Kd=0.3 tau=0"
echo "GUARDS error<=0.12rad speed<=0.6rad/s torque<=1.0Nm"
echo "FINALLY disables at least three times and verifies disabled feedback"
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
  --confirm-hardware-tx "$ACK"
