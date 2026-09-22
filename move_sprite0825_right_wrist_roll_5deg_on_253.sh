#!/usr/bin/env bash
set -euo pipefail

if [[ "${1:-}" != "ENABLE_RIGHT_WRIST_ROLL_5DEG_MOTION" ]]; then
  echo "LOADED-JOINT MOTION: right_wrist_roll_motor only" >&2
  echo "Robot must be supported; wrist path/cables must be clear;" >&2
  echo "a safety operator must hold the independent power cut-off." >&2
  echo "Joint path: current -> +5deg -> -5deg -> current; 15s at 50Hz." >&2
  echo "Motor direction applies frozen policy_to_motor_sign=-1." >&2
  echo "Run only after those conditions are true:" >&2
  echo "  $0 ENABLE_RIGHT_WRIST_ROLL_5DEG_MOTION" >&2
  exit 2
fi

ROOT=/home/tony/open_sprite_runtime
STAMP="$(date +%Y%m%d_%H%M%S)"
SNAPSHOT="$ROOT/reports/kcan4_right_wrist_roll_motion_${STAMP}.json"
OUTPUT="$ROOT/reports/right_wrist_roll_5deg_${STAMP}.json"

mkdir -p "$ROOT/reports"
ip -j -d link show type can > "$SNAPSHOT"

echo "ACTIVE LOADED-JOINT MOTION: exactly one distal wrist motor"
echo "MOTOR right_wrist_roll_motor; kcan4 ID 0x07; feedback 0x17; DM-J3507"
echo "JOINT TRAJECTORY current -> +5deg -> -5deg -> current; motor sign=-1"
echo "DURATION 15s; RATE 50Hz; Kp=2.0 Kd=0.2 tau=0"
echo "GUARDS error<=0.08rad speed<=0.6rad/s estimated_torque<=0.2Nm"
echo "FINALLY sends disable at least three times and verifies disabled"
echo "NO mode switch; NO set-zero; NO other motor command"
echo "OUTPUT $OUTPUT"

cd "$ROOT"
exec env PYTHONPATH="$ROOT/src" "$ROOT/.venv/bin/python" \
  "$ROOT/tools/move_right_wrist_roll_5deg.py" \
  --hardware "$ROOT/config/hardware.sprite0825.measurement.json" \
  --socketcan-snapshot "$SNAPSHOT" \
  --output "$OUTPUT" \
  --robot-supported \
  --safety-operator-ready \
  --right-wrist-clear \
  --confirm-hardware-tx ENABLE_RIGHT_WRIST_ROLL_5DEG_MOTION
