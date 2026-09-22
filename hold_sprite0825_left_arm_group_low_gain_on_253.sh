#!/usr/bin/env bash
set -euo pipefail

if [[ "${1:-}" != "ENABLE_LEFT_ARM_GROUP_LOW_GAIN_HOLD" ]]; then
  echo "FIXED MOTOR-GROUP HOLD: left-arm seven motors only; head yaw excluded" >&2
  echo "Robot must be supported; the complete left arm and cables must be clear;" >&2
  echo "a safety operator must hold the independent power cut-off." >&2
  echo "Fixed hold: measured positions, 2s, 50Hz, Kp=0.2 Kd=0.05 tau=0." >&2
  echo "This command never resets motor zero positions." >&2
  echo "Run only after those conditions are true:" >&2
  echo "  $0 ENABLE_LEFT_ARM_GROUP_LOW_GAIN_HOLD" >&2
  exit 2
fi

ROOT=/home/tony/open_sprite_runtime
STAMP="$(date +%Y%m%d_%H%M%S)"
SNAPSHOT="$ROOT/reports/kcan3_left_arm_group_hold_${STAMP}.json"
OUTPUT="$ROOT/reports/left_arm_group_low_gain_hold_${STAMP}.json"

mkdir -p "$ROOT/reports"
ip -j -d link show type can > "$SNAPSHOT"

echo "ACTIVE MOTOR-GROUP HOLD: exactly seven left-arm motors; head yaw excluded"
echo "BUS kcan3; IDs 0x01..0x07"
echo "HOLD measured positions for 2.0s at 50Hz; Kp=0.2 Kd=0.05 tau=0"
echo "GUARDS error<=0.05rad speed<=0.2rad/s; shoulder 4340 torque<=0.5Nm, others<=0.1Nm"
echo "ANY fault disables all seven; exit verifies all seven disabled"
echo "NO motion command; NO mode switch; NO set-zero"
echo "OUTPUT $OUTPUT"

cd "$ROOT"
exec env PYTHONPATH="$ROOT/src" "$ROOT/.venv/bin/python" \
  "$ROOT/tools/hold_left_arm_group_low_gain.py" \
  --hardware "$ROOT/config/hardware.sprite0825.measurement.json" \
  --socketcan-snapshot "$SNAPSHOT" \
  --output "$OUTPUT" \
  --robot-supported \
  --safety-operator-ready \
  --left-arm-clear \
  --confirm-hardware-tx ENABLE_LEFT_ARM_GROUP_LOW_GAIN_HOLD
