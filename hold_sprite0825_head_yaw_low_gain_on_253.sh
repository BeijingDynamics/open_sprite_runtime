#!/usr/bin/env bash
set -euo pipefail

if [[ "${1:-}" != "ENABLE_HEAD_YAW_LOW_GAIN_HOLD" ]]; then
  echo "FIRST POWERED CLOSED-LOOP TEST: head_yaw_motor only" >&2
  echo "Robot must be supported; head yaw must be mechanically free;" >&2
  echo "a safety operator must hold the independent power cut-off." >&2
  echo "Fixed command: measured-position hold, 2s, 50Hz, Kp=0.2, Kd=0.05, tau=0." >&2
  echo "Run only after those conditions are true:" >&2
  echo "  $0 ENABLE_HEAD_YAW_LOW_GAIN_HOLD" >&2
  exit 2
fi

ROOT=/home/tony/open_sprite_runtime
STAMP="$(date +%Y%m%d_%H%M%S)"
SNAPSHOT="$ROOT/reports/kcan3_head_yaw_hold_${STAMP}.json"
OUTPUT="$ROOT/reports/head_yaw_low_gain_hold_${STAMP}.json"

mkdir -p "$ROOT/reports"
ip -j -d link show type can > "$SNAPSHOT"

echo "ACTIVE HARDWARE CONTROL: exactly one motor"
echo "MOTOR head_yaw_motor; kcan3 ID 0x08; feedback 0x18; DM-J3507"
echo "HOLD measured position for 2.0s at 50Hz; Kp=0.2 Kd=0.05 tau=0"
echo "GUARDS error<=0.05rad speed<=0.2rad/s estimated_torque<=0.1Nm"
echo "FINALLY sends disable at least three times and polls zero-gain until disabled"
echo "NO mode switch; NO set-zero; NO other motor command"
echo "OUTPUT $OUTPUT"

cd "$ROOT"
exec env PYTHONPATH="$ROOT/src" "$ROOT/.venv/bin/python" \
  "$ROOT/tools/hold_head_yaw_low_gain.py" \
  --hardware "$ROOT/config/hardware.sprite0825.measurement.json" \
  --socketcan-snapshot "$SNAPSHOT" \
  --output "$OUTPUT" \
  --robot-supported \
  --safety-operator-ready \
  --head-yaw-mechanically-free \
  --confirm-hardware-tx ENABLE_HEAD_YAW_LOW_GAIN_HOLD
