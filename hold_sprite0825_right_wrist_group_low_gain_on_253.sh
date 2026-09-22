#!/usr/bin/env bash
set -euo pipefail

if [[ "${1:-}" != "ENABLE_RIGHT_WRIST_GROUP_LOW_GAIN_HOLD" ]]; then
  echo "FIRST FIXED MOTOR-GROUP HOLD: right wrist yaw/pitch/roll only" >&2
  echo "Robot must be supported; wrist path/cables must be clear;" >&2
  echo "a safety operator must hold the independent power cut-off." >&2
  echo "Fixed hold: measured positions, 2s, 50Hz, Kp=0.2 Kd=0.05 tau=0." >&2
  echo "Run only after those conditions are true:" >&2
  echo "  $0 ENABLE_RIGHT_WRIST_GROUP_LOW_GAIN_HOLD" >&2
  exit 2
fi

ROOT=/home/tony/open_sprite_runtime
STAMP="$(date +%Y%m%d_%H%M%S)"
SNAPSHOT="$ROOT/reports/kcan4_right_wrist_group_hold_${STAMP}.json"
OUTPUT="$ROOT/reports/right_wrist_group_low_gain_hold_${STAMP}.json"

mkdir -p "$ROOT/reports"
ip -j -d link show type can > "$SNAPSHOT"

echo "ACTIVE MOTOR-GROUP HOLD: exactly three right-wrist motors"
echo "BUS kcan4; IDs 0x05/0x06/0x07; no shoulder or elbow command"
echo "HOLD measured positions for 2.0s at 50Hz; Kp=0.2 Kd=0.05 tau=0"
echo "GUARDS each motor: error<=0.05rad speed<=0.2rad/s torque<=0.1Nm"
echo "ANY fault disables all three; exit verifies all three disabled"
echo "NO motion command; NO mode switch; NO set-zero"
echo "OUTPUT $OUTPUT"

cd "$ROOT"
exec env PYTHONPATH="$ROOT/src" "$ROOT/.venv/bin/python" \
  "$ROOT/tools/hold_right_wrist_group_low_gain.py" \
  --hardware "$ROOT/config/hardware.sprite0825.measurement.json" \
  --socketcan-snapshot "$SNAPSHOT" \
  --output "$OUTPUT" \
  --robot-supported \
  --safety-operator-ready \
  --right-wrist-clear \
  --confirm-hardware-tx ENABLE_RIGHT_WRIST_GROUP_LOW_GAIN_HOLD
