#!/usr/bin/env bash
set -euo pipefail

ACK=ENABLE_WAIST_GROUP_LOW_GAIN_HOLD
if [[ "${1:-}" != "$ACK" ]]; then
  echo "FIXED MOTOR-GROUP HOLD: waist yaw/roll only" >&2
  echo "Robot must be suspended; waist clear; safety operator at power cut-off." >&2
  echo "2s, 50Hz, Kp=0.2 Kd=0.05 tau=0; no set-zero or mode switch." >&2
  echo "Run only after explicit approval: $0 $ACK" >&2
  exit 2
fi

ROOT=/home/tony/open_sprite_runtime
STAMP="$(date +%Y%m%d_%H%M%S)"
SNAPSHOT="$ROOT/reports/kcan1_waist_group_hold_${STAMP}.json"
OUTPUT="$ROOT/reports/waist_group_low_gain_hold_${STAMP}.json"
mkdir -p "$ROOT/reports"
ip -j -d link show type can > "$SNAPSHOT"
echo "ACTIVE MOTOR-GROUP HOLD: waist yaw/roll kcan1 IDs 0x07/0x08 only"
echo "2.0s at 50Hz; Kp=0.2 Kd=0.05 tau=0"
echo "COMMAND torque guard 0.5Nm; FEEDBACK guards yaw 0.5Nm, roll 2.5Nm"
echo "Roll feedback guard is based on a disabled 500-sample noise baseline"
echo "NO set-zero; NO mode switch; any fault disables and verifies both motors"
cd "$ROOT"
exec env PYTHONPATH="$ROOT/src" "$ROOT/.venv/bin/python" \
  "$ROOT/tools/hold_waist_group_low_gain.py" \
  --hardware "$ROOT/config/hardware.sprite0825.measurement.json" \
  --socketcan-snapshot "$SNAPSHOT" --output "$OUTPUT" \
  --robot-supported --safety-operator-ready --waist-clear \
  --confirm-hardware-tx "$ACK"
