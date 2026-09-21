#!/usr/bin/env bash
set -euo pipefail

if [[ "${1:-}" != "SET_HEAD_ZERO" ]]; then
  echo "Persistent hardware operation: this sets the current positions of" >&2
  echo "kcan2 motor IDs 0x07 and 0x08 as their stored zero positions." >&2
  echo "Position the unloaded head pitch/roll mechanism at its exact neutral pose," >&2
  echo "confirm every motor is disabled, then run:" >&2
  echo "  $0 SET_HEAD_ZERO" >&2
  exit 2
fi

ROOT=/home/tony/open_sprite_runtime
STAMP="$(date +%Y%m%d_%H%M%S)"
SNAPSHOT="$ROOT/reports/kh_socketcan_head_zero_${STAMP}.json"
OUTPUT="$ROOT/reports/damiao_head_zero_${STAMP}.json"

mkdir -p "$ROOT/reports"
ip -j -d link show type can > "$SNAPSHOT"

echo "PERSISTENT DAMIAO ZERO WRITE"
echo "TARGET 1: kcan2 ID 0x07 head_motor_a, feedback 0x17"
echo "TARGET 2: kcan2 ID 0x08 head_motor_b, feedback 0x18"
echo "FRAME: FF FF FF FF FF FF FF FE"
echo "NO motor enable and NO mode switch"
echo "OUTPUT $OUTPUT"

cd "$ROOT"
exec env PYTHONPATH="$ROOT/src" python3 -m open_sprite_runtime.cli \
  damiao-set-head-zero \
  --hardware-config "$ROOT/config/hardware.sprite0825.measurement.json" \
  --snapshot "$SNAPSHOT" \
  --output "$OUTPUT" \
  --probe-duration 0.5 \
  --probe-rate-hz 50 \
  --feedback-timeout 0.2 \
  --settle-time 0.2 \
  --maximum-zero-error-rad 0.03 \
  --head-mechanism-positioned-at-zero \
  --all-motors-disabled-confirmed \
  --acknowledge-hardware-tx SET_HEAD_DIFFERENTIAL_ZERO_07_08
