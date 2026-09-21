#!/usr/bin/env bash
set -euo pipefail

DURATION="${1:-10}"
ROOT=/home/tony/open_sprite_runtime
CONFIG="$ROOT/build/native/motors.tsv"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUTPUT="$ROOT/reports/native_mixed_rate_can_shadow_${STAMP}.json"

PYTHONPATH="$ROOT/src" "$ROOT/.venv/bin/python" \
  "$ROOT/tools/export_native_motor_config.py" \
  --hardware "$ROOT/config/hardware.sprite0825.measurement.json" \
  --output "$CONFIG"

echo "NATIVE MIXED-RATE CAN SHADOW: ankle motors 500Hz, all other motors 50Hz"
echo "ACTIVE TX is restricted to position echo with velocity/Kp/Kd/torque all zero"
echo "NO motor enable, disable, mode switch, or nonzero control frame"
echo "The robot must remain mechanically supported and every motor disabled"
echo "DURATION ${DURATION}s; CPU 5; OUTPUT $OUTPUT"

exec "$ROOT/build/native/sprite_can_shadow" \
  --config "$CONFIG" \
  --duration "$DURATION" \
  --cpu 5 \
  --output "$OUTPUT" \
  --acknowledge-hardware-tx ZERO_GAIN_NATIVE_SHADOW \
  --all-motors-disabled-confirmed

