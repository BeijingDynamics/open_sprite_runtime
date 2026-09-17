#!/usr/bin/env bash
set -euo pipefail

ROOT=/home/tony/open_sprite_runtime
MOTOR="${1:-head_yaw_motor}"
DURATION="${2:-2.0}"
RATE_HZ="${3:-50.0}"
STAMP="$(date +%Y%m%d_%H%M%S)"
SNAPSHOT="$ROOT/reports/kh_socketcan_active_${STAMP}.json"
OUTPUT="$ROOT/reports/damiao_zero_gain_${MOTOR}_${STAMP}.json"

mkdir -p "$ROOT/reports"
ip -j -d link show type can > "$SNAPSHOT"

echo "ACTIVE HARDWARE TX: one motor only"
echo "MOTOR $MOTOR"
echo "DURATION $DURATION seconds; RATE $RATE_HZ Hz"
echo "PAYLOAD position echo, velocity=0, Kp=0, Kd=0, feedforward torque=0"
echo "NO automatic motor enable, disable, or mode switch"
echo "The selected mechanism must be supported/unloaded and the drive already in MIT mode."
echo "SNAPSHOT $SNAPSHOT"
echo "OUTPUT $OUTPUT"

cd "$ROOT"
exec env PYTHONPATH="$ROOT/src" python3 -m open_sprite_runtime.cli \
  damiao-zero-gain-probe \
  --hardware-config "$ROOT/config/hardware.sprite0825.measurement.json" \
  --snapshot "$SNAPSHOT" \
  --motor "$MOTOR" \
  --duration "$DURATION" \
  --rate-hz "$RATE_HZ" \
  --print-hz 10 \
  --feedback-timeout 0.2 \
  --output "$OUTPUT" \
  --mit-mode-confirmed \
  --supported-unloaded \
  --acknowledge-hardware-tx ZERO_GAIN_POSITION_ECHO
