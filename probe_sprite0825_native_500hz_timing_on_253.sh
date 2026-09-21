#!/usr/bin/env bash
set -euo pipefail

DURATION="${1:-30}"
CPU="${2:-5}"
ROOT=/home/tony/open_sprite_runtime
STAMP="$(date +%Y%m%d_%H%M%S)"
OUTPUT="$ROOT/reports/native_500hz_timing_${STAMP}.json"

echo "NATIVE 500 HZ TIMING: no CAN socket is opened; hardware TX attempts are zero"
echo "DURATION ${DURATION}s; CPU ${CPU}; OUTPUT $OUTPUT"
exec "$ROOT/build/native/sprite_rt_timing" \
  --duration "$DURATION" \
  --rate 500 \
  --cpu "$CPU" \
  --maximum-p99-lateness-ms 0.5 \
  --maximum-lateness-ms 2.0 \
  --output "$OUTPUT"
