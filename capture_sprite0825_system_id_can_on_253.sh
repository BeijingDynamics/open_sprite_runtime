#!/usr/bin/env bash
set -euo pipefail

DURATION="${1:-70}"
SESSION="${2:-$(date +%Y%m%d_%H%M%S)}"
CPU="${SPRITE_CAN_CAPTURE_CPU:-2}"
ROOT=/home/tony/open_sprite_runtime
OUTPUT_DIR="$ROOT/captures/system_id/$SESSION"
TRACE="$OUTPUT_DIR/can_raw.npz"
REPORT="$OUTPUT_DIR/can_capture_report.json"

mkdir -p "$OUTPUT_DIR"
echo "RECEIVE-ONLY system-identification CAN capture"
echo "DURATION $DURATION seconds; CPU $CPU"
echo "INTERFACES kcan1 kcan2 kcan3 kcan4"
echo "NO CAN transmission API is available to this recorder"
echo "TRACE $TRACE"

cd "$ROOT"
PYTHONPATH=src taskset -c "$CPU" .venv/bin/python \
  tools/capture_active_can_trace.py \
  --duration "$DURATION" \
  --trace "$TRACE" \
  --report "$REPORT"

sha256sum "$TRACE" "$REPORT" > "$OUTPUT_DIR/SHA256SUMS"
echo "CAPTURE_COMPLETE $OUTPUT_DIR"
