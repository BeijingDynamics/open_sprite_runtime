#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 2 || $# -gt 3 ]]; then
  echo "Usage: $0 CAN_TRACE.npz POLICY_TRACE.npz [OUTPUT_PREFIX]" >&2
  exit 2
fi

ROOT=/home/tony/open_sprite_runtime
CAN_TRACE="$1"
POLICY_TRACE="$2"
PREFIX="${3:-$ROOT/captures/system_id/merged_$(date +%Y%m%d_%H%M%S)}"

cd "$ROOT"
PYTHONPATH=src .venv/bin/python tools/merge_system_id_capture.py \
  --can-trace "$CAN_TRACE" \
  --policy-trace "$POLICY_TRACE" \
  --hardware-config config/hardware.sprite0825.measurement.json \
  --output "${PREFIX}.npz" \
  --report "${PREFIX}.json"

sha256sum "${PREFIX}.npz" "${PREFIX}.json" > "${PREFIX}.SHA256SUMS"
echo "MERGED_TRACE ${PREFIX}.npz"
echo "QUALITY_REPORT ${PREFIX}.json"
