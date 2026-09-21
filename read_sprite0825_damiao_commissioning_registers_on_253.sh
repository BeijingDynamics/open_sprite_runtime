#!/usr/bin/env bash
set -euo pipefail

ROOT=/home/tony/open_sprite_runtime
STAMP=$(date +%Y%m%d_%H%M%S)
SNAPSHOT="$ROOT/reports/kcan_active_snapshot_${STAMP}.json"
OUTPUT="$ROOT/reports/damiao_commissioning_registers_${STAMP}.json"

echo "READ-ONLY HARDWARE TX: Damiao parameter queries only"
echo "Allowed CAN ID: 0x7FF; opcode: 0x33; RIDs: 2/3/6/13/14/36"
echo "Reads OT/OC/MAX_SPD and hardware/software/sub-version only"
echo "No write-register, save, enable, disable, mode-switch, or MIT control frames"
echo "Robot must remain supported and all 31 motors must be disabled"
echo "OUTPUT $OUTPUT"

mkdir -p "$ROOT/reports"
ip -j -d link show type can >"$SNAPSHOT"
cd "$ROOT"
PYTHONPATH=src .venv/bin/python tools/read_damiao_commissioning_registers.py \
  --hardware config/hardware.sprite0825.measurement.json \
  --snapshot "$SNAPSHOT" \
  --output "$OUTPUT" \
  --supported-unloaded \
  --all-motors-disabled-confirmed \
  --acknowledge-hardware-tx READ_ONLY_DAMIAO_COMMISSIONING_REGISTERS
