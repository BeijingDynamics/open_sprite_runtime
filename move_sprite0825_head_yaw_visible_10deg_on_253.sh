#!/usr/bin/env bash
set -euo pipefail

if [[ "${1:-}" != "ENABLE_HEAD_YAW_VISIBLE_10DEG_MOTION" ]]; then
  echo "VISIBLE POWERED MOTION TEST: bare head_yaw_motor shaft only" >&2
  echo "Robot must be supported; the bare shaft must be mechanically free;" >&2
  echo "a safety operator must hold the independent power cut-off." >&2
  echo "Fixed path: current -> +10deg -> -10deg -> current; 15.0s at 50Hz." >&2
  echo "Fixed gains Kp=2.0 Kd=0.2 tau=0; output torque limited to 0.25Nm." >&2
  echo "Run only after those conditions are true:" >&2
  echo "  $0 ENABLE_HEAD_YAW_VISIBLE_10DEG_MOTION" >&2
  exit 2
fi

ROOT=/home/tony/open_sprite_runtime
STAMP="$(date +%Y%m%d_%H%M%S)"
SNAPSHOT="$ROOT/reports/kcan3_head_yaw_visible_10deg_${STAMP}.json"
OUTPUT="$ROOT/reports/head_yaw_visible_10deg_${STAMP}.json"

mkdir -p "$ROOT/reports"
ip -j -d link show type can > "$SNAPSHOT"

echo "ACTIVE VISIBLE HARDWARE MOTION: exactly one unloaded motor"
echo "MOTOR head_yaw_motor; kcan3 ID 0x08; feedback 0x18; DM-J3507"
echo "TRAJECTORY current -> +10deg -> -10deg -> current; quintic smoothstep"
echo "DURATION 15.0s; RATE 50Hz; Kp=2.0 Kd=0.2 tau=0"
echo "GUARDS error<=0.08rad speed<=0.8rad/s estimated_torque<=0.25Nm"
echo "FINALLY sends disable at least three times and polls zero-gain until disabled"
echo "NO mode switch; NO set-zero; NO other motor command"
echo "OUTPUT $OUTPUT"

cd "$ROOT"
exec env PYTHONPATH="$ROOT/src" "$ROOT/.venv/bin/python" \
  "$ROOT/tools/move_head_yaw_visible_10deg.py" \
  --hardware "$ROOT/config/hardware.sprite0825.measurement.json" \
  --socketcan-snapshot "$SNAPSHOT" \
  --output "$OUTPUT" \
  --robot-supported \
  --safety-operator-ready \
  --head-yaw-mechanically-free \
  --confirm-hardware-tx ENABLE_HEAD_YAW_VISIBLE_10DEG_MOTION
