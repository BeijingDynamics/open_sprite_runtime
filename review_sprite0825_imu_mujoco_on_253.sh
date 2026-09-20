#!/usr/bin/env bash
set -euo pipefail

ROOT=/home/tony/open_sprite_runtime
CANDIDATE=/home/tony/sprite_runtime/sprite0825_stage2_g74_model3000_sim2real_candidate
DURATION="${1:-120}"
DISPLAY_VALUE="${DISPLAY:-:1}"

echo "READ-ONLY IMU -> MUJOCO RELATIVE ATTITUDE"
echo "DURATION ${DURATION}s; DISPLAY ${DISPLAY_VALUE}"
echo "NO serial writes; NO CAN transmission; NO physics stepping"
echo "The startup IMU attitude is displayed as the upright neutral pose"

cd "$ROOT"
exec env DISPLAY="$DISPLAY_VALUE" PYTHONPATH="$ROOT/src" python3 \
  -m open_sprite_runtime.cli imu-mujoco-view \
  --device /dev/ttyCH341USB0 \
  --baud 115200 \
  --duration "$DURATION" \
  --viewer-hz 50 \
  --root-height 0.52 \
  --mjcf "$CANDIDATE/assets/mujoco/sprite0825_v5_4340_shoulders/sprite0825_v5_external_pd.xml"
