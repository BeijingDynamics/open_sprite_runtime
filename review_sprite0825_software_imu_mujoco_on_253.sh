#!/usr/bin/env bash
set -euo pipefail
ROOT=/home/tony/open_sprite_runtime
CANDIDATE="$ROOT/candidates/bno_software_20261014"
export DISPLAY="${DISPLAY:-:1}"
export PYTHONPATH="$ROOT/src${PYTHONPATH:+:$PYTHONPATH}"
exec python3 "$CANDIDATE/tools/view_software_imu.py" \
  --duration "${1:-120}" \
  --mjcf /home/tony/sprite_runtime/sprite0825_stage2_g74_model3000_sim2real_candidate/assets/mujoco/sprite0825_v5_4340_shoulders/sprite0825_v5_external_pd.xml
