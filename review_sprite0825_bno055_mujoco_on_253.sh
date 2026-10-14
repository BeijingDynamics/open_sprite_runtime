#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
MODEL=/home/tony/sprite_runtime/sprite0825_stage2_g74_model3000_sim2real_candidate/assets/mujoco/sprite0825_v5_4340_shoulders/sprite0825_v5_external_pd.xml
export DISPLAY="${DISPLAY:-:1}"
export PYTHONPATH="$ROOT/src${PYTHONPATH:+:$PYTHONPATH}"
echo 'BNO055 ONLY: 100Hz independent reader, latest-state MuJoCo display'
echo 'No motor enable, CAN transmission, policy execution, or IMU configuration writes'
exec python3 "$ROOT/tools/view_bno055.py" \
  --mjcf "$MODEL" --duration "${1:-120}" \
  --output "$ROOT/reports/bno055_view_$(date +%Y%m%d_%H%M%S).json"
