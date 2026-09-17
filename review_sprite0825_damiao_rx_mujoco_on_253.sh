#!/usr/bin/env bash
set -euo pipefail

ROOT=/home/tony/open_sprite_runtime
CANDIDATE=/home/tony/sprite_runtime/sprite0825_stage2_g74_model3000_sim2real_candidate
DURATION="${1:-600}"
DISPLAY_VALUE="${DISPLAY:-:1}"
STAMP="$(date +%Y%m%d_%H%M%S)"
SNAPSHOT="$ROOT/reports/kh_socketcan_snapshot_${STAMP}.json"
OUTPUT="$ROOT/reports/damiao_rx_mujoco_${STAMP}.json"

mkdir -p "$ROOT/reports"
ip -j -d link show type can > "$SNAPSHOT"

echo "MODE receive-only; this script has no CAN send or motor-enable path"
echo "DISPLAY $DISPLAY_VALUE"
echo "DURATION $DURATION seconds; close the MuJoCo window to stop early"
echo "SNAPSHOT $SNAPSHOT"
echo "OUTPUT $OUTPUT"
echo "NOTE head pitch/roll use the qualified differential calibration in the hardware config"
echo "NOTE existing motor feedback traffic is required; this script never polls a silent bus"

cd "$ROOT"
exec env DISPLAY="$DISPLAY_VALUE" PYTHONPATH="$ROOT/src" python3 -m open_sprite_runtime.cli \
  damiao-rx-audit \
  --hardware-config "$ROOT/config/hardware.sprite0825.measurement.json" \
  --snapshot "$SNAPSHOT" \
  --duration "$DURATION" \
  --output "$OUTPUT" \
  --viewer \
  --contract "$CANDIDATE/deploy/contract.json" \
  --mjcf "$CANDIDATE/assets/mujoco/sprite0825_v5_4340_shoulders/sprite0825_v5_external_pd.xml"
