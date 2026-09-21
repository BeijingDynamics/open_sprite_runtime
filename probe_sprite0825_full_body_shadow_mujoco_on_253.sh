#!/usr/bin/env bash
set -euo pipefail

ROOT=/home/tony/open_sprite_runtime
CANDIDATE=/home/tony/sprite_runtime/sprite0825_stage2_g74_model3000_sim2real_candidate
DURATION="${1:-30.0}"
RATE_HZ="${2:-50.0}"
DISPLAY_VALUE="${DISPLAY:-:1}"
STAMP="$(date +%Y%m%d_%H%M%S)"
SNAPSHOT="$ROOT/reports/kh_socketcan_full_body_${STAMP}.json"
POST_SNAPSHOT="$ROOT/reports/kh_socketcan_full_body_post_${STAMP}.json"
OUTPUT="$ROOT/reports/full_body_shadow_${STAMP}.json"

mkdir -p "$ROOT/reports"
ip -j -d link show type can > "$SNAPSHOT"

echo "FULL-BODY COMMISSIONING: 31 disabled motors + pelvis IMU"
echo "DURATION ${DURATION}s; RATE ${RATE_HZ}Hz per motor; DISPLAY ${DISPLAY_VALUE}"
echo "ACTIVE TX: position echo with velocity=0, Kp=0, Kd=0, torque=0"
echo "NO motor enable, disable, mode switch, or nonzero torque command"
echo "The complete robot must be mechanically supported and every motor disabled"
echo "Close the MuJoCo viewer to stop early"
echo "OUTPUT $OUTPUT"
echo "POST CAN STATS $POST_SNAPSHOT"

cd "$ROOT"
set +e
env DISPLAY="$DISPLAY_VALUE" PYTHONPATH="$ROOT/src" python3 \
  -m open_sprite_runtime.cli full-body-shadow-probe \
  --hardware-config "$ROOT/config/hardware.sprite0825.measurement.json" \
  --snapshot "$SNAPSHOT" \
  --contract "$CANDIDATE/deploy/contract.json" \
  --mjcf "$CANDIDATE/assets/mujoco/sprite0825_v5_4340_shoulders/sprite0825_v5_external_pd.xml" \
  --output "$OUTPUT" \
  --imu-device /dev/ttyCH341USB0 \
  --imu-baud 115200 \
  --duration "$DURATION" \
  --rate-hz "$RATE_HZ" \
  --feedback-timeout 0.2 \
  --minimum-sample-coverage 0.90 \
  --minimum-imu-rate-hz 80 \
  --display "$DISPLAY_VALUE" \
  --root-height 0.52 \
  --viewer-hz 50 \
  --mit-mode-confirmed \
  --supported-unloaded \
  --all-motors-disabled-confirmed \
  --acknowledge-hardware-tx ZERO_GAIN_FULL_BODY_POSITION_ECHO
status=$?
set -e

ip -j -d -s link show type can > "$POST_SNAPSHOT"
echo
echo "POST-RUN CAN STATE"
for interface in kcan1 kcan2 kcan3 kcan4; do
  ip -details -statistics link show dev "$interface"
done

exit "$status"
