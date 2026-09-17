#!/usr/bin/env bash
set -euo pipefail

ROOT=/home/tony/open_sprite_runtime
CANDIDATE=/home/tony/sprite_runtime/sprite0825_stage2_g74_model3000_sim2real_candidate
DURATION="${1:-120.0}"
RATE_HZ="${2:-50.0}"
DISPLAY_VALUE="${DISPLAY:-:1}"
STAMP="$(date +%Y%m%d_%H%M%S)"
SNAPSHOT="$ROOT/reports/kh_socketcan_active_${STAMP}.json"
OUTPUT="$ROOT/reports/damiao_zero_gain_mujoco_canfd3_left_arm_${STAMP}.json"

MOTORS=(
  left_shoulder_pitch_motor
  left_shoulder_roll_motor
  left_shoulder_yaw_motor
  left_elbow_motor
  left_wrist_yaw_motor
  left_wrist_pitch_motor
  left_wrist_roll_motor
  head_yaw_motor
)

mkdir -p "$ROOT/reports"
ip -j -d link show type can > "$SNAPSHOT"

echo "ACTIVE HARDWARE TX: CANFD3 left-arm and head-yaw group only"
echo "INTERFACE kcan3; MOTORS ${#MOTORS[@]}"
echo "DURATION $DURATION seconds; RATE $RATE_HZ Hz per motor; DISPLAY $DISPLAY_VALUE"
echo "PAYLOAD position echo, velocity=0, Kp=0, Kd=0, feedforward torque=0"
echo "NO automatic motor enable, disable, or mode switch"
echo "Support the arm while checking every joint direction and mechanical limit"
echo "OUTPUT $OUTPUT"

cd "$ROOT"
exec env DISPLAY="$DISPLAY_VALUE" PYTHONPATH="$ROOT/src" python3 -m open_sprite_runtime.cli \
  damiao-zero-gain-group-probe \
  --hardware-config "$ROOT/config/hardware.sprite0825.measurement.json" \
  --snapshot "$SNAPSHOT" \
  --interface kcan3 \
  --motors "${MOTORS[@]}" \
  --duration "$DURATION" \
  --rate-hz "$RATE_HZ" \
  --print-hz 2 \
  --feedback-timeout 0.2 \
  --minimum-sample-coverage 0.95 \
  --output "$OUTPUT" \
  --mit-mode-confirmed \
  --supported-unloaded \
  --acknowledge-hardware-tx ZERO_GAIN_GROUP_POSITION_ECHO \
  --viewer \
  --contract "$CANDIDATE/deploy/contract.json" \
  --mjcf "$CANDIDATE/assets/mujoco/sprite0825_v5_4340_shoulders/sprite0825_v5_external_pd.xml" \
  --display "$DISPLAY_VALUE" \
  --root-height 0.52 \
  --viewer-hz 60
