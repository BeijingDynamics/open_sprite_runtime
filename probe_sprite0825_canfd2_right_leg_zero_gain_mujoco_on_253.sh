#!/usr/bin/env bash
set -euo pipefail

ROOT=/home/tony/open_sprite_runtime
CANDIDATE=/home/tony/sprite_runtime/sprite0825_stage2_g74_model3000_sim2real_candidate
DURATION="${1:-120.0}"
RATE_HZ="${2:-500.0}"
DISPLAY_VALUE="${DISPLAY:-:1}"
STAMP="$(date +%Y%m%d_%H%M%S)"
SNAPSHOT="$ROOT/reports/kh_socketcan_active_${STAMP}.json"
OUTPUT="$ROOT/reports/damiao_zero_gain_mujoco_canfd2_${STAMP}.json"

MOTORS=(
  right_hip_pitch_motor
  right_hip_roll_motor
  right_hip_yaw_motor
  right_knee_motor
  right_ankle_motor_a
  right_ankle_motor_b
  head_motor_a
  head_motor_b
)

mkdir -p "$ROOT/reports"
ip -j -d link show type can > "$SNAPSHOT"

echo "ACTIVE HARDWARE TX: CANFD2 group only"
echo "INTERFACE kcan2; MOTORS ${#MOTORS[@]}"
echo "DURATION $DURATION seconds; RATE $RATE_HZ Hz per motor; DISPLAY $DISPLAY_VALUE"
echo "PAYLOAD position echo, velocity=0, Kp=0, Kd=0, feedforward torque=0"
echo "NO automatic motor enable, disable, or mode switch"
echo "The robot must be mechanically supported because only CANFD2 is observed"
echo "Right ankle and head differential feedback are converted to joint pitch/roll"
echo "OUTPUT $OUTPUT"

cd "$ROOT"
exec env DISPLAY="$DISPLAY_VALUE" PYTHONPATH="$ROOT/src" python3 -m open_sprite_runtime.cli \
  damiao-zero-gain-group-probe \
  --hardware-config "$ROOT/config/hardware.sprite0825.measurement.json" \
  --snapshot "$SNAPSHOT" \
  --interface kcan2 \
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
  --viewer-hz 50
