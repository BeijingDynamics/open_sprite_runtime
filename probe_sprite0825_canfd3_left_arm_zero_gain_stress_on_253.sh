#!/usr/bin/env bash
set -euo pipefail

ROOT=/home/tony/open_sprite_runtime
DURATION="${1:-10.0}"
RATE_HZ="${2:-500.0}"
STAMP="$(date +%Y%m%d_%H%M%S)"
SNAPSHOT="$ROOT/reports/kh_socketcan_active_${STAMP}.json"
OUTPUT="$ROOT/reports/damiao_zero_gain_stress_canfd3_left_arm_${STAMP}.json"
BEFORE="$ROOT/reports/kcan3_stats_before_${STAMP}.txt"
AFTER="$ROOT/reports/kcan3_stats_after_${STAMP}.txt"

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
ip -details -statistics link show kcan3 > "$BEFORE"

echo "HEADLESS ACTIVE HARDWARE TX STRESS: CANFD3 left-arm and head-yaw group only"
echo "INTERFACE kcan3; MOTORS ${#MOTORS[@]}"
echo "DURATION $DURATION seconds; RATE $RATE_HZ Hz per motor"
echo "TOTAL NOMINAL TX RATE $(awk "BEGIN {print ${#MOTORS[@]} * $RATE_HZ}") Hz"
echo "PAYLOAD position echo, velocity=0, Kp=0, Kd=0, feedforward torque=0"
echo "NO automatic motor enable, disable, mode switch, or MuJoCo viewer"
echo "OUTPUT $OUTPUT"

cd "$ROOT"
set +e
env PYTHONPATH="$ROOT/src" python3 -m open_sprite_runtime.cli \
  damiao-zero-gain-group-probe \
  --hardware-config "$ROOT/config/hardware.sprite0825.measurement.json" \
  --snapshot "$SNAPSHOT" \
  --interface kcan3 \
  --motors "${MOTORS[@]}" \
  --duration "$DURATION" \
  --rate-hz "$RATE_HZ" \
  --print-hz 0.1 \
  --feedback-timeout 0.2 \
  --minimum-sample-coverage 0.95 \
  --output "$OUTPUT" \
  --mit-mode-confirmed \
  --supported-unloaded \
  --acknowledge-hardware-tx ZERO_GAIN_GROUP_POSITION_ECHO
STATUS=$?
set -e

ip -details -statistics link show kcan3 > "$AFTER"
echo "CAN STATS BEFORE $BEFORE"
echo "CAN STATS AFTER  $AFTER"
exit "$STATUS"
