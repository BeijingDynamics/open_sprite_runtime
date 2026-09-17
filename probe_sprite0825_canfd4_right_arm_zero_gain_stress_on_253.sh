#!/usr/bin/env bash
set -euo pipefail

ROOT=/home/tony/open_sprite_runtime
DURATION="${1:-10.0}"
RATE_HZ="${2:-500.0}"
STAMP="$(date +%Y%m%d_%H%M%S)"
SNAPSHOT="$ROOT/reports/kh_socketcan_active_${STAMP}.json"
OUTPUT="$ROOT/reports/damiao_zero_gain_stress_canfd4_right_arm_${STAMP}.json"
BEFORE="$ROOT/reports/kcan4_stats_before_${STAMP}.txt"
AFTER="$ROOT/reports/kcan4_stats_after_${STAMP}.txt"

MOTORS=(
  right_shoulder_pitch_motor
  right_shoulder_roll_motor
  right_shoulder_yaw_motor
  right_elbow_motor
  right_wrist_yaw_motor
  right_wrist_pitch_motor
  right_wrist_roll_motor
)

mkdir -p "$ROOT/reports"
ip -j -d link show type can > "$SNAPSHOT"
ip -details -statistics link show kcan4 > "$BEFORE"

echo "HEADLESS ACTIVE HARDWARE TX STRESS: CANFD4 right-arm group only"
echo "INTERFACE kcan4; MOTORS ${#MOTORS[@]}"
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
  --interface kcan4 \
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

ip -details -statistics link show kcan4 > "$AFTER"
echo "CAN STATS BEFORE $BEFORE"
echo "CAN STATS AFTER  $AFTER"
exit "$STATUS"
