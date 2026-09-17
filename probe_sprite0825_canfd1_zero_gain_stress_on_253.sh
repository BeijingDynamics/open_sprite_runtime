#!/usr/bin/env bash
set -euo pipefail

ROOT=/home/tony/open_sprite_runtime
DURATION="${1:-10.0}"
RATE_HZ="${2:-500.0}"
STAMP="$(date +%Y%m%d_%H%M%S)"
SNAPSHOT="$ROOT/reports/kh_socketcan_active_${STAMP}.json"
OUTPUT="$ROOT/reports/damiao_zero_gain_stress_canfd1_${STAMP}.json"
BEFORE="$ROOT/reports/kcan1_stats_before_${STAMP}.txt"
AFTER="$ROOT/reports/kcan1_stats_after_${STAMP}.txt"

MOTORS=(
  left_hip_pitch_motor
  left_hip_roll_motor
  left_hip_yaw_motor
  left_knee_motor
  left_ankle_motor_a
  left_ankle_motor_b
  waist_yaw_motor
  waist_roll_motor
)

mkdir -p "$ROOT/reports"
ip -j -d link show type can > "$SNAPSHOT"
ip -details -statistics link show kcan1 > "$BEFORE"

echo "HEADLESS ACTIVE HARDWARE TX STRESS: CANFD1 group only"
echo "INTERFACE kcan1; MOTORS ${#MOTORS[@]}"
echo "DURATION $DURATION seconds; RATE $RATE_HZ Hz per motor"
echo "PAYLOAD position echo, velocity=0, Kp=0, Kd=0, feedforward torque=0"
echo "NO automatic motor enable, disable, mode switch, or MuJoCo viewer"
echo "OUTPUT $OUTPUT"

cd "$ROOT"
set +e
env PYTHONPATH="$ROOT/src" python3 -m open_sprite_runtime.cli \
  damiao-zero-gain-group-probe \
  --hardware-config "$ROOT/config/hardware.sprite0825.measurement.json" \
  --snapshot "$SNAPSHOT" \
  --interface kcan1 \
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

ip -details -statistics link show kcan1 > "$AFTER"
echo "CAN STATS BEFORE $BEFORE"
echo "CAN STATS AFTER  $AFTER"
exit "$STATUS"
