#!/usr/bin/env bash
set -euo pipefail

ROOT=/home/tony/open_sprite_runtime
CANDIDATE=/home/tony/sprite_runtime/sprite0825_stage2_g74_model3000_sim2real_candidate
MOTOR="${1:-head_yaw_motor}"
DURATION="${2:-120.0}"
RATE_HZ="${3:-50.0}"
DISPLAY_VALUE="${DISPLAY:-:1}"
STAMP="$(date +%Y%m%d_%H%M%S)"
SNAPSHOT="$ROOT/reports/kh_socketcan_active_${STAMP}.json"
OUTPUT="$ROOT/reports/damiao_zero_gain_mujoco_${MOTOR}_${STAMP}.json"

mkdir -p "$ROOT/reports"
ip -j -d link show type can > "$SNAPSHOT"

echo "ACTIVE HARDWARE TX: one motor only"
echo "MOTOR $MOTOR"
echo "DURATION $DURATION seconds; RATE $RATE_HZ Hz; DISPLAY $DISPLAY_VALUE"
echo "PAYLOAD position echo, velocity=0, Kp=0, Kd=0, feedforward torque=0"
echo "NO automatic motor enable, disable, or mode switch"
echo "Only the selected motor updates in MuJoCo; all unobserved joints remain neutral"
echo "OUTPUT $OUTPUT"

cd "$ROOT"
exec env DISPLAY="$DISPLAY_VALUE" PYTHONPATH="$ROOT/src" python3 -m open_sprite_runtime.cli \
  damiao-zero-gain-probe \
  --hardware-config "$ROOT/config/hardware.sprite0825.measurement.json" \
  --snapshot "$SNAPSHOT" \
  --motor "$MOTOR" \
  --duration "$DURATION" \
  --rate-hz "$RATE_HZ" \
  --print-hz 10 \
  --feedback-timeout 0.2 \
  --output "$OUTPUT" \
  --mit-mode-confirmed \
  --supported-unloaded \
  --acknowledge-hardware-tx ZERO_GAIN_POSITION_ECHO \
  --viewer \
  --contract "$CANDIDATE/deploy/contract.json" \
  --mjcf "$CANDIDATE/assets/mujoco/sprite0825_v5_4340_shoulders/sprite0825_v5_external_pd.xml" \
  --display "$DISPLAY_VALUE" \
  --root-height 0.52 \
  --viewer-hz 50
