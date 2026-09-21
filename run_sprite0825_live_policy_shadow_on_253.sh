#!/usr/bin/env bash
set -euo pipefail

ROOT=/home/tony/open_sprite_runtime
CANDIDATE=/home/tony/sprite_runtime/sprite0825_stage2_g74_model3000_sim2real_candidate
DURATION="${1:-10.0}"
VIEW_MODE="${2:-headless}"
STAMP="$(date +%Y%m%d_%H%M%S)"
SNAPSHOT="$ROOT/reports/kh_socketcan_policy_shadow_${STAMP}.json"
POST_SNAPSHOT="$ROOT/reports/kh_socketcan_policy_shadow_post_${STAMP}.json"
OUTPUT="$ROOT/reports/live_policy_shadow_${STAMP}.json"
VIEWER_ARGS=()

if [[ "$VIEW_MODE" == "viewer" ]]; then
  VIEWER_ARGS=(
    --viewer
    --display "${DISPLAY:-:1}"
    --mjcf "$CANDIDATE/assets/mujoco/sprite0825_v5_4340_shoulders/sprite0825_v5_external_pd.xml"
  )
elif [[ "$VIEW_MODE" != "headless" ]]; then
  echo "Usage: $0 [duration_s] [headless|viewer]" >&2
  exit 2
fi

mkdir -p "$ROOT/reports"
ip -j -d link show type can > "$SNAPSHOT"

echo "LIVE POLICY SHADOW: real 31-motor feedback + pelvis IMU -> G74 policy"
echo "DURATION ${DURATION}s; POLICY 50Hz; ANKLE CONTROL MATH 500Hz; MODE $VIEW_MODE"
echo "ACTIVE TX is restricted to position echo with velocity/Kp/Kd/torque all zero"
echo "NO nonzero motor command, enable, disable, or mode switch can be transmitted"
echo "The robot must remain mechanically supported and every motor disabled"
echo "COMMAND vx=0 vy=0 yaw_rate=0"
echo "OUTPUT $OUTPUT"

cd "$ROOT"
set +e
PYTHONPATH="$ROOT/src" "$ROOT/.venv/bin/python" \
  -m open_sprite_runtime.cli live-policy-shadow-probe \
  --hardware-config "$ROOT/config/hardware.sprite0825.measurement.json" \
  --snapshot "$SNAPSHOT" \
  --contract "$CANDIDATE/deploy/contract.json" \
  --output "$OUTPUT" \
  --imu-device /dev/ttyCH341USB0 \
  --imu-baud 115200 \
  --duration "$DURATION" \
  --rate-hz 50 \
  --feedback-timeout 0.2 \
  --minimum-sample-coverage 0.85 \
  --minimum-imu-rate-hz 80 \
  --vx 0 --vy 0 --yaw-rate 0 \
  --mit-mode-confirmed \
  --supported-unloaded \
  --all-motors-disabled-confirmed \
  --acknowledge-hardware-tx ZERO_GAIN_POLICY_SHADOW \
  "${VIEWER_ARGS[@]}"
status=$?
set -e

ip -j -d -s link show type can > "$POST_SNAPSHOT"
echo "POST CAN STATS $POST_SNAPSHOT"
exit "$status"
