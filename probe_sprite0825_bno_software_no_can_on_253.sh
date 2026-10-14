#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
seconds=${1:-60}
rate=${2:-100}
method=${3:-burst18}
if [[ "$rate" != 100 && "$rate" != 200 ]] || [[ "$method" != burst18 && "$method" != split6 ]]; then
  echo 'Rate:100 or200; method:burst18 or split6' >&2; exit 2
fi
if [[ ! "$seconds" =~ ^[0-9]+$ ]] || ((seconds<20 || seconds>120)); then
  echo 'Duration must be an integer20..120 seconds (includes15s static calibration)' >&2
  exit 2
fi
if pgrep -f 'sprite_can_shadow|sprite_policy_native|run_native_policy_hardware_shadow' >/dev/null; then
  echo 'Stop CAN/policy processes before changing IMU mode.' >&2; exit 2
fi
mkdir -p reports
run=$(mktemp -d "reports/bno_software_$(date +%Y%m%d_%H%M%S)_XXXXXX")
socket_dir=$(mktemp -d /tmp/sprite_software_imu_XXXXXX)
trap 'rmdir "$socket_dir" 2>/dev/null || true' EXIT
echo 'NO CAN. Configures BNO055 ACCGYRO/P0 and host six-axis estimation.'
echo 'Keep robot supported and completely still for first15 seconds.'
echo 'No startup leveling. No motor enable, zero reset, or policy execution.'
echo 'Leaves BNO in raw mode; old fusion launchers must reconfigure IMUPLUS.'
build/imu_software_candidate/sprite_imu_software --record "$seconds" \
  "$socket_dir/imu.sock" "$run/report.json" CONFIGURE_RAW_P0_NO_CAN "$rate" "$method"
echo "REPORT $PWD/$run/report.json"
