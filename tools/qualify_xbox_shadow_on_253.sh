#!/usr/bin/env bash
set -euo pipefail
[[ $EUID == 0 && ${SUDO_USER:-} == tony ]] || { echo 'Run with sudo as tony'; exit 2; }
[[ ${1:-} == ZERO_GAIN_ALL_DISABLED_SUPPORTED ]] || { echo 'Requires mechanical support, all motors disabled but powered, configured CAN'; exit 2; }
cd /home/tony/open_sprite_runtime
exec 9>/run/lock/sprite_icm_spi_pm.lock
flock -n 9 || exit 2
pm=/sys/bus/platform/devices/3210000.spi/power/control
old=$(cat "$pm")
[[ $old == auto || $old == on ]] || exit 2
trap 'printf "%s\n" "$old" > "$pm"' EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
printf 'on\n' > "$pm"
echo 'ZERO GAIN ONLY. No enable or nonzero impedance/torque. Keep still for initial 15s calibration.'
echo '40s shadow: first 10s stand; next 20s gamepad window; last 10s stop/stand.'
echo 'After window opens: center sticks, squeeze/release LT, hold LT again, move sticks.'
runuser -u tony -- env PYTHONPATH="$PWD/src" OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 \
  .venv/bin/python candidates/xbox_native_20261019/run_xbox_hardware_shadow.py \
  --runtime-root "$PWD" --icm-candidate "$PWD/candidates/icm_native_20261016" \
  --xbox-candidate "$PWD/candidates/xbox_native_20261019" --duration 40 \
  --command-profile xbox-shadow40 --confirm ZERO_GAIN_ALL_DISABLED_SUPPORTED
