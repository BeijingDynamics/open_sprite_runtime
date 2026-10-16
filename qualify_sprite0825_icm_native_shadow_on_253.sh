#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
duration="${1:-120}"
[[ "$duration" == 10 || "$duration" == 40 || "$duration" == 120 ]] || exit 2
[[ "$EUID" == 0 && "${SUDO_USER:-}" == tony ]] || { echo 'Run with sudo as tony; SPI runtime PM is restored on exit.'; exit 2; }
exec 9>/run/lock/sprite_icm_spi_pm.lock
flock -n 9 || { echo 'Another ICM test holds PM lock'; exit 2; }
pm=/sys/bus/platform/devices/3210000.spi/power/control
old=$(<"$pm")
trap 'printf "%s\n" "$old" > "$pm"' EXIT
printf 'on\n' > "$pm"
echo 'ZERO GAIN ONLY. No enable, motion, nonzero Kp/Kd/torque. Keep stationary first 15s.'
runuser -u tony -- env PYTHONPATH="$PWD/src" "$PWD/.venv/bin/python" \
  "$PWD/tools/run_native_policy_hardware_shadow.py" --runtime-root "$PWD" \
  --icm-candidate "$PWD/candidates/icm_native_20261016" --duration "$duration" \
  --confirm ZERO_GAIN_ALL_DISABLED_SUPPORTED
