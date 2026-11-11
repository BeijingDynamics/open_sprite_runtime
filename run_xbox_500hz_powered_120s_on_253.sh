#!/usr/bin/env bash
set -euo pipefail
experiment="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
candidate="$experiment/candidates/xbox_500hz_powered_20261111"
stable=/home/tony/open_sprite_runtime
[[ $# == 1 ]] || { echo 'Usage: --check | --preflight-only | ENABLE_XBOX_500HZ_TURN120'; exit 2; }
case "$1" in
  --check)
    "$stable/.venv/bin/python" "$experiment/tools/check_xbox500_powered.py"
    exec bash "$candidate/inner.sh" --check
    ;;
  --preflight-only) preflight=1 ;;
  ENABLE_XBOX_500HZ_TURN120) preflight=0 ;;
  *) echo 'Exact independent acknowledgement required'; exit 2 ;;
esac
[[ "$EUID" == 0 && "${SUDO_USER:-}" == tony ]] || { echo 'Run with sudo as tony'; exit 2; }
runuser -u tony -- "$stable/.venv/bin/python" "$experiment/tools/check_xbox500_powered.py"
getcap "$candidate/sprite_can_xbox500" | grep -q 'cap_sys_nice=ep' || {
  echo "Install capability: sudo setcap cap_sys_nice+ep $candidate/sprite_can_xbox500"; exit 2;
}
exec 9>/run/lock/sprite_icm_spi_pm.lock
flock -n 9 || { echo 'Another ICM/controller session holds lock'; exit 2; }
exec 8>"$experiment/reports/.xbox_500hz.lock"
flock -n 8 || { echo 'Another 500Hz session holds lock'; exit 2; }
pm=/sys/bus/platform/devices/3210000.spi/power/control
old="$(<"$pm")"
trap 'printf "%s\n" "$old" > "$pm"' EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
printf 'on\n' > "$pm"
echo 'EXPERIMENT: CAN31x500Hz, anklePD500Hz, actor50Hz, ICM800Hz. Initial15s stationary.'
echo 'Requires battery, fall arrest, clear area and onsite power safety operator.'
echo 'LT permission; forward/turn only; LB+RB3s disable; final10s stop then disable.'
runuser -u tony -- env SPRITE_PREFLIGHT_ONLY="$preflight" \
  bash "$candidate/inner.sh" ENABLE_XBOX_TURN120
