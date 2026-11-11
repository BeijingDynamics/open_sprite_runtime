#!/usr/bin/env bash
set -euo pipefail

experiment="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
runner="$experiment/tools/run_native_policy_hardware_shadow.py"
mode="${1:-}"
if [[ $# != 1 && $# != 2 ]]; then
  echo "Usage: $0 (--check | ZERO_GAIN_XBOX_500HZ_FULL_31_DISABLED) [--feedback-candidate]" >&2
  exit 2
fi
extra=()
if [[ $# == 2 ]]; then
  [[ "$2" == --feedback-candidate ]] || { echo 'Unknown optional flag'; exit 2; }
  extra=(--feedback-candidate)
fi

case "$mode" in
  --check)
    [[ "$EUID" != 0 ]] || { echo 'Run --check without sudo' >&2; exit 2; }
    stable="$HOME/open_sprite_runtime"
    exec env PYTHONDONTWRITEBYTECODE=1 PYTHONPATH="$stable/src" "$stable/.venv/bin/python" \
      "$runner" --runtime-root "$stable" --icm-candidate "$stable/candidates/icm_native_20261016" \
      --xbox-500hz --duration 120 --check-only --confirm ZERO_GAIN_ALL_DISABLED_SUPPORTED "${extra[@]}"
    ;;
  ZERO_GAIN_XBOX_500HZ_FULL_31_DISABLED)
    [[ "$EUID" == 0 && -n "${SUDO_USER:-}" && "$SUDO_USER" != root ]] || {
      echo 'Run the confirmed test with sudo; SPI power management is restored on exit.' >&2
      exit 2
    }
    home="$(getent passwd "$SUDO_USER" | cut -d: -f6)"
    [[ -n "$home" ]]
    stable="$home/open_sprite_runtime"
    ;;
  *) echo "Usage: $0 --check | ZERO_GAIN_XBOX_500HZ_FULL_31_DISABLED" >&2; exit 2 ;;
esac

exec 9>/run/lock/sprite_icm_spi_pm.lock
flock -n 9 || { echo 'Another ICM test holds the SPI PM lock' >&2; exit 2; }
pm=/sys/bus/platform/devices/3210000.spi/power/control
old="$(<"$pm")"
trap 'printf "%s\n" "$old" > "$pm"' EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
printf 'on\n' > "$pm"
echo '120 s ZERO GAIN ONLY. Keep still for the initial 15 s IMU calibration. No motor enable.'
runuser -u "$SUDO_USER" -- mkdir -p "$experiment/reports"
runuser -u "$SUDO_USER" -- flock -n "$experiment/reports/.xbox_500hz.lock" \
  env PYTHONDONTWRITEBYTECODE=1 PYTHONPATH="$stable/src" \
  "$stable/.venv/bin/python" "$runner" \
  --runtime-root "$stable" --icm-candidate "$stable/candidates/icm_native_20261016" \
  --xbox-500hz --duration 120 --confirm ZERO_GAIN_ALL_DISABLED_SUPPORTED "${extra[@]}"
