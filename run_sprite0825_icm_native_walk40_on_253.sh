#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
case "${1:-}" in
  --check) exec bash ./run_sprite0825_icm_native_walk40_inner.sh --check ;;
  --preflight-only) export SPRITE_PREFLIGHT_ONLY=1 ;;
  ENABLE_ICM45686_WALK40) export SPRITE_PREFLIGHT_ONLY=0 ;;
  *) echo 'Usage: --check | --preflight-only | ENABLE_ICM45686_WALK40'; exit 2 ;;
esac
[[ "$EUID" == 0 && "${SUDO_USER:-}" == tony ]] || { echo 'Use sudo as tony for scoped SPI power management'; exit 2; }
exec 9>/run/lock/sprite_icm_spi_pm.lock
flock -n 9 || { echo 'Another ICM test holds PM lock'; exit 2; }
pm=/sys/bus/platform/devices/3210000.spi/power/control
old=$(<"$pm")
trap 'printf "%s\n" "$old" > "$pm"' EXIT
printf 'on\n' > "$pm"
runuser -u tony -- env SPRITE_PREFLIGHT_ONLY="$SPRITE_PREFLIGHT_ONLY" \
  bash "$PWD/run_sprite0825_icm_native_walk40_inner.sh" ENABLE_ICM45686_WALK40
