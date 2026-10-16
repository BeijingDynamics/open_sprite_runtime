#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"
binary=build/icm45686_candidate/sprite_icm45686
if [[ ! -x "$binary" ]]; then
  echo 'First run: bash build_sprite0825_icm45686_on_253.sh' >&2
  exit 1
fi
spi="${ICM_SPI_DEVICE:-/dev/spidev0.0}"
case "${1:-help}" in
  probe)
    exec "$binary" probe "$spi"
    ;;
  400|800)
    if [[ "${3:-}" != CONFIGURE_ICM45686_RAW_NO_CAN ]]; then
      echo 'Requires explicit confirmation: bash probe_sprite0825_icm45686_on_253.sh 400 30 CONFIGURE_ICM45686_RAW_NO_CAN' >&2
      exit 1
    fi
    read -r chip offset < <(gpiofind PY.01)
    mkdir -p reports
    out="reports/icm45686_${1}hz_$(date +%Y%m%d_%H%M%S)_$$.csv"
    echo 'NO CAN. Sensor configuration writes only. Mount: X down, Z back.'
    echo "Keep stationary for first 15s. Total duration ${2:-30}s includes calibration. Output: $out"
    exec "$binary" capture "$spi" "/dev/$chip" "$offset" "$1" "${2:-30}" "$out" X_DOWN_Z_BACK "$3"
    ;;
  *)
    echo 'Read-only ID: bash probe_sprite0825_icm45686_on_253.sh probe'
    echo 'Capture: bash probe_sprite0825_icm45686_on_253.sh {400|800} 30 CONFIGURE_ICM45686_RAW_NO_CAN'
    echo 'No CAN, motor commands, policy execution, or automatic walking integration.'
    exit 2
    ;;
esac
