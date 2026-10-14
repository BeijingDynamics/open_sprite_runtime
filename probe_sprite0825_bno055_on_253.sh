#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export PYTHONPATH="$ROOT/src${PYTHONPATH:+:$PYTHONPATH}"
# SoC I2C8 on header pins3/5 is Linux bus7 on the inspected Jetson.
BUS="${BNO_BUS:-7}"
ADDR="${BNO_ADDRESS:-0x28}"
ACTION="${1:-probe}"
case "$ACTION" in
  probe)
    exec "$ROOT/.venv/bin/python" -m open_sprite_runtime.bno055 probe --bus "$BUS" --address "$ADDR"
    ;;
  configure)
    [[ "${2:-}" == CONFIGURE_BNO055_IMUPLUS_P0 ]] || {
      echo 'Requires: configure CONFIGURE_BNO055_IMUPLUS_P0' >&2; exit 2;
    }
    exec "$ROOT/.venv/bin/python" -m open_sprite_runtime.bno055 configure \
      --bus "$BUS" --address "$ADDR" --confirm "$2" \
      --output "$ROOT/reports/bno055_config_$(date +%Y%m%d_%H%M%S).json"
    ;;
  record)
    # User confirmed the physically aligned axes are the P0 output axes.
    MOUNT="${3:-p0-output-axes}"
    [[ "$MOUNT" == chip-axes || "$MOUNT" == p0-output-axes ]] || {
      echo 'Requires: record DURATION chip-axes|p0-output-axes' >&2; exit 2;
    }
    exec "$ROOT/.venv/bin/python" -m open_sprite_runtime.bno055 record \
      --bus "$BUS" --address "$ADDR" --duration "${2:-60}" --mount-basis "$MOUNT" \
      --output "$ROOT/reports/bno055_record_$(date +%Y%m%d_%H%M%S).json"
    ;;
  *) echo 'Actions: probe, configure, record. No CAN or policy access.' >&2; exit 2 ;;
esac
