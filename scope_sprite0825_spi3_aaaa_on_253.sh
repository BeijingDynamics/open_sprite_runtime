#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"
[[ $(readlink -f /sys/class/spidev/spidev1.0/device) == *'/3230000.spi/'* ]] || { echo 'SPI3 mapping mismatch'; exit 1; }
export SPRITE_SPI_DEVICE=/dev/spidev1.0
echo 'SPI3 CS0: MOSI PIN37, SCK PIN13, CS PIN18; GND PIN20 or25.'
echo 'Mode3, requested500kHz, AA AA,16clocks,10ms pause. RX ignored. NO CAN.'
echo 'Scope determines actual clock. Ctrl+C stops and restores SPI settings.'
exec build/icm45686_candidate/icm_spi_id_diagnostic --continuous-aa-500khz 3
