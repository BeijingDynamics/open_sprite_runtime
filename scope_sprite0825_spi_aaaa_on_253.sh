#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"
echo 'SPI0 CS0 Mode3, requested 500kHz, MSB first, TX AA AA, RX ignored. NO CAN.'
echo '16 clocks per transfer; 10ms pause between transfers. Ctrl+C stops.'
echo 'Measure actual SCK on PIN23; software speed readback is not a measurement.'
exec build/icm45686_candidate/icm_spi_id_diagnostic --continuous-aa-500khz 3
