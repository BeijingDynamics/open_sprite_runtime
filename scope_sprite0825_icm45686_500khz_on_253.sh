#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"
echo 'Read-only ID: SPI0 CS0, Mode3, request500kHz. No CAN. Ctrl+C stops.'
echo 'Remove external loopback jumper. Confirm actual SCK on scope: expected period2us.'
exec build/icm45686_candidate/icm_spi_id_diagnostic --continuous-id-500khz 3
