#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"
mode="${1:-3}"
case "$mode" in 0|3) ;; *) echo 'Usage: bash scope_sprite0825_icm45686_100khz_on_253.sh [0|3]' >&2; exit 2;; esac
echo 'ICM connected normally; REMOVE MOSI-MISO loopback jumper before running.'
echo 'Read-only WHO_AM_I, fixed 100kHz request, hardware CS0. No CAN or sensor register writes.'
echo 'Ctrl+C stops. The requested clock must be verified on the oscilloscope.'
exec build/icm45686_candidate/icm_spi_id_diagnostic --continuous-id-100khz "$mode"
