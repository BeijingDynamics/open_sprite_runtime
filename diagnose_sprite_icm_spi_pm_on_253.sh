#!/usr/bin/env bash
set -euo pipefail
[[ $EUID == 0 && ${SUDO_USER:-} == tony ]] || { echo 'Run using sudo as tony'; exit 1; }
cd "$(dirname "${BASH_SOURCE[0]}")"
duration="${1:-30}"
case "$duration" in 30|300|600) ;; *) echo 'Duration must be 30, 300 or 600 seconds'; exit 2;; esac
[[ $(readlink -f /sys/class/spidev/spidev0.0/device) == *'/3210000.spi/'* ]] || { echo 'SPI mapping mismatch'; exit 1; }
# Serialize PM ownership so another wrapper cannot restore it during this run.
exec 9>/run/lock/sprite_icm_spi_pm.lock
flock -n 9 || { echo 'Another SPI PM test owns the settings'; exit 1; }
power=/sys/bus/platform/devices/3210000.spi/power/control
old=$(cat "$power")
[[ $old == auto || $old == on ]] || { echo 'Unexpected PM control value'; exit 1; }
restore() { printf '%s\n' "$old" > "$power"; echo "RESTORED SPI power/control=$old"; }
trap restore EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
echo "NO CAN. Temporary SPI power/control: $old -> on. Exit restores original."
printf 'on\n' > "$power"
cat /sys/bus/platform/devices/3210000.spi/power/runtime_status
echo "Keep IMU stationary for 15s. 800Hz capture, ${duration}s, SPI request 1MHz."
runuser -u tony -- bash ./probe_sprite0825_icm45686_on_253.sh 800 "$duration" CONFIGURE_ICM45686_RAW_NO_CAN
