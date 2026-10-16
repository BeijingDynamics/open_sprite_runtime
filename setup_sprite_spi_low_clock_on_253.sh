#!/usr/bin/env bash
set -euo pipefail
# Runtime-only clock experiment. No DT/kernel edits or bus transactions.
[[ $EUID == 0 ]] || { echo 'Run using sudo'; exit 1; }
action="${1:-status}"
controller="${2:-spi1}"
case "$controller" in
  spi1) device=/dev/spidev0.0; expected=3210000.spi;;
  spi3) device=/dev/spidev1.0; expected=3230000.spi;;
  *) echo 'Supported controllers: spi1 or spi3'; exit 2;;
esac
[[ $(readlink -f "/sys/class/spidev/${device##*/}/device") == *"/$expected/"* ]] || { echo 'Controller mapping mismatch'; exit 1; }
clk=/sys/kernel/debug/bpmp/debug/clk/$controller
backup=/run/sprite_${controller}_clock_backup
echo "CONTROLLER=$controller DEVICE=$device"
exec 9>"$device"
flock -n 9 || { echo 'Stop SPI diagnostics first'; exit 1; }
[[ -r "$clk/parent" && -r "$clk/rate" ]] || { echo 'SPI1 BPMP clock interface unavailable'; exit 1; }
case "$action" in
  status) ;;
  apply)
    [[ ! -e "$backup" ]] || { echo 'Backup exists; use status or restore first'; exit 1; }
    mkdir -m 700 "$backup"
    cat "$clk/parent" > "$backup/parent"
    cat "$clk/rate" > "$backup/rate"
    rollback() { cat "$backup/parent" > "$clk/parent"; cat "$backup/rate" > "$clk/rate"; }
    trap 'rollback' ERR
    printf 'clk_m\n' > "$clk/parent"
    [[ $(cat "$clk/parent") == clk_m ]]
    printf '500000\n' > "$clk/rate"
    trap - ERR
    ;;
  restore)
    [[ -f "$backup/parent" && -f "$backup/rate" ]] || { echo 'No backup'; exit 1; }
    cat "$backup/parent" > "$clk/parent"
    cat "$backup/rate" > "$clk/rate"
    ;;
  *) echo 'Usage: sudo bash setup_sprite_spi_low_clock_on_253.sh status|apply|restore'; exit 2;;
esac
for field in parent rate min_rate max_rate; do
  if [[ -r "$clk/$field" ]]; then printf '%s=' "$field"; cat "$clk/$field"; fi
done
echo 'NO CAN/SPI transfers. Clock readback is NOT a scope measurement. Runtime-only; reboot restores boot configuration.'
