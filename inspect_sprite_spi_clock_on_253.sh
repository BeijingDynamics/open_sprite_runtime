#!/usr/bin/env bash
set -euo pipefail
[[ $EUID == 0 ]] || { echo 'Run using sudo'; exit 1; }
echo 'READ ONLY: no clock writes, no SPI/CAN transfers. Leave the scope probe running.'
readlink -f /sys/class/spidev/spidev0.0/device
for sample in {1..5}; do
  date --iso-8601=ns
  for clock in spi1 spi2; do
    root=/sys/kernel/debug/bpmp/debug/clk/$clock
    for field in parent rate min_rate max_rate mrq_rate_locked state; do
      if [[ -r "$root/$field" ]]; then
        printf '%s/%s=' "$clock" "$field"
        cat "$root/$field"
      fi
    done
  done
  for field in control runtime_status; do
    path=/sys/bus/platform/devices/3210000.spi/power/$field
    if [[ -r "$path" ]]; then printf 'power/%s=' "$field"; cat "$path"; fi
  done
  sleep 1
done
if [[ -r /sys/kernel/debug/clk/clk_summary ]]; then
  grep -E '(^ *clock|spi|clk_m|pllp)' /sys/kernel/debug/clk/clk_summary || true
fi
echo 'Original saved clock settings:'
for field in parent rate; do
  path=/run/sprite_spi1_clock_backup/$field
  if [[ -r "$path" ]]; then printf 'backup/%s=' "$field"; cat "$path"; fi
done
