#!/usr/bin/env bash
set -euo pipefail
# Read-only inventory. Does not enable tracing or access IMU/CAN hardware.
root=/sys/kernel/tracing
[[ -r "$root/available_events" ]] || root=/sys/kernel/debug/tracing
[[ -r "$root/available_events" ]] || { echo 'Run this inventory with sudo'; exit 1; }
printf 'TRACE_ROOT %s\n' "$root"
grep -E '^(i2c|sched|irq|power):' "$root/available_events"
for event in sched/sched_switch sched/sched_wakeup i2c/i2c_read i2c/i2c_reply i2c/i2c_result; do
  if [[ -r "$root/events/$event/format" ]]; then
    printf '\nFORMAT %s\n' "$event"
    cat "$root/events/$event/format"
  fi
done
