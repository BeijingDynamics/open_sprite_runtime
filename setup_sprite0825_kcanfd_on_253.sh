#!/usr/bin/env bash
set -euo pipefail

INTERFACES=(kcan1 kcan2 kcan3 kcan4)
ARBITRATION_BITRATE=1000000
DATA_BITRATE=5000000
TX_QUEUE_LENGTH=1000

if (( EUID != 0 )); then
  exec sudo -- "$0" "$@"
fi

missing=()
for interface in "${INTERFACES[@]}"; do
  if [[ ! -e "/sys/class/net/$interface" ]]; then
    missing+=("$interface")
  fi
done

if (( ${#missing[@]} > 0 )); then
  echo "ERROR: missing SocketCAN interfaces: ${missing[*]}" >&2
  echo "Connect the KH four-channel USB-CANFD adapter and verify its driver first." >&2
  exit 1
fi

for interface in "${INTERFACES[@]}"; do
  echo "CONFIGURE $interface arbitration=${ARBITRATION_BITRATE} data=${DATA_BITRATE} txqlen=${TX_QUEUE_LENGTH}"
  ip link set dev "$interface" down
  ip link set dev "$interface" type can \
    bitrate "$ARBITRATION_BITRATE" \
    dbitrate "$DATA_BITRATE" \
    fd on
  ip link set dev "$interface" txqueuelen "$TX_QUEUE_LENGTH"
  ip link set dev "$interface" up
done

echo
echo "Sprite0825 CAN-FD interfaces are configured:"
for interface in "${INTERFACES[@]}"; do
  ip -details -statistics link show dev "$interface"
done
