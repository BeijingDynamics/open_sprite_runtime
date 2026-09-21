#!/usr/bin/env bash
set -euo pipefail

RULE_PATH=/etc/udev/rules.d/99-sprite0825-imu.rules
RULE='SUBSYSTEM=="tty", KERNEL=="ttyCH341USB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", KERNELS=="1-2.2", SYMLINK+="sprite0825-imu", GROUP="dialout", MODE="0660"'

printf '%s\n' "$RULE" | sudo tee "$RULE_PATH" >/dev/null
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=tty
sudo udevadm settle

if [[ ! -e /dev/sprite0825-imu ]]; then
  echo "ERROR: /dev/sprite0825-imu was not created" >&2
  echo "Check that the IMU remains connected at Jetson USB topology 1-2.2." >&2
  exit 1
fi

resolved=$(readlink -f /dev/sprite0825-imu)
if [[ "$resolved" != "/dev/ttyCH341USB0" ]]; then
  echo "ERROR: stable link resolved to unexpected device: $resolved" >&2
  exit 1
fi

echo "IMU_UDEV_OK link=/dev/sprite0825-imu target=$resolved rule=$RULE_PATH"
