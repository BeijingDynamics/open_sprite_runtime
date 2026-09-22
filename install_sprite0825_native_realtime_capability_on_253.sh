#!/usr/bin/env bash
set -euo pipefail

ROOT=/home/tony/open_sprite_runtime
BINARY="$ROOT/build/native/sprite_can_shadow"
[[ -x "$BINARY" ]] || {
  echo "Native runtime not built: $BINARY" >&2
  exit 1
}

echo "Installing only CAP_SYS_NICE on the reviewed native transport binary"
echo "This capability permits SCHED_FIFO; it does not grant general root access"
sudo setcap cap_sys_nice+ep "$BINARY"
getcap "$BINARY" | grep -q 'cap_sys_nice=ep' || {
  echo "CAP_SYS_NICE verification failed" >&2
  exit 1
}
echo "NATIVE_RT_CAPABILITY_OK binary=$BINARY capability=cap_sys_nice+ep"
