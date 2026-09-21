#!/usr/bin/env bash
set -euo pipefail

BINARY=/home/tony/open_sprite_runtime/build/native/sprite_can_shadow

[[ -x "$BINARY" ]] || {
  echo "Native runtime not built: $BINARY" >&2
  exit 1
}

echo "Granting only CAP_SYS_NICE to the native 500Hz shadow runtime"
echo "This permits SCHED_FIFO but does not grant CAN, root, or device-management privileges"
sudo setcap cap_sys_nice=eip "$BINARY"

ACTUAL="$(getcap "$BINARY")"
echo "$ACTUAL"
[[ "$ACTUAL" == *"cap_sys_nice=eip"* ]] || {
  echo "CAP_SYS_NICE verification failed" >&2
  exit 1
}
