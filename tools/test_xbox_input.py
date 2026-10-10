"""Read-only Linux joystick probe. No CAN, policy, or force-feedback output."""
import argparse
import json
import math
import os
import select
import struct
import time


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--device', default='/dev/input/js0')
    p.add_argument('--seconds', type=float, default=30)
    args = p.parse_args()
    if not math.isfinite(args.seconds) or not 0 < args.seconds <= 300:
        p.error('seconds must be in (0, 300]')
    axes, buttons, observed = {}, {}, {}
    initial = events = 0
    disconnected = False
    fd = os.open(args.device, os.O_RDONLY | os.O_NONBLOCK)
    print('READ ONLY: move sticks and press buttons; NO ROBOT CONTROL', flush=True)
    deadline = time.monotonic() + args.seconds
    try:
        while time.monotonic() < deadline:
            if not select.select([fd], [], [], min(.2, max(0, deadline-time.monotonic())))[0]:
                continue
            try:
                data = os.read(fd, 8)
            except BlockingIOError:
                continue
            except OSError as exc:
                print('INPUT_ERROR', exc, flush=True)
                disconnected = True
                break
            if len(data) != 8:
                disconnected = True
                break
            _, value, kind, number = struct.unpack('IhBB', data)
            if kind & 0x80:
                initial += 1
                continue
            events += 1
            kind &= 0x7f
            if kind == 2:
                a = axes.setdefault(str(number), [value, value, value])
                a[:] = [min(a[0], value), max(a[1], value), value]
                bucket = round(value / 4096)
                if observed.get(number) != bucket:
                    print(f'AXIS {number}: {value / 32767:+.3f}', flush=True)
                    observed[number] = bucket
            elif kind == 1:
                buttons.setdefault(str(number), set()).add(value)
                print(f'BUTTON {number}: {value}', flush=True)
    finally:
        os.close(fd)
    print(json.dumps({'initial_events': initial, 'live_events': events,
                      'disconnected': disconnected, 'axes_min_max_last': axes,
                      'button_states_seen': {k: sorted(v) for k, v in buttons.items()},
                      'hardware_tx': False}, indent=2), flush=True)


if __name__ == '__main__':
    main()
