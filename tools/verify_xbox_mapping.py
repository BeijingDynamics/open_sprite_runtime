"""Guided read-only joystick mapping. No CAN, policy, or force feedback."""
import json
import os
from pathlib import Path
import select
import struct
import time
import argparse

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--interactive', action='store_true')
    args = parser.parse_args()
    root = Path('/home/tony/open_sprite_runtime/reports')
    root.mkdir(exist_ok=True)
    output = root / ('xbox_mapping_' + time.strftime('%Y%m%d_%H%M%S') + '.json')
    fd = os.open('/dev/input/js0', os.O_RDONLY | os.O_NONBLOCK)
    stages = [('neutral', 5), ('LT_press_release', 15), ('forward_only', 15),
              ('backward_only', 15), ('left_turn_only', 15), ('right_turn_only', 15), ('neutral_end', 5)]
    report = {'no_can': True, 'events': [], 'errors': [], 'output': str(output)}
    print('NO CAN. REPORT', output, flush=True)
    try:
        for name, duration in stages:
            if args.interactive:
                input('\nCenter sticks and release LT. Press ENTER to begin '+name+'; THEN move only that control. ')
                # Discard queued events from the operator preparation period.
                while select.select([fd], [], [], 0)[0]:
                    if len(os.read(fd, 8)) != 8:
                        raise RuntimeError('joystick disconnected')
            print('STAGE', name, 'SECONDS', duration, flush=True)
            deadline = time.monotonic() + duration
            while time.monotonic() < deadline:
                if not select.select([fd], [], [], min(.1, max(0, deadline-time.monotonic())))[0]:
                    continue
                data = os.read(fd, 8)
                if len(data) != 8:
                    raise RuntimeError('joystick disconnected')
                _, value, kind, axis = struct.unpack('<IhBB', data)
                report['events'].append([name, time.monotonic_ns(), kind, axis, value])
                if kind & 2:
                    print('AXIS', axis, 'VALUE', round(value/32767, 3), 'INIT', bool(kind & 128), flush=True)
    except Exception as exc:
        report['errors'].append(str(exc))
    finally:
        os.close(fd)
        output.write_text(json.dumps(report, indent=2))
    print('DONE', output, flush=True)
    if report['errors']:
        raise SystemExit(1)

if __name__ == '__main__':
    main()
