"""Guided serial-only IMU capture. No protocol writes, CAN, or motor enable."""

import argparse
from dataclasses import asdict
from datetime import datetime
import json
from pathlib import Path
import time

from open_sprite_runtime.yahboom_imu import YahboomStreamDecoder

PHASES = (
    (0, "STILL", "Keep pelvis still for 10 seconds"),
    (10, "YAW", "Slowly turn pelvis left, hold, then right and return; keep upright"),
    (25, "PITCH", "Gently tilt pelvis forward, hold, then backward and return"),
    (40, "ROLL", "Gently tilt pelvis left, hold, then right and return"),
    (55, "FINAL_STILL", "Return upright and keep still for 10 seconds"),
)


def phase_at(elapsed):
    return next(p for p in reversed(PHASES) if elapsed >= p[0])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--device', default='/dev/sprite0825-imu')
    parser.add_argument('--output-dir', default='reports')
    args = parser.parse_args()
    import serial

    output = Path(args.output_dir) / ('imu_six_axis_manual_' + datetime.now().strftime('%Y%m%d_%H%M%S') + '.json')
    output.parent.mkdir(parents=True, exist_ok=True)
    print('SERIAL READ ONLY: no motor enable, no CAN, no IMU configuration writes.', flush=True)
    print('Support robot mechanically. Move pelvis/IMU, not just waist or arms.', flush=True)
    print('65 seconds total. Do not force the sling or mechanisms. Skip any unsafe movement.', flush=True)
    input('Press Enter when ready: ')
    report = dict(mode='serial_read_only_manual_imu_audit', sensor_timestamps_available=False,
                  timestamp_kind='host_read_completion_not_sensor_sample_time',
                  expected_configuration='six_axis_command_sent_previously_not_read_back',
                  phases=PHASES, chunks=[], packets=[], errors=[], complete=False)
    decoder = YahboomStreamDecoder()
    try:
        with serial.Serial(args.device, 115200, timeout=.02, write_timeout=0, exclusive=True) as port:
            port.dtr = False
            port.rts = False
            port.reset_input_buffer()
            start = time.monotonic_ns()
            report['start_monotonic_ns'] = start
            last_phase = None
            while True:
                elapsed = (time.monotonic_ns() - start) / 1e9
                if elapsed >= 65:
                    break
                phase = phase_at(elapsed)
                if phase != last_phase:
                    print(f'PHASE {phase[1]} t={elapsed:.1f}s: {phase[2]}', flush=True)
                    last_phase = phase
                payload = port.read(min(max(port.in_waiting, 1), 4096))
                received = time.monotonic_ns()
                if not payload:
                    continue
                report['chunks'].append(dict(received_ns=received, hex=payload.hex()))
                for packet in decoder.feed(payload):
                    report['packets'].append(dict(received_ns=received,
                                                  kind=type(packet).__name__, data=asdict(packet)))
            report['complete'] = True
    except (Exception, KeyboardInterrupt) as exc:
        report['errors'].append(type(exc).__name__ + ': ' + str(exc))
    finally:
        report['rejected_frames'] = decoder.rejected_frame_count
        report['packet_count'] = len(report['packets'])
        output.write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
        print(f'SAVED {output.resolve()} packets={report["packet_count"]} complete={report["complete"]}', flush=True)
    if report['errors']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
