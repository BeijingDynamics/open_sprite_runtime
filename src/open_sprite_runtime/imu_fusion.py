"""Explicit, standalone Yahboom fusion configuration; no CAN or policy access."""

import argparse
import json
import math
from pathlib import Path
import time

from .yahboom_imu import YahboomQuaternion, YahboomRawImu, YahboomStreamDecoder

CONFIRMATION = "WRITE_IMU_CONFIGURATION"


def build_fusion_command(axes):
    if isinstance(axes, bool) or not isinstance(axes, int) or axes not in (6, 9):
        raise ValueError("fusion axes must be integer 6 or 9")
    frame = bytes((0x7E, 0x23, 7, 0x61, axes, 0x5F))
    return frame + bytes((sum(frame) & 255,))


def configure(device, baud, axes, confirmation, report_path):
    if confirmation != CONFIRMATION:
        raise ValueError("explicit WRITE_IMU_CONFIGURATION confirmation required")
    command = build_fusion_command(axes)
    if not device.startswith('/dev/') or baud <= 0:
        raise ValueError("require /dev/ device and positive baud")
    import serial

    output = Path(report_path)
    output.parent.mkdir(parents=True, exist_ok=True)
    report = dict(mode="imu_fusion_configuration_no_can", device=device,
                  requested_axes=axes, command_hex=command.hex(' '),
                  write_completed=False, mode_readback_verified=False,
                  protocol_ack_supported=False, persistence_verified=False,
                  stream_verified=False, errors=[])
    try:
        with serial.Serial(device, baud, timeout=0.02, write_timeout=1,
                           exclusive=True) as port:
            port.dtr = False
            port.rts = False
            port.reset_input_buffer()
            if port.write(command) != len(command):
                raise RuntimeError("short fusion configuration write")
            port.flush()
            report['write_completed'] = True
            # The vendor protocol specifies no ACK for 0x61.
            time.sleep(1)
            port.reset_input_buffer()
            decoder = YahboomStreamDecoder()
            raw_count = quat_count = 0
            bad_quat = 0
            start = time.monotonic()
            while time.monotonic() - start < 3:
                for packet in decoder.feed(port.read(min(max(port.in_waiting, 1), 4096))):
                    if isinstance(packet, YahboomRawImu):
                        raw_count += 1
                    elif isinstance(packet, YahboomQuaternion):
                        quat_count += 1
                        if not math.isfinite(packet.norm) or abs(packet.norm - 1) > .05:
                            bad_quat += 1
            elapsed = time.monotonic() - start
            report.update(raw_frames=raw_count, quaternion_frames=quat_count,
                          raw_hz=raw_count/elapsed, quaternion_hz=quat_count/elapsed,
                          rejected_frames=decoder.rejected_frame_count,
                          bad_quaternion_count=bad_quat)
            report['stream_verified'] = (
                80 <= raw_count/elapsed <= 120 and 80 <= quat_count/elapsed <= 120
                and bad_quat == 0 and decoder.rejected_frame_count == 0)
            if not report['stream_verified']:
                report['errors'].append("expected healthy approximately 100Hz raw and quaternion stream")
    except Exception as exc:
        report['errors'].append(str(exc))
    output.write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--device', default='/dev/sprite0825-imu')
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--axes', type=int, choices=(6, 9), required=True)
    parser.add_argument('--confirm-hardware-tx', required=True)
    parser.add_argument('--report', required=True)
    args = parser.parse_args()
    report = configure(args.device, args.baud, args.axes, args.confirm_hardware_tx, args.report)
    print(json.dumps(report, indent=2))
    if report['errors']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
