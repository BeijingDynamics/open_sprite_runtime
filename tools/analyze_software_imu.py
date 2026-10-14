"""Decode candidate packets after capture; no devices, CAN, or policy execution."""
import argparse
import csv
import json
import math
from pathlib import Path
import struct


def analyze(path):
    data = path.read_bytes()
    if not data or len(data) % 192:
        raise ValueError('empty/truncated software IMU packets')
    rows = []
    last_seq = last_start = 0
    for offset in range(0, len(data), 192):
        packet = data[offset:offset + 192]
        if packet[:8] != b'SPIMU002':
            raise ValueError('not a software-fusion packet')
        seq, start, end = struct.unpack_from('<QQQ', packet, 8)
        bias = struct.unpack_from('<3d', packet, 56)
        values = struct.unpack_from('<13d', packet, 80)
        if (seq <= last_seq or start <= last_start or not start or end < start
                or end-start > 30_000_000 or (last_start and start-last_start > 30_000_000)):
            raise ValueError('invalid sequence/time/gap')
        if not all(math.isfinite(v) for v in bias + values):
            raise ValueError('nonfinite sample')
        raw_w = [struct.unpack_from('<h', packet, 44+2*i)[0]/900 for i in range(3)]
        if any(abs(values[3+i] - (raw_w[i]-bias[i])) > 1e-12 for i in range(3)):
            raise ValueError('raw-minus-bias invariant violated')
        w, x, y, z = values[6:10]
        if abs(w*w+x*x+y*y+z*z-1) > 1e-6:
            raise ValueError('quaternion norm')
        # Same rear-pelvis P0 conversion as production policy_feedback.hpp.
        gx, gy, gz = 1-2*(x*x+y*y), 2*(y*z+x*w), 2*(x*z-y*w)
        row = [seq, start, (end-start)/1e6, (start-last_start)/1e6 if last_start else 0,
               *bias, -values[5], -values[4], -values[3], gx, gy, gz]
        rows.append(row)
        last_seq, last_start = seq, start
    output = path.with_suffix(path.suffix + '.csv')
    with output.open('x', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['sequence', 'read_start_ns', 'read_ms', 'gap_ms',
                         'bias_sx', 'bias_sy', 'bias_sz', 'gyro_bx', 'gyro_by', 'gyro_bz',
                         'gravity_bx', 'gravity_by', 'gravity_bz'])
        writer.writerows(rows)
    return dict(samples=len(rows), csv=str(output), max_read_ms=max(r[2] for r in rows),
                max_gap_ms=max(r[3] for r in rows), bias_p0_rad_s=rows[0][4:7],
                first_gravity_body=rows[0][10:13], last_gravity_body=rows[-1][10:13],
                note='Host timestamps, not physical measurement timestamps; no hardware qualification')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('packets', type=Path)
    print(json.dumps(analyze(parser.parse_args().packets), indent=2))
