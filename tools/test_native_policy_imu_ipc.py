"""Native IMU IPC tests against a fake Unix server; no I2C/CAN access."""
import argparse
import json
from pathlib import Path
import socket
import struct
import subprocess
import tempfile
import threading
import time


def packet(sequence, now):
    raw = bytearray(46)
    struct.pack_into('<h', raw, 24, 16384)
    return bytearray(b'SPIMU001' + struct.pack('<QQQ', sequence, now-1000000, now)
                     + raw + b'\0\0' + struct.pack('<13d', *([0]*6+[1,0,0,0]+[0]*3)) + b'\0'*8)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--binary', required=True)
    p.add_argument('--report', required=True)
    args = p.parse_args()
    results = []
    cases = {'normal': None, 'duplicate': None, 'silence': 'timeout',
             'stale': 'timestamp', 'future': 'timestamp', 'reverse_time': 'timestamp',
             'regression': 'sequence', 'size': 'size/version', 'magic': 'size/version',
             'reserved': 'reserved', 'mismatch': 'disagreement', 'nan': 'disagreement',
             'quaternion': 'quaternion', 'frozen': 'timestamp'}
    for case, expected in cases.items():
        with tempfile.TemporaryDirectory(prefix='sprite_fake_imu_') as directory:
            path = str(Path(directory)/'imu.sock')
            server = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
            server.bind(path)
            server.settimeout(.05)
            stop = threading.Event()
            errors = []
            def serve():
                count = 0
                frozen = None
                duplicate = None
                try:
                    while not stop.is_set():
                        try:
                            request, peer = server.recvfrom(100)
                        except socket.timeout:
                            continue
                        if request != b'GETIMU01':
                            raise AssertionError('unexpected request')
                        count += 1
                        now = time.monotonic_ns()
                        data = packet((count+1)//2 if case=='duplicate' else count, now)
                        if case=='duplicate':
                            if count % 2 == 1: duplicate = data
                            else: data = duplicate
                        if case=='silence': continue
                        if case=='stale': struct.pack_into('<Q', data, 16, now-40000000)
                        if case=='future': struct.pack_into('<Q', data, 24, now+1000000000)
                        if case=='reverse_time': struct.pack_into('<Q', data, 24, now-2000000)
                        if case=='regression': struct.pack_into('<Q', data, 8, 5 if count==1 else 4)
                        if case=='size': data += b'X'
                        if case=='magic': data[0] = 0
                        if case=='reserved': data[184] = 1
                        if case=='mismatch': struct.pack_into('<d', data, 80, 1)
                        if case=='nan': struct.pack_into('<d', data, 80, float('nan'))
                        if case=='quaternion': data[56:58] = b'\0\0'
                        if case=='frozen':
                            if frozen is None: frozen = data
                            data = frozen
                        try:
                            server.sendto(data, peer)
                        except (FileNotFoundError, ConnectionRefusedError):
                            break
                except Exception as exc:
                    errors.append(repr(exc))
            thread = threading.Thread(target=serve)
            thread.start()
            try:
                proc = subprocess.run([args.binary, path], capture_output=True, text=True, timeout=5)
            finally:
                stop.set()
                thread.join(1)
                server.close()
            passed = not errors and not thread.is_alive() and (
                proc.returncode==0 if expected is None else proc.returncode!=0 and expected in proc.stderr)
            results.append(dict(case=case, passed=passed, returncode=proc.returncode,
                                stdout=proc.stdout.strip(), stderr=proc.stderr.strip(), server_errors=errors))
    report = dict(mode='native_imu_ipc_fake_server_no_hardware', hardware_tx=0,
                  tests=results, passed=all(r['passed'] for r in results))
    destination = Path(args.report)
    with destination.open('x') as f:
        json.dump(report, f, indent=2)
    print(json.dumps(report, indent=2))
    if not report['passed']: raise SystemExit(1)


if __name__=='__main__': main()
