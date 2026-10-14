"""Native raw-six-axis attitude display. No CAN or policy execution."""
import argparse
import json
from pathlib import Path
import socket
import struct
import subprocess
import tempfile
import time

import numpy as np
import mujoco
import mujoco.viewer
from open_sprite_runtime.imu import ImuMount, matrix_to_quaternion_wxyz, quaternion_wxyz_to_matrix


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mjcf', required=True)
    parser.add_argument('--duration', type=int, default=120, choices=range(20, 181))
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():
            continue
        try:
            name = Path(entry.joinpath('exe').resolve(strict=True)).name
        except OSError:
            continue
        if name.startswith(('sprite_can_', 'sprite_policy_', 'sprite_imu_')):
            raise RuntimeError('Stop existing native control/IMU process: ' + name)
    model = mujoco.MjModel.from_xml_path(args.mjcf)
    data = mujoco.MjData(model)
    free = [i for i, kind in enumerate(model.jnt_type) if kind == mujoco.mjtJoint.mjJNT_FREE]
    if len(free) != 1:
        raise RuntimeError('Expected one floating root')
    address = int(model.jnt_qposadr[free[0]])
    data.qpos[address:address+3] = (0, 0, .65)
    mount = ImuMount.sprite0825_rear_pelvis().sensor_to_body_matrix
    out = Path(tempfile.mkdtemp(prefix='software_imu_view_', dir=root/'reports'))
    report = dict(mode='software_imu_attitude_only', can_tx_count=0, samples=0, errors=[])
    print('REPORT_DIRECTORY', out, flush=True)
    print('Keep still for15s. Then tilt robot. Joint poses are neutral, NOT measured.', flush=True)
    with tempfile.TemporaryDirectory(prefix='sprite_view_') as folder, (out/'producer.log').open('w') as log:
        endpoint = str(Path(folder)/'imu.sock')
        process = subprocess.Popen([str(root/'build/imu_software_candidate/sprite_imu_software'),
            '--record', str(args.duration+20), endpoint, str(out/'imu.json'),
            'CONFIGURE_RAW_P0_NO_CAN', '200', 'burst18'], stdout=log, stderr=subprocess.STDOUT)
        viewer = None
        try:
            with socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) as client:
                client.bind(str(Path(folder)/'view.sock'))
                client.settimeout(.1)
                startup = time.monotonic()+23
                finish = None
                last_seq = 0
                while finish is None or time.monotonic() < finish:
                    if process.poll() is not None:
                        raise RuntimeError('Native producer exited; see producer.log')
                    if finish is None and time.monotonic() > startup:
                        raise RuntimeError('Static calibration timed out')
                    if viewer is not None and not viewer.is_running():
                        break
                    try:
                        client.sendto(b'GETIMU01', endpoint)
                        packet = client.recv(193)
                    except (FileNotFoundError, socket.timeout):
                        if finish is not None:
                            raise RuntimeError('IMU display feed unavailable')
                        time.sleep(.01)
                        continue
                    if len(packet) != 192 or packet[:8] != b'SPIMU002':
                        raise RuntimeError('Wrong IMU packet version')
                    seq, start, end = struct.unpack_from('<QQQ', packet, 8)
                    values = struct.unpack_from('<13d', packet, 80)
                    if not start <= end <= time.monotonic_ns() or time.monotonic_ns()-start > 100_000_000:
                        raise RuntimeError('Stale display sample')
                    if seq < last_seq or not np.isfinite(values).all():
                        raise RuntimeError('Invalid display sample')
                    last_seq = seq
                    q = np.array(values[6:10])
                    if abs(np.dot(q, q)-1) > 1e-6:
                        raise RuntimeError('Invalid quaternion')
                    rotation = quaternion_wxyz_to_matrix(q) @ mount.T
                    data.qpos[address+3:address+7] = matrix_to_quaternion_wxyz(rotation)
                    mujoco.mj_forward(model, data)
                    if viewer is None:
                        viewer = mujoco.viewer.launch_passive(model, data)
                        viewer.cam.lookat[:] = (0, 0, .55)
                        viewer.cam.distance = 1.8
                        viewer.cam.azimuth = 90
                        viewer.cam.elevation = -8
                        finish = time.monotonic()+args.duration
                        print('VIEW_READY: software fusion, actual tilt; no startup leveling; NO CAN', flush=True)
                    viewer.sync()
                    report['samples'] += 1
                    time.sleep(.02)
        except Exception as exc:
            report['errors'].append(str(exc))
        finally:
            if viewer is not None:
                viewer.close()
                time.sleep(.5)
            if process.poll() is None:
                process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
            report['producer_exit'] = process.returncode
            report['producer_stopped_by_viewer'] = True
            (out/'viewer.json').write_text(json.dumps(report, indent=2))
    print(json.dumps(report), flush=True)
    if report['errors']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
