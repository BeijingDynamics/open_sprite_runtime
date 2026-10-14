"""Sensor-only BNO055 attitude viewer; no CAN imports or configuration writes."""
import argparse
from dataclasses import asdict
import gc
import json
import math
from pathlib import Path
import threading
import time

from open_sprite_runtime.bno055 import BNO055, I2CRegisters


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--mjcf', required=True)
    p.add_argument('--duration', type=float, default=120)
    p.add_argument('--headless', action='store_true')
    p.add_argument('--output', required=True)
    a = p.parse_args()
    if not math.isfinite(a.duration) or not 0 < a.duration <= 120:
        p.error('duration must be in (0,120]')
    output = Path(a.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    report = dict(mode='bno055_absolute_attitude_no_can', samples=[], errors=[],
                  can_tx_count=0, configuration_writes=0, skipped_slots=0)
    stop = threading.Event()
    lock = threading.Lock()
    latest = [None]

    def acquire():
        io = None
        try:
            io = I2CRegisters(7, 0x28)
            sensor = BNO055(io)
            deadline = time.monotonic_ns()
            next_check = 0
            while not stop.is_set():
                if time.monotonic_ns() >= next_check:
                    cfg = sensor.inspect()
                    report['configuration'] = cfg
                    if not cfg['matches'] or cfg['system_error'] or cfg['system_status'] != 5:
                        raise RuntimeError(f'IMUPLUS/P0 fusion not ready: {cfg}')
                    next_check = time.monotonic_ns() + 1_000_000_000
                sample = sensor.read_sample()
                row = asdict(sample)
                row['body'] = sample.body_state('p0-output-axes')
                report['samples'].append(row)
                with lock:
                    latest[0] = row
                deadline += 10_000_000
                now = time.monotonic_ns()
                if now > deadline:
                    skipped = (now-deadline)//10_000_000 + 1
                    report['skipped_slots'] += int(skipped)
                    deadline += skipped*10_000_000
                stop.wait(max(0, (deadline-time.monotonic_ns())/1e9))
        except Exception as exc:
            report['errors'].append(f'{type(exc).__name__}: {exc}')
            stop.set()
        finally:
            if io is not None:
                io.close()

    viewer = None
    worker = None
    with output.open('x') as handle:
        try:
            import mujoco
            import mujoco.viewer
            model = mujoco.MjModel.from_xml_path(a.mjcf)
            data = mujoco.MjData(model)
            free = [i for i, t in enumerate(model.jnt_type) if t == mujoco.mjtJoint.mjJNT_FREE]
            if len(free) != 1:
                raise ValueError('Expected exactly one floating root')
            root = int(model.jnt_qposadr[free[0]])
            data.qpos[root:root+3] = (0, 0, 0.52)
            if not a.headless:
                viewer = mujoco.viewer.launch_passive(model, data)
                viewer.cam.lookat[:] = (0, 0, 0.45)
                viewer.cam.distance = 1.8
                viewer.cam.azimuth = 90
                viewer.cam.elevation = -8
            worker = threading.Thread(target=acquire, name='bno055-latest', daemon=True)
            worker.start()
            start = time.monotonic()
            print('BNO055 LIVE: absolute attitude, no startup leveling, NO CAN', flush=True)
            while not stop.is_set() and time.monotonic()-start < a.duration:
                if viewer is not None and not viewer.is_running():
                    break
                with lock:
                    row = latest[0]
                if row is None:
                    if time.monotonic()-start > 2:
                        raise RuntimeError('No initial IMU sample within 2s')
                else:
                    age = (time.monotonic_ns()-row['read_start_ns'])/1e6
                    if age > 100:
                        raise RuntimeError(f'Viewer sample stale: {age:.1f}ms')
                    data.qpos[root+3:root+7] = row['body']['orientation_body_to_world_wxyz']
                    mujoco.mj_forward(model, data)
                    if viewer is not None:
                        viewer.sync()
                stop.wait(0.02)
            report['elapsed_s'] = time.monotonic()-start
        except Exception as exc:
            report['errors'].append(f'{type(exc).__name__}: {exc}')
        finally:
            stop.set()
            if worker is not None:
                worker.join(timeout=3)
                if worker.is_alive():
                    report['errors'].append('I2C worker did not stop within 3s')
            if viewer is not None:
                viewer.close()
                del viewer
                gc.collect()
            json.dump(report, handle, indent=2)
    print('SAMPLES', len(report['samples']), 'ERRORS', report['errors'], 'REPORT', output)
    raise SystemExit(bool(report['errors']))


if __name__ == '__main__':
    main()
