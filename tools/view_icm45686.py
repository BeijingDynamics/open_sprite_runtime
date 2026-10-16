"""ICM native body-frame attitude display only. No CAN or policy."""
import argparse
import json
import os
from pathlib import Path
import queue
import subprocess
import threading
import time

import mujoco
import mujoco.viewer
import numpy as np


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--mjcf', required=True)
    p.add_argument('--duration', type=int, default=120, choices=range(20, 181))
    args = p.parse_args()
    root = Path(__file__).resolve().parents[1]
    out = root / 'reports' / ('icm_view_' + time.strftime('%Y%m%d_%H%M%S') + '_' + str(os.getpid()))
    out.mkdir(parents=True)
    model = mujoco.MjModel.from_xml_path(args.mjcf)
    data = mujoco.MjData(model)
    free = np.flatnonzero(model.jnt_type == mujoco.mjtJoint.mjJNT_FREE)
    if len(free) != 1:
        raise RuntimeError('Expected one floating root')
    adr = int(model.jnt_qposadr[free[0]])
    data.qpos[adr:adr+3] = (0, 0, .65)
    latest = queue.Queue(maxsize=1)
    report = {'no_can': True, 'frames': 0, 'errors': []}
    env = dict(os.environ, ICM_VIEW_STDOUT='1')
    print('REPORT_DIRECTORY', out, flush=True)
    print('KEEP STILL 15s. IMU mount must be X down, Z back. NO CAN.', flush=True)
    process = subprocess.Popen(['bash', str(root/'probe_sprite0825_icm45686_on_253.sh'),
        '800', str(args.duration+25), 'CONFIGURE_ICM45686_RAW_NO_CAN'],
        cwd=root, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)

    def reader():
        with (out/'producer.log').open('w') as log:
            for line in process.stdout:
                log.write(line)
                log.flush()
                if line.startswith('POSE '):
                    try:
                        latest.get_nowait()
                    except queue.Empty:
                        pass
                    latest.put_nowait(line)
                else:
                    print(line.rstrip(), flush=True)

    thread = threading.Thread(target=reader, daemon=True)
    thread.start()
    viewer = None
    finish = None
    startup = time.monotonic()+25
    last_stamp = 0
    stop_reason = 'error'
    try:
        while finish is None or time.monotonic() < finish:
            if process.poll() is not None:
                raise RuntimeError('IMU producer exited; see producer.log')
            if viewer is not None and not viewer.is_running():
                stop_reason = 'window_closed'
                break
            try:
                line = latest.get(timeout=.05)
            except queue.Empty:
                if finish is None and time.monotonic() > startup:
                    raise RuntimeError('Calibration timed out')
                if last_stamp and time.monotonic_ns()-last_stamp > 200_000_000:
                    raise RuntimeError('Stale IMU display feed')
                continue
            fields = line.split()
            stamp = int(fields[1])
            q = np.array([float(x) for x in fields[2:]])
            if q.shape != (4,) or not np.isfinite(q).all() or abs(np.dot(q, q)-1) > 1e-5:
                raise RuntimeError('Invalid body quaternion')
            if not 0 <= time.monotonic_ns()-stamp < 200_000_000 or stamp <= last_stamp:
                raise RuntimeError('Stale or regressed IMU sample')
            last_stamp = stamp
            # Native estimator already receives body-frame accel/gyro.
            data.qpos[adr+3:adr+7] = q
            mujoco.mj_forward(model, data)
            if viewer is None:
                viewer = mujoco.viewer.launch_passive(model, data)
                viewer.cam.lookat[:] = (0, 0, .55)
                viewer.cam.distance = 1.8
                viewer.cam.azimuth = 90
                viewer.cam.elevation = -8
                finish = time.monotonic()+args.duration
                print('VIEW_READY: tilt now; neutral joints; no startup leveling; NO CAN', flush=True)
            viewer.sync()
            report['frames'] += 1
            time.sleep(.02)
        if stop_reason != 'window_closed':
            stop_reason = 'display_duration_completed'
    except Exception as exc:
        report['errors'].append(str(exc))
    finally:
        if viewer is not None:
            viewer.close()
            time.sleep(.5)
        stopped_by_viewer = process.poll() is None
        if stopped_by_viewer:
            process.terminate()
        try:
            process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
        thread.join(timeout=3)
        report['producer_exit'] = process.returncode
        report['producer_stopped_by_viewer'] = stopped_by_viewer
        report['stop_reason'] = stop_reason
        report['display_completed'] = not report['errors'] and stop_reason == 'display_duration_completed'
        (out/'viewer.json').write_text(json.dumps(report, indent=2))
    print(json.dumps(report), flush=True)
    if report['errors']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
