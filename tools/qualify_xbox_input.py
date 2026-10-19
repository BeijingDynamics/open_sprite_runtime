"""Operator-guided native input qualification. NO CAN, NO POLICY, NO motor commands."""
import json
from pathlib import Path
import signal
import subprocess
import threading
import time


def evaluate(name, rows):
    if len(rows) < 20:
        return {'passed': False, 'reason': 'insufficient samples'}
    tail = rows[-20:]
    zero = lambda r: abs(r['vx']) < 1e-6 and abs(r['wz']) < 1e-6
    if name == 'startup':
        ok = all(zero(r) and not r['LT_permission'] for r in rows)
    elif name in ('forward', 'rearm_disconnect', 'rearm_reconnected'):
        ok = any(r['connected'] and r['LT_permission'] and r['vx'] > .04 for r in rows)
    elif name == 'release':
        active = next((i for i, r in enumerate(rows) if r['LT_permission'] and abs(r['vx']) > .04), None)
        if active is None:
            return {'passed': False, 'reason': '未记录到松开前的前进指令；请按Enter后先保持LT和前推1秒，再松开LT'}
        revoked = next((i for i in range(active+1, len(rows)) if not rows[i]['LT_permission']), None)
        stopped = next((r for r in rows[revoked:] if zero(r)), None) if revoked is not None else None
        seconds = (stopped['stamp_ns']-rows[revoked]['stamp_ns'])/1e9 if stopped else None
        ok = seconds is not None and seconds <= 1.2 and all(zero(r) and not r['LT_permission'] for r in tail)
        return {'passed': ok, 'permission_revoked_to_zero_s': seconds}
    elif name == 'disconnect':
        lost = next((i for i, r in enumerate(rows) if not r['connected']), None)
        if lost is None:
            return {'passed': False, 'reason': 'kernel disconnect was not observed'}
        stopped = next((r for r in rows[lost:] if zero(r)), None)
        seconds = (stopped['stamp_ns']-rows[lost]['stamp_ns'])/1e9 if stopped else None
        ok = seconds is not None and seconds <= 1.2 and all(zero(r) for r in tail)
        moving_before_disconnect = lost > 0 and not zero(rows[lost-1])
        return {'passed': ok, 'disconnect_detected': True,
                'moving_disconnect_observed': moving_before_disconnect,
                'kernel_disconnect_to_zero_s': seconds if moving_before_disconnect else None,
                'radio_failure_detection_latency_measured': False}
    elif name == 'reconnect':
        ok = all(r['connected'] and not r['LT_permission'] and zero(r) for r in tail)
    else:
        raise ValueError(name)
    return {'passed': bool(ok)}


def main():
    here = Path(__file__).resolve().parent
    mapping = json.loads((here/'mapping_verified.json').read_text())
    if not mapping['mapping_verified']:
        raise RuntimeError('axis mapping is not verified')
    out = Path('/home/tony/open_sprite_runtime/reports') / ('xbox_stop_' + time.strftime('%Y%m%d_%H%M%S'))
    out.mkdir()
    report = {'no_can': True, 'powered_control_qualified': False, 'stages': {}, 'errors': []}
    rows = []
    lock = threading.Lock()
    process = subprocess.Popen([str(here/'gamepad_probe'), mapping['device'], '300',
        str(mapping['forward_sign']), str(mapping['turn_sign'])], stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT, text=True)

    def reader():
        with (out/'native.log').open('w') as f:
            for line in process.stdout:
                f.write(line)
                if not line.startswith('stamp_ns='):
                    continue
                values = dict(field.split('=', 1) for field in line.split())
                row = {k: int(v) if k == 'stamp_ns' else float(v) for k, v in values.items()}
                with lock:
                    rows.append(row)

    thread = threading.Thread(target=reader, daemon=True)
    thread.start()
    stages = [
        ('startup', 4, '松开LT，两个摇杆回中，不操作。'),
        ('forward', 8, '摇杆回中，捏放一次LT；再按住LT，并把前后摇杆向前推到底，保持。'),
        ('release', 5, '按Enter后，先保持LT和前推1秒，再只松开LT，摇杆继续向前。不要提前松开。'),
        ('rearm_disconnect', 8, '摇杆回中，松开LT；再按住LT，并向前推摇杆，保持。此项不要关机。'),
        ('disconnect', 20, '按Enter后真正关闭手柄电源，确认手柄指示灯完全熄灭。仅松开LT不算断连。此阶段不要重新开机。'),
        ('reconnect', 20, '重新开启手柄，等待连接；LT松开、摇杆回中，不要重新解锁。'),
        ('rearm_reconnected', 8, '摇杆回中，捏放一次LT；再按住LT，向前推摇杆。'),
    ]
    print('只读手柄，不发送CAN、不使能电机。报告目录：', out, flush=True)
    try:
        for name, seconds, instruction in stages:
            input('\n下一项 '+name+'：'+instruction+'\n先按键盘Enter开始计时，再按上述要求操作。 ')
            with lock:
                start = len(rows)
            for remaining in range(seconds, 0, -1):
                if process.poll() is not None:
                    raise RuntimeError('native reader exited; 300-second total test limit')
                with lock:
                    last = dict(rows[-1]) if rows else {}
                print(name, remaining, 'seconds', {k: last.get(k) for k in ('connected','LT_permission','vx','wz')}, flush=True)
                time.sleep(1)
            with lock:
                selected = list(rows[start:])
            result = evaluate(name, selected)
            report['stages'][name] = result
            (out/(name+'.json')).write_text(json.dumps(selected))
            print('RESULT', name, result, flush=True)
            if not result['passed']:
                raise RuntimeError('stage failed: '+name+'; inspect before proceeding')
    except (Exception, KeyboardInterrupt) as exc:
        report['errors'].append(str(exc) or 'interrupted')
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            process.kill(); process.wait()
        thread.join(timeout=3)
        report['passed'] = not report['errors'] and len(report['stages']) == len(stages) and process.returncode == 0
        report['native_exit'] = process.returncode
        (out/'report.json').write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2), '\nREPORT_DIRECTORY', out, flush=True)


if __name__ == '__main__':
    main()
