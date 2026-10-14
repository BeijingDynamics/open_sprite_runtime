"""Bounded, per-instance I2C/scheduler trace. No CAN or IMU configuration writes."""
import json
import os
from pathlib import Path
import pwd
import re
import signal
import subprocess
import tempfile
import time

ROOT = Path('/home/tony/open_sprite_runtime')


def main():
    if os.geteuid() != 0:
        raise SystemExit('Run with sudo /usr/bin/python3; only tracefs setup requires root')
    user = pwd.getpwnam('tony')
    binary = ROOT / 'build/imu_native_stage1/sprite_imu_bno_diagnostic'
    if not binary.is_file():
        raise SystemExit('Diagnostic binary missing')
    # Do not collide with an active controller or another IMU reader.
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():
            continue
        try:
            exe = Path(os.readlink(entry / 'exe')).name
        except OSError:
            continue
        if exe.startswith(('sprite_can_', 'sprite_imu_', 'sprite_policy_')):
            raise SystemExit('Stop existing runtime process first: ' + entry.name)
    trace_root = Path('/sys/kernel/tracing')
    if not (trace_root / 'available_events').exists():
        trace_root = Path('/sys/kernel/debug/tracing')
    out = Path(tempfile.mkdtemp(prefix='bno_kernel_', dir=ROOT / 'reports'))
    os.chown(out, user.pw_uid, user.pw_gid)
    instance = trace_root / 'instances' / ('sprite_bno_' + str(os.getpid()))
    instance.mkdir()
    child = consumer = None
    handles = []
    report = dict(mode='bno_kernel_trace_no_can', duration_s=180, can_tx_count=0,
                  imu_configuration_writes=0, errors=[], events={})
    socket_path = '/tmp/sprite_bno_kernel_' + str(os.getpid()) + '.sock'

    def write(relative, value):
        (instance / relative).write_text(str(value))

    def event(name, expression):
        path = 'events/' + name
        report['events'][name] = expression
        write(path + '/filter', expression)
        (out / (name.replace('/', '_') + '.format')).write_text(
            (instance / path / 'format').read_text())
        write(path + '/enable', '1')

    def stop(proc):
        if proc is not None and proc.poll() is None:
            proc.send_signal(signal.SIGCONT)
            proc.terminate()
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()

    try:
        write('tracing_on', '0')
        write('trace_clock', 'mono')
        write('buffer_size_kb', '16384')
        log = (out / 'imu.log').open('w'); handles.append(log)
        # Bash execs as the same PID; stop before opening hardware so filters
        # cover every transaction. The diagnostic itself runs unprivileged.
        child = subprocess.Popen(
            ['/bin/bash', '-c', 'kill -STOP $$; exec "$@"', 'bno-trace',
             str(binary), '--record', '180', socket_path, str(out / 'imu.json')],
            user=user.pw_uid, group=user.pw_gid,
            extra_groups=os.getgrouplist(user.pw_name, user.pw_gid),
            cwd=ROOT, stdout=log, stderr=subprocess.STDOUT)
        _, status = os.waitpid(child.pid, os.WUNTRACED)
        if not os.WIFSTOPPED(status):
            raise RuntimeError('Diagnostic did not enter setup barrier')
        pid = child.pid
        report['imu_pid'] = pid
        for name in ('i2c_write', 'i2c_read', 'i2c_reply', 'i2c_result'):
            event('i2c/' + name, f'adapter_nr == 7 && common_pid == {pid}')
        event('sched/sched_switch', f'prev_pid == {pid} || next_pid == {pid}')
        event('sched/sched_wakeup', f'pid == {pid}')
        interrupts = Path('/proc/interrupts').read_text()
        (out / 'interrupts_before.txt').write_text(interrupts)
        matches = re.findall(r'^\s*(\d+):[^\n]*c250000\.i2c', interrupts, re.M)
        if len(matches) != 1:
            raise RuntimeError('Cannot uniquely identify I2C7 IRQ')
        report['i2c_irq'] = int(matches[0])
        irq_threads=[]
        for entry in Path('/proc').iterdir():
            if not entry.name.isdigit():
                continue
            try:
                comm=(entry/'comm').read_text().strip()
            except OSError:
                continue
            if comm.startswith('irq/'+matches[0]+'-'):
                irq_threads.append(int(entry.name))
        report['irq_thread_pids']=irq_threads
        tracked=[pid,*irq_threads]
        event('sched/sched_switch', ' || '.join(
            f'prev_pid == {p} || next_pid == {p}' for p in tracked))
        event('sched/sched_wakeup', ' || '.join(f'pid == {p}' for p in tracked))
        for name in ('irq_handler_entry', 'irq_handler_exit'):
            event('irq/' + name, 'irq == ' + matches[0])
        # Runtime-PM callbacks in the issuing thread help identify resume cost.
        for name in ('device_pm_callback_start', 'device_pm_callback_end'):
            if (instance / 'events/power' / name / 'format').exists():
                event('power/' + name, f'common_pid == {pid}')
        write('tracing_on', '1')
        print('NO CAN / NO ENABLE / NO IMU WRITES. REPORT_DIRECTORY', out, flush=True)
        child.send_signal(signal.SIGCONT)
        deadline = time.monotonic() + 5
        while not Path(socket_path).is_socket():
            if child.poll() is not None or time.monotonic() > deadline:
                raise RuntimeError('IMU startup failed; see imu.log')
            time.sleep(.02)
        log = (out / 'consumer.log').open('w'); handles.append(log)
        consumer = subprocess.Popen(
            [str(ROOT / '.venv/bin/python'), 'tools/check_native_imu_no_can.py',
             '--socket', socket_path, '--seconds', '170'],
            user=user.pw_uid, group=user.pw_gid,
            extra_groups=os.getgrouplist(user.pw_name, user.pw_gid), cwd=ROOT,
            env=dict(os.environ, PYTHONPATH=str(ROOT / 'src')),
            stdout=log, stderr=subprocess.STDOUT)
        report['imu_exit'] = child.wait(timeout=195)
        report['consumer_exit'] = consumer.wait(timeout=5)
        if report['imu_exit'] or report['consumer_exit']:
            raise RuntimeError('Acquisition/consumer failed; preserve trace for diagnosis')
    except BaseException as exc:
        report['errors'].append(repr(exc))
    finally:
        stop(consumer)
        stop(child)
        try:
            write('tracing_on', '0')
            (out / 'trace.txt').write_text((instance / 'trace').read_text())
            report['cpu_buffer_stats'] = {p.parent.name:p.read_text()
                for p in (instance / 'per_cpu').glob('cpu*/stats')}
            for stats in report['cpu_buffer_stats'].values():
                if any(int(n) for n in re.findall(r'^(?:overrun|commit overrun|dropped events):\s*(\d+)', stats, re.M)):
                    report['errors'].append('Trace buffer loss: attribution may be incomplete')
        except Exception as exc:
            report['errors'].append('trace export: ' + repr(exc))
        finally:
            try:
                write('events/enable', '0')
                instance.rmdir()
            except Exception as exc:
                report['errors'].append('trace cleanup: ' + repr(exc))
        for handle in handles:
            handle.close()
        report['passed'] = not report['errors']
        (out / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
        for path in out.iterdir():
            os.chown(path, user.pw_uid, user.pw_gid)
        print(json.dumps(report, indent=2), flush=True)
        print('REPORT_DIRECTORY', out, flush=True)
    raise SystemExit(0 if report['passed'] else 1)


if __name__ == '__main__':
    main()
