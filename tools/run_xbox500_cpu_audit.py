"""Two zero-gain runs with isolated CPU5 ftrace and scoped PM QoS. Root wrapper."""
import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import select
import shutil
import signal
import struct
import subprocess
import tempfile
import threading
import time

from analyze_xbox500_cpu_trace import analyze

ROOT = Path(__file__).resolve().parents[1]
STABLE = Path('/home/tony/open_sprite_runtime')
EVENTS = ('sched/sched_switch', 'sched/sched_wakeup', 'power/cpu_idle',
          'irq/irq_handler_entry', 'irq/irq_handler_exit',
          'irq/softirq_entry', 'irq/softirq_exit',
          'timer/hrtimer_start', 'timer/hrtimer_expire_entry', 'timer/hrtimer_expire_exit')
OPTIONAL_EVENTS = ('ipi/ipi_entry', 'ipi/ipi_exit', 'power/cpu_frequency', 'sched/sched_waking')
CONFIRM = 'ZERO_GAIN_CPU_AUDIT_31_DISABLED'


def save(path, data):
    path.write_text(json.dumps(data, indent=2)+'\n')


def read(path):
    try:
        return Path(path).read_text().strip()
    except OSError as error:
        return str(error)


def snapshot():
    paths = ['/proc/interrupts', '/proc/softirqs', '/proc/sys/kernel/sched_rt_runtime_us',
             '/proc/sys/kernel/sched_schedstats']
    paths += [str(p) for p in Path('/sys/devices/system/cpu/cpu5/cpuidle').glob('state*/*')]
    paths += [str(Path('/sys/devices/system/cpu/cpu5/cpufreq')/n) for n in
              ('scaling_governor', 'scaling_cur_freq', 'scaling_min_freq', 'scaling_max_freq')]
    return {'monotonic_ns': time.monotonic_ns(), 'values': {p: read(p) for p in paths}}


class Trace:
    def __init__(self, out):
        self.out = out
        self.path = Path('/sys/kernel/tracing/instances')/('xbox500_'+str(os.getpid()))
        self.stop = threading.Event()
        self.thread = None
        self.errors = []
        self.bytes = 0

    def __enter__(self):
        self.path.mkdir()  # Never modify an existing trace instance.
        try:
            (self.path/'tracing_on').write_text('0')
            (self.path/'trace_clock').write_text('mono')
            (self.path/'tracing_cpumask').write_text('20')  # CPU5 only.
            (self.path/'buffer_size_kb').write_text('8192')
            formats = {}
            for event in EVENTS:
                directory = self.path/'events'/event
                formats[event] = (directory/'format').read_text()
                (directory/'enable').write_text('1')
            for event in OPTIONAL_EVENTS:
                directory = self.path/'events'/event
                if (directory/'enable').exists():
                    formats[event] = (directory/'format').read_text()
                    (directory/'enable').write_text('1')
            save(self.out/'event_formats.json', formats)
            self.thread = threading.Thread(target=self.pump, daemon=True)
            self.thread.start()
            (self.path/'tracing_on').write_text('1')
            return self
        except BaseException:
            self.__exit__(None, None, None)
            raise

    def pump(self):
        fd = None
        try:
            fd = os.open(self.path/'trace_pipe', os.O_RDONLY | os.O_NONBLOCK)
            with (self.out/'cpu5.trace').open('wb') as output:
                while not self.stop.is_set():
                    if not select.select([fd], [], [], 0.1)[0]:
                        continue
                    try:
                        data = os.read(fd, 65536)
                    except BlockingIOError:
                        continue
                    if not data:
                        continue
                    if self.bytes+len(data) > 256*1024*1024:
                        raise RuntimeError('Trace size cap reached; trace incomplete')
                    output.write(data)
                    self.bytes += len(data)
        except BaseException as error:
            self.errors.append(repr(error))
        finally:
            if fd is not None:
                os.close(fd)

    def __exit__(self, *unused):
        if self.path.exists():
            (self.path/'tracing_on').write_text('0')
            time.sleep(0.2)
            self.stop.set()
            if self.thread:
                self.thread.join(timeout=3)
                if self.thread.is_alive():
                    self.errors.append('Trace reader failed to stop')
            stats = read(self.path/'per_cpu/cpu5/stats')
            save(self.out/'trace_status.json', dict(bytes=self.bytes, errors=self.errors, stats=stats))
            (self.path/'events/enable').write_text('0')
            self.path.rmdir()


def can_processes():
    found = {}
    for p in Path('/proc').glob('[0-9]*/exe'):
        try:
            if Path(os.readlink(p)).name != 'sprite_can_xbox500':
                continue
            base = p.parent
            found[int(base.name)] = {n: read(base/n) for n in ('stat', 'schedstat', 'status')}
        except OSError:
            continue
    return found


def phase(name, out, qos):
    out.mkdir()
    print('PHASE',name,'keep robot stationary; zero-gain only',flush=True)
    before = set((ROOT/'reports').glob('xbox_500hz_full_shadow_120s_*'))
    save(out/'before.json', snapshot())
    fd = None
    proc = None
    pids = set()
    started = time.monotonic()
    try:
        if qos:
            fd = os.open('/dev/cpu_dma_latency', os.O_RDWR | os.O_CLOEXEC)
            if os.write(fd, struct.pack('=i', 0)) != 4:
                raise RuntimeError('PM QoS request failed')
        save(out/'condition.json', dict(name=name, qos_latency_us=0 if qos else None,
             qos_scope='system-wide only while parent fd open', governor_unchanged=True,
             irq_affinity_unchanged=True, thresholds_unchanged=True))
        with Trace(out) as trace, (out/'launcher.log').open('w') as log, (out/'process.jsonl').open('w') as samples:
            proc = subprocess.Popen(['bash', str(ROOT/'run_xbox_500hz_full_shadow_120s_on_253.sh'),
                                     'ZERO_GAIN_XBOX_500HZ_FULL_31_DISABLED', '--feedback-candidate'],
                                    stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            deadline = time.monotonic()+240
            while proc.poll() is None:
                observed = can_processes()
                pids.update(observed)
                samples.write(json.dumps(dict(monotonic_ns=time.monotonic_ns(), processes=observed))+'\n')
                if time.monotonic() > deadline:
                    raise RuntimeError('Zero-gain test timed out')
                if trace.errors:
                    raise RuntimeError('Trace acquisition failed: '+str(trace.errors))
                time.sleep(0.5)
            rc = proc.returncode
        save(out/'trace_analysis.json', analyze(out/'cpu5.trace', pids))
        created = set((ROOT/'reports').glob('xbox_500hz_full_shadow_120s_*'))-before
        if len(created) != 1:
            raise RuntimeError('Expected exactly one new zero-gain report directory')
        report_dir = created.pop()
        report = json.loads((report_dir/'report.json').read_text())
        can = json.loads((report_dir/'can.json').read_text())
        save(out/'result.json', dict(returncode=rc, report_directory=str(report_dir),
             report=report, can=can, can_pids=sorted(pids), duration_wall_s=time.monotonic()-started))
        # A completed timing rejection remains FAILED, but may be compared with B.
        if (can.get('automatic_enable_attempts') != 0 or can.get('nonzero_gain_or_torque_tx_attempts') != 0
                or report.get('frames') != 6000 or not report.get('feedback_workload_passed')
                or not report.get('exactly_60000_tx_per_motor') or report.get('actor_exit') != 0
                or report.get('imu_exit') != 0 or can.get('elapsed_s', 0) < 120
                or not report.get('offline_byte_exact') or report.get('minimum_motor_rx_coverage',0) < 0.99
                or report.get('unready_links_after')
                or any(v != 0 for counters in report.get('can_error_counter_delta',{}).values() for v in counters.values())
                or set(report.get('can_qualification_reasons',[])) - {
                    'transport_slot_overruns','state_tick_deadline_misses','state_tick_lateness'}):
            raise RuntimeError('Incomplete/unsafe run; no automatic next phase')
        print(name, 'completed; passed=',report['passed'],flush=True)
    finally:
        if proc is not None and proc.poll() is None:
            os.killpg(proc.pid, signal.SIGTERM)
            try:
                proc.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait()
        if fd is not None:
            os.close(fd)  # Remove only our QoS request, including on exceptions.
        save(out/'after.json', snapshot())


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('confirm', choices=[CONFIRM])
    args = parser.parse_args()
    if os.geteuid() != 0 or os.environ.get('SUDO_USER') != 'tony':
        raise SystemExit('Run with sudo as tony; zero-gain only')
    if ROOT != Path('/home/tony/open_sprite_runtime_xbox_500hz'):
        raise SystemExit('Independent experimental checkout required')
    subprocess.run(['runuser','-u','tony','--','bash',str(ROOT/'run_xbox_500hz_full_shadow_120s_on_253.sh'),
                    '--check','--feedback-candidate'],check=True)
    if shutil.disk_usage(ROOT).free < 2*1024**3:
        raise SystemExit('Need at least 2GiB free for bounded trace evidence')
    lock = (ROOT/'reports/.cpu_audit.lock').open('w')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    out=Path(tempfile.mkdtemp(prefix='xbox500_cpu_ab_',dir=ROOT/'reports'))
    out.chmod(0o755)  # Allow the operator/SSH reader to inspect root-captured evidence.
    save(out/'source.json', {str(p):hashlib.sha256(p.read_bytes()).hexdigest()
                            for p in (Path(__file__),ROOT/'tools/analyze_xbox500_cpu_trace.py')})
    print('ZERO GAIN ONLY. No enable. Two120s phases; keep robot stationary throughout (about6min).',flush=True)
    print('REPORT_DIRECTORY',out,flush=True)
    def interrupted(signum, frame):
        raise KeyboardInterrupt('signal '+str(signum))
    signal.signal(signal.SIGTERM, interrupted)
    try:
        phase('A_current',out/'A_current',False)
        phase('B_qos0',out/'B_qos0',True)
    finally:
        print('CPU_AUDIT_FINISHED; temporary QoS request released; no powered test',flush=True)


if __name__ == '__main__':
    main()
