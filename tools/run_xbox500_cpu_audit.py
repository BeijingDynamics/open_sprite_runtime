"""Two zero-gain runs with isolated CPU5 ftrace and scoped PM QoS. Root wrapper."""
import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import struct
import subprocess
import sys
import tempfile
import time
import zlib

from analyze_xbox500_cpu_trace import analyze

ROOT = Path(__file__).resolve().parents[1]
STABLE = Path('/home/tony/open_sprite_runtime')
EVENTS = ('sched/sched_switch', 'sched/sched_wakeup', 'power/cpu_idle',
          'irq/irq_handler_entry', 'irq/irq_handler_exit',
          'irq/softirq_entry', 'irq/softirq_exit',
          'timer/hrtimer_start', 'timer/hrtimer_expire_entry')
OPTIONAL_EVENTS = ('ipi/ipi_entry', 'ipi/ipi_exit')
TRACE_SECONDS = 20
TRACE_BYTES = 64*1024*1024
CONFIRM = 'ZERO_GAIN_CPU_AUDIT_31_DISABLED'


def compress_trace(source, output, limit):
    """Export a stopped kernel buffer without discarding events or buffering text."""
    compressor = zlib.compressobj(level=1, wbits=31)
    digest = hashlib.sha256()
    stored = expanded = 0

    def emit(data):
        nonlocal stored
        if stored+len(data) > limit:
            raise RuntimeError('Trace size cap reached; trace incomplete')
        output.write(data)
        stored += len(data)

    while data := source.read(65536):
        digest.update(data)
        expanded += len(data)
        emit(compressor.compress(data))
    emit(compressor.flush())
    return stored, expanded, digest.hexdigest()


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
        self.errors = []
        self.bytes = 0
        self.expanded_bytes = 0
        self.text_sha256 = None
        self.started_ns = None
        self.stopped_ns = None
        self.pid = None
        self.stats = ''

    def __enter__(self):
        self.path.mkdir()  # Never modify an existing trace instance.
        try:
            (self.path/'tracing_on').write_text('0')
            (self.path/'current_tracer').write_text('nop')
            (self.path/'events/enable').write_text('0')
            (self.path/'trace_clock').write_text('mono')
            (self.path/'tracing_cpumask').write_text('20')  # CPU5 only.
            (self.path/'buffer_size_kb').write_text('1')
            (self.path/'per_cpu/cpu5/buffer_size_kb').write_text('32768')
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
            return self
        except BaseException:
            self.__exit__(None, None, None)
            raise

    def start_for_pid(self, pid):
        if self.pid is not None:
            if self.pid != pid:
                raise RuntimeError('Trace target changed')
            return
        self.pid = int(pid)
        filters = {'sched/sched_switch': f'prev_pid == {pid} || next_pid == {pid}',
                   'sched/sched_wakeup': f'pid == {pid}',
                   'timer/hrtimer_start': f'common_pid == {pid}'}
        for event, value in filters.items():
            (self.path/'events'/event/'filter').write_text(value)
        save(self.out/'target_filters.json', filters)
        self.started_ns = time.monotonic_ns()
        (self.path/'tracing_on').write_text('1')

    def stop_capture(self):
        if self.started_ns is not None and self.stopped_ns is None:
            (self.path/'tracing_on').write_text('0')
            self.stopped_ns = time.monotonic_ns()

    def update_window(self):
        if self.started_ns is not None and time.monotonic_ns()-self.started_ns >= TRACE_SECONDS*1_000_000_000:
            self.stop_capture()

    def collect_stopped(self):
        # Reading trace_pipe while tracing IRQ/scheduler activity can trace the
        # reader's own wakeups recursively. Never consume a live trace here.
        if (self.path/'tracing_on').read_text().strip() != '0':
            raise RuntimeError('Refuse to read an active trace')
        self.stats = (self.path/'per_cpu/cpu5/stats').read_text()
        counters = dict(line.split(':', 1) for line in self.stats.splitlines() if ':' in line)
        for key in ('overrun', 'commit overrun', 'dropped events'):
            if key not in counters or int(counters[key].strip()) != 0:
                self.errors.append('Trace counter missing or nonzero: '+key)
        with (self.path/'per_cpu/cpu5/trace').open('rb') as source, (self.out/'cpu5.trace.gz').open('wb') as output:
            try:
                self.bytes, self.expanded_bytes, self.text_sha256 = compress_trace(source, output, TRACE_BYTES)
            finally:
                self.bytes = output.tell()

    def __exit__(self, *unused):
        if not self.path.exists():
            return
        try:
            self.stop_capture()
            (self.path/'tracing_on').write_text('0')
            self.collect_stopped()
        except Exception as error:
            self.errors.append(repr(error))
        finally:
            (self.path/'events/enable').write_text('0')
            self.path.rmdir()
            save(self.out/'trace_status.json', dict(bytes=self.bytes, errors=self.errors, stats=self.stats,
                 pid=self.pid, capture_start_ns=self.started_ns, capture_end_ns=self.stopped_ns,
                 requested_capture_seconds=TRACE_SECONDS, maximum_bytes=TRACE_BYTES,
                 encoding='gzip', expanded_bytes=self.expanded_bytes, text_sha256=self.text_sha256,
                 read_only_after_tracing_stopped=True,
                 coverage='first20s after target detected; remaining zero-gain run is untraced'))


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
                if len(observed) > 1:
                    raise RuntimeError('Multiple CAN candidates detected')
                if observed:
                    trace.start_for_pid(next(iter(observed)))
                trace.update_window()
                samples.write(json.dumps(dict(monotonic_ns=time.monotonic_ns(), processes=observed))+'\n')
                if time.monotonic() > deadline:
                    raise RuntimeError('Zero-gain test timed out')
                if trace.errors:
                    raise RuntimeError('Trace acquisition failed: '+str(trace.errors))
                time.sleep(0.5)
            rc = proc.returncode
        created = set((ROOT/'reports').glob('xbox_500hz_full_shadow_120s_*'))-before
        if len(created) != 1:
            raise RuntimeError('Expected exactly one new zero-gain report directory')
        report_dir = created.pop()
        report = json.loads((report_dir/'report.json').read_text())
        can = json.loads((report_dir/'can.json').read_text())
        save(out/'result.json', dict(returncode=rc, report_directory=str(report_dir),
             report=report, can=can, can_pids=sorted(pids), duration_wall_s=time.monotonic()-started))
        print(name, 'zero-gain report passed=',report['passed'], 'trace_errors=',trace.errors,flush=True)
        if trace.errors or trace.pid is None:
            raise RuntimeError('Invalid trace; no next phase: '+str(trace.errors))
        analysis = analyze(out/'cpu5.trace.gz', pids)
        save(out/'trace_analysis.json', analysis)
        if analysis['matched_cycles'] < 100:
            raise RuntimeError('Insufficient CAN timer cycles; no next phase')
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


def trace_smoke(out):
    """Exercise actual kernel event format/volume before any CAN/SPI is launched."""
    out.mkdir()
    print('TRACE_SMOKE: 5s CPU5 timer only; no CAN/SPI',flush=True)
    proc = None
    timer_code = ('import time; time.sleep(0.2); end=time.monotonic()+5; '
                  '\nwhile time.monotonic()<end: time.sleep(0.0005)')
    try:
        with Trace(out) as trace:
            proc = subprocess.Popen(['taskset','-c','5',sys.executable,'-c',timer_code])
            trace.start_for_pid(proc.pid)
            proc.wait(timeout=10)
            if proc.returncode or trace.errors:
                raise RuntimeError('No-hardware trace smoke failed: '+str(trace.errors))
        if trace.errors:
            raise RuntimeError('No-hardware trace smoke incomplete: '+str(trace.errors))
        result = analyze(out/'cpu5.trace.gz',{proc.pid})
        save(out/'trace_analysis.json',result)
        if result['matched_cycles'] < 100:
            raise RuntimeError('Insufficient matching timer events; do not start hardware tests')
        print('TRACE_SMOKE_OK no CAN/SPI; bytes=',trace.bytes,flush=True)
    finally:
        if proc is not None and proc.poll() is None:
            proc.terminate()
            try: proc.wait(timeout=3)
            except subprocess.TimeoutExpired: proc.kill(); proc.wait()


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
        trace_smoke(out/'trace_smoke_no_hardware')
        phase('A_current',out/'A_current',False)
        phase('B_qos0',out/'B_qos0',True)
    finally:
        print('CPU_AUDIT_FINISHED; temporary QoS request released; no powered test',flush=True)


if __name__ == '__main__':
    main()
