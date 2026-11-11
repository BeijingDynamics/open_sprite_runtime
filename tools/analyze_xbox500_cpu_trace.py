"""Offline ftrace attribution. No hardware or configuration access."""
import argparse
import gzip
import json
import re
from pathlib import Path

LINE = re.compile(r'\s*(.*)-(\d+)\s+\[(\d+)\].*?\s(\d+\.\d+):\s+(\w+):\s*(.*)')


def field(text, name):
    match = re.search(r'\b'+re.escape(name)+r'=([^\s]+)', text)
    return match.group(1) if match else None


def analyze(path, pids):
    opener = gzip.open if path.suffix == '.gz' else open
    with opener(path, 'rt', errors='replace') as lines:
        return analyze_lines(lines, pids)


def analyze_lines(lines, pids):
    timers, waking, cycles = {}, {}, {}
    worst = []
    count = parsed = expires = switches = 0
    for line in lines:
        match = LINE.match(line)
        if not match:
            continue
        parsed += 1
        _, pid, cpu, seconds, event, data = match.groups()
        pid, cpu = int(pid), int(cpu)
        whole, fraction = seconds.split('.')
        stamp = int(whole)*1_000_000_000 + int(fraction.ljust(9, '0')[:9])
        if event == 'hrtimer_start' and pid in pids:
            address = field(data, 'hrtimer')
            deadline = field(data, 'softexpires')
            if address and deadline and deadline.isdigit() and 'hrtimer_wakeup' in data:
                timers[address] = (pid, int(deadline))
        elif event == 'hrtimer_expire_entry':
            target = timers.get(field(data, 'hrtimer'))
            now = field(data, 'now')
            if target and now and now.isdigit():
                owner, deadline = target
                cycles[owner] = dict(pid=owner, cpu=cpu, deadline_ns=deadline,
                                     timer_expiry_ns=int(now))
                waking.pop(owner, None)
                expires += 1
        elif event == 'sched_wakeup':
            target = field(data, 'pid')
            if target and int(target) in pids:
                waking[int(target)] = stamp
        elif event == 'sched_switch':
            target = field(data, 'next_pid')
            if not target or int(target) not in pids:
                continue
            owner = int(target)
            switches += 1
            cycle = cycles.pop(owner, None)
            if cycle is None:
                continue
            cycle['switch_in_ns'] = stamp
            wake = waking.pop(owner, None)
            cycle['wakeup_ns'] = wake
            cycle['timer_lateness_us'] = (cycle['timer_expiry_ns']-cycle['deadline_ns'])/1000
            cycle['switch_lateness_us'] = (stamp-cycle['deadline_ns'])/1000
            cycle['wake_to_run_us'] = (stamp-wake)/1000 if wake is not None else None
            count += 1
            worst.append(cycle)
            worst.sort(key=lambda r: r['switch_lateness_us'], reverse=True)
            del worst[20:]
    return dict(parsed_events=parsed, matched_timer_expiries=expires,
                matched_switches=switches, matched_cycles=count, worst_cycles=worst,
                attribution_available=count > 0,
                caveat='Trace overhead applies; timer callback delay and wake-to-run are not intrinsic sensor latency. Inspect IRQ/idle events around worst timestamps; no automatic root-cause verdict.')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('trace', type=Path)
    parser.add_argument('--pid', type=int, action='append', required=True)
    args = parser.parse_args()
    print(json.dumps(analyze(args.trace, set(args.pid)), indent=2))
