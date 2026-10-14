"""Extract kernel event windows around the slowest measured BNO reads."""
import argparse
import bisect
import csv
import json
from pathlib import Path
import re


def main():
    p=argparse.ArgumentParser()
    p.add_argument('directory',type=Path)
    args=p.parse_args()
    report=json.loads((args.directory/'report.json').read_text())
    if not report['passed']:
        raise SystemExit('Capture was not complete/loss-free; inspect report first')
    with (args.directory/'imu.json.timing.csv').open(newline='') as f:
        rows=[{k:int(v) for k,v in row.items()} for row in csv.DictReader(f)]
    events=[]
    for line in (args.directory/'trace.txt').read_text().splitlines():
        match=re.search(r'\s(\d+)\.(\d+): (\w+): (.*)$',line)
        if match:
            sec,frac,name,body=match.groups()
            ns=int(sec)*10**9+int(frac.ljust(9,'0'))
            events.append((ns,name,body))
    events.sort(key=lambda e:e[0])
    times=[e[0] for e in events]
    windows=[]
    waiting={}
    dispatch={pid:[] for pid in report.get('irq_thread_pids',[])}
    for stamp,name,body in events:
        if name=='sched_wakeup':
            match=re.search(r'\bpid=(\d+)\b',body)
            if match and int(match[1]) in dispatch:
                waiting.setdefault(int(match[1]),stamp)
        elif name=='sched_switch':
            match=re.search(r'\bnext_pid=(\d+)\b',body)
            if match and int(match[1]) in waiting:
                pid=int(match[1])
                dispatch[pid].append((stamp-waiting.pop(pid))/1e6)
    for row in sorted(rows,key=lambda r:r['read_ns'],reverse=True)[:5]:
        start=row['start_ns'];end=start+row['read_ns']
        subset=events[bisect.bisect_left(times,start):bisect.bisect_right(times,end)]
        windows.append(dict(sequence=row['sequence'],read_ms=row['read_ns']/1e6,
            cpu_ms=row['read_cpu_ns']/1e6,runqueue_ms=row['runqueue_wait_ns']/1e6,
            events=[dict(offset_ms=(t-start)/1e6,event=n,details=b) for t,n,b in subset]))
    output=dict(imu_pid=report['imu_pid'],irq_thread_pids=report.get('irq_thread_pids',[]),
                parsed_events=len(events),slow_read_windows=windows,
                irq_wakeup_to_run_ms={str(pid):dict(count=len(values),
                    maximum=max(values,default=0),
                    p99=sorted(values)[int(.99*(len(values)-1))] if values else None)
                    for pid,values in dispatch.items()})
    (args.directory/'kernel_analysis.json').write_text(json.dumps(output,indent=2)+'\n')
    print(json.dumps(output,indent=2))


if __name__=='__main__':
    main()
