"""Live BNO source plus recorded-observation ONNX inference; never opens CAN."""
import argparse
from datetime import datetime
import gc
import json
from pathlib import Path
import time
import numpy as np
import onnxruntime as ort
from open_sprite_runtime.bno055_source import Bno055Source


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model',required=True)
    parser.add_argument('--trace',required=True)
    parser.add_argument('--seconds',type=int,choices=range(1,121),default=60)
    args=parser.parse_args()
    opts=ort.SessionOptions(); opts.intra_op_num_threads=1; opts.inter_op_num_threads=1
    session=ort.InferenceSession(args.model,sess_options=opts,providers=['CPUExecutionProvider'])
    observations=np.load(args.trace)['observation'].astype(np.float32)
    inp=session.get_inputs()[0]
    for i in range(20): session.run(None,{inp.name:observations[i:i+1]})
    timing=np.zeros((args.seconds*50,4))
    latest=[0]; errors=[]; gc_events=[]; count=0
    def accept(packet,stamp): latest[0]=stamp
    def callback(phase,info): gc_events.append((time.monotonic_ns(),phase,info['generation']))
    gc.collect(); gc.callbacks.append(callback)
    try:
        with Bno055Source(initialize=False) as source:
            deadline=time.monotonic()+3
            while not latest[0]:
                source.drain(accept)
                if time.monotonic()>deadline: raise RuntimeError('IMU startup timeout')
                time.sleep(.002)
            start=time.monotonic_ns()
            for i in range(len(timing)):
                target=start+i*20_000_000
                time.sleep(max(0,(target-time.monotonic_ns())/1e9))
                source.drain(accept)
                before=time.monotonic_ns(); stamp=latest[0]
                j=i%len(observations)
                session.run(None,{inp.name:observations[j:j+1]})
                after=time.monotonic_ns()
                if after-stamp>30_000_000: raise RuntimeError('IMU older than30ms after inference')
                timing[i]=[(before-target)/1e6,(after-before)/1e6,(before-stamp)/1e6,(after-stamp)/1e6]
                count+=1
            source_report=source.report()
    except Exception as exc:
        errors.append(repr(exc)); source_report=source.report()
    finally:
        gc.callbacks.remove(callback)
    report=dict(mode='live_bno_recorded_observation_onnx_no_can',can_tx_count=0,
        observation_source=args.trace,model=args.model,live_imu_used_for_inference=False,
        note='Tests concurrent live source and ONNX timing, not full native IPC/control loop.',
        ticks=count,expected_ticks=len(timing),gc_enabled=gc.isenabled(),gc_events=gc_events,
        source=source_report,errors=errors,passed=not errors and count==len(timing))
    if count:
        report['p50_p99_max_ms']={key:np.percentile(timing[:count,j],[50,99,100]).tolist()
            for j,key in enumerate(('wake_lateness','inference','imu_age_before','imu_age_after'))}
    path=Path('reports')/datetime.now().strftime('bno_policy_no_can_%Y%m%d_%H%M%S.json')
    path.write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2));print(path)
    raise SystemExit(not report['passed'])


if __name__=='__main__': main()
