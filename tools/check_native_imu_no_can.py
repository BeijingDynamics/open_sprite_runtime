"""No-CAN native IMU client with deliberate consumer stall and optional ONNX load."""
import argparse
from datetime import datetime
import json
from pathlib import Path
import socket
import tempfile
import time
import numpy as np
from open_sprite_runtime.native_imu_packet import decode_packet


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--socket',required=True)
    parser.add_argument('--seconds',type=int,choices=range(1,601),default=120)
    parser.add_argument('--model')
    parser.add_argument('--trace')
    args=parser.parse_args()
    if bool(args.model)!=bool(args.trace): parser.error('model and trace required together')
    session=None
    if args.model:
        import onnxruntime as ort
        options=ort.SessionOptions();options.intra_op_num_threads=1;options.inter_op_num_threads=1
        session=ort.InferenceSession(args.model,sess_options=options,providers=['CPUExecutionProvider'])
        observations=np.load(args.trace)['observation'].astype(np.float32)
        inp=session.get_inputs()[0].name
        for i in range(20):session.run(None,{inp:observations[i:i+1]})
    rows=np.zeros((args.seconds*50,4)); count=0; previous=0; errors=[]; pauses=[]
    try:
        with tempfile.TemporaryDirectory(prefix='sprite_imu_') as tmp, socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM) as client:
            client.bind(str(Path(tmp)/'client.sock')); client.connect(args.socket); client.settimeout(.03)
            start=time.monotonic();deadline=start
            for i in range(len(rows)):
                if i==1000:
                    before=time.monotonic_ns();time.sleep(.15)
                    pauses.append(dict(start_ns=before,end_ns=time.monotonic_ns()))
                    deadline=time.monotonic()
                time.sleep(max(0,deadline-time.monotonic()))
                client.send(b'GETIMU01');payload=client.recv(193); received=time.monotonic_ns()
                packet=decode_packet(payload,received,previous)
                stamp=packet.sample.read_start_ns
                before=time.monotonic_ns()
                if session:
                    j=i%len(observations);session.run(None,{inp:observations[j:j+1]})
                after=time.monotonic_ns()
                if after-stamp>30_000_000:raise RuntimeError('IMU stale after inference')
                rows[count]=[(received-stamp)/1e6,(after-stamp)/1e6,(after-before)/1e6,packet.sequence-previous]
                previous=packet.sequence;count+=1;deadline+=.02
    except Exception as exc:errors.append(repr(exc))
    report=dict(mode='native_imu_no_can_consumer',can_tx_count=0,live_imu_used_for_inference=False,
        note='Recorded observations for inference timing only, native live IMU checked concurrently.',
        ticks=count,expected_ticks=len(rows),pauses=pauses,errors=errors,passed=not errors and count==len(rows))
    if count:
        report['p50_p99_max_ms']={key:np.percentile(rows[:count,j],[50,99,100]).tolist()
            for j,key in enumerate(('sample_age_at_receive','sample_age_after_inference','inference'))}
        report['maximum_sequence_step']=int(rows[1:count,3].max()) if count>1 else 0
    path=Path('reports')/datetime.now().strftime('native_imu_consumer_%Y%m%d_%H%M%S.json')
    path.write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2));print(path)
    raise SystemExit(not report['passed'])


if __name__=='__main__':main()
