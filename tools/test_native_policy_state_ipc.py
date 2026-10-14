"""Fake state server; checks native IPC bytes against existing Python ABI."""
import argparse
import json
from pathlib import Path
import socket
import struct
import subprocess
import tempfile
import threading
import time
from open_sprite_runtime.native_ipc import NativeStatePacket, PolicyTargetPacket, ordered_name_hash


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--binary',required=True)
    parser.add_argument('--report',required=True)
    args=parser.parse_args()
    mh=ordered_name_hash(f'motor_{i}' for i in range(31))
    jh=ordered_name_hash(f'joint_{i}' for i in range(31))
    cases={'normal':None,'oversize':'ABI','truncated':'ABI','hash':'ABI','magic':'ABI',
           'repeat':'sequence','nan':'nonfinite','stale':'timestamp','future':'timestamp',
           'zero_stamp':'timestamp','disconnect':'disconnected','silence':'timeout','late_send':'timestamp'}
    results=[]
    for case,expected in cases.items():
        with tempfile.TemporaryDirectory(prefix='sprite_fake_state_') as directory:
            path=str(Path(directory)/'state.sock')
            server=socket.socket(socket.AF_UNIX,socket.SOCK_SEQPACKET)
            server.bind(path);server.listen(1);server.settimeout(3)
            errors=[];received=[]
            def serve():
                try:
                    conn,_=server.accept()
                    with conn:
                        conn.settimeout(2)
                        for seq in range(1,4):
                            if case=='disconnect':return
                            if case=='silence':
                                time.sleep(.2);return
                            stamp=time.monotonic_ns()
                            pos=tuple((i-15)*.01*seq for i in range(31))
                            vel=tuple(-v*.3 for v in pos)
                            b=bytearray(NativeStatePacket(seq,stamp,mh,pos,vel).pack())
                            if case=='oversize':b+=b'X'
                            if case=='truncated':b=b[:-1]
                            if case=='hash':struct.pack_into('<Q',b,24,mh^1)
                            if case=='magic':b[0]=0
                            if case=='repeat':struct.pack_into('<Q',b,8,1)
                            if case=='nan':struct.pack_into('<d',b,32,float('nan'))
                            if case=='stale':struct.pack_into('<q',b,16,stamp-200000000)
                            if case=='future':struct.pack_into('<q',b,16,stamp+1000000000)
                            if case=='zero_stamp':struct.pack_into('<q',b,16,0)
                            conn.sendall(b)
                            payload=conn.recv(4096)
                            if not payload:return
                            target=PolicyTargetPacket.unpack(payload)
                            assert target.sequence==seq and target.source_state_sequence==seq
                            assert target.joint_order_hash==jh
                            assert stamp<=target.monotonic_ns<=time.monotonic_ns()
                            assert target.position_rad==pos and target.velocity_rad_s==vel
                            assert target.kp==tuple(i+.5 for i in range(31))
                            assert target.kd==tuple(i*.01 for i in range(31))
                            assert target.feedforward_torque_nm==tuple(-i*.001 for i in range(31))
                            assert payload==target.pack()
                            received.append(seq)
                except (BrokenPipeError,ConnectionResetError):
                    pass
                except Exception as exc:errors.append(repr(exc))
            thread=threading.Thread(target=serve);thread.start()
            try:
                process=subprocess.run([args.binary,path,'150' if case=='late_send' else '0'],
                                       capture_output=True,text=True,timeout=5)
            finally:
                thread.join(3);server.close()
            ok=not errors and not thread.is_alive() and (
                process.returncode==0 and received==[1,2,3] if expected is None else
                process.returncode!=0 and expected in process.stderr)
            # Malformed state must produce no target, except the first valid repeated-sequence frame.
            if expected is not None and received!=([1] if case=='repeat' else []):ok=False
            results.append(dict(case=case,passed=ok,targets=received,stderr=process.stderr.strip(),errors=errors))
    report=dict(mode='fake_state_target_ipc_no_hardware',hardware_tx=0,tests=results,
                passed=all(r['passed'] for r in results))
    with Path(args.report).open('x') as f:json.dump(report,f,indent=2)
    print(json.dumps(report,indent=2))
    if not report['passed']:raise SystemExit(1)


if __name__=='__main__':main()
