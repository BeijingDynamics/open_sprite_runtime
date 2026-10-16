"""Paced recorded-state / fake-IMU integration test. Never opens CAN or I2C."""
import argparse
import json
from pathlib import Path
import socket
import struct
import subprocess
import tempfile
import threading
import time
import numpy as np
from open_sprite_runtime.native_ipc import NativeStatePacket, PolicyTargetPacket, ordered_name_hash
from open_sprite_runtime.bno055 import decode_sample


def main():
    p=argparse.ArgumentParser()
    for name in ('binary','offline-binary','model','fixture','output-dir'):p.add_argument('--'+name,required=True)
    p.add_argument('--fault',choices=('none','imu_freeze','state_disconnect'),default='none')
    p.add_argument('--command-profile', choices=('stand-walk-stop-40s','stand-walk010-stop-40s'))
    p.add_argument('--powered-candidate', action='store_true', help='Fake peers only, software V2 IMU')
    p.add_argument('--icm-input', action='store_true', help='Fake ICM V3 packets; no sensor access')
    args=p.parse_args();out=Path(args.output_dir);out.mkdir(parents=True,exist_ok=False)
    data=Path(args.fixture).read_bytes();count=struct.unpack_from('<I',data,8)[0]
    header_size=len(data)-count*72*8
    rows=np.frombuffer(data[header_size:],dtype='<f8').reshape(count,72)
    mh=ordered_name_hash(f'motor_{i}' for i in range(31));jh=ordered_name_hash(f'joint_{i}' for i in range(31))
    errors=[];latencies=[];targets=[];stop=threading.Event();shared={'index':0};fault_time=[None]
    with tempfile.TemporaryDirectory(prefix='sprite_loop_fake_') as temp:
        state_path=str(Path(temp)/'state.sock');imu_path=str(Path(temp)/'imu.sock')
        server=socket.socket(socket.AF_UNIX,socket.SOCK_SEQPACKET);server.bind(state_path);server.listen(1);server.settimeout(.1)
        imu=socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM);imu.bind(imu_path);imu.settimeout(.05)
        def state_worker():
            try:
                while not stop.is_set():
                    try:conn,_=server.accept();break
                    except socket.timeout:continue
                else:return
                with conn:
                    conn.settimeout(1)
                    start=time.monotonic()
                    for i,row in enumerate(rows):
                        if stop.wait(max(0,start+i*.02-time.monotonic())):return
                        shared['index']=i
                        if args.fault=='state_disconnect' and i==25:
                            fault_time[0]=time.monotonic();return
                        stamp=time.monotonic_ns()
                        conn.sendall(NativeStatePacket(i+1,stamp,mh,tuple(row[:31]),tuple(row[31:62])).pack())
                        payload=conn.recv(4096)
                        if not payload:return
                        value=PolicyTargetPacket.unpack(payload)
                        if value.joint_order_hash!=jh or value.sequence!=i+1 or value.source_state_sequence!=i+1:
                            raise AssertionError('target metadata mismatch')
                        latencies.append((time.monotonic_ns()-stamp)/1e6);targets.append(payload)
                    if args.powered_candidate:
                        time.sleep(.35)  # Model the CAN controller's final disable verification.
            except (BrokenPipeError,ConnectionResetError):pass
            except Exception as exc:errors.append('state: '+repr(exc))
        def imu_worker():
            seq=0;packet=None;next_sample=0
            try:
                while not stop.is_set():
                    try:request,peer=imu.recvfrom(100)
                    except socket.timeout:continue
                    if request!=b'GETIMU01':raise AssertionError('IMU request')
                    now=time.monotonic_ns()
                    freeze=args.fault=='imu_freeze' and shared['index']>=25
                    if freeze and fault_time[0] is None:fault_time[0]=time.monotonic()
                    if packet is None or (now>=next_sample and not freeze):
                        seq+=1;next_sample=now+10000000
                        row=rows[shared['index']];raw=bytearray(46)
                        for j in range(3):struct.pack_into('<h',raw,12+2*j,int(round(row[62+j]*900)))
                        for j in range(4):struct.pack_into('<h',raw,24+2*j,int(round(row[65+j]*16384)))
                        sample=decode_sample(bytes(raw),now-1000000,now)
                        values=(*sample.acceleration_p0_m_s2,*sample.gyro_p0_rad_s,*sample.quaternion_p0_to_world_wxyz,*sample.gravity_p0_m_s2)
                        if args.powered_candidate:
                            q=np.array(row[65:69]);q=q/np.linalg.norm(q)
                            w,x,y,z=q
                            up=(2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y))
                            gyro=tuple(struct.unpack_from('<h',raw,12+2*j)[0]/900 for j in range(3))
                            values=(0.,0.,0.,*gyro,*q,*(9.80665*v for v in up))
                            packet=b'SPIMU002'+struct.pack('<QQQ',seq,now-1000000,now)+raw[:18]+b'\0'*6+struct.pack('<3d',0,0,0)+struct.pack('<13d',*values)+b'\0'*8
                            if args.icm_input:
                                scale=2000*np.pi/180/32768
                                counts=tuple(int(round(row[62+j]/scale)) for j in range(3))
                                gyro=tuple(v*scale for v in counts)
                                values=(0.,0.,0.,*gyro,*q,*(9.80665*v for v in up))
                                packet=b'SPIMU003'+struct.pack('<QQQ',seq,now-1000000,now)+struct.pack('<6h',0,0,0,*counts)+bytes(12)+struct.pack('<3d',0,0,0)+struct.pack('<13d',*values)+bytes(8)
                        else:
                            packet=b'SPIMU001'+struct.pack('<QQQ',seq,now-1000000,now)+raw+b'\0\0'+struct.pack('<13d',*values)+b'\0'*8
                    try:imu.sendto(packet,peer)
                    except (FileNotFoundError,ConnectionRefusedError):return
            except Exception as exc:errors.append('imu: '+repr(exc))
        threads=[threading.Thread(target=f) for f in (state_worker,imu_worker)]
        for t in threads:t.start()
        begin=time.monotonic()
        try:
            profile_args=[args.command_profile] if args.command_profile else []
            if args.powered_candidate:
                assert args.command_profile == 'stand-walk010-stop-40s'
                names=out/'fake_names.txt'
                names.write_text('SPRITE_POWERED_WALK40_V1\n'+'\n'.join(
                    [f'motor_{i}' for i in range(31)]+[f'joint_{i}' for i in range(31)])+'\n')
                profile_args=[str(names),args.command_profile,'ENABLE_NATIVE_ACTOR_WALK40']
            proc=subprocess.run([args.binary,args.model,args.fixture,str(out/'live.bin'),state_path,imu_path]+profile_args,
                                capture_output=True,text=True,timeout=count*.02+20)
            ended=time.monotonic()
        finally:
            stop.set()
            for t in threads:t.join(2)
            server.close();imu.close()
        (out/'native.log').write_text(proc.stdout+proc.stderr)
    parity=None
    ipc_matches=None
    if proc.returncode==0 and args.fault=='none':
        (out/'consumed.bin').write_bytes(data[:header_size]+(out/'live.bin.inputs').read_bytes())
        offline=subprocess.run([args.offline_binary,args.model,str(out/'consumed.bin'),str(out/'offline.bin')],capture_output=True,text=True)
        (out/'offline.log').write_text(offline.stdout+offline.stderr)
        parity=offline.returncode==0 and (out/'live.bin').read_bytes()==(out/'offline.bin').read_bytes()
        dtype=np.dtype([('obs','<f4',(795,)),('raw','<f4',(31,)),('handoff','<f8',(31,)),
                        ('target','<f8',(31,)),('protected','<f8',(5,31))])
        computed=np.fromfile(out/'live.bin',dtype=dtype)
        ipc_matches=len(computed)==len(targets)
        for row,payload in zip(computed,targets):
            packet=PolicyTargetPacket.unpack(payload)
            transmitted=np.array([packet.position_rad,packet.velocity_rad_s,packet.kp,packet.kd,packet.feedforward_torque_nm])
            ipc_matches=ipc_matches and np.array_equal(row['protected'],transmitted)
    expected_error='stale' if args.fault=='imu_freeze' else 'disconnected'
    passed=not errors and not any(t.is_alive() for t in threads) and (
        proc.returncode==0 and len(targets)==count and parity and ipc_matches and max(latencies)<20 if args.fault=='none' else
        proc.returncode!=0 and expected_error in proc.stderr and fault_time[0] is not None and ended-fault_time[0]<1)
    (out/'targets.bin').write_bytes(b''.join(targets))
    result=dict(mode='paced_native_policy_fake_peers_no_hardware',hardware_tx=0,fault=args.fault,
                frames=len(targets),expected_frames=count,elapsed_s=ended-begin,returncode=proc.returncode,
                offline_byte_exact=parity,ipc_matches_computed_target=ipc_matches,
                roundtrip_over_20ms=sum(x>=20 for x in latencies),errors=errors,stderr=proc.stderr,
                state_roundtrip_ms={'median':float(np.median(latencies)),'p99':float(np.percentile(latencies,99)),
                                    'max':max(latencies)} if latencies else None,
                fault_exit_delay_s=ended-fault_time[0] if fault_time[0] is not None else None,passed=bool(passed))
    (out/'report.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
    if not passed:raise SystemExit('realtime loop test failed')


if __name__=='__main__':main()
