"""Bounded real-feedback native actor test; zero-gain CAN and zero-gain IPC only."""
import argparse
import hashlib
import json
import os
import socket
from pathlib import Path
import struct
import subprocess
import tempfile
import time
import numpy as np
from open_sprite_runtime.native_ipc import ordered_name_hash
from open_sprite_runtime.native_policy_client import _native_motor_order


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--confirm',required=True,choices=['ZERO_GAIN_ALL_DISABLED_SUPPORTED'])
    p.add_argument('--duration',type=int,default=10,choices=range(1,121))
    p.add_argument('--software-imu-candidate',type=Path)
    p.add_argument('--icm-candidate',type=Path)
    p.add_argument('--runtime-root',type=Path)
    p.add_argument('--command-profile',choices=['stand-walk010-stop-40s'])
    args=p.parse_args()
    if args.icm_candidate and args.software_imu_candidate:
        p.error('select exactly one IMU candidate')
    icm=args.icm_candidate.resolve() if args.icm_candidate else None
    if args.command_profile and args.duration != 40:
        p.error('walk40 shadow profile requires duration=40')
    frames=args.duration*50
    root=(args.runtime_root or Path(__file__).resolve().parents[1]).resolve();os.chdir(root)
    links=json.loads(subprocess.check_output(['ip','-json','-details','link','show','type','can'],text=True))
    by_name={link['ifname']:link for link in links}
    for name in ('kcan1','kcan2','kcan3','kcan4'):
        link=by_name.get(name,{})
        info=link.get('linkinfo',{}).get('info_data',{})
        if ('UP' not in link.get('flags',[]) or link.get('mtu')!=72
                or info.get('bittiming',{}).get('bitrate')!=1000000
                or info.get('data_bittiming',{}).get('bitrate')!=5000000
                or info.get('state')!='ERROR-ACTIVE'):
            raise RuntimeError(name+' requires UP, CAN-FD MTU72, ERROR-ACTIVE, 1M/5M; run CAN setup first')
    candidate=args.software_imu_candidate.resolve() if args.software_imu_candidate else None
    if icm:
        checked=subprocess.run(['ldd',str(icm/'build/sprite_policy_native_icm_shadow')],capture_output=True,text=True,check=True)
        if 'not found' in checked.stdout:raise RuntimeError('ICM inference shared library missing: '+checked.stdout)
    if candidate:
        checked=subprocess.run(['ldd',str(candidate/'build/imu_software_candidate/sprite_policy_native_software_shadow')],capture_output=True,text=True,check=True)
        if 'not found' in checked.stdout:raise RuntimeError('candidate inference shared library missing: '+checked.stdout)
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():continue
        try:exe=os.readlink(entry/'exe')
        except OSError:continue
        if Path(exe).name in ('sprite_can_shadow','sprite_can_shadow_quantized_v1','sprite_imu_bno','sprite_policy_native_shadow','sprite_imu_software','sprite_policy_native_software_shadow','sprite_icm45686','sprite_policy_native_icm_shadow'):
            raise RuntimeError('existing hardware process: '+entry.name)
    out=Path(tempfile.mkdtemp(prefix='native_actor_hardware_shadow_',dir=root/'reports'))
    print('REPORT_DIRECTORY',out,flush=True)
    verified=root/'reports/native_policy_chain_155525_v2'
    proof=json.loads((verified/'report.json').read_text())
    hardware=root/'config/hardware.sprite0825.kp110_kd5.json'
    contract=Path('/home/tony/sprite_runtime/sprite0825_stage2_g74_model3000_sim2real_candidate_kp130_kd5_20261010_arms7_kd3_20261012_follow_v1_waist50_hiproll150_waist70_kd3_kd35_waist60kd3/deploy/contract.json')
    limits=root/'artifacts/g60_model3450/reports/sprite0825_urdf_limit_candidates.json'
    for name,path in [('hardware',hardware),('contract',contract),('limits',limits)]:
        if hashlib.sha256(path.read_bytes()).hexdigest()!=proof['source_sha256'][name]:raise ValueError(name+' changed since parity')
    hw=json.loads(hardware.read_text());c=json.loads(contract.read_text());model=contract.parent/c['policy_onnx']
    if hashlib.sha256(model.read_bytes()).hexdigest()!=proof['model_sha256']:raise ValueError('model changed')
    trace=np.load(root/'reports/native_protected_policy_trace_20261013_155525.npz',allow_pickle=False)
    motors=_native_motor_order(hw);joints=tuple(c['joint_names'])
    if tuple(trace['motor_names'])!=motors or tuple(trace['joint_names'])!=joints:raise ValueError('fixture order differs from live CAN')
    data=(verified/'input.bin').read_bytes();count=struct.unpack_from('<I',data,8)[0]
    header=bytearray(data[:-count*72*8]);struct.pack_into('<I',header,8,frames)
    # Recorded sensor slots are never consumed by the live path; commands are stand-only.
    rows=np.zeros((frames,72),dtype='<f8')
    fixture=out/'input.bin';fixture.write_bytes(header+rows.tobytes())
    names=out/'names.txt';names.write_text('SPRITE_ZERO_GAIN_SHADOW_V1\n'+'\n'.join((*motors,*joints))+'\n')
    manifest=dict(mode='native_actor_zero_gain_hardware_shadow',hardware_tx='zero-gain only',
                  source_proof=proof,fixture_sha256=hashlib.sha256(fixture.read_bytes()).hexdigest(),
                  motor_order=list(motors),joint_order=list(joints),duration_s=args.duration,
                  software_imu_candidate=str(candidate) if candidate else None,
                  icm_candidate=str(icm) if icm else None)
    (out/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    # Fixed, user-approved boot configuration. Never reconfigure during control.
    from open_sprite_runtime.bno055 import BNO055, I2CRegisters, CONFIRM
    io=None if candidate or icm else I2CRegisters(7,0x28)
    try:
        sensor=None if candidate or icm else BNO055(io)
        configuration={} if candidate or icm else sensor.configure(CONFIRM)
        deadline=time.monotonic()+5
        consecutive=0
        while not (candidate or icm) and consecutive<20:
            if time.monotonic()>deadline:
                raise RuntimeError('BNO055 fusion did not become continuously valid')
            configuration=sensor.inspect()
            if not configuration['matches'] or configuration['system_error']:
                raise RuntimeError('BNO055 startup configuration/health failed')
            try:
                sensor.read_sample()
                consecutive=consecutive+1 if configuration['system_status']==5 else 0
            except RuntimeError:
                consecutive=0
            time.sleep(.01)
        (out/'imu_startup.json').write_text(json.dumps(configuration,indent=2)+'\n')
    finally:
        if io is not None:io.close()
    env=dict(os.environ,PYTHONPATH=str(root/'src'),OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1')
    python=str(root/'.venv/bin/python')
    for tool,extra,dest in [
        ('export_native_motor_config.py',[],out/'motors.tsv'),
        ('export_native_kinematics_config.py',['--contract',str(contract)],out/'kinematics.tsv')]:
        subprocess.run([python,'tools/'+tool,'--hardware',str(hardware),*extra,'--output',str(dest)],env=env,check=True)
    with tempfile.TemporaryDirectory(prefix='sprite_native_shadow_') as sockets:
        imu_socket=str(Path(sockets)/'imu.sock');state_socket=str(Path(sockets)/'state.sock')
        processes=[];handles=[]
        def launch(command,name):
            handle=(out/(name+'.log')).open('w');handles.append(handle)
            process=subprocess.Popen(command,stdout=handle,stderr=subprocess.STDOUT,env=env)
            processes.append(process);return process
        def wait_socket(path,process):
            deadline=time.monotonic()+3
            while not Path(path).is_socket():
                if process.poll() is not None or time.monotonic()>deadline:raise RuntimeError('producer not ready: '+path)
                time.sleep(.02)
        actor_rc=can_rc=imu_rc=None
        failure=None
        try:
            imu_command=([str(candidate/'build/imu_software_candidate/sprite_imu_software'),'--record',str(args.duration+35),imu_socket,str(out/'imu.json'),'CONFIGURE_RAW_P0_NO_CAN','200','burst18'] if candidate else
                         ['build/imu_native_stage1/sprite_imu_bno','--record',str(args.duration+15),imu_socket,str(out/'imu.json')])
            if icm:
                pm=Path('/sys/bus/platform/devices/3210000.spi/power/control').read_text().strip()
                if pm!='on':raise RuntimeError('ICM qualification requires SPI runtime PM=on; use sudo wrapper')
                chip,offset=subprocess.check_output(['gpiofind','PY.01'],text=True).split()
                env['ICM_IPC_SOCKET']=imu_socket
                imu_command=[str(icm/'build/sprite_icm45686'),'capture','/dev/spidev0.0','/dev/'+chip,offset,'800',str(args.duration+30),str(out/'imu.csv'),'X_DOWN_Z_BACK','CONFIGURE_ICM45686_RAW_NO_CAN']
            imu=launch(imu_command,'imu')
            wait_socket(imu_socket,imu)
            # Read-only adapter validation before any CAN transmission.
            if candidate or icm:
                print('STATIC_CALIBRATION: keep robot still for15s; CAN has not started',flush=True)
                with socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM) as client:
                    client.bind(str(Path(sockets)/'preflight.sock'));client.settimeout(.1)
                    deadline=time.monotonic()+22
                    while True:
                        if imu.poll() is not None or time.monotonic()>deadline:raise RuntimeError('software IMU startup failed')
                        client.sendto(b'GETIMU01',imu_socket)
                        try:packet=client.recv(193)
                        except socket.timeout:continue
                        if len(packet)!=192 or packet[:8]!=(b'SPIMU003' if icm else b'SPIMU002'):raise RuntimeError('software packet mismatch')
                        seq,start,end=struct.unpack_from('<QQQ',packet,8)
                        if not seq or not start<=end<=time.monotonic_ns() or time.monotonic_ns()-start>30_000_000:raise RuntimeError('stale software IMU startup')
                        break
                print('SOFTWARE_IMU_READY: starting zero-gain CAN and native actor',flush=True)
            else:
                from open_sprite_runtime.native_bno_source import NativeBnoSource
                with NativeBnoSource(imu_socket) as source:
                    packets=[];deadline=time.monotonic()+1
                    while not packets and time.monotonic()<deadline:
                        source.drain(lambda *v:packets.append(v));time.sleep(.005)
                    if not packets:raise RuntimeError('no validated IMU sample')
            can=launch(['build/native/sprite_can_shadow_quantized_v1','--config',str(out/'motors.tsv'),
                '--duration',str(args.duration),'--cpu','5','--realtime-priority','50','--output',str(out/'can.json'),
                '--ipc-socket',state_socket,'--policy-joint-hash',hex(ordered_name_hash(joints)),
                '--kinematics-config',str(out/'kinematics.tsv'),'--bounded-shadow-preview',
                '--acknowledge-hardware-tx','ZERO_GAIN_NATIVE_SHADOW','--all-motors-disabled-confirmed'],'can')
            wait_socket(state_socket,can)
            actor_binary=str(candidate/'build/imu_software_candidate/sprite_policy_native_software_shadow') if candidate else 'build/policy_native_stage2/sprite_policy_native_shadow'
            if icm:actor_binary=str(icm/'build/sprite_policy_native_icm_shadow')
            actor=launch(['taskset','-c','4',actor_binary,str(model),
                str(fixture),str(out/'live.bin'),state_socket,imu_socket,str(names)]
                +([args.command_profile] if args.command_profile else []),'actor')
            deadline=time.monotonic()+args.duration+10
            while actor.poll() is None or can.poll() is None:
                for name,proc in [('actor',actor),('can',can),('imu',imu)]:
                    rc=proc.poll()
                    if rc is not None and rc!=0:raise RuntimeError(f'{name} exited with code {rc}; see {name}.log')
                if time.monotonic()>deadline:raise RuntimeError('actor/CAN completion timeout')
                time.sleep(.05)
            actor_rc=actor.returncode;can_rc=can.returncode
            if actor_rc or can_rc:raise RuntimeError(f'actor={actor_rc}, can={can_rc}')
            imu_rc=imu.wait(timeout=35)
        except Exception as exc:
            failure=f'{type(exc).__name__}: {exc}'
        finally:
            for proc in reversed(processes):
                if proc.poll() is None:proc.terminate()
            for proc in processes:
                try:proc.wait(timeout=3)
                except subprocess.TimeoutExpired:proc.kill();proc.wait()
            for handle in handles:handle.close()
        if failure:
            result=dict(passed=False,error=failure,
                        process_exit_codes=[proc.returncode for proc in processes],
                        logs={n:(out/(n+'.log')).read_text()[-4000:] for n in ('actor','can','imu') if (out/(n+'.log')).exists()})
            (out/'report.json').write_text(json.dumps(result,indent=2)+'\n')
            print(json.dumps(result,indent=2),flush=True)
            raise SystemExit('native shadow failed; primary error and logs preserved')
    imu_report_path=out/('imu.csv.json' if icm else 'imu.json')
    if actor_rc or can_rc or imu_rc or not (out/'can.json').exists() or not imu_report_path.exists():
        result=dict(passed=False,actor_exit=actor_rc,can_exit=can_rc,imu_exit=imu_rc,
                    logs={n:(out/(n+'.log')).read_text()[-4000:] for n in ('actor','can','imu')})
        (out/'report.json').write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps(result,indent=2),flush=True)
        raise SystemExit('native hardware shadow failed; original logs preserved')
    can_report=json.loads((out/'can.json').read_text());imu_report=json.loads(imu_report_path.read_text())
    imu_passed=(imu_report['completed'] and imu_report['max_sample_gap_ns']<30_000_000) if icm else imu_report['passed']
    timing=np.genfromtxt(out/'live.bin.timing',delimiter=',',names=True)
    n=timing.size
    consumed=out/'consumed.bin';consumed.write_bytes(header+(out/'live.bin.inputs').read_bytes())
    parity=False
    if actor_rc==0 and n==frames:
        with (out/'offline.log').open('w') as log:
            check=subprocess.run(['build/policy_native_stage2/sprite_policy_replay',str(model),str(consumed),str(out/'offline.bin')],stdout=log,stderr=subprocess.STDOUT)
        parity=check.returncode==0 and (out/'offline.bin').read_bytes()==(out/'live.bin').read_bytes()
    passed=actor_rc==can_rc==imu_rc==0 and can_report['passed'] and imu_passed and parity
    result=dict(mode='native_policy_real_feedback_zero_gain_shadow',passed=bool(passed),frames=int(n),
        command_profile=args.command_profile,
        actor_exit=actor_rc,can_exit=can_rc,imu_exit=imu_rc,offline_byte_exact=parity,
        nonzero_can_tx_attempts=can_report['nonzero_gain_or_torque_tx_attempts'],
        enable_attempts=can_report['automatic_enable_attempts'],
        maximum_ms={key:float(np.max(timing[key])) for key in timing.dtype.names})
    if result['nonzero_can_tx_attempts'] or result['enable_attempts']:result['passed']=False
    (out/'report.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2),flush=True)
    if not result['passed']:raise SystemExit('native hardware shadow not qualified')


if __name__=='__main__':main()
