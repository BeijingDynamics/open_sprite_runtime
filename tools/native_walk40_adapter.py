"""Prepare/launch a fixed qualified native actor; Python never performs inference.

Only a generated admission script should call this. It does not enable motors;
the existing protected CAN controller owns all hardware actions and shutdown.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import struct
import subprocess
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
PROOF = ROOT/'reports/software_imu_walk_native_parity_131201'
REFERENCE = ROOT/'reports/native_protected_policy_actor_20261014_131201.json'
BINARY = ROOT/'build/policy_native_stage2/sprite_policy_powered_candidate'


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def parser():
    p = argparse.ArgumentParser()
    p.add_argument('--prepare-only', action='store_true')
    for name in ('socket','hardware-config','contract','joint-limit-candidates','output','trace-output','native-imu-socket'):
        p.add_argument('--'+name, required=True)
    p.add_argument('--imu-source', choices=['native-bno055'], required=True)
    p.add_argument('--imu-device')
    p.add_argument('--imu-baud', type=int, default=115200)
    p.add_argument('--command-profile', choices=['stand-walk010-stop-40s'], required=True)
    for name in ('vx','vy','yaw-rate'):
        p.add_argument('--'+name, type=float, default=0)
    for name in ('gain-scale','physical-startup-hold-seconds','physical-startup-ramp-seconds'):
        p.add_argument('--'+name, type=float, required=True)
    for name in ('joint-gain-multiplier','joint-damping-multiplier'):
        p.add_argument('--'+name, action='append', default=[])
    return p


def multipliers(items):
    result = {}
    for item in items:
        name, value = item.split('=')
        if name in result:
            raise ValueError('duplicate multiplier '+name)
        result[name] = float(value)
    return result


def validate(args):
    if os.environ.get('SPRITE_NATIVE_IMU_PROTOCOL') != 'software-v2':
        raise ValueError('software V2 IMU required')
    proof = json.loads((PROOF/'report.json').read_text())
    if proof.get('passed') is not True:
        raise ValueError('reference parity not qualified')
    for key, path in [('contract',args.contract), ('hardware',args.hardware_config),
                      ('limits',args.joint_limit_candidates), ('actor-report',REFERENCE)]:
        if digest(path) != proof['source_sha256'][key]:
            raise ValueError('changed qualified input: '+key)
    contract = json.loads(Path(args.contract).read_text())
    model = Path(args.contract).parent/contract['policy_onnx']
    if digest(model) != proof['model_sha256']:
        raise ValueError('model changed')
    ref = json.loads(REFERENCE.read_text())
    cfg = ref['protected_target_projection']
    ramp = ref['physical_startup_ramp']
    if (args.vx,args.vy,args.yaw_rate) != (0,0,0):
        raise ValueError('only qualified scheduled commands supported')
    for actual, expected in [
        (args.gain_scale,cfg['gain_scale']),
        (multipliers(args.joint_gain_multiplier),cfg['joint_gain_multiplier_overrides']),
        (multipliers(args.joint_damping_multiplier),cfg['joint_damping_multiplier_overrides']),
        (args.physical_startup_hold_seconds,ramp['hold_seconds']),
        (args.physical_startup_ramp_seconds,ramp['ramp_seconds'])]:
        if actual != expected:
            raise ValueError('actor settings differ from visually accepted baseline')
    return model, contract


def main():
    args = parser().parse_args()
    model, contract = validate(args)
    bundle = Path(args.output+'.native')
    if args.prepare_only:
        bundle.mkdir(parents=True, exist_ok=False)
        data = (PROOF/'input.bin').read_bytes()
        count = struct.unpack_from('<I',data,8)[0]
        if count != 2001 or data[:8] != b'SPCHN001':
            raise ValueError('unexpected qualified fixture')
        # Sensor fields are replaced by live IPC before inference, never replayed.
        header = data[:-count*72*8]
        (bundle/'input.bin').write_bytes(header+bytes(count*72*8))
        trace = np.load(ROOT/'reports/native_protected_policy_trace_20261014_131201.npz',allow_pickle=False)
        motors = [str(n) for n in trace['motor_names']]
        joints = contract['joint_names']
        if len(motors)!=31 or len(set(motors))!=31 or list(trace['joint_names'])!=joints:
            raise ValueError('reference joint order mismatch')
        (bundle/'names.txt').write_text('SPRITE_POWERED_WALK40_V1\n'+'\n'.join(motors+joints)+'\n')
        files = [BINARY,model,Path(args.contract),Path(args.hardware_config),Path(args.joint_limit_candidates),
                 bundle/'input.bin',bundle/'names.txt',ROOT/'build/native/sprite_can_shadow_quantized_v1']
        for name in ('motors.tsv','kinematics.tsv','joint_safety.tsv'):
            destination=bundle/name
            destination.write_bytes((ROOT/'build/native'/name).read_bytes())
            files.append(destination)
        (bundle/'manifest.json').write_text(json.dumps({str(p.resolve()):digest(p) for p in files},indent=2))
        print('NATIVE_BUNDLE_READY',bundle)
        return
    manifest=json.loads((bundle/'manifest.json').read_text())
    if any(digest(path)!=expected for path,expected in manifest.items()):
        raise ValueError('frozen bundle changed')
    command=[str(BINARY),str(model),str(bundle/'input.bin'),str(bundle/'actor.bin'),
             args.socket,args.native_imu_socket,str(bundle/'names.txt'),args.command_profile,
             'ENABLE_NATIVE_ACTOR_WALK40']
    child=subprocess.Popen(command)
    def stop(signum, _frame):
        child.terminate()
        child.wait(timeout=3)
        raise SystemExit(128+signum)
    signal.signal(signal.SIGTERM,stop)
    signal.signal(signal.SIGINT,stop)
    status=child.wait()
    report={'mode':'native_actor_powered_ipc_candidate','actor_exit':status,
            'controller_report_required':True,'binary_trace':str(bundle/'actor.bin'),
            'passed':status==0}
    Path(args.output).write_text(json.dumps(report,indent=2))
    raise SystemExit(status if status>=0 else 1)


if __name__=='__main__':
    main()
