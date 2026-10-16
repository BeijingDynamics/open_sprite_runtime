"""Audit actual generated shell settings and frozen native bundle; NO hardware."""
import argparse
import csv
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import socket
import struct
import subprocess
import sys
import tempfile
import numpy as np

root=Path(__file__).resolve().parents[1]
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--candidate',type=Path,default=root/'candidates/upper_body_pd_20261014')
c=parser.parse_args().candidate.resolve();q=c/'qualification'
entry=root/'run_sprite0825_upper_body_pd_native_admission.sh'
wrapper=root/'run_sprite0825_upper_body_pd_native_walk40_on_253.sh'
adapter=root/'tools/upper_body_native_walk40_adapter.py'
for case in ('none','imu_freeze','state_disconnect'):
    result=json.loads((q/f'fake_{case}/report.json').read_text())
    if result.get('passed') is not True or result.get('hardware_tx')!=0:raise ValueError('fake-peer failure')
with (c/'joint_safety.tsv').open() as f:joint=list(csv.DictReader(f,delimiter='\t'))
with (c/'motors.tsv').open() as f:motors=list(csv.DictReader(f,delimiter='\t'))
expected=np.array([[float(r['kp_max']) for r in joint],[float(r['kd_max']) for r in joint]])
dtype=np.dtype([('obs','<f4',(795,)),('raw','<f4',(31,)),('handoff','<f8',(31,)),('target','<f8',(31,)),('protected','<f8',(5,31))])
live=np.fromfile(q/'fake_none/live.bin',dtype=dtype)
assert len(live)==2001
np.testing.assert_allclose(live['protected'][300:,2:4],np.broadcast_to(expected,live['protected'][300:,2:4].shape),atol=1e-12,rtol=0)
assert np.max(np.abs(live['protected'][:50,2:4]))==0
consumed=np.fromfile(q/'fake_none/live.bin.inputs',dtype='<f8').reshape(2001,72)
cmd=consumed[:,69:]
assert np.max(np.abs(cmd[:500]))==0
np.testing.assert_allclose(cmd[650:1350,0],.1,atol=1e-10,rtol=0)
assert np.max(np.abs(cmd[1500:]))==0
assert np.max(np.abs(cmd[:,1:]))==0
# Seal validated inputs, test evidence, entry code and exact actor/controller.
paths=[c/'deploy/contract.json',c/'deploy/policy.onnx',c/'joint_safety.tsv',c/'motors.tsv',
       q/'fixture40.bin',q/'reference.json',q/'reference.npz',q/'parity/report.json',
       root/'config/hardware.sprite0825.kp110_kd5.json',
       root/'artifacts/g60_model3450/reports/sprite0825_urdf_limit_candidates.json',
       root/'build/policy_native_stage2/sprite_policy_powered_candidate',
       root/'build/native/sprite_can_shadow_quantized_v1',entry,wrapper,adapter,c/'shadow.sh']
paths += [q/f'fake_{case}/report.json' for case in ('none','imu_freeze','state_disconnect')]
seal={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
(q/'seal.json').write_text(json.dumps(seal,indent=2)+'\n')
text=entry.read_text();prefix=text[:text.index('BOUNDED_SHADOW_PREVIEW=0')]
needle='"$ROOT/tools/upper_body_native_walk40_adapter.py" --prepare-only'
start=text.rfind('OMP_NUM_THREADS=1',0,text.index(needle));end=text.index('\n\n',text.index(needle))
prepare=text[start:end]
with tempfile.TemporaryDirectory(prefix='upperpd_no_hardware_') as tmp:
    with socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM) as fake:
        fake.bind(tmp+'/imu.sock')
        env=dict(os.environ,SPRITE_NATIVE_IMU_SOCKET=tmp+'/imu.sock',SPRITE_NATIVE_IMU_PROTOCOL='software-v2')
        script=Path(tmp)/'settings.sh'
        script.write_text(prefix+f'\nPOLICY_REPORT="{tmp}/actor.json"\nPOLICY_TRACE="{tmp}/trace.npz"\n'+prepare+'\n')
        subprocess.run(['bash',str(script),'ENABLE_UPPER_BODY_PD_WALK40'],env=env,check=True)
        bundle=Path(tmp)/'actor.json.native'
        # Verify the actual adapter bundle, not only the earlier fixture.
        data=(bundle/'input.bin').read_bytes();n=struct.unpack_from('<I',data,8)[0]
        fixture=(q/'fixture40.bin').read_bytes();h=len(fixture)-2001*72*8
        assert n==2001 and data[:h]==fixture[:h] and not any(data[h:])
        for name in ('joint_safety.tsv','motors.tsv'):
            assert (bundle/name).read_bytes()==(c/name).read_bytes()
        assert all(hashlib.sha256(Path(p).read_bytes()).hexdigest()==v for p,v in seal.items())
        # Invoke validation on intentionally wrong gains; rejection precedes IPC.
        spec=importlib.util.spec_from_file_location('upper_adapter',adapter);module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        os.environ['SPRITE_NATIVE_IMU_PROTOCOL']='software-v2'
        args=module.parser().parse_args(['--socket','/missing','--native-imu-socket','/missing','--imu-source','native-bno055',
            '--contract',str(c/'deploy/contract.json'),'--hardware-config',str(root/'config/hardware.sprite0825.kp110_kd5.json'),
            '--joint-limit-candidates',str(root/'artifacts/g60_model3450/reports/sprite0825_urdf_limit_candidates.json'),
            '--output',tmp+'/actor.json','--trace-output',tmp+'/trace.npz','--command-profile','stand-walk010-stop-40s',
            '--gain-scale','1','--physical-startup-hold-seconds','1','--physical-startup-ramp-seconds','5'])
        module.validate(args)
        args.joint_gain_multiplier=['waist_yaw_joint=0.125']
        try:module.validate(args)
        except ValueError:pass
        else:raise AssertionError('old multiplier accepted')
result={'hardware_tx':0,'passed':True,'joint_effective_gain_count':31,'full_native_fake_frames':2001,
        'startup_hold_seconds':1,'startup_ramp_seconds':5,'command_profile':'stand-walk010-stop-40s',
        'old_multiplier_rejected':True,'actual_shell_bundle_matches_fixture':True,
        'powered_test_performed':False}
(q/'entry_checks.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result,indent=2))
