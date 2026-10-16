"""Offline rebuild of upper-body PD native fixture; never opens CAN/I2C."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import struct
import subprocess
import sys
import numpy as np
from open_sprite_runtime.physical_startup import PhysicalStartupRamp
from open_sprite_runtime.multirate_control import JointImpedanceTarget

root=Path(__file__).resolve().parents[1]
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--candidate',type=Path,default=root/'candidates/upper_body_pd_20261014')
parser.add_argument('--derive-effective-gains',action='store_true')
args=parser.parse_args()
c=args.candidate.resolve()
out=c/'qualification';out.mkdir(exist_ok=False)
source_report=root/'reports/native_policy_ipc_actor_upperpd_20261014_175424.json'
source_trace=root/'reports/native_policy_ipc_trace_upperpd_20261014_175424.npz'
report=json.loads(source_report.read_text())
if not report['passed']:raise ValueError('zero-gain source did not pass')
with np.load(source_trace,allow_pickle=False) as d:trace={k:d[k].copy() for k in d.files}
if args.derive_effective_gains:
    contract=json.loads((c/'deploy/contract.json').read_text())
    cfg=report['protected_target_projection']
    if cfg['gain_scale']!=1 or cfg['joint_gain_multiplier_overrides'] or cfg['joint_damping_multiplier_overrides']:
        raise ValueError('effective gain derivation requires unit multipliers')
    if list(trace['joint_names'])!=contract['joint_names']:
        raise ValueError('joint order mismatch')
    trace['projected_kp'][:]=np.asarray(contract['stiffness'])
    trace['projected_kd'][:]=np.minimum(contract['damping'],cfg['maximum_embedded_kd'])
# Recompute only startup fields for the ORIGINAL powered 1s hold +5s ramp.
# This is explicitly derived offline evidence, not a new hardware recording.
ramp=PhysicalStartupRamp(50,1,5)
keys=['startup_target_position_rad','startup_target_velocity_rad_s','startup_kp','startup_kd','startup_feedforward_torque_nm']
for i in range(len(trace['state_sequence'])):
    target=JointImpedanceTarget(*(trace[k][i] for k in
        ['projected_target_position_rad','projected_target_velocity_rad_s','projected_kp','projected_kd','projected_feedforward_torque_nm']))
    result=ramp.apply(target,trace['joint_position_rad'][i])
    for key, value in zip(keys,[result.position_rad,result.velocity_rad_s,result.kp,result.kd,result.feedforward_torque_nm]):trace[key][i]=value
    trace['startup_alpha'][i]=ramp.last_alpha
report['physical_startup_ramp']=ramp.report()
report['offline_derivation']={'source_report':str(source_report),'source_trace':str(source_trace),
    'hardware_tx':0,'change':'startup1s hold +5s ramp recomputed from recorded projected targets'}
report['offline_derivation']['effective_gains_replaced']=args.derive_effective_gains
report['offline_derivation']['not_new_hardware_evidence']=True
reference=out/'reference.json';reference.write_text(json.dumps(report,indent=2)+'\n')
np.savez(out/'reference.npz',**trace)
subprocess.run([sys.executable,str(root/'tools/verify_native_policy_chain.py'),
    '--contract',str(c/'deploy/contract.json'),'--hardware',str(root/'config/hardware.sprite0825.kp110_kd5.json'),
    '--limits',str(root/'artifacts/g60_model3450/reports/sprite0825_urdf_limit_candidates.json'),
    '--actor-report',str(reference),'--trace',str(out/'reference.npz'),
    '--binary',str(root/'build/policy_native_stage2/sprite_policy_replay'),'--output-dir',str(out/'parity')],check=True)
data=(out/'parity/input.bin').read_bytes();n=struct.unpack_from('<I',data,8)[0]
header=data[:-n*72*8];rows=np.frombuffer(data[-n*72*8:],dtype='<f8').reshape(n,72)
#40s fake-peer pacing only: stationary recorded states, last state held to2001.
# Live runtime replaces every sensor row with actual IPC feedback.
extended=np.concatenate([rows,np.repeat(rows[-1:],2001-n,axis=0)])
header=header[:8]+struct.pack('<I',2001)+header[12:]
(out/'fixture40.bin').write_bytes(header+extended.astype('<f8').tobytes())
with (c/'joint_safety.tsv').open() as f:expected=list(csv.DictReader(f,delimiter='\t'))
for field,key in [('projected_kp','kp_max'),('projected_kd','kd_max')]:
    values=np.array([float(r[key]) for r in expected]);np.testing.assert_allclose(trace[field],np.broadcast_to(values,trace[field].shape),rtol=0,atol=1e-12)
    np.testing.assert_allclose(trace['startup_'+('kp' if key=='kp_max' else 'kd')][-1],values,rtol=0,atol=1e-12)
meta={'hardware_tx':0,'effective_gains_checked':31,'startup_hold_s':1,'startup_ramp_s':5,
      'fixture_rows':2001,'fixture_is_synthetic_pacing_not_walking_evidence':True,
      'source_sha256':{str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in [source_report,source_trace]},
      'passed':True}
(out/'preparation.json').write_text(json.dumps(meta,indent=2)+'\n')
print(json.dumps(meta,indent=2))
