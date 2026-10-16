"""Build isolated ZERO_GAIN_NATIVE_SHADOW entry; no hardware access."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

root=Path(__file__).resolve().parents[1]
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--candidate',type=Path,default=root/'candidates/upper_body_pd_20261014')
candidate=parser.parse_args().candidate.resolve()
report=json.loads((candidate/'report.json').read_text())
for name, expected in report['output_sha256'].items():
    if hashlib.sha256((candidate/name).read_bytes()).hexdigest()!=expected:
        raise ValueError('candidate table changed: '+name)
source=Path(next(p for p in report['source_sha256'] if p.endswith('contract.json')))
if hashlib.sha256(source.read_bytes()).hexdigest()!=report['source_sha256'][str(source)]:
    raise ValueError('source contract changed')
contract=json.loads(source.read_text())
with (candidate/'joint_safety.tsv').open() as f:
    rows=list(csv.DictReader(f,delimiter='\t'))
if [r['joint_name'] for r in rows]!=contract['joint_names']:
    raise ValueError('joint order mismatch')
contract['stiffness']=[float(r['kp_max']) for r in rows]
contract['damping']=[float(r['kd_max']) for r in rows]
contract['effort_limit']=[float(r['feedforward_torque_max_nm']) for r in rows]
contract['label']=candidate.name+'_shadow_effective_gains'
deploy=candidate/'deploy';deploy.mkdir(exist_ok=True)
for key in ('policy_onnx','policy_jit'):
    model=source.parent/contract[key]
    if hashlib.sha256(model.read_bytes()).hexdigest()!=contract[key+'_sha256']:
        raise ValueError('source model hash mismatch')
    shutil.copy2(model,deploy/model.name);contract[key]=model.name
(deploy/'contract.json').write_text(json.dumps(contract,indent=2)+'\n')
text=(root/'probe_sprite0825_bno055_policy_shadow_on_253.sh').read_text()
lines=text.splitlines()
lines=[f'CANDIDATE={candidate}' if x.startswith('CANDIDATE=') else x for x in lines]
text='\n'.join(lines)+'\n'
text=text.replace('$ROOT/build/native/kinematics.tsv',str(candidate/'kinematics.tsv'))
text=text.replace('$ROOT/build/native/joint_safety.tsv',str(candidate/'shadow_joint_safety.tsv'))
text=text.replace('same waist60/3 and hiproll150/5 candidate','new upper-body effective PD candidate')
text=text.replace('# Arms use shoulder pitch/roll 20/0.5; shoulder yaw/elbow 7/0.45 directly; waist, wrists, head and torque caps unchanged.',
                  '# Explicit effective PD; all gain multipliers must be one. No powered control.')
text=text.replace('STAMP="$(date +%Y%m%d_%H%M%S)"','STAMP="upperpd_$(date +%Y%m%d_%H%M%S)"')
entry=candidate/'shadow.sh';entry.write_text(text)
wrapper=(root/'run_sprite0825_full_native_walk40_candidate_on_253.sh').read_text()
wrapper=wrapper.replace('ENABLE_NATIVE_ACTOR_WALK40','ZERO_GAIN_UPPER_BODY_CHECK')
wrapper=wrapper.replace('baseline=run_sprite0825_native_actor_admission_candidate.sh',f'baseline={entry}')
wrapper=wrapper.replace('Candidate: native200Hz IMU + native50Hz ONNX; existing protected CAN controller.',
                        'SHADOW ONLY: native200Hz BNO + Python50Hz ONNX reference + native zero-gain CAN. No enable.')
wrapper=wrapper.replace('reports/software_imu_walk40_','reports/upper_body_imu_shadow_')
precheck='''"$ROOT/.venv/bin/python" - <<'PYCAN'
import json, subprocess
links=json.loads(subprocess.check_output(['ip','-details','-json','link','show']))
by_name={x['ifname']:x for x in links}
for name in ('kcan1','kcan2','kcan3','kcan4'):
    item=by_name.get(name,{})
    data=item.get('linkinfo',{}).get('info_data',{})
    if ('UP' not in item.get('flags',[]) or item.get('mtu')!=72
        or data.get('bittiming',{}).get('bitrate')!=1000000
        or data.get('data_bittiming',{}).get('bitrate')!=5000000):
        raise SystemExit(name+': require UP, CAN-FD MTU72, 1M/5M. Run setup_sprite0825_kcanfd_on_253.sh')
print('CAN configuration checked; no CAN transmission')
PYCAN
'''
wrapper=wrapper.replace("echo 'Requires fall arrest",precheck+"echo 'Requires fall arrest")
wrapper=wrapper.replace('bash "$baseline" ENABLE_BNO055_WAIST60_KD3_WALK40 &',
    f'SPRITE_PREPARED_MOTOR_CONFIG="{candidate}/motors.tsv" SPRITE_BOUNDED_SHADOW_PREVIEW=1 bash "$baseline" 10 1 2 4 1 0 1 1 1 1 1 1 1 &')
path=root/'probe_sprite0825_upper_body_pd_bno_shadow_on_253.sh';path.write_text(wrapper)
for p in (entry,path):subprocess.run(['bash','-n',str(p)],check=True)
if '--policy-actuation' in text or 'ENABLE_NATIVE_PROTECTED_POLICY_ACTUATION' in text:
    raise ValueError('powered invocation forbidden')
if '--acknowledge-hardware-tx ZERO_GAIN_NATIVE_SHADOW' not in text.replace('\\\n  ',' '):
    raise ValueError('zero gain acknowledgement missing')
print('PREPARED',path,'NO HARDWARE ACCESS')
