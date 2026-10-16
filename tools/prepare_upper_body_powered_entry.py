"""Generate isolated upper-body entry; generation never runs hardware scripts."""
from pathlib import Path
import argparse
import json
import subprocess

root=Path(__file__).resolve().parents[1]
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--candidate',type=Path,default=root/'candidates/upper_body_pd_20261014')
c=parser.parse_args().candidate.resolve()
q=c/'qualification'
def replace_once(text,old,new):
    if text.count(old)!=1:raise ValueError('expected exactly one marker: '+old[:100])
    return text.replace(old,new)

adapter=(root/'tools/native_walk40_adapter.py').read_text()
adapter=replace_once(adapter,"PROOF = ROOT/'reports/software_imu_walk_native_parity_131201'",f"PROOF = Path({str(q/'parity')!r})")
adapter=replace_once(adapter,"REFERENCE = ROOT/'reports/native_protected_policy_actor_20261014_131201.json'",f"REFERENCE = Path({str(q/'reference.json')!r})")
adapter=replace_once(adapter,"data = (PROOF/'input.bin').read_bytes()",f"data = Path({str(q/'fixture40.bin')!r}).read_bytes()")
adapter=replace_once(adapter,"ROOT/'reports/native_protected_policy_trace_20261014_131201.npz'",f"Path({str(q/'reference.npz')!r})")
adapter=replace_once(adapter,"(ROOT/'build/native'/name).read_bytes()",f"(Path({str(c)!r})/name).read_bytes()")
guard='''    qualification = Path(QUALIFICATION)
    for test in ('fake_none','fake_imu_freeze','fake_state_disconnect'):
        result = json.loads((qualification/test/'report.json').read_text())
        if result.get('passed') is not True or result.get('hardware_tx') != 0:
            raise ValueError('candidate fake-peer test not passed: '+test)
    seal=json.loads((qualification/'seal.json').read_text())
    if any(digest(path)!=value for path,value in seal.items()):
        raise ValueError('candidate qualification seal changed')
'''
guard=guard.replace('QUALIFICATION',repr(str(q)))
adapter=replace_once(adapter,'def validate(args):\n','def validate(args):\n'+guard)
adapter_path=root/'tools/upper_body_native_walk40_adapter.py';adapter_path.write_text(adapter)

text=(root/'run_sprite0825_native_actor_admission_candidate.sh').read_text()
lines=text.splitlines()
contract=json.loads((c/'deploy/contract.json').read_text())
gain_message='; '.join(f"{name}={contract['stiffness'][i]:g}/{contract['damping'][i]:g}" for i,name in enumerate(contract['joint_names']) if 'wrist' in name)
lines=[(f'echo "UPPER BODY PD: waist yaw and shoulder pitch/roll130/5; {gain_message}; other gains unchanged"'
        if line.startswith('echo "WAIST COMPARISON:') else
        'echo "Use native BNO raw-six-axis software-v2 only; new upper-body candidate, not baseline gains"'
        if line.startswith('echo "BNO055 IMUPLUS/P0 comparison:') else line) for line in lines]
text='\n'.join(lines)+'\n'
text=replace_once(text,'[[ "$ACK" == "ENABLE_BNO055_WAIST60_KD3_WALK40" ]]',
                  '[[ "$ACK" == "ENABLE_UPPER_BODY_PD_WALK40" ]]')
text='\n'.join(f'CANDIDATE={c}' if line.startswith('CANDIDATE=') else line for line in text.splitlines())+'\n'
text=replace_once(text,'JOINT_GAIN_ARGS=()',
'''# Contract already contains effective commissioning gains. Never scale twice.
GAIN_SCALE=1.0
DM3507_GAIN_MULTIPLIER=1.0
HEAD_GAIN_MULTIPLIER=1.0
NON_HIP_GAIN_MULTIPLIER=1.0
WAIST_ROLL_GAIN_MULTIPLIER=1.0
LEG_GAIN_MULTIPLIER=1.0
LEG_DAMPING_MULTIPLIER=1.0
ANKLE_GAIN_MULTIPLIER=1.0
HIP_GAIN_MULTIPLIER=1.0
JOINT_GAIN_ARGS=()''')
# Copy the reviewed motor table, not historical cap-generation branches.
start=text.index('PYTHONPATH="$ROOT/src" "$ROOT/.venv/bin/python" \\\n  "$ROOT/tools/export_native_motor_config.py"')
end=text.index('PYTHONPATH="$ROOT/src" "$ROOT/.venv/bin/python" \\\n  "$ROOT/tools/export_native_kinematics_config.py"',start)
text=text[:start]+text[end:]
for filename in ('kinematics.tsv','joint_safety.tsv'):
    text=text.replace('$ROOT/build/native/'+filename,str(c/filename))
text=text.replace('$ROOT/build/native/motors.tsv',str(c/'motors.tsv'))
text=text.replace('$ROOT/tools/native_walk40_adapter.py','$ROOT/tools/upper_body_native_walk40_adapter.py')
text=replace_once(text,'"$ROOT/probe_sprite0825_bno055_policy_shadow_on_253.sh"',f'"{c}/shadow.sh"')
# The old preview checker contains hardcoded3Nm shoulder limits: replace its
# entire body with the exact table consumed by the native controller.
start=text.index('import json\nimport sys\n\nreport = json.load(')
end=text.index('\nPY\nPREFLIGHT_LIMIT_GATE_ARGS',start)
preview=f'''import csv,json,sys
report=json.load(open(sys.argv[1]))
with open({str(c/'motors.tsv')!r}) as f:
    caps={{r['motor_name']:float(r['commissioning_torque_cap_nm']) for r in csv.DictReader(f,delimiter='\\t')}}
values=report['preview_maximum_abs_estimated_torque_nm_by_motor']
if set(values)!=set(caps):raise SystemExit('preview motor set mismatch')
if not report.get('passed'):raise SystemExit('native preflight failed')
bad=[f'{{name}}={{value:.4f}}>{{caps[name]*1.1:.4f}}Nm' for name,value in values.items() if value>caps[name]*1.1]
if bad:raise SystemExit('startup torque preview exceeded: '+', '.join(bad))
print('UPPER_BODY_STARTUP_TORQUE_PASSED; exact candidate caps; no limit increase')
'''
text=text[:start]+preview+text[end:]
text=text.replace('STAMP="$(date +%Y%m%d_%H%M%S)"','STAMP="upperpd_$(date +%Y%m%d_%H%M%S)"')
text=text.replace('echo "ARM COMMAND CAPS shoulder pitch/roll=3Nm; shoulder yaw/elbow=3Nm; wrists unchanged"',
                  'echo "ACTIVE CAPS from candidate table: shoulder pitch/roll and waist yaw28Nm; waist roll30Nm; others unchanged"')
entry=root/'run_sprite0825_upper_body_pd_native_admission.sh';entry.write_text(text)
wrapper=(root/'probe_sprite0825_upper_body_pd_bno_shadow_on_253.sh').read_text()
wrapper=replace_once(wrapper,f'baseline={c}/shadow.sh',f'baseline={entry}')
wrapper=wrapper.replace('ZERO_GAIN_UPPER_BODY_CHECK','ENABLE_UPPER_BODY_PD_WALK40')
wrapper=replace_once(wrapper,f'SPRITE_PREPARED_MOTOR_CONFIG="{c}/motors.tsv" SPRITE_BOUNDED_SHADOW_PREVIEW=1 bash "$baseline" 10 1 2 4 1 0 1 1 1 1 1 1 1 &',
                     'bash "$baseline" ENABLE_UPPER_BODY_PD_WALK40 &')
wrapper=wrapper.replace('SHADOW ONLY: native200Hz BNO + Python50Hz ONNX reference + native zero-gain CAN. No enable.',
                        'POWERED CANDIDATE: native BNO200Hz + native ONNX50Hz + CAN; new upper-body PD. Explicit onsite approval required.')
wrapper=wrapper.replace('reports/upper_body_imu_shadow_','reports/upper_body_imu_walk40_')
static_check=f'''"$ROOT/.venv/bin/python" - <<'PYQUAL'
import hashlib,json
from pathlib import Path
q=Path({str(q)!r})
if json.loads((q/'entry_checks.json').read_text()).get('passed') is not True:
    raise SystemExit('new entry offline checks missing')
seal=json.loads((q/'seal.json').read_text())
if any(hashlib.sha256(Path(p).read_bytes()).hexdigest()!=v for p,v in seal.items()):
    raise SystemExit('new candidate qualification seal changed')
print('NEW_PARAMETER_BUNDLE_QUALIFIED; no hardware access')
PYQUAL
'''
wrapper=replace_once(wrapper,'if [[ "$1" == --check ]]; then',static_check+'if [[ "$1" == --check ]]; then')
wrapper_path=root/'run_sprite0825_upper_body_pd_native_walk40_on_253.sh';wrapper_path.write_text(wrapper)
for p in (entry,wrapper_path):subprocess.run(['bash','-n',str(p)],check=True)
print('GENERATED ONLY; no hardware execution',wrapper_path)
