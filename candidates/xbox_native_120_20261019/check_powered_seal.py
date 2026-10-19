"""Check/seal the120s candidate after offline and physical button-only checks."""
import csv
import hashlib
import json
from pathlib import Path
import subprocess
import sys

R=Path('/home/tony/open_sprite_runtime');C=R/'candidates/xbox_native_120_20261019'
buttons=json.loads((C/'buttons_verified.json').read_text())
if (buttons.get('passed') is not True or buttons.get('native_combo_passed') is not True
    or buttons.get('hardware_tx')!=0 or buttons['lb']==buttons['rb']
    or not all(isinstance(buttons[k],int) and 0<=buttons[k]<32 for k in ('lb','rb'))):
    raise SystemExit('Physical LB/RB verification required; no motor control allowed')
faults=('none','imu_freeze','state_disconnect','operator_disable')
for fault in faults:
    report=json.loads((C/'qualification'/fault/'report.json').read_text())
    if report.get('passed') is not True or report.get('hardware_tx')!=0:
        raise SystemExit('Offline qualification failed: '+fault)
if sys.argv[1:]==['--seal']:
    subprocess.run([str(C/'gamepad_test')],check=True)
    subprocess.run([str(C/'test_options')],check=True)
    rows=list(csv.DictReader((C/'qualification/none/live.bin.gamepad.csv').open()))
    assert len(rows)==6001 and max(float(r['vx']) for r in rows)>.099
    assert all(0<=float(r['vx'])<=.100001 and float(r['wz'])==0 for r in rows)
    assert all(float(r['vx'])==0 for r in rows if float(r['elapsed_s'])>=112)
    refusal=subprocess.run([str(C/'sprite_policy_xbox_powered')],capture_output=True,text=True)
    assert refusal.returncode!=0 and 'explicit forward120 admission' in refusal.stderr
    files=[C/x for x in ['sprite_policy_xbox_powered','sprite_can_xbox120','powered_adapter.py','check_completion.py',
        'check_powered_seal.py','admission.sh','buttons_verified.json','mapping_verified.json','libonnxruntime.so.1',
        'sprite_policy_xbox_fake','sprite_policy_xbox_fake_stop','gamepad_disable_probe','gamepad_test','test_options']]
    files+=list((C/'src').glob('*'))
    files+=[R/'run_sprite0825_xbox_native_forward120_on_253.sh',R/'run_sprite0825_xbox_native_forward120_inner.sh']
    files+=[C/'qualification'/f/'report.json' for f in faults]
    (C/'powered_seal.json').write_text(json.dumps({str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in files if p.is_file()},indent=2)+'\n')
elif sys.argv[1:]:raise SystemExit('Usage: [--seal]')
for name,digest in json.loads((C/'powered_seal.json').read_text()).items():
    if hashlib.sha256(Path(name).read_bytes()).hexdigest()!=digest:raise SystemExit('Candidate changed: '+name)
print('XBOX120_SEAL_OK; no hardware access; physical powered120s test still requires onsite approval')
