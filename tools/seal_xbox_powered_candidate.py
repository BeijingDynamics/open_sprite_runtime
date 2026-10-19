"""Seal a built candidate after fake-peer qualification; no hardware access."""
import csv
import hashlib
import json
from pathlib import Path
import subprocess

R=Path('/home/tony/open_sprite_runtime')
C=R/'candidates/xbox_native_20261019'
for fault in ('none','imu_freeze','state_disconnect'):
    report=json.loads((C/'qualification'/fault/'report.json').read_text())
    assert report['passed'] and report['hardware_tx']==0
rows=list(csv.DictReader((C/'qualification/none/live.bin.gamepad.csv').open()))
assert len(rows)==2001
assert max(float(r['vx']) for r in rows)>.099
assert all(0<=float(r['vx'])<=.100001 and float(r['wz'])==0 for r in rows)
assert any(float(r['axis1'])>.9 and float(r['target_vx'])==0 for r in rows)
for a,b in [(0,9),(26.5,27.5),(32,40.1)]:
    selected=[r for r in rows if a<=float(r['elapsed_s'])<b]
    assert selected and all(float(r['vx'])==float(r['wz'])==0 for r in selected),(a,b)
# Without the explicit acknowledgement the real binary must refuse to start.
refusal=subprocess.run([str(C/'sprite_policy_xbox_powered')],capture_output=True,text=True)
assert refusal.returncode!=0 and 'explicit forward40 admission' in refusal.stderr
files=[C/'sprite_policy_xbox_powered',C/'sprite_policy_xbox_fake',C/'powered_adapter.py',
       C/'mapping_verified.json',C/'check_powered_seal.py',C/'libonnxruntime.so.1',
       R/'run_sprite0825_xbox_native_forward40_on_253.sh',
       R/'run_sprite0825_xbox_native_forward40_inner.sh',R/'run_sprite0825_xbox_native_admission.sh']
files+=list((C/'src').glob('*.hpp'))+[C/'src/sprite_policy_replay.cpp']
files += [C/'qualification'/fault/'report.json' for fault in ('none','imu_freeze','state_disconnect')]
seal={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
(C/'powered_seal.json').write_text(json.dumps(seal,indent=2)+'\n')
print('SEALED: fake integration, command bounds, reconnect latch, stop, explicit acknowledgement. Hardware TX=0.')
