"""Read-only integrity gate. Does not open hardware devices."""
import hashlib
import json
from pathlib import Path

C=Path('/home/tony/open_sprite_runtime/candidates/xbox_native_20261019')
seal=json.loads((C/'powered_seal.json').read_text())
for name,expected in seal.items():
    if hashlib.sha256(Path(name).read_bytes()).hexdigest()!=expected:
        raise SystemExit('Xbox candidate changed after qualification: '+name)
for fault in ('none','imu_freeze','state_disconnect'):
    r=json.loads((C/'qualification'/fault/'report.json').read_text())
    if not r.get('passed') or r.get('hardware_tx')!=0:
        raise SystemExit('Xbox fake-peer qualification failed: '+fault)
print('XBOX_FORWARD_ONLY_SEAL_OK; no hardware access')
