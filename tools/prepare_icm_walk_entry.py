"""Generate isolated ICM entry from sealed BNO baseline. Never runs hardware."""
from pathlib import Path
import hashlib
import json
import shutil
import subprocess

root=Path(__file__).resolve().parents[1]
c=root/'candidates/icm_native_20261016'
b=root/'candidates/upper_body_pd_wrist7_20261014'
q=c/'offline_bVVn81XH'
def digest(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def once(text,old,new):
    if text.count(old)!=1:raise ValueError('marker count: '+old[:100])
    return text.replace(old,new)
seal=json.loads((b/'qualification/seal.json').read_text())
if any(digest(p)!=v for p,v in seal.items()):raise ValueError('baseline seal changed')
for fault in ('none','imu_freeze','state_disconnect'):
    report=json.loads((q/fault/'report.json').read_text())
    if not report['passed'] or report['hardware_tx']!=0:raise ValueError('offline qualification failed')
# Keep the existing BNO package untouched. Only the candidate searches this copy.
dest=c/'python/open_sprite_runtime'
shutil.copytree(root/'src/open_sprite_runtime',dest,dirs_exist_ok=True,ignore=shutil.ignore_patterns('__pycache__','*.pyc'))
p=dest/'native_imu_packet.py'
text=p.read_text()
text=once(text,'def decode_software_packet(payload, now_ns, previous_sequence=0):','def decode_software_packet(payload, now_ns, previous_sequence=0, icm=False):')
text=once(text,"payload[:8]!=b'SPIMU002'","payload[:8]!=(b'SPIMU003' if icm else b'SPIMU002')")
text=once(text,'payload[50:56]!=bytes(6)','payload[44 if icm else 50:56]!=bytes(12 if icm else 6)')
text=once(text,"accel=tuple(x/100 for x in struct.unpack_from('<3h',payload,32))","accel=tuple(x*(4*9.80665/32768 if icm else .01) for x in struct.unpack_from('<3h',payload,32))")
text=once(text,"gyro=tuple(x/900-b for x,b in zip(struct.unpack_from('<3h',payload,44),bias))","gyro=tuple(x*(2000*math.pi/180/32768 if icm else 1/900)-b for x,b in zip(struct.unpack_from('<3h',payload,38 if icm else 44),bias))")
text+='\n\ndef decode_icm_packet(payload, now_ns, previous_sequence=0):\n    return decode_software_packet(payload, now_ns, previous_sequence, icm=True)\n'
p.write_text(text)
p=dest/'native_bno_source.py';text=p.read_text()
text=once(text,'import decode_packet, decode_software_packet','import decode_packet, decode_software_packet, decode_icm_packet')
text=text.replace("('hardware-v1','software-v2')","('hardware-v1','software-v2','icm-v3')")
text=once(text,"decoder=decode_software_packet if self.protocol=='software-v2' else decode_packet","decoder=decode_icm_packet if self.protocol=='icm-v3' else decode_software_packet if self.protocol=='software-v2' else decode_packet")
text=once(text,"source='native-bno055'","source='native-icm45686' if self.protocol=='icm-v3' else 'native-bno055'")
text=once(text,"timestamp_basis='native host read start, not sensor capture time'","timestamp_basis='GPIO DRDY edge, not intrinsic sensor/filter delay' if self.protocol=='icm-v3' else 'native host read start, not sensor capture time'")
p.write_text(text)
adapter=(root/'tools/upper_body_native_walk40_adapter.py').read_text()
adapter=once(adapter,"BINARY = ROOT/'build/policy_native_stage2/sprite_policy_powered_candidate'",f'BINARY = Path({str(c/"build/sprite_policy_powered_icm_candidate")!r})')
adapter=adapter.replace("!= 'software-v2'","!= 'icm-v3'").replace('software V2 IMU required','ICM V3 IMU required')
adapter=once(adapter,'def validate(args):','def validate(args):\n    check=json.loads(Path('+repr(str(c/'entry_seal.json'))+').read_text())\n    if any(digest(p)!=v for p,v in check.items()):raise ValueError("ICM entry seal changed")')
adapter_path=root/'tools/icm_native_walk40_adapter.py';adapter_path.write_text(adapter)
shadow=(b/'shadow.sh').read_text().replace('$ROOT/src',str(c/'python'))
(c/'shadow.sh').write_text(shadow)
entry=(root/'run_sprite0825_upper_body_pd_native_admission.sh').read_text()
entry=entry.replace('ENABLE_UPPER_BODY_PD_WALK40','ENABLE_ICM45686_WALK40')
entry=entry.replace('native BNO raw-six-axis software-v2','native ICM raw-six-axis icm-v3')
entry=entry.replace('$ROOT/src',str(c/'python'))
entry=entry.replace('$ROOT/tools/upper_body_native_walk40_adapter.py','$ROOT/tools/icm_native_walk40_adapter.py')
entry=entry.replace(str(b/'shadow.sh'),str(c/'shadow.sh'))
entry=entry.replace('STAMP="upperpd_', 'STAMP="icm_')
entry_path=root/'run_sprite0825_icm_native_admission.sh';entry_path.write_text(entry)
wrapper=(root/'run_sprite0825_upper_body_pd_native_walk40_on_253.sh').read_text()
wrapper=wrapper.replace('ENABLE_UPPER_BODY_PD_WALK40','ENABLE_ICM45686_WALK40')
wrapper=wrapper.replace('run_sprite0825_upper_body_pd_native_admission.sh','run_sprite0825_icm_native_admission.sh')
wrapper=once(wrapper,'producer="$CANDIDATE/build/imu_software_candidate/sprite_imu_software"',f'producer="{c}/build/sprite_icm45686"')
wrapper=wrapper.replace('software-v2','icm-v3').replace('decode_software_packet','decode_icm_packet')
wrapper=wrapper.replace('PYTHONPATH=src',f'PYTHONPATH="{c}/python"')
wrapper=wrapper.replace('native BNO200Hz','native ICM800Hz').replace('reports/upper_body_imu_walk40_','reports/icm_imu_walk40_')
wrapper=wrapper.replace("[s]prite_imu_|", "[s]prite_icm45686|[s]prite_imu_|")
old='"$producer" --record 180 "$sockdir/imu.sock" "$run/imu.json" CONFIGURE_RAW_P0_NO_CAN 200 burst18 >"$run/imu.log" 2>&1 &'
new='read -r chip offset < <(gpiofind PY.01)\nICM_IPC_SOCKET="$sockdir/imu.sock" "$producer" capture /dev/spidev0.0 "/dev/$chip" "$offset" 800 180 "$run/imu.csv" X_DOWN_Z_BACK CONFIGURE_ICM45686_RAW_NO_CAN >"$run/imu.log" 2>&1 &'
wrapper=once(wrapper,old,new).replace("grep -q '^SOFTWARE_IMU_READY'", "grep -q '^BIAS_READY'")
check='''"$ROOT/.venv/bin/python" - <<'PYICM'
import hashlib,json
from pathlib import Path
p=Path('/home/tony/open_sprite_runtime/candidates/icm_native_20261016/entry_seal.json')
if any(hashlib.sha256(Path(k).read_bytes()).hexdigest()!=v for k,v in json.loads(p.read_text()).items()):
    raise SystemExit('ICM entry changed since offline qualification')
PYICM
'''
wrapper=once(wrapper,'if [[ "$1" == --check ]]; then',check+'if [[ "$1" == --check ]]; then')
wrapper=once(wrapper,"run=$(mktemp -d",'[[ "$(cat /sys/bus/platform/devices/3210000.spi/power/control)" == on ]] || { echo "Use PM wrapper"; exit 1; }\nrun=$(mktemp -d')
wrapper_path=root/'run_sprite0825_icm_native_walk40_inner.sh';wrapper_path.write_text(wrapper)
for p in (c/'shadow.sh',entry_path,wrapper_path):subprocess.run(['bash','-n',str(p)],check=True)
paths=[adapter_path,entry_path,wrapper_path,c/'shadow.sh',c/'build/sprite_icm45686',c/'build/sprite_policy_powered_icm_candidate']
paths+=list(dest.rglob('*.py'))
paths += [q/f/'report.json' for f in ('none','imu_freeze','state_disconnect')]
(c/'entry_seal.json').write_text(json.dumps({str(p):digest(p) for p in paths},indent=2)+'\n')
print('GENERATED ICM ENTRY; no hardware accessed; powered test not authorized')
