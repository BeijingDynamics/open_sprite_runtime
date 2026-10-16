"""Exercise generated shell parameter preparation; no CAN/SPI/I2C access."""
from pathlib import Path
import os,json,struct,subprocess,tempfile,socket
root=Path(__file__).resolve().parents[1]
c=root/'candidates/icm_native_20261016'
b=root/'candidates/upper_body_pd_wrist7_20261014'
text=(root/'run_sprite0825_icm_native_admission.sh').read_text()
prefix=text[:text.index('BOUNDED_SHADOW_PREVIEW=0')]
# The prefix only exports tables; no hardware execution occurs before the marker.
needle='"$ROOT/tools/icm_native_walk40_adapter.py" --prepare-only'
start=text.rfind('OMP_NUM_THREADS=1',0,text.index(needle));end=text.index('\n\n',text.index(needle))
prepare=text[start:end]
with tempfile.TemporaryDirectory(prefix='icm_entry_offline_') as tmp:
    with socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM) as fake:
        fake.bind(tmp+'/imu.sock')
        env=dict(os.environ,SPRITE_NATIVE_IMU_SOCKET=tmp+'/imu.sock',SPRITE_NATIVE_IMU_PROTOCOL='icm-v3')
        p=Path(tmp)/'prepare.sh'
        p.write_text(prefix+f'\nPOLICY_REPORT="{tmp}/actor.json"\nPOLICY_TRACE="{tmp}/trace.npz"\n'+prepare+'\n')
        subprocess.run(['bash',str(p),'ENABLE_ICM45686_WALK40'],env=env,check=True)
        bundle=Path(tmp)/'actor.json.native'
        data=(bundle/'input.bin').read_bytes();baseline=(b/'qualification/fixture40.bin').read_bytes()
        n=struct.unpack_from('<I',data,8)[0];h=len(baseline)-2001*72*8
        if n!=2001 or data[:h]!=baseline[:h] or any(data[h:]):raise AssertionError('fixture changed')
        for name in ('motors.tsv','kinematics.tsv','joint_safety.tsv'):
            if (bundle/name).read_bytes()!=(b/name).read_bytes():raise AssertionError(name+' differs')
        manifest=json.loads((bundle/'manifest.json').read_text())
        if str(c/'build/sprite_policy_powered_icm_candidate') not in manifest:raise AssertionError('wrong actor')
subprocess.run(['bash',str(root/'run_sprite0825_icm_native_walk40_on_253.sh'),'--check'],check=True)
report=dict(passed=True,hardware_tx=0,actual_entry_bundle_matches_baseline=True,
            powered_execution=False,frames=2001,startup_hold_s=1,startup_ramp_s=5)
(c/'entry_checks.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
