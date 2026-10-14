"""Build a bundle and reject tampering without connecting any IPC socket."""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
from native_walk40_adapter import ROOT,REFERENCE,validate,parser

os.environ['SPRITE_NATIVE_IMU_PROTOCOL']='software-v2'
ref=json.loads(REFERENCE.read_text())
proof=json.loads((ROOT/'reports/software_imu_walk_native_parity_131201/report.json').read_text())
# The paths below select the same already-reviewed baseline; hashes are checked by the adapter.
contract=Path('/home/tony/sprite_runtime/sprite0825_stage2_g74_model3000_sim2real_candidate_kp130_kd5_20261010_arms7_kd3_20261012_follow_v1_waist50_hiproll150_waist70_kd3_kd35_waist60kd3/deploy/contract.json')
with tempfile.TemporaryDirectory(prefix='native_bundle_check_') as temp:
    args=['--socket','/nonexistent/fake.sock','--native-imu-socket','/nonexistent/imu.sock',
          '--imu-source','native-bno055','--contract',str(contract),
          '--hardware-config',str(ROOT/'config/hardware.sprite0825.kp110_kd5.json'),
          '--joint-limit-candidates',str(ROOT/'artifacts/g60_model3450/reports/sprite0825_urdf_limit_candidates.json'),
          '--output',temp+'/actor.json','--trace-output',temp+'/trace.npz',
          '--command-profile','stand-walk010-stop-40s','--gain-scale',str(ref['protected_target_projection']['gain_scale']),
          '--physical-startup-hold-seconds','1','--physical-startup-ramp-seconds','5']
    for field,flag in [('joint_gain_multiplier_overrides','--joint-gain-multiplier'),
                       ('joint_damping_multiplier_overrides','--joint-damping-multiplier')]:
        for name,value in ref['protected_target_projection'][field].items():
            args.extend([flag,f'{name}={value}'])
    validate(parser().parse_args(args))
    bad=parser().parse_args(args);bad.gain_scale+=.1
    try:validate(bad)
    except ValueError:pass
    else:raise AssertionError('gain mismatch accepted')
    subprocess.run([sys.executable,str(ROOT/'tools/native_walk40_adapter.py'),'--prepare-only',*args],check=True)
    bundle=Path(temp+'/actor.json.native')
    (bundle/'input.bin').write_bytes(b'tampered')
    result=subprocess.run([sys.executable,str(ROOT/'tools/native_walk40_adapter.py'),*args],capture_output=True,text=True)
    assert result.returncode!=0 and 'frozen bundle changed' in result.stderr, result.stderr
print('PASS bundle preparation, gain mismatch and tamper rejection; hardware_tx=0')
