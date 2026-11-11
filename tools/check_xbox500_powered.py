"""Verify separate experimental provenance. No hardware access or auto-sealing."""
import hashlib
import json
from pathlib import Path
import subprocess

E = Path(__file__).resolve().parents[1]
R = Path('/home/tony/open_sprite_runtime')
C = E/'candidates/xbox_500hz_powered_20261111'


def main():
    seal = json.loads((C/'seal.json').read_text())
    if seal.get('mode') != 'xbox500_prepared_not_hardware_qualified':
        raise SystemExit('Wrong candidate seal')
    for path, expected in seal['sha256'].items():
        if hashlib.sha256(Path(path).read_bytes()).hexdigest() != expected:
            raise SystemExit('Candidate changed: '+path)
    parameter = R/'candidates/upper_body_pd_wrist7_20261014'
    for name in ('motors.tsv','kinematics.tsv','joint_safety.tsv','deploy/contract.json','deploy/policy.onnx'):
        if (parameter/name).read_bytes() != (C/'parameters'/name).read_bytes():
            raise SystemExit('Experimental parameters differ from stable: '+name)
    subprocess.run([str(R/'.venv/bin/python'),
                    str(R/'candidates/xbox_native_turn120_20261019/check_powered_seal.py')], check=True)
    print('XBOX500_OFFLINE_SEAL_OK; powered hardware qualification still pending; no hardware access')


if __name__ == '__main__':
    main()
