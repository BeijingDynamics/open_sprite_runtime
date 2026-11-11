"""Build a separate powered candidate from validated onsite scripts. NO hardware TX."""
import ast
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

R = Path('/home/tony/open_sprite_runtime')
E = Path(__file__).resolve().parents[1]
C = E/'candidates/xbox_500hz_powered_20261111'
OLD = R/'candidates/xbox_native_turn120_20261019'
PARAM = R/'candidates/upper_body_pd_wrist7_20261014'
NEW_PARAM = C/'parameters'
CAN = C/'sprite_can_xbox500'


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def once(text, old, new):
    if text.count(old) != 1:
        raise ValueError('Unexpected source anchor: '+old[:120])
    return text.replace(old, new)


def common_shell(text):
    text = text.replace(str(PARAM), str(NEW_PARAM))
    text = text.replace('$ROOT/reports', str(E/'reports'))
    text = text.replace('$ROOT/build/native', str(C/'scratch'))
    return text


def main():
    if E == R or C.exists():
        raise SystemExit('Require separate checkout and new candidate directory')
    subprocess.run(['bash', str(R/'run_sprite0825_xbox_native_turn120_on_253.sh'), '--check'], check=True)
    stable_can_source = R/'candidates/xbox_native_120_20261019/src/sprite_can_shadow.cpp'
    if digest(stable_can_source) != '50d9befb29d80da248103f7d843ab59117b0b615ab73e8477fc57db6f1e680e5':
        raise ValueError('Stable CAN source changed; review baseline diff again')
    evidence = E/'reports/xbox_500hz_full_shadow_120s_nhmm1j8m/report.json'
    proof = json.loads(evidence.read_text())
    if (proof.get('passed') is not True or proof.get('frames') != 6000
            or proof.get('transport_slot_overruns') != 0
            or proof.get('enable_attempts') != 0 or proof.get('nonzero_can_tx_attempts') != 0):
        raise ValueError('Full-chain zero-gain qualification missing')
    C.mkdir(parents=True)
    NEW_PARAM.mkdir()
    (C/'scratch').mkdir()
    (E/'reports').mkdir(exist_ok=True)
    shutil.copytree(PARAM/'deploy', NEW_PARAM/'deploy')
    for name in ('motors.tsv', 'kinematics.tsv', 'joint_safety.tsv'):
        shutil.copyfile(PARAM/name, NEW_PARAM/name)
    source_paths = [stable_can_source, OLD/'admission.sh', OLD/'powered_adapter.py', OLD/'check_completion.py',
                    R/'run_sprite0825_xbox_native_turn120_inner.sh',
                    R/'candidates/icm_native_20261016/shadow.sh']
    source_hashes = {str(p): digest(p) for p in source_paths}

    admission = common_shell((OLD/'admission.sh').read_text())
    admission = admission.replace(str(OLD/'sprite_can_xbox120'), str(CAN))
    admission = admission.replace(str(OLD/'powered_adapter.py'), str(C/'powered_adapter.py'))
    admission = admission.replace(str(OLD/'check_completion.py'), str(C/'check_completion.py'))
    admission = admission.replace(str(R/'candidates/icm_native_20261016/shadow.sh'), str(C/'shadow.sh'))
    admission = once(admission, '  --policy-actuation \\\n',
                     '  --policy-actuation --xbox-500hz-actuation \\\n'
                     '  --xbox-500hz-actuation-acknowledgement ENABLE_XBOX_500HZ_TURN120 \\\n')

    shadow = common_shell(source_paths[-1].read_text())
    shadow = once(shadow, '"'+str(C/'scratch')+'/sprite_can_shadow_quantized_v1" \\\n',
                  '"'+str(CAN)+'" --xbox-500hz-shadow \\\n')

    inner = (R/'run_sprite0825_xbox_native_turn120_inner.sh').read_text()
    inner = once(inner, 'baseline='+str(OLD/'admission.sh'), 'baseline='+str(C/'admission.sh'))
    inner = once(inner, 'run=$(mktemp -d reports/xbox_imu_turn120_XXXXXXXX)',
                 'run=$(mktemp -d '+str(E/'reports/xbox500_imu_XXXXXXXX')+')')
    inner = once(inner, 'echo "SOFTWARE_IMU_REPORT $ROOT/$run"', 'echo "SOFTWARE_IMU_REPORT $run"')
    inner = once(inner, '[s]prite_can_shadow|', '[s]prite_can_shadow|[s]prite_can_xbox|')

    adapter = (OLD/'powered_adapter.py').read_text()
    adapter = adapter.replace(str(OLD/'sprite_can_xbox120'), str(CAN))
    adapter = once(adapter, 'def validate(args):',
        'def validate(args):\n    subprocess.run([str(ROOT/".venv/bin/python"), '+repr(str(E/'tools/check_xbox500_powered.py'))+'], check=True)')
    completion = (OLD/'check_completion.py').read_text()
    completion = once(completion, '    errors = []',
        "    errors = []\n    if report.get('all_motors_requested_hz') != 500 or report.get('transport_slot_overruns') != 0:\n        errors.append('500Hz schedule/timing qualification failed')")

    for name, text in [('admission.sh', admission), ('shadow.sh', shadow),
                       ('inner.sh', inner), ('powered_adapter.py', adapter),
                       ('check_completion.py', completion)]:
        path = C/name
        path.write_text(text)
        if name.endswith('.sh'):
            subprocess.run(['bash', '-n', str(path)], check=True)
        else:
            ast.parse(text)
    subprocess.run(['g++', '-std=c++17', '-O2', '-Wall', '-Wextra', '-Wpedantic', '-Werror',
                    '-pthread', str(E/'native/src/sprite_can_shadow.cpp'), '-o', str(CAN)], check=True)
    subprocess.run(['g++', '-std=c++17', '-O2', '-Wall', '-Wextra', '-Werror', '-pthread',
                    str(E/'native/tests/test_xbox_500hz_powered.cpp'), '-o', str(C/'test_powered')], check=True)
    subprocess.run([str(C/'test_powered'), str(NEW_PARAM)], check=True)
    subprocess.run(['g++', '-std=c++17', '-O2', '-Wall', '-Wextra', '-Werror', '-pthread',
                    str(E/'native/tests/test_xbox_500hz_shutdown.cpp'), '-o', str(C/'test_shutdown')], check=True)
    subprocess.run([str(C/'test_shutdown')], check=True)
    subprocess.run(['g++', '-std=c++17', '-O2', '-Wall', '-Wextra', '-Werror',
                    str(E/'native/tests/test_first_slot_wait.cpp'), '-o', str(C/'test_first_slot_wait')], check=True)
    subprocess.run([str(C/'test_first_slot_wait')], check=True)
    subprocess.run(['g++', '-std=c++17', '-O2', '-Wall', '-Wextra', '-Werror', '-pthread',
                    str(E/'native/tests/test_xbox_500hz_feedback.cpp'), '-o', str(C/'test_feedback')], check=True)
    subprocess.run([str(C/'test_feedback')], check=True)
    files = [*source_paths, CAN, C/'test_powered', C/'test_shutdown', evidence,
             E/'native/src/sprite_can_shadow.cpp', E/'native/src/transport_slot_timing.hpp',
             E/'native/src/first_slot_wait.hpp', E/'native/tests/test_first_slot_wait.cpp', C/'test_first_slot_wait',
             E/'native/src/fixed_rms_window.hpp', E/'native/src/receive_timing.hpp',
             E/'native/tests/test_xbox_500hz_feedback.cpp', C/'test_feedback',
             E/'tools/prepare_xbox500_powered.py', E/'tools/check_xbox500_powered.py',
             E/'tools/run_native_policy_hardware_shadow.py', E/'run_xbox_500hz_full_shadow_120s_on_253.sh',
             E/'run_xbox_500hz_powered_120s_on_253.sh']
    files += [C/name for name in ('admission.sh','shadow.sh','inner.sh','powered_adapter.py','check_completion.py')]
    files += list((NEW_PARAM/'deploy').rglob('*'))
    files += [NEW_PARAM/name for name in ('motors.tsv','kinematics.tsv','joint_safety.tsv')]
    seal = {'mode':'xbox500_prepared_not_hardware_qualified',
            'source_sha256':source_hashes,
            'sha256':{str(p):digest(p) for p in files if p.is_file()},
            'powered_test_run':False}
    (C/'seal.json').write_text(json.dumps(seal, indent=2)+'\n')
    subprocess.run([str(R/'.venv/bin/python'), str(E/'tools/check_xbox500_powered.py')], check=True)
    print('PREPARED_NO_CAN_NO_SPI_NO_ENABLE', C)
    print('Install CAP_SYS_NICE only on', CAN)


if __name__ == '__main__':
    main()
