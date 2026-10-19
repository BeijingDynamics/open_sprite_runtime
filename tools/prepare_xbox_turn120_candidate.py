"""Build an isolated turn candidate. No sensor, joystick or CAN access."""
from pathlib import Path
import shutil
import subprocess

R=Path('/home/tony/open_sprite_runtime')
OLD=R/'candidates/xbox_native_120_20261019'
C=R/'candidates/xbox_native_turn120_20261019'
UPLOAD=Path('/tmp/sprite_turn120_sources')

def main():
    subprocess.run(['python3',str(OLD/'check_powered_seal.py')],check=True)
    C.mkdir(exist_ok=False)
    shutil.copytree(OLD/'src',C/'src')
    for name in ('policy_gamepad.hpp','sprite_policy_replay.cpp','policy_gamepad_fake.hpp','gamepad_test.cpp'):
        shutil.copy2(UPLOAD/name,C/'src'/name)
    for name in ('fixture120.bin','buttons_verified.json','mapping_verified.json','check_completion.py','test_options','gamepad_disable_probe'):
        shutil.copy2(OLD/name,C/name)
    # Reuse the exact qualified CAN binary, including its realtime capability.
    for name in ('sprite_can_xbox120','libonnxruntime.so.1'):
        (C/name).symlink_to(OLD/name)
    pairs=[(OLD/name,C/name) for name in ('powered_adapter.py','admission.sh','check_powered_seal.py')]
    pairs += [(R/name,R/name.replace('forward120','turn120')) for name in (
        'run_sprite0825_xbox_native_forward120_on_253.sh','run_sprite0825_xbox_native_forward120_inner.sh')]
    for src,dst in pairs:
        s=src.read_text().replace(str(OLD),str(C)).replace('xbox_native_120_20261019','xbox_native_turn120_20261019')
        s=s.replace('forward120','turn120').replace('FORWARD120','TURN120')
        s=s.replace('forward only 0..0.10m/s','forward 0..0.10m/s, turn +/-0.10rad/s; no reverse')
        if dst.name=='admission.sh':
            old_check=f'getcap "{C}/sprite_can_xbox120"'
            if s.count(old_check)!=1:
                raise ValueError('Unexpected capability check')
            s=s.replace(old_check,f'getcap "$(readlink -e "{C}/sprite_can_xbox120")"')
        if dst.name=='check_powered_seal.py':
            s=s.replace("float(r['wz'])==0 for r in rows", "abs(float(r['wz']))<=.100001 for r in rows")
            s=s.replace("assert all(float(r['vx'])==0 for r in rows if float(r['elapsed_s'])>=112)",
                "assert max(float(r['wz']) for r in rows)>.099 and min(float(r['wz']) for r in rows)<-.099\n    assert all(float(r['vx'])==0 and float(r['wz'])==0 for r in rows if float(r['elapsed_s'])>=112)")
            s=s.replace('XBOX120_SEAL_OK','XBOX_TURN120_SEAL_OK')
        dst.write_text(s)
        if dst.suffix=='.sh':subprocess.run(['bash','-n',str(dst)],check=True)
        else:compile(s,str(dst),'exec')
    libdir=subprocess.check_output([str(R/'.venv/bin/python'),'-c',
        'import onnxruntime,pathlib;print(pathlib.Path(onnxruntime.__file__).parent/"capi")'],text=True).strip()
    cmd=['g++','-std=c++17','-O2','-ffp-contract=off','-Wall','-Wextra','-Werror','-pthread',
        '-DSPRITE_LOOP_TEST','-DSPRITE_POWERED_LOOP','-DSPRITE_ICM_IMU_INPUT','-DSPRITE_GAMEPAD_INPUT',
        '-DSPRITE_XBOX_120','-DSPRITE_XBOX_TURN','-DSPRITE_GAMEPAD_STOP_COMBO',
        '-I'+str(R/'build/policy_native_stage2/include'),'-I'+str(C/'src'),
        '-I'+str(R/'candidates/icm_native_20261016/native/src'),str(C/'src/sprite_policy_replay.cpp'),
        libdir+'/libonnxruntime.so.1.23.2','-Wl,-rpath,$ORIGIN']
    for name,flags in [('sprite_policy_xbox_powered',[]),('sprite_policy_xbox_fake',['-DSPRITE_GAMEPAD_FAKE_PEERS'])]:
        subprocess.run(cmd+flags+['-o',str(C/name)],check=True)
    (C/'sprite_policy_xbox_fake_stop').symlink_to(C/'sprite_policy_xbox_fake')
    subprocess.run(['g++','-std=c++17','-O2','-Wall','-Wextra','-Werror','-I'+str(C/'src'),
        str(C/'src/gamepad_test.cpp'),'-o',str(C/'gamepad_test')],check=True)
    subprocess.run([str(C/'gamepad_test')],check=True)
    subprocess.run([str(C/'test_options')],check=True)
    print('BUILT ONLY; unsealed until offline qualification passes. NO HARDWARE ACCESS.')

if __name__=='__main__':main()
