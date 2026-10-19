"""Build isolated forward-only Xbox entry. No hardware access or execution."""
import hashlib
import json
from pathlib import Path
import subprocess

ROOT = Path('/home/tony/open_sprite_runtime')
C = ROOT/'candidates/xbox_native_20261019'

def replace_once(text, old, new):
    if text.count(old) != 1:
        raise ValueError('baseline changed: '+old[:100])
    return text.replace(old, new)

def main():
    wrapper=(ROOT/'run_sprite0825_icm_native_walk40_on_253.sh').read_text()
    wrapper=wrapper.replace('run_sprite0825_icm_native_walk40_inner.sh','run_sprite0825_xbox_native_forward40_inner.sh')
    wrapper=wrapper.replace('ENABLE_ICM45686_WALK40','ENABLE_XBOX_FORWARD40')
    inner=(ROOT/'run_sprite0825_icm_native_walk40_inner.sh').read_text()
    inner=inner.replace('run_sprite0825_icm_native_admission.sh','run_sprite0825_xbox_native_admission.sh')
    inner=inner.replace('ENABLE_ICM45686_WALK40','ENABLE_XBOX_FORWARD40')
    inner=inner.replace('reports/icm_imu_walk40_','reports/xbox_imu_forward40_')
    inner=replace_once(inner,'if [[ "$1" == --check ]]; then',
        '"$ROOT/.venv/bin/python" "$ROOT/candidates/xbox_native_20261019/check_powered_seal.py"\nif [[ "$1" == --check ]]; then')
    admission=(ROOT/'run_sprite0825_icm_native_admission.sh').read_text()
    admission=admission.replace('ENABLE_ICM45686_WALK40','ENABLE_XBOX_FORWARD40')
    admission=admission.replace('"$ROOT/tools/icm_native_walk40_adapter.py"',f'"{C}/powered_adapter.py"')
    # The adapter receives the existing profile to validate baseline settings,
    # but the native actor receives only its dedicated Xbox profile.
    admission=replace_once(admission,
        'echo "TIMELINE 0-10s stand; 10-13s ramp to ${PROFILE_FORWARD_SPEED}m/s; 13-27s forward; 27-30s ramp to zero; 30-40s stand then disable"',
        'echo "XBOX: 0-10s stand; 10-30s deliberate LT rearm and forward only 0..0.10m/s; 30s ramp to zero, stand until40s then disable"')
    adapter=(ROOT/'tools/icm_native_walk40_adapter.py').read_text()
    adapter=replace_once(adapter,'ROOT = Path(__file__).resolve().parents[1]',f'ROOT = Path({str(ROOT)!r})')
    adapter=replace_once(adapter,"BINARY = Path('/home/tony/open_sprite_runtime/candidates/icm_native_20261016/build/sprite_policy_powered_icm_candidate')",f'BINARY = Path({str(C/"sprite_policy_xbox_powered")!r})')
    adapter=replace_once(adapter,'def validate(args):','def validate(args):\n    subprocess.run([str(ROOT/".venv/bin/python"),str(BINARY.parent/"check_powered_seal.py")],check=True)')
    adapter=replace_once(adapter,"args.socket,args.native_imu_socket,str(bundle/'names.txt'),args.command_profile,\n             'ENABLE_NATIVE_ACTOR_WALK40']",
        "args.socket,args.native_imu_socket,str(bundle/'names.txt'),'xbox-forward40',\n             'ENABLE_XBOX_FORWARD40']")
    adapter=replace_once(adapter,'child=subprocess.Popen(command)',
        'mapping=json.loads((BINARY.parent/"mapping_verified.json").read_text())\n'
        '    if not mapping.get("mapping_verified") or mapping["forward_axis"]!=1 or mapping["turn_axis"]!=2 or mapping["permission_axis"]!=5:raise ValueError("unverified mapping")\n'
        '    env=os.environ.copy()\n'
        '    env.update(SPRITE_GAMEPAD_DEVICE=mapping["device"],SPRITE_GAMEPAD_FORWARD_SIGN=str(mapping["forward_sign"]),SPRITE_GAMEPAD_TURN_SIGN=str(mapping["turn_sign"]))\n'
        '    child=subprocess.Popen(command,env=env)')
    outputs={ROOT/'run_sprite0825_xbox_native_forward40_on_253.sh':wrapper,
             ROOT/'run_sprite0825_xbox_native_forward40_inner.sh':inner,
             ROOT/'run_sprite0825_xbox_native_admission.sh':admission,
             C/'powered_adapter.py':adapter}
    for p,text in outputs.items():
        p.write_text(text)
        if p.suffix=='.sh':subprocess.run(['bash','-n',str(p)],check=True)
        else:compile(text,str(p),'exec')
    libdir=subprocess.check_output([str(ROOT/'.venv/bin/python'),'-c',
        'import onnxruntime,pathlib; print(pathlib.Path(onnxruntime.__file__).parent/"capi")'],text=True).strip()
    cmd=['g++','-std=c++17','-O2','-ffp-contract=off','-Wall','-Wextra','-Werror','-pthread',
         '-DSPRITE_LOOP_TEST','-DSPRITE_POWERED_LOOP','-DSPRITE_ICM_IMU_INPUT','-DSPRITE_GAMEPAD_INPUT',
         '-I'+str(ROOT/'build/policy_native_stage2/include'),'-I'+str(C/'src'),
         '-I'+str(ROOT/'candidates/icm_native_20261016/native/src'),str(C/'src/sprite_policy_replay.cpp'),
         libdir+'/libonnxruntime.so.1.23.2','-Wl,-rpath,$ORIGIN']
    for name,flags in [('sprite_policy_xbox_powered',[]),('sprite_policy_xbox_fake',['-DSPRITE_GAMEPAD_FAKE_PEERS'])]:
        subprocess.run(cmd+flags+['-o',str(C/name)],check=True)
    print('BUILT ONLY. Run fake-peer tests and generate qualification seal before entry can run.')

if __name__=='__main__':main()
