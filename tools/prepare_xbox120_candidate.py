"""Generate isolated120s candidate. Build only; never execute hardware control."""
import json
from pathlib import Path
import subprocess
import struct

R=Path('/home/tony/open_sprite_runtime')
OLD=R/'candidates/xbox_native_20261019'
C=R/'candidates/xbox_native_120_20261019'

def once(s,a,b):
    if s.count(a)!=1:raise ValueError('Unexpected baseline: '+a[:80])
    return s.replace(a,b)

def main():
    # Copy generated40s files; keep both baseline entries and seals intact.
    pairs=[(R/'run_sprite0825_xbox_native_forward40_on_253.sh',R/'run_sprite0825_xbox_native_forward120_on_253.sh'),
           (R/'run_sprite0825_xbox_native_forward40_inner.sh',R/'run_sprite0825_xbox_native_forward120_inner.sh'),
           (R/'run_sprite0825_xbox_native_admission.sh',C/'admission.sh'),
           (OLD/'powered_adapter.py',C/'powered_adapter.py')]
    for src,dst in pairs:
        s=src.read_text().replace(str(OLD),str(C))
        s=s.replace('$ROOT/candidates/xbox_native_20261019/', '$ROOT/candidates/xbox_native_120_20261019/')
        s=s.replace('forward40','forward120').replace('ENABLE_XBOX_FORWARD40','ENABLE_XBOX_FORWARD120')
        s=s.replace('run_sprite0825_xbox_native_admission.sh','candidates/xbox_native_120_20261019/admission.sh')
        if dst.name=='admission.sh':
            s=once(s,'# Contract already contains effective commissioning gains.',
                'DURATION=120.0\nEXTENDED_NATIVE_ACK_ARGS=(--extended-policy-actuation-acknowledgement ENABLE_120_SECOND_XBOX_FORWARD_TEST)\n# Contract already contains effective commissioning gains.')
            s=s.replace('$ROOT/build/native/sprite_can_shadow_quantized_v1',str(C/'sprite_can_xbox120'))
            s=s.replace('$ROOT/tools/check_native_powered_completion.py',str(C/'check_completion.py'))
            s=s.replace('10-30s deliberate LT rearm','10-110s deliberate LT rearm').replace('30s ramp to zero, stand until40s','110s ramp to zero, stand until120s')
        if dst.name=='powered_adapter.py':
            s=once(s,"header+bytes(count*72*8)","header[:8]+struct.pack('<I',6001)+header[12:]+bytes(6001*72*8)")
            s=once(s,"ROOT/'build/native/sprite_can_shadow_quantized_v1'",f'Path({str(C/"sprite_can_xbox120")!r})')
            s=once(s,"env.update(SPRITE_GAMEPAD_DEVICE=mapping[\"device\"]",'buttons=json.loads((BINARY.parent/"buttons_verified.json").read_text())\n    if not buttons.get("passed") or not buttons.get("native_combo_passed"):raise ValueError("LB/RB unverified")\n    env.update(SPRITE_GAMEPAD_LB=str(buttons["lb"]),SPRITE_GAMEPAD_RB=str(buttons["rb"]))\n    env.update(SPRITE_GAMEPAD_DEVICE=mapping["device"]')
            # Verify before prepare, not only after the controller has launched.
            s=once(s,'def validate(args):','def validate(args):\n    buttons=json.loads((BINARY.parent/"buttons_verified.json").read_text())\n    if not buttons.get("passed") or not buttons.get("native_combo_passed"):raise ValueError("LB/RB unverified")')
        dst.write_text(s)
        if dst.suffix=='.sh':subprocess.run(['bash','-n',str(dst)],check=True)
        else:compile(s,str(dst),'exec')
    checker=(R/'tools/check_native_powered_completion.py').read_text()
    checker=once(checker,"report.get('duration_s') != 40","report.get('duration_s') != 120")
    (C/'check_completion.py').write_text(checker)
    can=(R/'candidates/icm_native_20261016/native/src/sprite_can_shadow.cpp').read_text()
    if can != (R/'tools/sprite_can_shadow.cpp').read_text():
        raise ValueError('Stable CAN source copies disagree; verify provenance before building')
    can=once(can,'    const bool acknowledged_20_seconds =',
        '    const bool acknowledged_120_seconds = result.extended_policy_actuation_acknowledgement == "ENABLE_120_SECOND_XBOX_FORWARD_TEST";\n    const bool acknowledged_20_seconds =')
    can=once(can,'        !acknowledged_60_seconds) {','        !acknowledged_60_seconds && !acknowledged_120_seconds) {')
    can=once(can,'!acknowledged_40_seconds && !acknowledged_60_seconds) {','!acknowledged_40_seconds && !acknowledged_60_seconds && !acknowledged_120_seconds) {')
    can=once(can,'result.duration_s > 40.0 && !acknowledged_60_seconds','result.duration_s > 40.0 && !acknowledged_60_seconds && !acknowledged_120_seconds')
    can=once(can,'if (result.duration_s > 60.0) {','if (result.duration_s > 60.0 && !acknowledged_120_seconds) {')
    can=once(can,'        !acknowledged_short_sim_pd_saturation))) {',
        '        result.extended_policy_actuation_acknowledgement != "ENABLE_120_SECOND_XBOX_FORWARD_TEST" &&\n        !acknowledged_short_sim_pd_saturation))) {')
    (C/'src/sprite_can_shadow.cpp').write_text(can)
    subprocess.run(['g++','-std=c++17','-O2','-Wall','-Wextra','-Wpedantic','-Werror','-pthread','-I'+str(R/'native/src'),str(C/'src/sprite_can_shadow.cpp'),'-o',str(C/'sprite_can_xbox120')],check=True)
    libdir=subprocess.check_output([str(R/'.venv/bin/python'),'-c','import onnxruntime,pathlib;print(pathlib.Path(onnxruntime.__file__).parent/"capi")'],text=True).strip()
    cmd=['g++','-std=c++17','-O2','-ffp-contract=off','-Wall','-Wextra','-Werror','-pthread',
         '-DSPRITE_LOOP_TEST','-DSPRITE_POWERED_LOOP','-DSPRITE_ICM_IMU_INPUT','-DSPRITE_GAMEPAD_INPUT','-DSPRITE_XBOX_120','-DSPRITE_GAMEPAD_STOP_COMBO',
         '-I'+str(R/'build/policy_native_stage2/include'),'-I'+str(C/'src'),'-I'+str(OLD/'src'),
         '-I'+str(R/'candidates/icm_native_20261016/native/src'),str(C/'src/sprite_policy_replay.cpp'),
         libdir+'/libonnxruntime.so.1.23.2','-Wl,-rpath,$ORIGIN']
    for name,flags in [('sprite_policy_xbox_powered',[]),('sprite_policy_xbox_fake',['-DSPRITE_GAMEPAD_FAKE_PEERS'])]:
        subprocess.run(cmd+flags+['-o',str(C/name)],check=True)
    lib=C/'libonnxruntime.so.1'
    if not lib.exists():lib.symlink_to(libdir+'/libonnxruntime.so.1.23.2')
    subprocess.run(['g++','-std=c++17','-O2','-pthread','-DSPRITE_GAMEPAD_STOP_COMBO','-I'+str(C/'src'),str(C/'src/gamepad_disable_probe.cpp'),'-o',str(C/'gamepad_disable_probe')],check=True)
    (C/'mapping_verified.json').write_bytes((OLD/'mapping_verified.json').read_bytes())
    # Fake-peer fixture preserves the exact header/settings; cycle recorded states.
    b=(R/'candidates/upper_body_pd_wrist7_20261014/qualification/fixture40.bin').read_bytes()
    n=struct.unpack_from('<I',b,8)[0];h=b[:-n*576];rows=b[-n*576:]
    (C/'fixture120.bin').write_bytes(h[:8]+struct.pack('<I',6001)+h[12:]+(rows*4)[:6001*576])
    print('BUILT ONLY. Button qualification, offline tests, seal and CAP_SYS_NICE still required.')

if __name__=='__main__':main()
