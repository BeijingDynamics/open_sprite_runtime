"""Generate an isolated admission entry from the existing qualified script; no hardware."""
from pathlib import Path
import subprocess

root=Path(__file__).resolve().parents[1]
source=root/'run_sprite0825_bno055_waist60kd3_walk40_on_253.sh'
text=source.read_text()
needle='"$ROOT/.venv/bin/python" -m open_sprite_runtime.native_policy_client'
if text.count(needle)!=1:
    raise ValueError('baseline actor invocation changed')
start=text.rfind('OMP_NUM_THREADS=1',0,text.index(needle))
end=text.index('\nPOLICY_STATUS=$?',start)
old=text[start:end]
new=old.replace(needle,'"$ROOT/.venv/bin/python" "$ROOT/tools/native_walk40_adapter.py"')
prepare=new.replace('"$ROOT/tools/native_walk40_adapter.py"',
                    '"$ROOT/tools/native_walk40_adapter.py" --prepare-only')
text=text[:start]+new+text[end:]
controller='"$ROOT/build/native/sprite_can_shadow_quantized_v1" \\\n  --config'
if text.count(controller)!=1:
    raise ValueError('baseline controller invocation changed')
where=text.index(controller)
text=text[:where]+prepare+'\n\n'+text[where:]
# Only the active controller consumes the frozen files; preflight remains unchanged.
where=text.index(controller)
prefix,active=text[:where],text[where:]
for name in ('motors.tsv','kinematics.tsv','joint_safety.tsv'):
    active=active.replace(f'"$ROOT/build/native/{name}"',f'"${{POLICY_REPORT}}.native/{name}"')
text=prefix+active
text+='\nPYTHONPATH="$ROOT/src" "$ROOT/.venv/bin/python" "$ROOT/tools/check_native_powered_completion.py" \\\n  --report "$NATIVE_REPORT" --names "${POLICY_REPORT}.native/names.txt" \\\n  --actor-exit "$POLICY_STATUS" --controller-exit "$NATIVE_STATUS"\n'
destination=root/'run_sprite0825_native_actor_admission_candidate.sh'
destination.write_text(text)
wrapper=(root/'run_sprite0825_software_imu_walk40_on_253.sh').read_text()
wrapper=wrapper.replace(source.name,destination.name)
wrapper=wrapper.replace('ENABLE_SOFTWARE_IMU_WALK40','ENABLE_NATIVE_ACTOR_WALK40')
wrapper=wrapper.replace('Only IMU changes: raw ACCGYRO/P0, native200Hz fusion. Baseline walking controller unchanged.',
                        'Candidate: native200Hz IMU + native50Hz ONNX; existing protected CAN controller.')
entry=root/'run_sprite0825_full_native_walk40_candidate_on_253.sh'
entry.write_text(wrapper)
for path in (destination,entry):
    subprocess.run(['bash','-n',str(path)],check=True)
print('GENERATED',entry,'; syntax only, no hardware')
