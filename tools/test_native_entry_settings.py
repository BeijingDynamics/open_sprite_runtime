"""Evaluate baseline shell settings only, stopping before all hardware/preflight actions."""
import os
from pathlib import Path
import socket
import subprocess
import tempfile

root=Path(__file__).resolve().parents[1]
text=(root/'run_sprite0825_native_actor_admission_candidate.sh').read_text()
marker='PYTHONPATH="$ROOT/src" "$ROOT/.venv/bin/python" \\\n  "$ROOT/tools/export_native_motor_config.py"'
prefix=text[:text.index(marker)]
needle='"$ROOT/tools/native_walk40_adapter.py" --prepare-only'
start=text.rfind('OMP_NUM_THREADS=1',0,text.index(needle))
end=text.index('\n\n',text.index(needle))
prepare=text[start:end]
with tempfile.TemporaryDirectory(prefix='native_settings_') as temp:
    with socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM) as fake:
        fake.bind(temp+'/imu.sock')
        env=dict(os.environ,SPRITE_NATIVE_IMU_SOCKET=temp+'/imu.sock',SPRITE_NATIVE_IMU_PROTOCOL='software-v2')
        script=Path(temp)/'settings.sh'
        script.write_text(prefix+f'\nPOLICY_REPORT="{temp}/actor.json"\nPOLICY_TRACE="{temp}/trace.npz"\n'+prepare+'\n')
        subprocess.run(['bash',str(script),'ENABLE_BNO055_WAIST60_KD3_WALK40'],env=env,check=True)
print('PASS actual shell settings match qualified actor; no CAN/I2C, no policy launch')
