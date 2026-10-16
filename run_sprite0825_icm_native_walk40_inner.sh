#!/usr/bin/env bash
set -euo pipefail
ROOT=/home/tony/open_sprite_runtime
CANDIDATE="$ROOT/candidates/bno_software_20261014"
cd "$ROOT"
case "${1:-}" in
  --check|ENABLE_ICM45686_WALK40) ;;
  *) echo "Usage: bash $0 --check | ENABLE_ICM45686_WALK40"; exit 2 ;;
esac
baseline=/home/tony/open_sprite_runtime/run_sprite0825_icm_native_admission.sh
producer="/home/tony/open_sprite_runtime/candidates/icm_native_20261016/build/sprite_icm45686"
[[ -x "$producer" && -f "$baseline" ]]
bash -n "$baseline"
export SPRITE_NATIVE_IMU_PROTOCOL=icm-v3
PYTHONPATH="/home/tony/open_sprite_runtime/candidates/icm_native_20261016/python" .venv/bin/python -c 'from open_sprite_runtime.native_imu_packet import decode_icm_packet; from open_sprite_runtime.native_bno_source import NativeBnoSource; assert NativeBnoSource("unused").protocol == "icm-v3"'
"$ROOT/.venv/bin/python" - <<'PYQUAL'
import hashlib,json
from pathlib import Path
q=Path('/home/tony/open_sprite_runtime/candidates/upper_body_pd_wrist7_20261014/qualification')
if json.loads((q/'entry_checks.json').read_text()).get('passed') is not True:
    raise SystemExit('new entry offline checks missing')
seal=json.loads((q/'seal.json').read_text())
if any(hashlib.sha256(Path(p).read_bytes()).hexdigest()!=v for p,v in seal.items()):
    raise SystemExit('new candidate qualification seal changed')
print('NEW_PARAMETER_BUNDLE_QUALIFIED; no hardware access')
PYQUAL
"$ROOT/.venv/bin/python" - <<'PYICM'
import hashlib,json
from pathlib import Path
p=Path('/home/tony/open_sprite_runtime/candidates/icm_native_20261016/entry_seal.json')
if any(hashlib.sha256(Path(k).read_bytes()).hexdigest()!=v for k,v in json.loads(p.read_text()).items()):
    raise SystemExit('ICM entry changed since offline qualification')
PYICM
if [[ "$1" == --check ]]; then
  echo 'STATIC CHECK PASSED; no I2C/CAN, no enable'; exit 0
fi
if pgrep -f '[s]prite_icm45686|[s]prite_imu_|[s]prite_policy_|[s]prite_can_shadow|[o]pen_sprite_runtime.native_policy_client' >/dev/null; then
  echo 'Another runtime/IMU process is active'; exit 1
fi
"$ROOT/.venv/bin/python" - <<'PYCAN'
import json, subprocess
links=json.loads(subprocess.check_output(['ip','-details','-json','link','show']))
by_name={x['ifname']:x for x in links}
for name in ('kcan1','kcan2','kcan3','kcan4'):
    item=by_name.get(name,{})
    data=item.get('linkinfo',{}).get('info_data',{})
    if ('UP' not in item.get('flags',[]) or item.get('mtu')!=72
        or data.get('bittiming',{}).get('bitrate')!=1000000
        or data.get('data_bittiming',{}).get('bitrate')!=5000000):
        raise SystemExit(name+': require UP, CAN-FD MTU72, 1M/5M. Run setup_sprite0825_kcanfd_on_253.sh')
print('CAN configuration checked; no CAN transmission')
PYCAN
echo 'Requires fall arrest, clear space, and power safety operator. Hold still first15s.'
echo 'POWERED CANDIDATE: native ICM800Hz + native ONNX50Hz + CAN; new upper-body PD. Explicit onsite approval required.'
[[ "$(cat /sys/bus/platform/devices/3210000.spi/power/control)" == on ]] || { echo "Use PM wrapper"; exit 1; }
run=$(mktemp -d reports/icm_imu_walk40_XXXXXXXX)
sockdir=$(mktemp -d /tmp/sprite_sw_walk_XXXXXXXX)
producer_pid=''; walk_pid=''
cleanup() {
  if [[ -n "$walk_pid" ]] && kill -0 "$walk_pid" 2>/dev/null; then
    kill -TERM "$walk_pid" 2>/dev/null || true
    wait "$walk_pid" || true
  fi
  if [[ -n "$producer_pid" ]] && kill -0 "$producer_pid" 2>/dev/null; then
    kill -TERM "$producer_pid" 2>/dev/null || true
    wait "$producer_pid" || true
  fi
  rmdir "$sockdir" 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
read -r chip offset < <(gpiofind PY.01)
ICM_IPC_SOCKET="$sockdir/imu.sock" "$producer" capture /dev/spidev0.0 "/dev/$chip" "$offset" 800 180 "$run/imu.csv" X_DOWN_Z_BACK CONFIGURE_ICM45686_RAW_NO_CAN >"$run/imu.log" 2>&1 &
producer_pid=$!
ready=0
for _ in $(seq 1 240); do
  kill -0 "$producer_pid" 2>/dev/null || { cat "$run/imu.log"; exit 1; }
  if grep -q '^BIAS_READY' "$run/imu.log"; then ready=1; break; fi
  sleep .1
done
[[ "$ready" == 1 ]] || { echo 'Static calibration did not finish'; exit 1; }
export SPRITE_NATIVE_IMU_SOCKET="$sockdir/imu.sock"
PYTHONPATH="/home/tony/open_sprite_runtime/candidates/icm_native_20261016/python" .venv/bin/python -c '
import os,time
from open_sprite_runtime.native_bno_source import NativeBnoSource
with NativeBnoSource(os.environ["SPRITE_NATIVE_IMU_SOCKET"]) as source:
    samples=[]; deadline=time.monotonic()+1
    while not samples and time.monotonic()<deadline:
        source.drain(lambda *v:samples.append(v));time.sleep(.005)
    if not samples:raise RuntimeError("No fresh validated software IMU input")
    print(source.report())
'
echo "SOFTWARE_IMU_REPORT $ROOT/$run"
bash "$baseline" ENABLE_ICM45686_WALK40 &
walk_pid=$!
set +e
wait "$walk_pid"
status=$?
set -e
walk_pid=''
exit "$status"
