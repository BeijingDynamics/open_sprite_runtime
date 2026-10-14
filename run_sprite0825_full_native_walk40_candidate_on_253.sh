#!/usr/bin/env bash
set -euo pipefail
ROOT=/home/tony/open_sprite_runtime
CANDIDATE="$ROOT/candidates/bno_software_20261014"
cd "$ROOT"
case "${1:-}" in
  --check|ENABLE_NATIVE_ACTOR_WALK40) ;;
  *) echo "Usage: bash $0 --check | ENABLE_NATIVE_ACTOR_WALK40"; exit 2 ;;
esac
baseline=run_sprite0825_native_actor_admission_candidate.sh
producer="$CANDIDATE/build/imu_software_candidate/sprite_imu_software"
[[ -x "$producer" && -f "$baseline" ]]
bash -n "$baseline"
export SPRITE_NATIVE_IMU_PROTOCOL=software-v2
PYTHONPATH=src .venv/bin/python -c 'from open_sprite_runtime.native_imu_packet import decode_software_packet; from open_sprite_runtime.native_bno_source import NativeBnoSource; assert NativeBnoSource("unused").protocol == "software-v2"'
if [[ "$1" == --check ]]; then
  echo 'STATIC CHECK PASSED; no I2C/CAN, no enable'; exit 0
fi
if pgrep -f '[s]prite_imu_|[s]prite_policy_|[s]prite_can_shadow|[o]pen_sprite_runtime.native_policy_client' >/dev/null; then
  echo 'Another runtime/IMU process is active'; exit 1
fi
echo 'Requires fall arrest, clear space, and power safety operator. Hold still first15s.'
echo 'Candidate: native200Hz IMU + native50Hz ONNX; existing protected CAN controller.'
run=$(mktemp -d reports/software_imu_walk40_XXXXXXXX)
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
"$producer" --record 180 "$sockdir/imu.sock" "$run/imu.json" CONFIGURE_RAW_P0_NO_CAN 200 burst18 >"$run/imu.log" 2>&1 &
producer_pid=$!
ready=0
for _ in $(seq 1 240); do
  kill -0 "$producer_pid" 2>/dev/null || { cat "$run/imu.log"; exit 1; }
  if grep -q '^SOFTWARE_IMU_READY' "$run/imu.log"; then ready=1; break; fi
  sleep .1
done
[[ "$ready" == 1 ]] || { echo 'Static calibration did not finish'; exit 1; }
export SPRITE_NATIVE_IMU_SOCKET="$sockdir/imu.sock"
PYTHONPATH=src .venv/bin/python -c '
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
bash "$baseline" ENABLE_BNO055_WAIST60_KD3_WALK40 &
walk_pid=$!
set +e
wait "$walk_pid"
status=$?
set -e
walk_pid=''
exit "$status"
