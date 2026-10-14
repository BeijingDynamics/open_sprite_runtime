#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"
mode="${1:-}"
case "$mode" in
  --check|ENABLE_NATIVE_BNO_WAIST60_KD3_WALK40) ;;
  *) echo "Usage: bash $0 --check | ENABLE_NATIVE_BNO_WAIST60_KD3_WALK40" >&2; exit 2 ;;
esac
producer=build/imu_native_stage1/sprite_imu_bno
baseline=run_sprite0825_bno055_waist60kd3_walk40_on_253.sh
[[ -x "$producer" && -x .venv/bin/python && -f "$baseline" ]] || exit 1
bash -n "$baseline"
PYTHONPATH=src .venv/bin/python -c 'from open_sprite_runtime.native_bno_source import NativeBnoSource'
if [[ "$mode" == --check ]]; then
  echo "STATIC CHECK PASSED: no I2C/CAN access, no policy or motor enable"
  exit 0
fi
if pgrep -f '[s]prite_imu_bno|[o]pen_sprite_runtime.native_policy_client|[s]prite_can_shadow' >/dev/null; then
  echo "Another IMU/policy/CAN process is active; refusing concurrent test" >&2
  exit 1
fi
echo "ACTIVE 40s comparison: native BNO input only; baseline policy/gains/caps/timeline unchanged"
echo "Requires supported robot, clear walking/arm space, and power safety operator ready"
echo "No motor zero reset or IMU configuration writes. Existing admission checks remain enforced."
run_dir=$(mktemp -d reports/native_bno_walk40_XXXXXXXX)
socket_path="/tmp/sprite-native-walk-$$.sock"
producer_pid=""
walk_pid=""
cleanup() {
  # Let the existing controller perform disable/verification before stopping IMU.
  if [[ -n "$walk_pid" ]] && kill -0 "$walk_pid" 2>/dev/null; then
    kill -TERM "$walk_pid" 2>/dev/null || true
    wait "$walk_pid" || true
  fi
  if [[ -n "$producer_pid" ]] && kill -0 "$producer_pid" 2>/dev/null; then
    kill -TERM "$producer_pid" 2>/dev/null || true
    wait "$producer_pid" || true
  fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
"$producer" --record 120 "$socket_path" "$run_dir/producer.json" >"$run_dir/producer.log" 2>&1 &
producer_pid=$!
sleep 1
kill -0 "$producer_pid" 2>/dev/null || { cat "$run_dir/producer.log"; exit 1; }
PYTHONPATH=src .venv/bin/python -c '
import sys,time
from open_sprite_runtime.native_bno_source import NativeBnoSource
with NativeBnoSource(sys.argv[1]) as source:
    deadline=time.monotonic()+1
    packets=[]
    while not packets and time.monotonic()<deadline:
        source.drain(lambda *value: packets.append(value))
        time.sleep(.005)
    if not packets: raise RuntimeError("No fresh native IMU sample")
    print(source.report())
' "$socket_path"
echo "NATIVE_IMU_REPORT_DIRECTORY $PWD/$run_dir"
export SPRITE_NATIVE_IMU_SOCKET="$socket_path"
bash "$baseline" ENABLE_BNO055_WAIST60_KD3_WALK40 &
walk_pid=$!
set +e
wait "$walk_pid"
status=$?
set -e
walk_pid=""
echo "WALK_EXIT_STATUS $status"
echo "Stopping diagnostic IMU producer after controller exit; capture interrupted is expected here."
exit "$status"
