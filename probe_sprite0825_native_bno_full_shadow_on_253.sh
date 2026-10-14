#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"
[[ "${1:-}" == ZERO_GAIN_ALL_DISABLED_SUPPORTED ]] || {
  echo "Requires ZERO_GAIN_ALL_DISABLED_SUPPORTED: supported robot, all drives disabled" >&2
  exit 1
}
duration="${2:-10}"
rounds="${3:-1}"
[[ "$duration" =~ ^[0-9]+$ && "$rounds" =~ ^[0-9]+$ ]] || exit 2
(( duration >= 1 && duration <= 120 && rounds >= 1 && rounds <= 5 )) || exit 2
producer_seconds=$((duration * rounds + 8 * rounds + 10))
(( producer_seconds <= 600 )) || { echo "IMU diagnostic exceeds 600s" >&2; exit 2; }
echo "${rounds} x ${duration}s FULL SHADOW: zero-gain CAN polling only; no enable or IMU configuration writes"
run_dir=$(mktemp -d reports/native_bno_full_shadow_XXXXXXXX)
socket_path="/tmp/sprite-native-full-$$.sock"
producer_pid=""
cleanup() {
  if [[ -n "$producer_pid" ]] && kill -0 "$producer_pid" 2>/dev/null; then
    kill -TERM "$producer_pid"
    wait "$producer_pid" || true
  fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
build/imu_native_stage1/sprite_imu_bno --record "$producer_seconds" "$socket_path" "$run_dir/producer.json" >"$run_dir/producer.log" 2>&1 &
producer_pid=$!
sleep 1
if ! kill -0 "$producer_pid" 2>/dev/null; then
  cat "$run_dir/producer.log"
  exit 1
fi
# No CAN until one fresh, valid packet has traversed the real adapter.
PYTHONPATH=src .venv/bin/python -c '
import sys,time
from open_sprite_runtime.native_bno_source import NativeBnoSource
with NativeBnoSource(sys.argv[1]) as source:
    deadline=time.monotonic()+1
    packets=[]
    while not packets and time.monotonic()<deadline:
        source.drain(lambda *value: packets.append(value))
        time.sleep(.005)
    if not packets: raise RuntimeError("No native IMU sample before CAN start")
    print(source.report())
' "$socket_path"
status=0
for (( round=1; round<=rounds; round++ )); do
  kill -0 "$producer_pid" || { cat "$run_dir/producer.log"; exit 1; }
  echo "SHADOW_ROUND $round/$rounds REPORT_DIRECTORY $PWD/$run_dir"
  set +e
  SPRITE_NATIVE_IMU_SOCKET="$socket_path" bash ./probe_sprite0825_bno055_policy_shadow_on_253.sh "$duration" 2>&1 | tee "$run_dir/shadow_${round}.log"
  status=${PIPESTATUS[0]}
  set -e
  (( status == 0 )) || { cat "$run_dir/producer.log"; exit "$status"; }
done
wait "$producer_pid"
producer_pid=""
cat "$run_dir/producer.json"
echo "REPORT DIRECTORY: $PWD/$run_dir"
exit "$status"
