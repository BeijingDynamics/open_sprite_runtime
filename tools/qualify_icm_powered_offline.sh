#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
root="$PWD"
c="$root/candidates/icm_native_20261016"
b="$root/candidates/upper_body_pd_wrist7_20261014"
libdir=$(.venv/bin/python -c 'import onnxruntime,pathlib; print(pathlib.Path(onnxruntime.__file__).parent/"capi")')
g++ -std=c++17 -O2 -ffp-contract=off -Wall -Wextra -Werror -pthread \
  -DSPRITE_LOOP_TEST -DSPRITE_POWERED_LOOP -DSPRITE_ICM_IMU_INPUT \
  -Ibuild/policy_native_stage2/include "$c/native/src/sprite_policy_replay.cpp" \
  "$libdir/libonnxruntime.so.1.23.2" -Wl,-rpath,'$ORIGIN' -o "$c/build/sprite_policy_powered_icm_candidate"
ldd "$c/build/sprite_policy_powered_icm_candidate"
output=$(mktemp -d "$c/offline_XXXXXXXX")
echo "NO HARDWARE. Unix fake peers only: $output"
for fault in imu_freeze state_disconnect none; do
  PYTHONPATH=src .venv/bin/python tools/test_native_policy_realtime_loop.py \
    --binary "$c/build/sprite_policy_powered_icm_candidate" --powered-candidate --icm-input \
    --offline-binary build/policy_native_stage2/sprite_policy_replay \
    --model "$b/deploy/policy.onnx" --fixture "$b/qualification/fixture40.bin" \
    --output-dir "$output/$fault" --fault "$fault" --command-profile stand-walk010-stop-40s
done
echo "OFFLINE_QUALIFICATION $output"
