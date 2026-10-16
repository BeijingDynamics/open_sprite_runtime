#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
candidate="$PWD/candidates/icm_native_20261016"
libdir=$(.venv/bin/python -c 'import onnxruntime,pathlib; print(pathlib.Path(onnxruntime.__file__).parent/"capi")')
g++ -std=c++17 -O2 -ffp-contract=off -Wall -Wextra -Werror -pthread \
  -DSPRITE_LOOP_TEST -DSPRITE_SHADOW_LOOP -DSPRITE_ICM_IMU_INPUT \
  -Ibuild/policy_native_stage2/include "$candidate/native/src/sprite_policy_replay.cpp" \
  "$libdir/libonnxruntime.so.1.23.2" -Wl,-rpath,'$ORIGIN' \
  -o "$candidate/build/sprite_policy_native_icm_shadow"
ln -sfn "$libdir/libonnxruntime.so.1.23.2" "$candidate/build/libonnxruntime.so.1"
dependencies=$(ldd "$candidate/build/sprite_policy_native_icm_shadow")
printf '%s\n' "$dependencies"
if grep -q 'not found' <<< "$dependencies"; then exit 1; fi
echo 'ICM zero-gain-only actor built; no powered actor changed.'
