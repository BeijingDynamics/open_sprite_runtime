#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
runtime_root=${1:-$PWD}
libdir=$("$runtime_root/.venv/bin/python" -c 'import onnxruntime,pathlib; print(pathlib.Path(onnxruntime.__file__).parent/"capi")')
mkdir -p build/imu_software_candidate
g++ -std=c++17 -O2 -ffp-contract=off -Wall -Wextra -Werror -pthread \
  -DSPRITE_LOOP_TEST -DSPRITE_SHADOW_LOOP -DSPRITE_SOFTWARE_IMU_INPUT \
  -I"$runtime_root/build/policy_native_stage2/include" native/src/sprite_policy_replay.cpp \
  "$libdir/libonnxruntime.so.1.23.2" -Wl,-rpath,"$runtime_root/build/policy_native_stage2:$libdir" \
  -o build/imu_software_candidate/sprite_policy_native_software_shadow
echo 'Built software-IMU zero-gain shadow consumer only. No hardware run.'
