#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
libdir=$(.venv/bin/python -c 'import onnxruntime,pathlib; print(pathlib.Path(onnxruntime.__file__).parent/"capi")')
stage=build/policy_native_stage2
g++ -std=c++17 -O2 -ffp-contract=off -Wall -Wextra -Werror -pthread \
  -DSPRITE_LOOP_TEST -DSPRITE_POWERED_LOOP -DSPRITE_SOFTWARE_IMU_INPUT \
  -I"$stage/include" native/src/sprite_policy_replay.cpp \
  "$libdir/libonnxruntime.so.1.23.2" -Wl,-rpath,'$ORIGIN' \
  -o "$stage/sprite_policy_powered_candidate"
echo 'Compiled candidate only; no CAN, IMU or powered execution.'
