#!/usr/bin/env bash
set -euo pipefail
cd /home/tony/open_sprite_runtime
c="$PWD/candidates/xbox_native_20261019"
libdir=$(.venv/bin/python -c 'import onnxruntime,pathlib; print(pathlib.Path(onnxruntime.__file__).parent/"capi")')
g++ -std=c++17 -O2 -ffp-contract=off -Wall -Wextra -Werror -pthread \
  -DSPRITE_LOOP_TEST -DSPRITE_SHADOW_LOOP -DSPRITE_ICM_IMU_INPUT -DSPRITE_GAMEPAD_INPUT \
  -Ibuild/policy_native_stage2/include -I"$c/src" -Icandidates/icm_native_20261016/native/src \
  "$c/src/sprite_policy_replay.cpp" "$libdir/libonnxruntime.so.1.23.2" -Wl,-rpath,'$ORIGIN' \
  -o "$c/sprite_policy_xbox_shadow"
ln -sfn "$libdir/libonnxruntime.so.1.23.2" "$c/libonnxruntime.so.1"
echo 'SHADOW ONLY: gamepad build cannot be compiled as powered actor.'
