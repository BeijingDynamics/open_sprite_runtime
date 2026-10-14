#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
stage=build/policy_native_stage2
mkdir -p "$stage/include"
# Match the installed Python runtime, never silently upgrade a deployed runtime.
version=$(.venv/bin/python -c 'import onnxruntime; print(onnxruntime.__version__)')
[[ "$version" == 1.23.2 ]] || { echo "Expected ORT1.23.2, found $version" >&2; exit 1; }
for name in onnxruntime_c_api.h onnxruntime_cxx_api.h onnxruntime_cxx_inline.h onnxruntime_float16.h onnxruntime_ep_c_api.h; do
  [[ -s "$stage/include/$name" ]] && continue
  curl --fail --location --max-time 60 --retry 2 "https://raw.githubusercontent.com/microsoft/onnxruntime/v1.23.2/include/onnxruntime/core/session/$name" -o "$stage/include/$name"
done
libdir=$(.venv/bin/python -c 'import onnxruntime,pathlib; print(pathlib.Path(onnxruntime.__file__).parent/"capi")')
ln -sfn "$libdir/libonnxruntime.so.1.23.2" "$stage/libonnxruntime.so.1"
g++ -std=c++17 -O2 -ffp-contract=off -Wall -Wextra -Werror -I"$stage/include" native/src/sprite_policy_replay.cpp \
  "$libdir/libonnxruntime.so.1.23.2" -Wl,-rpath,'$ORIGIN' -o "$stage/sprite_policy_replay"
echo "Built offline policy only; CAN binary and capabilities untouched"
