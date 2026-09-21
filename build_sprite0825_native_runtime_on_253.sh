#!/usr/bin/env bash
set -euo pipefail

ROOT=/home/tony/open_sprite_runtime
BUILD="$ROOT/build/native"

cmake -S "$ROOT/native" -B "$BUILD" -DCMAKE_BUILD_TYPE=Release
cmake --build "$BUILD" --parallel
echo "BUILT $BUILD/sprite_rt_timing"
echo "BUILT $BUILD/sprite_can_shadow"
