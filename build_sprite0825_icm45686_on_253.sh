#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"
headers="$PWD/native/src"
if [[ ! -f "$headers/imu_six_axis.hpp" ]]; then
  headers="$PWD/candidates/bno_software_20261014/native/src"
fi
test -f "$headers/imu_six_axis.hpp"
cmake -S native/icm45686 -B build/icm45686_candidate -DCMAKE_BUILD_TYPE=Release -DSPRITE_IMU_INCLUDE_DIR="$headers"
cmake --build build/icm45686_candidate -j2
ctest --test-dir build/icm45686_candidate --output-on-failure
