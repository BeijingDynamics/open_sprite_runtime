#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
stage=build/imu_software_candidate
mkdir -p "$stage"
g++ -std=c++17 -O2 -Wall -Wextra -Werror -pthread -Inative/src \
  native/tests/test_imu_six_axis.cpp -o "$stage/test_imu_six_axis"
"$stage/test_imu_six_axis"
g++ -std=c++17 -O2 -Wall -Wextra -Werror -DSPRITE_BNO_SOFTWARE_FUSION \
  native/src/sprite_imu_bno.cpp -o "$stage/sprite_imu_software"
# Compile the old path too, without overwriting its deployed binary.
g++ -std=c++17 -O2 -Wall -Wextra -Werror \
  native/src/sprite_imu_bno.cpp -o "$stage/sprite_imu_hardware_regression"
echo 'Candidate built and offline tests passed. No hardware accessed.'
