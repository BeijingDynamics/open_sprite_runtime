#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
mkdir -p build/policy_native_stage2
for component in feedback protection; do
    g++ -std=c++17 -O2 -ffp-contract=off -Wall -Wextra -Werror \
        "native/src/sprite_${component}_replay.cpp" \
        -o "build/policy_native_stage2/sprite_${component}_replay"
done
g++ -std=c++17 -O2 -Wall -Wextra -Werror -pthread \
    native/src/sprite_imu_ipc_probe.cpp \
    -o build/policy_native_stage2/sprite_imu_ipc_probe
printf '%s\n' 'Offline math replays built. No live binaries or capabilities modified.'
g++ -std=c++17 -O2 -Wall -Wextra -Werror -pthread \
    native/src/sprite_state_ipc_probe.cpp \
    -o build/policy_native_stage2/sprite_state_ipc_probe
