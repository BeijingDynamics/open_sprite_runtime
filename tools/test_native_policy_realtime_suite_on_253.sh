#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
model=/home/tony/sprite_runtime/sprite0825_stage2_g74_model3000_sim2real_candidate_kp130_kd5_20261010_arms7_kd3_20261012_follow_v1_waist50_hiproll150_waist70_kd3_kd35_waist60kd3/deploy/policy.onnx
for fault in imu_freeze state_disconnect none; do
    PYTHONPATH=src .venv/bin/python tools/test_native_policy_realtime_loop.py \
        --binary build/policy_native_stage2/sprite_policy_loop_test \
        --offline-binary build/policy_native_stage2/sprite_policy_replay --model "$model" \
        --fixture reports/native_policy_chain_155850_v2/input.bin \
        --output-dir "reports/native_policy_realtime_${fault}_v2" --fault "$fault"
done
