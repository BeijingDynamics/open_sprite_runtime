#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
CONTRACT=/home/tony/sprite_runtime/sprite0825_stage2_g74_model3000_sim2real_candidate_kp130_kd5_20261010_arms7_kd3_20261012_follow_v1_waist50_hiproll150_waist70_kd3_kd35_waist60kd3/deploy/contract.json
for stamp in 155525 155850; do
    PYTHONPATH=src OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 taskset -c 4 \
        .venv/bin/python tools/verify_native_policy_chain.py \
        --contract "$CONTRACT" --hardware config/hardware.sprite0825.kp110_kd5.json \
        --limits artifacts/g60_model3450/reports/sprite0825_urdf_limit_candidates.json \
        --actor-report "reports/native_protected_policy_actor_20261013_${stamp}.json" \
        --trace "reports/native_protected_policy_trace_20261013_${stamp}.npz" \
        --binary build/policy_native_stage2/sprite_policy_replay \
        --output-dir "reports/native_policy_chain_${stamp}_v2"
done
