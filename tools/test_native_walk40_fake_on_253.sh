#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
model=/home/tony/sprite_runtime/sprite0825_stage2_g74_model3000_sim2real_candidate_kp130_kd5_20261010_arms7_kd3_20261012_follow_v1_waist50_hiproll150_waist70_kd3_kd35_waist60kd3/deploy/policy.onnx
output=$(mktemp -d reports/native_walk40_fake_XXXXXXXX)
binary=build/policy_native_stage2/sprite_policy_loop_test
extra=()
case "${1:-}" in
  '') ;;
  --powered-candidate) binary=build/policy_native_stage2/sprite_policy_powered_candidate; extra=(--powered-candidate) ;;
  *) exit 2 ;;
esac
echo "NO HARDWARE: fake Unix socket peers only; reports=$output"
for fault in imu_freeze state_disconnect none; do
  PYTHONPATH=src .venv/bin/python tools/test_native_policy_realtime_loop.py \
    --binary "$binary" "${extra[@]}" \
    --offline-binary build/policy_native_stage2/sprite_policy_replay \
    --model "$model" --fixture reports/software_imu_walk_native_parity_131201/input.bin \
    --output-dir "$output/$fault" --fault "$fault" \
    --command-profile stand-walk010-stop-40s
done
