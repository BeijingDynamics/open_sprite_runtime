#!/usr/bin/env bash
set -euo pipefail
root=/home/tony/open_sprite_runtime
mode="${1:-}"
case "$mode" in
  preflight)
    export SPRITE_PREFLIGHT_ONLY=1
    ;;
  ENABLE_STAND_WALK_STOP_40S)
    unset SPRITE_PREFLIGHT_ONLY
    ;;
  *)
    echo 'Usage: bash review_sprite0825_walk005_stop_on_253.sh preflight|ENABLE_STAND_WALK_STOP_40S'
    echo 'The active mode enables all motors; operator and fall arrest are required.'
    exit 2
    ;;
esac
exec bash "$root/run_sprite0825_native_protected_policy_admission_on_253.sh" \
  ENABLE_NATIVE_PROTECTED_POLICY_WALK005_STOP_40S grounded_full_weight_walk005_stop_40s_tier
