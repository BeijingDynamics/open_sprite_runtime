#!/usr/bin/env bash
set -euo pipefail

ACK="${1:-}"
TIER="${2:-first_admission}"
COMMAND_VX=0.0
LEG_COMMAND_CAP_NM=""
LEG_FEEDBACK_CAP_NM=""
ANKLE_COMMAND_CAP_NM=""
ANKLE_FEEDBACK_CAP_NM=""
HIP_PITCH_ROLL_COMMAND_CAP_NM=""
HIP_PITCH_ROLL_FEEDBACK_CAP_NM=""
PREFLIGHT_MAXIMUM_GATED_OVERSHOOT_RAD=0.01
PREFLIGHT_MAXIMUM_GATED_VIOLATION_FRACTION=0.01
PREFLIGHT_MAXIMUM_GATED_CONSECUTIVE_TICKS=2
NON_HIP_GAIN_MULTIPLIER=1.0
LEG_GAIN_MULTIPLIER=""
EXTENDED_NATIVE_ACK_ARGS=()
CLAMP_WATCHDOG_ARGS=()
SUPPORT_INSTRUCTION="Robot must remain suspended"

case "$TIER" in
  first_admission)
    EXPECTED_ACK=ENABLE_NATIVE_PROTECTED_POLICY_ACTUATION
    DURATION=2.0
    GAIN_SCALE=0.02
    DM3507_GAIN_MULTIPLIER=1.0
    MAXIMUM_COMMAND_TORQUE_NM=1.4
    ;;
  full_ramp_dm3507_tier)
    EXPECTED_ACK=ENABLE_NATIVE_PROTECTED_POLICY_FULL_RAMP
    DURATION=6.0
    GAIN_SCALE=0.015
    DM3507_GAIN_MULTIPLIER=0.1
    MAXIMUM_COMMAND_TORQUE_NM=1.4
    ;;
  full_ramp_02nm_tier)
    EXPECTED_ACK=ENABLE_NATIVE_PROTECTED_POLICY_02NM_FULL_RAMP
    DURATION=6.0
    GAIN_SCALE=0.006
    DM3507_GAIN_MULTIPLIER=0.1
    MAXIMUM_COMMAND_TORQUE_NM=0.2
    ;;
  full_ramp_05nm_tier)
    EXPECTED_ACK=ENABLE_NATIVE_PROTECTED_POLICY_05NM_FULL_RAMP
    DURATION=6.0
    GAIN_SCALE=0.015
    DM3507_GAIN_MULTIPLIER=0.1
    MAXIMUM_COMMAND_TORQUE_NM=0.5
    ;;
  stand_10nm_tier)
    EXPECTED_ACK=ENABLE_NATIVE_PROTECTED_POLICY_10NM_STAND
    DURATION=8.0
    GAIN_SCALE=0.02
    DM3507_GAIN_MULTIPLIER=0.1
    MAXIMUM_COMMAND_TORQUE_NM=1.0
    ;;
  stand_leg_20nm_tier)
    EXPECTED_ACK=ENABLE_NATIVE_PROTECTED_POLICY_LEG_20NM_STAND
    DURATION=8.0
    GAIN_SCALE=0.04
    DM3507_GAIN_MULTIPLIER=0.1
    MAXIMUM_COMMAND_TORQUE_NM=1.0
    LEG_COMMAND_CAP_NM=2.0
    LEG_FEEDBACK_CAP_NM=2.2
    ;;
  stand_leg_20nm_balance_20s_tier)
    EXPECTED_ACK=ENABLE_NATIVE_PROTECTED_POLICY_LEG_20NM_BALANCE_20S
    DURATION=20.0
    GAIN_SCALE=0.04
    DM3507_GAIN_MULTIPLIER=0.1
    MAXIMUM_COMMAND_TORQUE_NM=1.0
    LEG_COMMAND_CAP_NM=2.0
    LEG_FEEDBACK_CAP_NM=2.2
    EXTENDED_NATIVE_ACK_ARGS=(
      --extended-policy-actuation-acknowledgement
      ENABLE_20_SECOND_SUSPENDED_BALANCE_TEST
    )
    ;;
  stand_leg_20nm_partial_contact_tier)
    EXPECTED_ACK=ENABLE_NATIVE_PROTECTED_POLICY_LEG_20NM_PARTIAL_CONTACT
    DURATION=8.0
    GAIN_SCALE=0.04
    DM3507_GAIN_MULTIPLIER=0.1
    MAXIMUM_COMMAND_TORQUE_NM=1.0
    LEG_COMMAND_CAP_NM=2.0
    LEG_FEEDBACK_CAP_NM=2.2
    SUPPORT_INSTRUCTION="Lifting frame must carry most weight; both soles may only touch a flat floor"
    for joint in \
      left_ankle_pitch_joint right_ankle_pitch_joint \
      left_ankle_roll_joint right_ankle_roll_joint; do
      CLAMP_WATCHDOG_ARGS+=(--fail-on-consecutive-clamp-joint "$joint")
    done
    CLAMP_WATCHDOG_ARGS+=(
      --clamp-watchdog-minimum-overshoot-rad 0.05
      --clamp-watchdog-maximum-consecutive-ticks 5
    )
    ;;
  stand_leg_20nm_partial_contact_20s_tier)
    EXPECTED_ACK=ENABLE_NATIVE_PROTECTED_POLICY_LEG_20NM_PARTIAL_CONTACT_20S
    DURATION=20.0
    GAIN_SCALE=0.04
    DM3507_GAIN_MULTIPLIER=0.1
    MAXIMUM_COMMAND_TORQUE_NM=1.0
    LEG_COMMAND_CAP_NM=2.0
    LEG_FEEDBACK_CAP_NM=2.2
    SUPPORT_INSTRUCTION="Lifting frame must carry most weight; both soles may only touch a flat floor"
    EXTENDED_NATIVE_ACK_ARGS=(
      --extended-policy-actuation-acknowledgement
      ENABLE_20_SECOND_SUSPENDED_BALANCE_TEST
    )
    for joint in \
      left_ankle_pitch_joint right_ankle_pitch_joint \
      left_ankle_roll_joint right_ankle_roll_joint; do
      CLAMP_WATCHDOG_ARGS+=(--fail-on-consecutive-clamp-joint "$joint")
    done
    CLAMP_WATCHDOG_ARGS+=(
      --clamp-watchdog-minimum-overshoot-rad 0.05
      --clamp-watchdog-maximum-consecutive-ticks 5
    )
    ;;
  stand_hip_pr_45nm_partial_contact_tier)
    EXPECTED_ACK=ENABLE_NATIVE_PROTECTED_POLICY_HIP_PR_45NM_PARTIAL_CONTACT
    DURATION=8.0
    GAIN_SCALE=0.06
    DM3507_GAIN_MULTIPLIER=0.1
    MAXIMUM_COMMAND_TORQUE_NM=1.0
    LEG_COMMAND_CAP_NM=2.0
    LEG_FEEDBACK_CAP_NM=2.2
    HIP_PITCH_ROLL_COMMAND_CAP_NM=4.5
    HIP_PITCH_ROLL_FEEDBACK_CAP_NM=5.0
    SUPPORT_INSTRUCTION="Lifting frame must retain the previously qualified support height; both soles remain on a flat floor; no disturbance"
    for joint in \
      left_ankle_pitch_joint right_ankle_pitch_joint \
      left_ankle_roll_joint right_ankle_roll_joint; do
      CLAMP_WATCHDOG_ARGS+=(--fail-on-consecutive-clamp-joint "$joint")
    done
    CLAMP_WATCHDOG_ARGS+=(
      --clamp-watchdog-minimum-overshoot-rad 0.05
      --clamp-watchdog-maximum-consecutive-ticks 5
    )
    ;;
  stand_hip_pr_45nm_gain08_partial_contact_tier)
    EXPECTED_ACK=ENABLE_NATIVE_PROTECTED_POLICY_HIP_PR_45NM_GAIN08_PARTIAL_CONTACT
    DURATION=8.0
    GAIN_SCALE=0.08
    DM3507_GAIN_MULTIPLIER=0.075
    NON_HIP_GAIN_MULTIPLIER=0.75
    MAXIMUM_COMMAND_TORQUE_NM=1.0
    LEG_COMMAND_CAP_NM=2.0
    LEG_FEEDBACK_CAP_NM=2.2
    HIP_PITCH_ROLL_COMMAND_CAP_NM=4.5
    HIP_PITCH_ROLL_FEEDBACK_CAP_NM=5.0
    SUPPORT_INSTRUCTION="Lifting frame must retain the previously qualified support height; both soles remain on a flat floor; no disturbance"
    for joint in \
      left_ankle_pitch_joint right_ankle_pitch_joint \
      left_ankle_roll_joint right_ankle_roll_joint; do
      CLAMP_WATCHDOG_ARGS+=(--fail-on-consecutive-clamp-joint "$joint")
    done
    CLAMP_WATCHDOG_ARGS+=(
      --clamp-watchdog-minimum-overshoot-rad 0.05
      --clamp-watchdog-maximum-consecutive-ticks 5
    )
    ;;
  stand_hip_pr_45nm_gain08_partial_contact_20s_tier)
    EXPECTED_ACK=ENABLE_NATIVE_PROTECTED_POLICY_HIP_PR_45NM_GAIN08_PARTIAL_CONTACT_20S
    DURATION=20.0
    GAIN_SCALE=0.08
    DM3507_GAIN_MULTIPLIER=0.075
    NON_HIP_GAIN_MULTIPLIER=0.75
    MAXIMUM_COMMAND_TORQUE_NM=1.0
    LEG_COMMAND_CAP_NM=2.0
    LEG_FEEDBACK_CAP_NM=2.2
    HIP_PITCH_ROLL_COMMAND_CAP_NM=4.5
    HIP_PITCH_ROLL_FEEDBACK_CAP_NM=5.0
    SUPPORT_INSTRUCTION="Lifting frame must retain the previously qualified support height; both soles remain on a flat floor; no disturbance"
    EXTENDED_NATIVE_ACK_ARGS=(
      --extended-policy-actuation-acknowledgement
      ENABLE_20_SECOND_SUSPENDED_BALANCE_TEST
    )
    for joint in \
      left_ankle_pitch_joint right_ankle_pitch_joint \
      left_ankle_roll_joint right_ankle_roll_joint; do
      CLAMP_WATCHDOG_ARGS+=(--fail-on-consecutive-clamp-joint "$joint")
    done
    CLAMP_WATCHDOG_ARGS+=(
      --clamp-watchdog-minimum-overshoot-rad 0.05
      --clamp-watchdog-maximum-consecutive-ticks 5
    )
    ;;
  stand_hip_pr_45nm_gain08_lowered_harness_static_tier)
    EXPECTED_ACK=ENABLE_NATIVE_PROTECTED_POLICY_HIP_PR_45NM_GAIN08_LOWERED_HARNESS_STATIC
    DURATION=8.0
    GAIN_SCALE=0.08
    DM3507_GAIN_MULTIPLIER=0.075
    NON_HIP_GAIN_MULTIPLIER=0.75
    MAXIMUM_COMMAND_TORQUE_NM=1.0
    LEG_COMMAND_CAP_NM=2.0
    LEG_FEEDBACK_CAP_NM=2.2
    HIP_PITCH_ROLL_COMMAND_CAP_NM=4.5
    HIP_PITCH_ROLL_FEEDBACK_CAP_NM=5.0
    SUPPORT_INSTRUCTION="Lifting frame has been lowered slightly from the qualified height; both soles remain on a flat floor; no disturbance"
    for joint in \
      left_ankle_pitch_joint right_ankle_pitch_joint \
      left_ankle_roll_joint right_ankle_roll_joint; do
      CLAMP_WATCHDOG_ARGS+=(--fail-on-consecutive-clamp-joint "$joint")
    done
    CLAMP_WATCHDOG_ARGS+=(
      --clamp-watchdog-minimum-overshoot-rad 0.05
      --clamp-watchdog-maximum-consecutive-ticks 5
    )
    ;;
  stand_leg_gain08_lowered_harness_static_tier)
    EXPECTED_ACK=ENABLE_NATIVE_PROTECTED_POLICY_LEG_GAIN08_LOWERED_HARNESS_STATIC
    DURATION=8.0
    GAIN_SCALE=0.08
    DM3507_GAIN_MULTIPLIER=0.075
    NON_HIP_GAIN_MULTIPLIER=0.75
    LEG_GAIN_MULTIPLIER=1.0
    MAXIMUM_COMMAND_TORQUE_NM=1.0
    LEG_COMMAND_CAP_NM=2.0
    LEG_FEEDBACK_CAP_NM=2.2
    HIP_PITCH_ROLL_COMMAND_CAP_NM=4.5
    HIP_PITCH_ROLL_FEEDBACK_CAP_NM=5.0
    SUPPORT_INSTRUCTION="Lifting frame remains at the adjusted intermediate height; both soles remain on a flat floor; no disturbance"
    for joint in \
      left_ankle_pitch_joint right_ankle_pitch_joint \
      left_ankle_roll_joint right_ankle_roll_joint; do
      CLAMP_WATCHDOG_ARGS+=(--fail-on-consecutive-clamp-joint "$joint")
    done
    CLAMP_WATCHDOG_ARGS+=(
      --clamp-watchdog-minimum-overshoot-rad 0.05
      --clamp-watchdog-maximum-consecutive-ticks 5
    )
    ;;
  stand_leg_gain08_lower_support_leg22_static_tier)
    EXPECTED_ACK=ENABLE_NATIVE_PROTECTED_POLICY_LEG_GAIN08_LOWER_SUPPORT_LEG22_STATIC
    DURATION=8.0
    GAIN_SCALE=0.08
    DM3507_GAIN_MULTIPLIER=0.075
    NON_HIP_GAIN_MULTIPLIER=0.75
    LEG_GAIN_MULTIPLIER=1.0
    MAXIMUM_COMMAND_TORQUE_NM=1.0
    LEG_COMMAND_CAP_NM=2.2
    LEG_FEEDBACK_CAP_NM=2.5
    HIP_PITCH_ROLL_COMMAND_CAP_NM=4.5
    HIP_PITCH_ROLL_FEEDBACK_CAP_NM=5.0
    SUPPORT_INSTRUCTION="Lifting frame is at the newly lowered height; both soles remain on a flat floor; no disturbance"
    for joint in \
      left_ankle_pitch_joint right_ankle_pitch_joint \
      left_ankle_roll_joint right_ankle_roll_joint; do
      CLAMP_WATCHDOG_ARGS+=(--fail-on-consecutive-clamp-joint "$joint")
    done
    CLAMP_WATCHDOG_ARGS+=(
      --clamp-watchdog-minimum-overshoot-rad 0.05
      --clamp-watchdog-maximum-consecutive-ticks 5
    )
    ;;
  stand_leg_gain08_lower_support_leg22_static_20s_tier)
    EXPECTED_ACK=ENABLE_NATIVE_PROTECTED_POLICY_LEG_GAIN08_LOWER_SUPPORT_LEG22_STATIC_20S
    DURATION=20.0
    GAIN_SCALE=0.08
    DM3507_GAIN_MULTIPLIER=0.075
    NON_HIP_GAIN_MULTIPLIER=0.75
    LEG_GAIN_MULTIPLIER=1.0
    MAXIMUM_COMMAND_TORQUE_NM=1.0
    LEG_COMMAND_CAP_NM=2.2
    LEG_FEEDBACK_CAP_NM=2.5
    HIP_PITCH_ROLL_COMMAND_CAP_NM=4.5
    HIP_PITCH_ROLL_FEEDBACK_CAP_NM=5.0
    SUPPORT_INSTRUCTION="Lifting frame remains at the qualified lower-support height; both soles remain on a flat floor; no disturbance"
    EXTENDED_NATIVE_ACK_ARGS=(
      --extended-policy-actuation-acknowledgement
      ENABLE_20_SECOND_SUSPENDED_BALANCE_TEST
    )
    for joint in \
      left_ankle_pitch_joint right_ankle_pitch_joint \
      left_ankle_roll_joint right_ankle_roll_joint; do
      CLAMP_WATCHDOG_ARGS+=(--fail-on-consecutive-clamp-joint "$joint")
    done
    CLAMP_WATCHDOG_ARGS+=(
      --clamp-watchdog-minimum-overshoot-rad 0.05
      --clamp-watchdog-maximum-consecutive-ticks 5
    )
    ;;
  stand_leg_gain08_lower_support_recovery_tier)
    EXPECTED_ACK=ENABLE_NATIVE_PROTECTED_POLICY_LEG_GAIN08_LOWER_SUPPORT_RECOVERY
    DURATION=8.0
    GAIN_SCALE=0.08
    DM3507_GAIN_MULTIPLIER=0.075
    NON_HIP_GAIN_MULTIPLIER=0.75
    LEG_GAIN_MULTIPLIER=1.0
    MAXIMUM_COMMAND_TORQUE_NM=1.0
    LEG_COMMAND_CAP_NM=3.0
    LEG_FEEDBACK_CAP_NM=3.5
    ANKLE_COMMAND_CAP_NM=2.2
    ANKLE_FEEDBACK_CAP_NM=2.5
    HIP_PITCH_ROLL_COMMAND_CAP_NM=4.5
    HIP_PITCH_ROLL_FEEDBACK_CAP_NM=5.0
    PREFLIGHT_MAXIMUM_GATED_OVERSHOOT_RAD=0.065
    PREFLIGHT_MAXIMUM_GATED_VIOLATION_FRACTION=1.0
    PREFLIGHT_MAXIMUM_GATED_CONSECUTIVE_TICKS=300
    SUPPORT_INSTRUCTION="Lifting frame is at the newly lowered recovery height; both soles remain on a flat floor; no disturbance"
    for joint in \
      left_ankle_pitch_joint right_ankle_pitch_joint \
      left_ankle_roll_joint right_ankle_roll_joint; do
      CLAMP_WATCHDOG_ARGS+=(--fail-on-consecutive-clamp-joint "$joint")
    done
    CLAMP_WATCHDOG_ARGS+=(
      --clamp-watchdog-minimum-overshoot-rad 0.05
      --clamp-watchdog-maximum-consecutive-ticks 5
      --clamp-watchdog-ignored-initial-ticks 250
    )
    ;;
  stand_leg_gain08_lowered_harness_static_20s_tier)
    EXPECTED_ACK=ENABLE_NATIVE_PROTECTED_POLICY_LEG_GAIN08_LOWERED_HARNESS_STATIC_20S
    DURATION=20.0
    GAIN_SCALE=0.08
    DM3507_GAIN_MULTIPLIER=0.075
    NON_HIP_GAIN_MULTIPLIER=0.75
    LEG_GAIN_MULTIPLIER=1.0
    MAXIMUM_COMMAND_TORQUE_NM=1.0
    LEG_COMMAND_CAP_NM=2.0
    LEG_FEEDBACK_CAP_NM=2.2
    HIP_PITCH_ROLL_COMMAND_CAP_NM=4.5
    HIP_PITCH_ROLL_FEEDBACK_CAP_NM=5.0
    SUPPORT_INSTRUCTION="Lifting frame remains at the qualified intermediate height; both soles remain on a flat floor; no disturbance"
    EXTENDED_NATIVE_ACK_ARGS=(
      --extended-policy-actuation-acknowledgement
      ENABLE_20_SECOND_SUSPENDED_BALANCE_TEST
    )
    for joint in \
      left_ankle_pitch_joint right_ankle_pitch_joint \
      left_ankle_roll_joint right_ankle_roll_joint; do
      CLAMP_WATCHDOG_ARGS+=(--fail-on-consecutive-clamp-joint "$joint")
    done
    CLAMP_WATCHDOG_ARGS+=(
      --clamp-watchdog-minimum-overshoot-rad 0.05
      --clamp-watchdog-maximum-consecutive-ticks 5
    )
    ;;
  suspended_walk_10nm_tier)
    EXPECTED_ACK=ENABLE_NATIVE_PROTECTED_POLICY_10NM_SUSPENDED_WALK
    DURATION=8.0
    GAIN_SCALE=0.018
    DM3507_GAIN_MULTIPLIER=0.1
    MAXIMUM_COMMAND_TORQUE_NM=1.0
    COMMAND_VX=0.15
    ;;
  *)
    echo "Unknown protected-policy tier: $TIER" >&2
    exit 2
    ;;
esac
[[ -n "$LEG_GAIN_MULTIPLIER" ]] || LEG_GAIN_MULTIPLIER="$NON_HIP_GAIN_MULTIPLIER"
[[ "$ACK" == "$EXPECTED_ACK" ]] || {
  echo "Exact acknowledgement required: $EXPECTED_ACK" >&2
  exit 2
}

ROOT=/home/tony/open_sprite_runtime
CANDIDATE=/home/tony/sprite_runtime/sprite0825_stage2_g74_model3000_sim2real_candidate
STARTUP_HOLD_SECONDS=1.0
STARTUP_RAMP_SECONDS=4.0
STAMP="$(date +%Y%m%d_%H%M%S)"
SOCKET="/tmp/open_sprite_policy_active_${$}.sock"
NATIVE_REPORT="$ROOT/reports/native_protected_policy_admission_${STAMP}.json"
POLICY_REPORT="$ROOT/reports/native_protected_policy_actor_${STAMP}.json"
POLICY_TRACE="$ROOT/reports/native_protected_policy_trace_${STAMP}.npz"
NATIVE_LOG="$ROOT/reports/native_protected_policy_admission_${STAMP}.log"
JOINT_LIMITS="$ROOT/artifacts/g60_model3450/reports/sprite0825_urdf_limit_candidates.json"
[[ -f "$JOINT_LIMITS" ]] || JOINT_LIMITS="$ROOT/reports/sprite0825_urdf_limit_candidates.json"
PREFLIGHT_LOG="$ROOT/reports/native_protected_policy_preflight_${STAMP}.log"
PREFLIGHT_REPORT="$ROOT/reports/native_protected_policy_preflight_${STAMP}.json"

mkdir -p "$ROOT/build/native" "$ROOT/reports"
getcap "$ROOT/build/native/sprite_can_shadow" | grep -q 'cap_sys_nice' || {
  echo "Native runtime lacks CAP_SYS_NICE; refuse active policy control" >&2
  echo "Reinstall the reviewed capability after every rebuild" >&2
  exit 1
}
[[ -f "$JOINT_LIMITS" ]] || {
  echo "Joint limit report not found: $JOINT_LIMITS" >&2
  exit 1
}

echo "ZERO-GAIN STARTUP READINESS PREFLIGHT: 6.0s"
echo "Warms policy history for 1.0s, then requires ankle excursions <=0.01rad, <=1% ticks, <=2 consecutive ticks; horizontal projected gravity <=0.10"
"$ROOT/probe_sprite0825_native_policy_ipc_shadow_on_253.sh" \
  6.0 "$GAIN_SCALE" 0 0 "$DM3507_GAIN_MULTIPLIER" "$COMMAND_VX" \
  "$NON_HIP_GAIN_MULTIPLIER" "$LEG_GAIN_MULTIPLIER" | tee "$PREFLIGHT_LOG"
PREFLIGHT_TRACE="$(awk '/^REPLAYABLE_TRACE / {print $2}' "$PREFLIGHT_LOG" | tail -1)"
PREFLIGHT_NATIVE_REPORT="$(awk '/^DURATION / {for (i=1; i<=NF; ++i) if ($i == "NATIVE_REPORT") {gsub(/;/, "", $(i+1)); print $(i+1)}}' "$PREFLIGHT_LOG" | tail -1)"
[[ -n "$PREFLIGHT_TRACE" && -f "$PREFLIGHT_TRACE" ]] || {
  echo "Startup readiness preflight did not produce a trace" >&2
  exit 1
}
[[ -n "$PREFLIGHT_NATIVE_REPORT" && -f "$PREFLIGHT_NATIVE_REPORT" ]] || {
  echo "Startup readiness preflight did not produce a native report" >&2
  exit 1
}
"$ROOT/.venv/bin/python" - \
  "$PREFLIGHT_NATIVE_REPORT" \
  "$MAXIMUM_COMMAND_TORQUE_NM" \
  "$LEG_COMMAND_CAP_NM" \
  "$HIP_PITCH_ROLL_COMMAND_CAP_NM" \
  "$ANKLE_COMMAND_CAP_NM" <<'PY'
import json
import sys

report = json.load(open(sys.argv[1], encoding="utf-8"))
default_limit = float(sys.argv[2])
leg_limit = float(sys.argv[3]) if sys.argv[3] else None
hip_pitch_roll_limit = float(sys.argv[4]) if sys.argv[4] else None
ankle_limit = float(sys.argv[5]) if sys.argv[5] else None
leg_motors = {
    f"{side}_{joint}_motor"
    for side in ("left", "right")
    for joint in ("hip_pitch", "hip_roll", "hip_yaw", "knee")
}
leg_motors.update(
    f"{side}_ankle_motor_{motor}"
    for side in ("left", "right")
    for motor in ("a", "b")
)
violations = []
for name, raw_observed in report[
    "preview_maximum_abs_estimated_torque_nm_by_motor"
].items():
    observed = float(raw_observed)
    if ankle_limit is not None and name in {
        "left_ankle_motor_a",
        "left_ankle_motor_b",
        "right_ankle_motor_a",
        "right_ankle_motor_b",
    }:
        limit = ankle_limit
    elif hip_pitch_roll_limit is not None and name in {
        "left_hip_pitch_motor",
        "left_hip_roll_motor",
        "right_hip_pitch_motor",
        "right_hip_roll_motor",
    }:
        limit = hip_pitch_roll_limit
    else:
        limit = leg_limit if leg_limit is not None and name in leg_motors else default_limit
    if observed > limit:
        violations.append(f"{name}={observed:.6f}>{limit:.6f}Nm")
if violations:
    raise SystemExit(
        "startup command torque preview exceeds per-motor limits: "
        + ", ".join(violations)
    )
print(
    "STARTUP_COMMAND_TORQUE_PASSED "
    f"maximum={report['preview_maximum_abs_estimated_torque_nm']:.6f}Nm "
    f"default_limit={default_limit:.6f}Nm leg_limit={leg_limit} "
    f"hip_pitch_roll_limit={hip_pitch_roll_limit} ankle_limit={ankle_limit}"
)
PY
PYTHONPATH="$ROOT/src" "$ROOT/.venv/bin/python" \
  "$ROOT/tools/analyze_policy_joint_limit_clamps.py" \
  --trace "$PREFLIGHT_TRACE" \
  --joint-limit-candidates "$JOINT_LIMITS" \
  --contract "$CANDIDATE/deploy/contract.json" \
  --fail-on-violation-joint left_ankle_pitch_joint \
  --fail-on-violation-joint right_ankle_pitch_joint \
  --fail-on-violation-joint left_ankle_roll_joint \
  --fail-on-violation-joint right_ankle_roll_joint \
  --maximum-gated-overshoot-rad "$PREFLIGHT_MAXIMUM_GATED_OVERSHOOT_RAD" \
  --maximum-gated-violation-fraction "$PREFLIGHT_MAXIMUM_GATED_VIOLATION_FRACTION" \
  --maximum-gated-consecutive-violation-ticks "$PREFLIGHT_MAXIMUM_GATED_CONSECUTIVE_TICKS" \
  --ignore-initial-ticks 50 \
  --maximum-horizontal-gravity-norm 0.10 \
  --output "$PREFLIGHT_REPORT" >/dev/null
echo "STARTUP_READINESS_PASSED report=$PREFLIGHT_REPORT"

JOINT_GAIN_ARGS=()
if [[ "$DM3507_GAIN_MULTIPLIER" != "1.0" ]]; then
  for joint in \
    head_pitch_joint head_roll_joint head_yaw_joint \
    left_wrist_pitch_joint left_wrist_roll_joint \
    right_wrist_pitch_joint right_wrist_roll_joint; do
    JOINT_GAIN_ARGS+=(--joint-gain-multiplier "$joint=$DM3507_GAIN_MULTIPLIER")
  done
fi
if [[ "$NON_HIP_GAIN_MULTIPLIER" != "1.0" ]]; then
  for joint in \
    waist_roll_joint waist_yaw_joint \
    left_shoulder_pitch_joint right_shoulder_pitch_joint \
    left_shoulder_roll_joint right_shoulder_roll_joint \
    left_shoulder_yaw_joint right_shoulder_yaw_joint \
    left_elbow_joint right_elbow_joint \
    left_wrist_yaw_joint right_wrist_yaw_joint; do
    JOINT_GAIN_ARGS+=(--joint-gain-multiplier "$joint=$NON_HIP_GAIN_MULTIPLIER")
  done
fi
if [[ "$LEG_GAIN_MULTIPLIER" != "1.0" ]]; then
  for joint in \
    left_hip_yaw_joint right_hip_yaw_joint \
    left_knee_joint right_knee_joint \
    left_ankle_pitch_joint right_ankle_pitch_joint \
    left_ankle_roll_joint right_ankle_roll_joint; do
    JOINT_GAIN_ARGS+=(--joint-gain-multiplier "$joint=$LEG_GAIN_MULTIPLIER")
  done
fi
MOTOR_CAP_ARGS=()
if [[ -n "$LEG_COMMAND_CAP_NM" ]]; then
  for motor in \
    left_hip_pitch_motor left_hip_roll_motor left_hip_yaw_motor left_knee_motor \
    left_ankle_motor_a left_ankle_motor_b \
    right_hip_pitch_motor right_hip_roll_motor right_hip_yaw_motor right_knee_motor \
    right_ankle_motor_a right_ankle_motor_b; do
    command_cap="$LEG_COMMAND_CAP_NM"
    feedback_cap="$LEG_FEEDBACK_CAP_NM"
    case "$motor" in
      left_ankle_motor_a|left_ankle_motor_b|right_ankle_motor_a|right_ankle_motor_b)
        if [[ -n "$ANKLE_COMMAND_CAP_NM" ]]; then
          command_cap="$ANKLE_COMMAND_CAP_NM"
          feedback_cap="$ANKLE_FEEDBACK_CAP_NM"
        fi
        ;;
      left_hip_pitch_motor|left_hip_roll_motor|right_hip_pitch_motor|right_hip_roll_motor)
        if [[ -n "$HIP_PITCH_ROLL_COMMAND_CAP_NM" ]]; then
          command_cap="$HIP_PITCH_ROLL_COMMAND_CAP_NM"
          feedback_cap="$HIP_PITCH_ROLL_FEEDBACK_CAP_NM"
        fi
        ;;
    esac
    MOTOR_CAP_ARGS+=(--motor-command-cap "$motor=$command_cap")
    MOTOR_CAP_ARGS+=(--motor-feedback-cap "$motor=$feedback_cap")
  done
fi
PYTHONPATH="$ROOT/src" "$ROOT/.venv/bin/python" \
  "$ROOT/tools/export_native_motor_config.py" \
  --hardware "$ROOT/config/hardware.sprite0825.measurement.json" \
  --maximum-commissioning-torque-nm "$MAXIMUM_COMMAND_TORQUE_NM" \
  "${MOTOR_CAP_ARGS[@]}" \
  --output "$ROOT/build/native/motors.tsv"
PYTHONPATH="$ROOT/src" "$ROOT/.venv/bin/python" \
  "$ROOT/tools/export_native_kinematics_config.py" \
  --hardware "$ROOT/config/hardware.sprite0825.measurement.json" \
  --contract "$CANDIDATE/deploy/contract.json" \
  --output "$ROOT/build/native/kinematics.tsv"
PYTHONPATH="$ROOT/src" "$ROOT/.venv/bin/python" \
  "$ROOT/tools/export_native_joint_safety_config.py" \
  --contract "$CANDIDATE/deploy/contract.json" \
  --joint-limit-candidates "$JOINT_LIMITS" \
  --gain-scale "$GAIN_SCALE" \
  --maximum-embedded-kd 3.0 \
  "${JOINT_GAIN_ARGS[@]}" \
  --output "$ROOT/build/native/joint_safety.tsv"

JOINT_HASH="$(PYTHONPATH="$ROOT/src" "$ROOT/.venv/bin/python" -c \
  'import json,sys; from open_sprite_runtime.native_ipc import ordered_name_hash; d=json.load(open(sys.argv[1])); print(hex(ordered_name_hash(d["joint_names"])))' \
  "$CANDIDATE/deploy/contract.json")"

echo "ACTIVE HARDWARE CONTROL: suspended protected-policy admission"
echo "Fixed tier: name=${TIER} duration=${DURATION}s gain_scale=${GAIN_SCALE} non_hip_multiplier=${NON_HIP_GAIN_MULTIPLIER} leg_multiplier=${LEG_GAIN_MULTIPLIER} DM3507_multiplier=${DM3507_GAIN_MULTIPLIER} vx=${COMMAND_VX} hold=${STARTUP_HOLD_SECONDS}s ramp=${STARTUP_RAMP_SECONDS}s"
if [[ -n "$LEG_COMMAND_CAP_NM" ]]; then
  if [[ -n "$HIP_PITCH_ROLL_COMMAND_CAP_NM" ]]; then
    echo "Per-motor command cap: hip pitch/roll=${HIP_PITCH_ROLL_COMMAND_CAP_NM}Nm; other legs=${LEG_COMMAND_CAP_NM}Nm; ankles=${ANKLE_COMMAND_CAP_NM:-$LEG_COMMAND_CAP_NM}Nm; other motors=min(10% rated, ${MAXIMUM_COMMAND_TORQUE_NM}Nm), checked after MIT quantization"
  else
    echo "Per-motor command cap: legs=${LEG_COMMAND_CAP_NM}Nm; other motors=min(10% rated, ${MAXIMUM_COMMAND_TORQUE_NM}Nm), checked after MIT quantization"
  fi
else
  echo "Per-motor command cap: min(10% of rated torque, ${MAXIMUM_COMMAND_TORQUE_NM} Nm), checked after MIT quantization"
fi
echo "Native watchdogs cover target age, status, hard position, speed, torque, temperature, and timing"
echo "Any fault or SIGINT/SIGTERM performs whole-body disable and verifies all 31 disabled"
echo "No mode switch and no zero-position reset are implemented in this path"
echo "$SUPPORT_INSTRUCTION; independent power safety operator must be ready"
echo "NATIVE_REPORT $NATIVE_REPORT"
echo "POLICY_REPORT $POLICY_REPORT"

"$ROOT/build/native/sprite_can_shadow" \
  --config "$ROOT/build/native/motors.tsv" \
  --duration "$DURATION" \
  --cpu 5 \
  --realtime-priority 50 \
  --output "$NATIVE_REPORT" \
  --ipc-socket "$SOCKET" \
  --policy-joint-hash "$JOINT_HASH" \
  --kinematics-config "$ROOT/build/native/kinematics.tsv" \
  --joint-safety-config "$ROOT/build/native/joint_safety.tsv" \
  --policy-actuation \
  "${EXTENDED_NATIVE_ACK_ARGS[@]}" \
  --acknowledge-hardware-tx ENABLE_NATIVE_PROTECTED_POLICY_ACTUATION \
  --all-motors-disabled-confirmed >"$NATIVE_LOG" 2>&1 &
NATIVE_PID=$!

shutdown_native() {
  if kill -0 "$NATIVE_PID" 2>/dev/null; then
    kill -TERM "$NATIVE_PID" 2>/dev/null || true
    wait "$NATIVE_PID" 2>/dev/null || true
  fi
}
trap shutdown_native INT TERM EXIT

for _ in $(seq 1 100); do
  [[ -S "$SOCKET" ]] && break
  kill -0 "$NATIVE_PID" 2>/dev/null || {
    cat "$NATIVE_LOG"
    exit 1
  }
  sleep 0.05
done
[[ -S "$SOCKET" ]] || {
  echo "Native IPC socket did not appear" >&2
  cat "$NATIVE_LOG"
  exit 1
}

set +e
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 taskset -c 4 env PYTHONPATH="$ROOT/src" \
  "$ROOT/.venv/bin/python" -m open_sprite_runtime.native_policy_client \
  --socket "$SOCKET" \
  --hardware-config "$ROOT/config/hardware.sprite0825.measurement.json" \
  --contract "$CANDIDATE/deploy/contract.json" \
  --imu-device /dev/sprite0825-imu \
  --imu-baud 115200 \
  --vx "$COMMAND_VX" --vy 0 --yaw-rate 0 \
  --joint-limit-candidates "$JOINT_LIMITS" \
  --gain-scale "$GAIN_SCALE" \
  "${JOINT_GAIN_ARGS[@]}" \
  "${CLAMP_WATCHDOG_ARGS[@]}" \
  --physical-startup-hold-seconds "$STARTUP_HOLD_SECONDS" \
  --physical-startup-ramp-seconds "$STARTUP_RAMP_SECONDS" \
  --output "$POLICY_REPORT" \
  --trace-output "$POLICY_TRACE"
POLICY_STATUS=$?
wait "$NATIVE_PID"
NATIVE_STATUS=$?
set -e
trap - INT TERM EXIT
cat "$NATIVE_LOG"
if (( POLICY_STATUS != 0 || NATIVE_STATUS != 0 )); then
  echo "Protected-policy admission failed: policy_status=$POLICY_STATUS native_status=$NATIVE_STATUS" >&2
  exit 1
fi
