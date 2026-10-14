#!/usr/bin/env bash
set -euo pipefail
echo "WAIST COMPARISON: waist Kp=60 Kd=3.0 cap=30Nm; bilateral hip roll150/5; all other gains and protections unchanged"
# Arms use shoulder pitch/roll 20/0.5; shoulder yaw/elbow 7/0.45 directly; waist, wrists and head unchanged; arm caps explicitly overridden below.

ACK="${1:-}"
[[ "$ACK" == "ENABLE_BNO055_WAIST60_KD3_WALK40" ]] || exit 2
echo "BNO055 IMUPLUS/P0 comparison: original policy, PD gains and safety limits unchanged"
ACK=ENABLE_NATIVE_PROTECTED_POLICY_WALK010_STOP_40S
TIER=grounded_full_weight_walk010_stop_40s_tier
PREFLIGHT_ONLY="${SPRITE_PREFLIGHT_ONLY:-0}"
IMU_ARGS=(--imu-source bno055 --bno-bus 7 --bno-address 0x28)
if [[ -n "${SPRITE_NATIVE_IMU_SOCKET:-}" ]]; then
  [[ -S "$SPRITE_NATIVE_IMU_SOCKET" ]] || { echo "Native IMU socket missing" >&2; exit 1; }
  IMU_ARGS=(--imu-source native-bno055 --native-imu-socket "$SPRITE_NATIVE_IMU_SOCKET")
fi
COMMAND_VX=0.0
COMMAND_PROFILE_ARGS=()
LEG_COMMAND_CAP_NM=""
LEG_FEEDBACK_CAP_NM=""
KNEE_COMMAND_CAP_NM=""
KNEE_FEEDBACK_CAP_NM=""
ANKLE_COMMAND_CAP_NM=""
ANKLE_FEEDBACK_CAP_NM=""
HIP_PITCH_ROLL_COMMAND_CAP_NM=""
HIP_PITCH_ROLL_FEEDBACK_CAP_NM=""
WAIST_ROLL_COMMAND_CAP_NM=""
WAIST_ROLL_FEEDBACK_CAP_NM=""
SHOULDER_PITCH_FEEDBACK_CAP_NM=""
DM3507_COMMAND_CAP_NM=""
HARDWARE_FEEDBACK_CAPS=0
PREFLIGHT_MAXIMUM_GATED_OVERSHOOT_RAD=0.01
PREFLIGHT_MAXIMUM_GATED_VIOLATION_FRACTION=0.01
PREFLIGHT_MAXIMUM_GATED_CONSECUTIVE_TICKS=2
PREFLIGHT_TORQUE_MULTIPLIER=1.0
PREFLIGHT_ENFORCE_POLICY_SOFT_LIMITS=1
NON_HIP_GAIN_MULTIPLIER=1.0
HEAD_GAIN_MULTIPLIER=""
WAIST_ROLL_GAIN_MULTIPLIER=""
LEG_GAIN_MULTIPLIER=""
LEG_DAMPING_MULTIPLIER=1.0
ANKLE_GAIN_MULTIPLIER=""
HIP_GAIN_MULTIPLIER=1.0
EXTENDED_NATIVE_ACK_ARGS=()
TORQUE_SATURATION_ARGS=()
CLAMP_WATCHDOG_ARGS=()
POLICY_REPLAY_ARGS=()
REPLAY_ACTION_TRACE=""
REPLAY_AMPLITUDE_SCALE=0.2
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
    PREFLIGHT_MAXIMUM_GATED_OVERSHOOT_RAD=0.15
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
      --clamp-watchdog-ignored-initial-ticks 350
    )
    ;;
  stand_leg_gain16_ankle_recovery_tier)
    EXPECTED_ACK=ENABLE_NATIVE_PROTECTED_POLICY_ANKLE_GAIN16_LOWER_SUPPORT_RECOVERY
    DURATION=8.0
    GAIN_SCALE=0.16
    DM3507_GAIN_MULTIPLIER=0.0375
    NON_HIP_GAIN_MULTIPLIER=0.375
    LEG_GAIN_MULTIPLIER=0.5
    ANKLE_GAIN_MULTIPLIER=1.0
    HIP_GAIN_MULTIPLIER=0.5
    MAXIMUM_COMMAND_TORQUE_NM=1.0
    LEG_COMMAND_CAP_NM=3.0
    LEG_FEEDBACK_CAP_NM=3.5
    ANKLE_COMMAND_CAP_NM=2.2
    ANKLE_FEEDBACK_CAP_NM=2.5
    HIP_PITCH_ROLL_COMMAND_CAP_NM=4.5
    HIP_PITCH_ROLL_FEEDBACK_CAP_NM=5.0
    PREFLIGHT_MAXIMUM_GATED_OVERSHOOT_RAD=0.15
    PREFLIGHT_MAXIMUM_GATED_VIOLATION_FRACTION=1.0
    PREFLIGHT_MAXIMUM_GATED_CONSECUTIVE_TICKS=300
    SUPPORT_INSTRUCTION="Lifting frame is at the lower-support recovery height; both soles remain on a flat floor; no disturbance; only ankle gains are doubled relative to the failed recovery tier"
    for joint in \
      left_ankle_pitch_joint right_ankle_pitch_joint \
      left_ankle_roll_joint right_ankle_roll_joint; do
      CLAMP_WATCHDOG_ARGS+=(--fail-on-consecutive-clamp-joint "$joint")
    done
    CLAMP_WATCHDOG_ARGS+=(
      --clamp-watchdog-minimum-overshoot-rad 0.05
      --clamp-watchdog-maximum-consecutive-ticks 5
      --clamp-watchdog-ignored-initial-ticks 350
    )
    ;;
  suspended_ankle_gain16_pose_recovery_35nm_tier)
    EXPECTED_ACK=ENABLE_NATIVE_PROTECTED_POLICY_SUSPENDED_ANKLE_GAIN16_POSE_RECOVERY_35NM
    DURATION=8.0
    GAIN_SCALE=0.16
    DM3507_GAIN_MULTIPLIER=0.0375
    NON_HIP_GAIN_MULTIPLIER=0.375
    LEG_GAIN_MULTIPLIER=0.5
    ANKLE_GAIN_MULTIPLIER=1.0
    HIP_GAIN_MULTIPLIER=0.5
    MAXIMUM_COMMAND_TORQUE_NM=1.0
    LEG_COMMAND_CAP_NM=3.5
    LEG_FEEDBACK_CAP_NM=3.5
    ANKLE_COMMAND_CAP_NM=3.5
    ANKLE_FEEDBACK_CAP_NM=3.5
    HIP_PITCH_ROLL_COMMAND_CAP_NM=3.5
    HIP_PITCH_ROLL_FEEDBACK_CAP_NM=3.5
    SUPPORT_INSTRUCTION="Robot is securely suspended; both feet remain at least 4cm above the floor; no contact and no disturbance; safety operator controls independent power cutoff"
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
  suspended_leg_gain12_ankle_gain24_pose_recovery_35nm_20s_tier)
    EXPECTED_ACK=ENABLE_NATIVE_PROTECTED_POLICY_SUSPENDED_LEG_GAIN12_ANKLE_GAIN24_POSE_RECOVERY_35NM_20S
    DURATION=20.0
    GAIN_SCALE=0.24
    DM3507_GAIN_MULTIPLIER=0.025
    NON_HIP_GAIN_MULTIPLIER=0.2
    LEG_GAIN_MULTIPLIER=0.5
    ANKLE_GAIN_MULTIPLIER=1.0
    HIP_GAIN_MULTIPLIER=0.5
    MAXIMUM_COMMAND_TORQUE_NM=1.0
    LEG_COMMAND_CAP_NM=3.5
    LEG_FEEDBACK_CAP_NM=3.5
    ANKLE_COMMAND_CAP_NM=3.5
    ANKLE_FEEDBACK_CAP_NM=3.5
    HIP_PITCH_ROLL_COMMAND_CAP_NM=3.5
    HIP_PITCH_ROLL_FEEDBACK_CAP_NM=3.5
    SUPPORT_INSTRUCTION="Robot is securely suspended; both feet remain at least 4cm above the floor; all leg joints start visibly displaced from the policy pose; no contact and no disturbance; safety operator controls independent power cutoff"
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
  suspended_far_pose_leg_gain055_ankle_gain11_recovery_35nm_20s_tier)
    EXPECTED_ACK=ENABLE_NATIVE_PROTECTED_POLICY_SUSPENDED_FAR_POSE_LEG_GAIN055_ANKLE_GAIN11_RECOVERY_35NM_20S
    DURATION=20.0
    GAIN_SCALE=0.11
    DM3507_GAIN_MULTIPLIER=0.05
    NON_HIP_GAIN_MULTIPLIER=0.4
    LEG_GAIN_MULTIPLIER=0.48
    ANKLE_GAIN_MULTIPLIER=1.0
    HIP_GAIN_MULTIPLIER=0.48
    MAXIMUM_COMMAND_TORQUE_NM=1.0
    LEG_COMMAND_CAP_NM=3.5
    LEG_FEEDBACK_CAP_NM=3.5
    ANKLE_COMMAND_CAP_NM=3.5
    ANKLE_FEEDBACK_CAP_NM=3.5
    HIP_PITCH_ROLL_COMMAND_CAP_NM=3.5
    HIP_PITCH_ROLL_FEEDBACK_CAP_NM=3.5
    SUPPORT_INSTRUCTION="Robot is securely suspended; both feet remain at least 4cm above the floor; leg joints start far from the policy pose; this low-gain first stage must complete before the higher-gain recovery tier; no contact and no disturbance; safety operator controls independent power cutoff"
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
  suspended_direction_audit_vx015_35nm_20s_tier)
    EXPECTED_ACK=ENABLE_NATIVE_PROTECTED_POLICY_SUSPENDED_DIRECTION_AUDIT_VX015_35NM_20S
    DURATION=20.0
    GAIN_SCALE=0.11
    DM3507_GAIN_MULTIPLIER=0.05
    NON_HIP_GAIN_MULTIPLIER=0.4
    LEG_GAIN_MULTIPLIER=0.48
    ANKLE_GAIN_MULTIPLIER=1.0
    HIP_GAIN_MULTIPLIER=0.48
    MAXIMUM_COMMAND_TORQUE_NM=1.0
    LEG_COMMAND_CAP_NM=3.5
    LEG_FEEDBACK_CAP_NM=3.5
    ANKLE_COMMAND_CAP_NM=3.5
    ANKLE_FEEDBACK_CAP_NM=3.5
    HIP_PITCH_ROLL_COMMAND_CAP_NM=3.5
    HIP_PITCH_ROLL_FEEDBACK_CAP_NM=3.5
    COMMAND_VX=0.15
    SUPPORT_INSTRUCTION="Robot is securely suspended; both feet remain at least 4cm above the floor; start from the qualified policy pose; no contact and no disturbance; safety operator controls independent power cutoff; this test audits commanded-versus-measured joint direction and phase only"
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
  suspended_trace_direction_audit_scale020_35nm_20s_tier)
    EXPECTED_ACK=ENABLE_NATIVE_PROTECTED_POLICY_SUSPENDED_TRACE_DIRECTION_AUDIT_SCALE020_35NM_20S
    DURATION=20.0
    GAIN_SCALE=0.11
    DM3507_GAIN_MULTIPLIER=0.05
    NON_HIP_GAIN_MULTIPLIER=0.4
    LEG_GAIN_MULTIPLIER=0.48
    ANKLE_GAIN_MULTIPLIER=1.0
    HIP_GAIN_MULTIPLIER=0.48
    MAXIMUM_COMMAND_TORQUE_NM=1.0
    LEG_COMMAND_CAP_NM=3.5
    LEG_FEEDBACK_CAP_NM=3.5
    ANKLE_COMMAND_CAP_NM=3.5
    ANKLE_FEEDBACK_CAP_NM=3.5
    HIP_PITCH_ROLL_COMMAND_CAP_NM=3.5
    HIP_PITCH_ROLL_FEEDBACK_CAP_NM=3.5
    COMMAND_VX=0.15
    REPLAY_ACTION_TRACE="/home/tony/sprite_runtime/sprite0825_stage2_g74_model3000_sim2real_candidate_kp130_kd5_20261010_arms7_kd3_20261012_follow_v1_waist50_hiproll150_waist70_kd3_kd35_waist60kd3/evaluation/mujoco_matrix/straight_60s_trace.json"
    REPLAY_AMPLITUDE_SCALE=0.2
    POLICY_REPLAY_ARGS=(
      --replay-action-trace "$REPLAY_ACTION_TRACE"
      --replay-source-hz 50.0
      --replay-start-seconds 10.0
      --replay-duration-seconds 20.0
      --replay-amplitude-scale "$REPLAY_AMPLITUDE_SCALE"
    )
    SUPPORT_INSTRUCTION="Robot is securely suspended; both feet remain at least 4cm above the floor; start from the qualified policy pose; no contact and no disturbance; safety operator controls independent power cutoff; replays a centered 20-percent-amplitude qualified MuJoCo policy action trace for direction audit only"
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
  suspended_trace_direction_audit_scale050_gain012_35nm_20s_tier)
    EXPECTED_ACK=ENABLE_NATIVE_PROTECTED_POLICY_SUSPENDED_TRACE_DIRECTION_AUDIT_SCALE050_GAIN012_35NM_20S
    DURATION=20.0
    GAIN_SCALE=0.24
    DM3507_GAIN_MULTIPLIER=0.025
    NON_HIP_GAIN_MULTIPLIER=0.2
    LEG_GAIN_MULTIPLIER=0.5
    ANKLE_GAIN_MULTIPLIER=1.0
    HIP_GAIN_MULTIPLIER=0.5
    MAXIMUM_COMMAND_TORQUE_NM=1.0
    LEG_COMMAND_CAP_NM=3.5
    LEG_FEEDBACK_CAP_NM=3.5
    ANKLE_COMMAND_CAP_NM=3.5
    ANKLE_FEEDBACK_CAP_NM=3.5
    HIP_PITCH_ROLL_COMMAND_CAP_NM=3.5
    HIP_PITCH_ROLL_FEEDBACK_CAP_NM=3.5
    COMMAND_VX=0.15
    REPLAY_ACTION_TRACE="/home/tony/sprite_runtime/sprite0825_stage2_g74_model3000_sim2real_candidate_kp130_kd5_20261010_arms7_kd3_20261012_follow_v1_waist50_hiproll150_waist70_kd3_kd35_waist60kd3/evaluation/mujoco_matrix/straight_60s_trace.json"
    REPLAY_AMPLITUDE_SCALE=0.5
    POLICY_REPLAY_ARGS=(
      --replay-action-trace "$REPLAY_ACTION_TRACE"
      --replay-source-hz 50.0
      --replay-start-seconds 10.0
      --replay-duration-seconds 20.0
      --replay-amplitude-scale "$REPLAY_AMPLITUDE_SCALE"
    )
    SUPPORT_INSTRUCTION="Robot is securely suspended; both feet remain at least 4cm above the floor; start from the qualified policy pose; no contact and no disturbance; safety operator controls independent power cutoff; replays a centered 50-percent-amplitude qualified MuJoCo policy action trace for visible direction audit only"
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
  grounded_full_weight_stand_ankle100_rated_20s_tier)
    EXPECTED_ACK=ENABLE_NATIVE_PROTECTED_POLICY_FULL_WEIGHT_STAND_ANKLE100_RATED_20S
    DURATION=20.0
    GAIN_SCALE=1.0
    DM3507_GAIN_MULTIPLIER=0.01
    NON_HIP_GAIN_MULTIPLIER=0.125
    LEG_GAIN_MULTIPLIER=0.25
    ANKLE_GAIN_MULTIPLIER=1.0
    HIP_GAIN_MULTIPLIER=0.25
    MAXIMUM_COMMAND_TORQUE_NM=3.0
    LEG_COMMAND_CAP_NM=8.0
    LEG_FEEDBACK_CAP_NM=9.0
    ANKLE_COMMAND_CAP_NM=3.5
    ANKLE_FEEDBACK_CAP_NM=3.5
    HIP_PITCH_ROLL_COMMAND_CAP_NM=8.0
    HIP_PITCH_ROLL_FEEDBACK_CAP_NM=9.0
    COMMAND_VX=0.0
    SUPPORT_INSTRUCTION="Robot full weight is carried by both soles on a flat floor; lifting frame is slack and serves only as fall arrest; after 8s stable standing apply one gentle disturbance direction at a time; ankle gains are full contract values while ankle torque remains capped at 3.5Nm; safety operator controls independent power cutoff"
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
      --clamp-watchdog-ignored-initial-ticks 250
    )
    ;;
  grounded_full_weight_walk010_stop_40s_tier|grounded_full_weight_walk005_stop_40s_tier|grounded_full_weight_stand_lower_body_sim_pd_8s_tier|grounded_full_weight_stand_lower_body_sim_pd_40s_tier|grounded_full_weight_stand_lower_body_sim_pd_60s_tier)
    if [[ "$TIER" == "grounded_full_weight_walk010_stop_40s_tier" || "$TIER" == "grounded_full_weight_walk005_stop_40s_tier" ]]; then
      EXPECTED_ACK=ENABLE_NATIVE_PROTECTED_POLICY_WALK005_STOP_40S
      DURATION=40.0
      COMMAND_PROFILE_ARGS=(--command-profile stand-walk-stop-40s)
      PROFILE_FORWARD_SPEED=0.05
      if [[ "$TIER" == "grounded_full_weight_walk010_stop_40s_tier" ]]; then
        EXPECTED_ACK=ENABLE_NATIVE_PROTECTED_POLICY_WALK010_STOP_40S
        COMMAND_PROFILE_ARGS=(--command-profile stand-walk010-stop-40s)
        PROFILE_FORWARD_SPEED=0.10
      fi
      EXTENDED_NATIVE_ACK_ARGS=(
        --extended-policy-actuation-acknowledgement
        ENABLE_40_SECOND_GROUNDED_BALANCE_TEST
      )
    elif [[ "$TIER" == "grounded_full_weight_stand_lower_body_sim_pd_60s_tier" ]]; then
      EXPECTED_ACK=ENABLE_NATIVE_PROTECTED_POLICY_FULL_WEIGHT_STAND_LOWER_BODY_SIM_PD_60S
      DURATION=60.0
      EXTENDED_NATIVE_ACK_ARGS=(
        --extended-policy-actuation-acknowledgement
        ENABLE_60_SECOND_GROUNDED_LOWER_BODY_SIM_PD_TEST
      )
    elif [[ "$TIER" == "grounded_full_weight_stand_lower_body_sim_pd_40s_tier" ]]; then
      EXPECTED_ACK=ENABLE_NATIVE_PROTECTED_POLICY_FULL_WEIGHT_STAND_LOWER_BODY_SIM_PD_40S
      DURATION=40.0
      EXTENDED_NATIVE_ACK_ARGS=(
        --extended-policy-actuation-acknowledgement
        ENABLE_40_SECOND_GROUNDED_BALANCE_TEST
      )
    else
      EXPECTED_ACK=ENABLE_NATIVE_PROTECTED_POLICY_FULL_WEIGHT_STAND_LOWER_BODY_SIM_PD_8S
      DURATION=8.0
      EXTENDED_NATIVE_ACK_ARGS=(
        --extended-policy-actuation-acknowledgement
        ENABLE_8_SECOND_FULL_WEIGHT_LOWER_BODY_SIM_PD_TEST
      )
    fi
    GAIN_SCALE=1.0
    DM3507_GAIN_MULTIPLIER=0.01
    HEAD_GAIN_MULTIPLIER=0.05
    NON_HIP_GAIN_MULTIPLIER=0.125
    WAIST_ROLL_GAIN_MULTIPLIER=1.0
    LEG_GAIN_MULTIPLIER=1.0
    LEG_DAMPING_MULTIPLIER=1.0
    ANKLE_GAIN_MULTIPLIER=1.0
    HIP_GAIN_MULTIPLIER=1.0
    MAXIMUM_COMMAND_TORQUE_NM=3.0
    DM3507_COMMAND_CAP_NM=0.5
    LEG_COMMAND_CAP_NM=28.0
    LEG_FEEDBACK_CAP_NM=28.0
    KNEE_COMMAND_CAP_NM=28.0
    KNEE_FEEDBACK_CAP_NM=28.0
    ANKLE_COMMAND_CAP_NM=10.0
    ANKLE_FEEDBACK_CAP_NM=10.0
    HIP_PITCH_ROLL_COMMAND_CAP_NM=28.0
    HIP_PITCH_ROLL_FEEDBACK_CAP_NM=28.0
    WAIST_ROLL_COMMAND_CAP_NM=30.0
    HARDWARE_FEEDBACK_CAPS=1
    COMMAND_VX=0.0
    # Admission margin covers MIT quantization and small initial pose offsets;
    # the native runtime still hard-clamps actual commands to the configured caps.
    PREFLIGHT_TORQUE_MULTIPLIER=1.1
    PREFLIGHT_ENFORCE_POLICY_SOFT_LIMITS=0
    SUPPORT_INSTRUCTION="Robot full weight is carried by both soles on a flat floor; lifting frame is slack and serves only as fall arrest; lower-body gains match the deployment contract except hip roll Kp=150 Kd=5; hip pitch/yaw and knee Kp=130 Kd=5; other joint gains retain baseline values; arm gains are an unqualified candidate: shoulder pitch/roll 20/0.5, yaw/elbow 7/0.45; safety operator controls independent power cutoff"
    TORQUE_SATURATION_ARGS=(--saturate-policy-torque-to-commissioning-cap)
    ;;
  grounded_full_weight_stand_ankle100_rated_40s_tier|grounded_full_weight_stand_waist050_ankle100_rated_40s_tier|grounded_full_weight_stand_waist050_ankle5_40s_tier|grounded_full_weight_stand_waist050_ankle5_physical_feedback_40s_tier|grounded_full_weight_stand_waist050_ankle5_legkd050_20s_tier|grounded_full_weight_stand_waist050_ankle5_legkd050_40s_tier)
    if [[ "$TIER" == "grounded_full_weight_stand_waist050_ankle5_legkd050_20s_tier" || "$TIER" == "grounded_full_weight_stand_waist050_ankle5_legkd050_40s_tier" ]]; then
      if [[ "$TIER" == "grounded_full_weight_stand_waist050_ankle5_legkd050_40s_tier" ]]; then
        EXPECTED_ACK=ENABLE_NATIVE_PROTECTED_POLICY_FULL_WEIGHT_STAND_WAIST050_ANKLE5_LEGKD050_40S
        DURATION=40.0
      else
        EXPECTED_ACK=ENABLE_NATIVE_PROTECTED_POLICY_FULL_WEIGHT_STAND_WAIST050_ANKLE5_LEGKD050_20S
        DURATION=20.0
      fi
      WAIST_ROLL_GAIN_MULTIPLIER=0.5
      WAIST_ROLL_COMMAND_CAP_NM=10.0
      HARDWARE_FEEDBACK_CAPS=1
      LEG_DAMPING_MULTIPLIER=2.0
      KNEE_COMMAND_CAP_NM=28.0
      KNEE_FEEDBACK_CAP_NM=28.0
    elif [[ "$TIER" == "grounded_full_weight_stand_waist050_ankle5_physical_feedback_40s_tier" ]]; then
      EXPECTED_ACK=ENABLE_NATIVE_PROTECTED_POLICY_FULL_WEIGHT_STAND_WAIST050_ANKLE5_PHYSICAL_FEEDBACK_40S
      WAIST_ROLL_GAIN_MULTIPLIER=0.5
      WAIST_ROLL_COMMAND_CAP_NM=10.0
      HARDWARE_FEEDBACK_CAPS=1
    elif [[ "$TIER" == "grounded_full_weight_stand_waist050_ankle5_40s_tier" ]]; then
      EXPECTED_ACK=ENABLE_NATIVE_PROTECTED_POLICY_FULL_WEIGHT_STAND_WAIST050_ANKLE5_40S
      WAIST_ROLL_GAIN_MULTIPLIER=0.5
      WAIST_ROLL_COMMAND_CAP_NM=10.0
      WAIST_ROLL_FEEDBACK_CAP_NM=12.0
      SHOULDER_PITCH_FEEDBACK_CAP_NM=1.6
    elif [[ "$TIER" == "grounded_full_weight_stand_waist050_ankle100_rated_40s_tier" ]]; then
      EXPECTED_ACK=ENABLE_NATIVE_PROTECTED_POLICY_FULL_WEIGHT_STAND_WAIST050_ANKLE100_RATED_40S
      WAIST_ROLL_GAIN_MULTIPLIER=0.5
      WAIST_ROLL_COMMAND_CAP_NM=10.0
      WAIST_ROLL_FEEDBACK_CAP_NM=12.0
      SHOULDER_PITCH_FEEDBACK_CAP_NM=1.6
    else
      EXPECTED_ACK=ENABLE_NATIVE_PROTECTED_POLICY_FULL_WEIGHT_STAND_ANKLE100_RATED_40S
    fi
    DURATION="${DURATION:-40.0}"
    GAIN_SCALE=1.0
    DM3507_GAIN_MULTIPLIER=0.01
    HEAD_GAIN_MULTIPLIER=0.05
    NON_HIP_GAIN_MULTIPLIER=0.125
    LEG_GAIN_MULTIPLIER=0.25
    ANKLE_GAIN_MULTIPLIER=1.0
    HIP_GAIN_MULTIPLIER=0.25
    MAXIMUM_COMMAND_TORQUE_NM=3.0
    DM3507_COMMAND_CAP_NM=0.5
    LEG_COMMAND_CAP_NM=8.0
    LEG_FEEDBACK_CAP_NM=9.0
    ANKLE_COMMAND_CAP_NM=3.5
    ANKLE_FEEDBACK_CAP_NM=3.5
    HIP_PITCH_ROLL_COMMAND_CAP_NM=8.0
    HIP_PITCH_ROLL_FEEDBACK_CAP_NM=9.0
    COMMAND_VX=0.0
    PREFLIGHT_TORQUE_MULTIPLIER=1.5
    PREFLIGHT_ENFORCE_POLICY_SOFT_LIMITS=0
    if [[ "$TIER" == "grounded_full_weight_stand_waist050_ankle5_40s_tier" || "$TIER" == "grounded_full_weight_stand_waist050_ankle5_physical_feedback_40s_tier" || "$TIER" == "grounded_full_weight_stand_waist050_ankle5_legkd050_20s_tier" || "$TIER" == "grounded_full_weight_stand_waist050_ankle5_legkd050_40s_tier" ]]; then
      ANKLE_COMMAND_CAP_NM=5.0
      ANKLE_FEEDBACK_CAP_NM=6.0
    elif [[ "$TIER" == "grounded_full_weight_stand_waist050_ankle100_rated_40s_tier" ]]; then
      ANKLE_FEEDBACK_CAP_NM=4.0
    fi
    SUPPORT_INSTRUCTION="Robot full weight is carried by both soles on a flat floor; lifting frame is slack and serves only as fall arrest; after 8s stable standing apply one gentle disturbance direction at a time; ankle gains are full contract values while ankle torque remains capped at ${ANKLE_COMMAND_CAP_NM}Nm; safety operator controls independent power cutoff"
    EXTENDED_NATIVE_ACK_ARGS=(
      --extended-policy-actuation-acknowledgement
      ENABLE_40_SECOND_GROUNDED_BALANCE_TEST
    )
    TORQUE_SATURATION_ARGS=(--saturate-policy-torque-to-commissioning-cap)
    ;;
  grounded_full_weight_stand_gain025_rated_20s_tier)
    EXPECTED_ACK=ENABLE_NATIVE_PROTECTED_POLICY_FULL_WEIGHT_STAND_GAIN025_RATED_20S
    DURATION=20.0
    GAIN_SCALE=0.5
    DM3507_GAIN_MULTIPLIER=0.02
    NON_HIP_GAIN_MULTIPLIER=0.25
    LEG_GAIN_MULTIPLIER=0.5
    ANKLE_GAIN_MULTIPLIER=1.0
    HIP_GAIN_MULTIPLIER=0.5
    MAXIMUM_COMMAND_TORQUE_NM=3.0
    LEG_COMMAND_CAP_NM=8.0
    LEG_FEEDBACK_CAP_NM=9.0
    ANKLE_COMMAND_CAP_NM=3.5
    ANKLE_FEEDBACK_CAP_NM=3.5
    HIP_PITCH_ROLL_COMMAND_CAP_NM=8.0
    HIP_PITCH_ROLL_FEEDBACK_CAP_NM=9.0
    COMMAND_VX=0.0
    SUPPORT_INSTRUCTION="Robot full weight is carried by both soles on a flat floor; lifting frame is slack and serves only as fall arrest; after 8s stable standing apply one gentle disturbance direction at a time; safety operator controls independent power cutoff"
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
      --clamp-watchdog-ignored-initial-ticks 250
    )
    ;;
  grounded_walk_vx010_gain016_rated_tier)
    EXPECTED_ACK=ENABLE_NATIVE_PROTECTED_POLICY_GROUNDED_WALK_VX010_GAIN016_RATED
    DURATION=8.0
    GAIN_SCALE=0.32
    DM3507_GAIN_MULTIPLIER=0.02
    NON_HIP_GAIN_MULTIPLIER=0.15
    LEG_GAIN_MULTIPLIER=0.5
    ANKLE_GAIN_MULTIPLIER=1.0
    HIP_GAIN_MULTIPLIER=0.5
    MAXIMUM_COMMAND_TORQUE_NM=1.0
    LEG_COMMAND_CAP_NM=8.0
    LEG_FEEDBACK_CAP_NM=9.0
    ANKLE_COMMAND_CAP_NM=3.5
    ANKLE_FEEDBACK_CAP_NM=3.5
    HIP_PITCH_ROLL_COMMAND_CAP_NM=8.0
    HIP_PITCH_ROLL_FEEDBACK_CAP_NM=9.0
    COMMAND_VX=0.10
    SUPPORT_INSTRUCTION="Both soles are on a flat floor; lifting frame remains attached as a fall arrest; safety operator controls independent power cutoff; clear the walking area"
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
[[ -n "$ANKLE_GAIN_MULTIPLIER" ]] || ANKLE_GAIN_MULTIPLIER="$LEG_GAIN_MULTIPLIER"
[[ -n "$WAIST_ROLL_GAIN_MULTIPLIER" ]] || WAIST_ROLL_GAIN_MULTIPLIER="$NON_HIP_GAIN_MULTIPLIER"
[[ "$ACK" == "$EXPECTED_ACK" ]] || {
  echo "Exact acknowledgement required: $EXPECTED_ACK" >&2
  exit 2
}

ROOT=/home/tony/open_sprite_runtime
CANDIDATE=/home/tony/sprite_runtime/sprite0825_stage2_g74_model3000_sim2real_candidate_kp130_kd5_20261010_arms7_kd3_20261012_follow_v1_waist50_hiproll150_waist70_kd3_kd35_waist60kd3
STARTUP_HOLD_SECONDS=1.0
STARTUP_RAMP_SECONDS=4.0
if [[ ${#COMMAND_PROFILE_ARGS[@]} -gt 0 ]]; then
  STARTUP_RAMP_SECONDS=5.0
fi
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
getcap "$ROOT/build/native/sprite_can_shadow_quantized_v1" | grep -q 'cap_sys_nice' || {
  echo "Native runtime lacks CAP_SYS_NICE; refuse active policy control" >&2
  echo "Reinstall the reviewed capability after every rebuild" >&2
  exit 1
}
[[ -f "$JOINT_LIMITS" ]] || {
  echo "Joint limit report not found: $JOINT_LIMITS" >&2
  exit 1
}

JOINT_GAIN_ARGS=()
JOINT_DAMPING_ARGS=()
if [[ "$DM3507_GAIN_MULTIPLIER" != "1.0" ]]; then
  for joint in \
    left_wrist_pitch_joint left_wrist_roll_joint \
    right_wrist_pitch_joint right_wrist_roll_joint; do
    JOINT_GAIN_ARGS+=(--joint-gain-multiplier "$joint=$DM3507_GAIN_MULTIPLIER")
  done
fi
if [[ -z "$HEAD_GAIN_MULTIPLIER" ]]; then
  HEAD_GAIN_MULTIPLIER="$DM3507_GAIN_MULTIPLIER"
fi
if [[ "$HEAD_GAIN_MULTIPLIER" != "1.0" ]]; then
  for joint in head_pitch_joint head_roll_joint head_yaw_joint; do
    JOINT_GAIN_ARGS+=(--joint-gain-multiplier "$joint=$HEAD_GAIN_MULTIPLIER")
  done
fi
if [[ "$NON_HIP_GAIN_MULTIPLIER" != "1.0" ]]; then
  for joint in \
    waist_yaw_joint \
    left_wrist_yaw_joint right_wrist_yaw_joint; do
    JOINT_GAIN_ARGS+=(--joint-gain-multiplier "$joint=$NON_HIP_GAIN_MULTIPLIER")
  done
fi
if [[ "$WAIST_ROLL_GAIN_MULTIPLIER" != "1.0" ]]; then
  JOINT_GAIN_ARGS+=(--joint-gain-multiplier "waist_roll_joint=$WAIST_ROLL_GAIN_MULTIPLIER")
fi
if [[ "$LEG_GAIN_MULTIPLIER" != "1.0" ]]; then
  for joint in \
    left_hip_yaw_joint right_hip_yaw_joint \
    left_knee_joint right_knee_joint; do
    JOINT_GAIN_ARGS+=(--joint-gain-multiplier "$joint=$LEG_GAIN_MULTIPLIER")
  done
fi
if [[ "$ANKLE_GAIN_MULTIPLIER" != "1.0" ]]; then
  for joint in \
    left_ankle_pitch_joint right_ankle_pitch_joint \
    left_ankle_roll_joint right_ankle_roll_joint; do
    JOINT_GAIN_ARGS+=(--joint-gain-multiplier "$joint=$ANKLE_GAIN_MULTIPLIER")
  done
fi
if [[ "$HIP_GAIN_MULTIPLIER" != "1.0" ]]; then
  for joint in \
    left_hip_pitch_joint right_hip_pitch_joint \
    left_hip_roll_joint right_hip_roll_joint; do
    JOINT_GAIN_ARGS+=(--joint-gain-multiplier "$joint=$HIP_GAIN_MULTIPLIER")
  done
fi
if [[ "$LEG_DAMPING_MULTIPLIER" != "1.0" ]]; then
  for joint in \
    left_hip_pitch_joint right_hip_pitch_joint \
    left_hip_roll_joint right_hip_roll_joint \
    left_hip_yaw_joint right_hip_yaw_joint \
    left_knee_joint right_knee_joint; do
    JOINT_DAMPING_ARGS+=(--joint-damping-multiplier "$joint=$LEG_DAMPING_MULTIPLIER")
  done
fi
MOTOR_CAP_ARGS=()
FEEDBACK_CAP_MODE_ARGS=()
if [[ "$HARDWARE_FEEDBACK_CAPS" == "1" ]]; then
  FEEDBACK_CAP_MODE_ARGS=(--feedback-caps-at-hardware-limit)
fi
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
      left_knee_motor|right_knee_motor)
        if [[ -n "$KNEE_COMMAND_CAP_NM" ]]; then
          command_cap="$KNEE_COMMAND_CAP_NM"
          feedback_cap="$KNEE_FEEDBACK_CAP_NM"
        fi
        ;;
    esac
    MOTOR_CAP_ARGS+=(--motor-command-cap "$motor=$command_cap")
    if [[ "$HARDWARE_FEEDBACK_CAPS" != "1" ]]; then
      MOTOR_CAP_ARGS+=(--motor-feedback-cap "$motor=$feedback_cap")
    fi
  done
fi
if [[ -n "$WAIST_ROLL_COMMAND_CAP_NM" ]]; then
  MOTOR_CAP_ARGS+=(--motor-command-cap "waist_roll_motor=$WAIST_ROLL_COMMAND_CAP_NM")
  if [[ "$HARDWARE_FEEDBACK_CAPS" != "1" ]]; then
    MOTOR_CAP_ARGS+=(--motor-feedback-cap "waist_roll_motor=$WAIST_ROLL_FEEDBACK_CAP_NM")
  fi
fi
if [[ -n "$SHOULDER_PITCH_FEEDBACK_CAP_NM" ]]; then
  MOTOR_CAP_ARGS+=(--motor-feedback-cap "left_shoulder_pitch_motor=$SHOULDER_PITCH_FEEDBACK_CAP_NM")
  MOTOR_CAP_ARGS+=(--motor-feedback-cap "right_shoulder_pitch_motor=$SHOULDER_PITCH_FEEDBACK_CAP_NM")
fi
if [[ -n "$DM3507_COMMAND_CAP_NM" ]]; then
  for motor in \
    head_motor_a head_motor_b head_yaw_motor \
    left_wrist_pitch_motor left_wrist_roll_motor \
    right_wrist_pitch_motor right_wrist_roll_motor; do
    MOTOR_CAP_ARGS+=(--motor-command-cap "$motor=$DM3507_COMMAND_CAP_NM")
  done
fi
# Explicit arm caps shared by bounded preflight and active control.
for side in left right; do
  for joint in shoulder_pitch shoulder_roll; do
    MOTOR_CAP_ARGS+=(--motor-command-cap "${side}_${joint}_motor=3.0")
  done
  for joint in shoulder_yaw elbow; do
    MOTOR_CAP_ARGS+=(--motor-command-cap "${side}_${joint}_motor=3.0")
  done
done
echo "ARM COMMAND CAPS shoulder pitch/roll=3Nm; shoulder yaw/elbow=3Nm; wrists unchanged"
PYTHONPATH="$ROOT/src" "$ROOT/.venv/bin/python" \
  "$ROOT/tools/export_native_motor_config.py" \
  --hardware "$ROOT/config/hardware.sprite0825.kp110_kd5.json" \
  --maximum-commissioning-torque-nm "$MAXIMUM_COMMAND_TORQUE_NM" \
  "${FEEDBACK_CAP_MODE_ARGS[@]}" \
  "${MOTOR_CAP_ARGS[@]}" \
  --output "$ROOT/build/native/motors.tsv"
PYTHONPATH="$ROOT/src" "$ROOT/.venv/bin/python" \
  "$ROOT/tools/export_native_kinematics_config.py" \
  --hardware "$ROOT/config/hardware.sprite0825.kp110_kd5.json" \
  --contract "$CANDIDATE/deploy/contract.json" \
  --output "$ROOT/build/native/kinematics.tsv"
PYTHONPATH="$ROOT/src" "$ROOT/.venv/bin/python" \
  "$ROOT/tools/export_native_joint_safety_config.py" \
  --contract "$CANDIDATE/deploy/contract.json" \
  --joint-limit-candidates "$JOINT_LIMITS" \
  --gain-scale "$GAIN_SCALE" \
  --maximum-embedded-kd 5.0 \
  "${JOINT_GAIN_ARGS[@]}" \
  "${JOINT_DAMPING_ARGS[@]}" \
  --output "$ROOT/build/native/joint_safety.tsv"

BOUNDED_SHADOW_PREVIEW=0
if [[ ${#TORQUE_SATURATION_ARGS[@]} -gt 0 ]]; then
  BOUNDED_SHADOW_PREVIEW=1
fi
echo "ZERO-GAIN STARTUP READINESS PREFLIGHT: 8.0s"
if [[ "$PREFLIGHT_ENFORCE_POLICY_SOFT_LIMITS" == "1" ]]; then
  echo "Warms policy history for 1.0s, then requires ankle excursions <=0.01rad, <=1% ticks, <=2 consecutive ticks; horizontal projected gravity <=0.10"
else
  echo "Warms policy history for 1.0s; policy soft-limit excursions are advisory; horizontal projected gravity must remain <=0.10"
fi
SPRITE_PREPARED_MOTOR_CONFIG="$ROOT/build/native/motors.tsv" \
SPRITE_BOUNDED_SHADOW_PREVIEW="$BOUNDED_SHADOW_PREVIEW" \
SPRITE_REPLAY_ACTION_TRACE="$REPLAY_ACTION_TRACE" \
SPRITE_REPLAY_SOURCE_HZ=50.0 \
SPRITE_REPLAY_START_SECONDS=10.0 \
SPRITE_REPLAY_DURATION_SECONDS=20.0 \
SPRITE_REPLAY_AMPLITUDE_SCALE="$REPLAY_AMPLITUDE_SCALE" \
bash "$ROOT/probe_sprite0825_bno055_policy_shadow_on_253.sh" \
  8.0 "$GAIN_SCALE" "$STARTUP_HOLD_SECONDS" "$STARTUP_RAMP_SECONDS" "$DM3507_GAIN_MULTIPLIER" "$COMMAND_VX" \
  "$NON_HIP_GAIN_MULTIPLIER" "$LEG_GAIN_MULTIPLIER" \
  "$ANKLE_GAIN_MULTIPLIER" "$HIP_GAIN_MULTIPLIER" \
  "$WAIST_ROLL_GAIN_MULTIPLIER" "$HEAD_GAIN_MULTIPLIER" \
  "$LEG_DAMPING_MULTIPLIER" | tee "$PREFLIGHT_LOG"
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
  "$KNEE_COMMAND_CAP_NM" \
  "$HIP_PITCH_ROLL_COMMAND_CAP_NM" \
  "$ANKLE_COMMAND_CAP_NM" \
  "$PREFLIGHT_TORQUE_MULTIPLIER" \
  "$WAIST_ROLL_COMMAND_CAP_NM" \
  "$DM3507_COMMAND_CAP_NM" <<'PY'
import json
import sys

report = json.load(open(sys.argv[1], encoding="utf-8"))
default_limit = float(sys.argv[2])
leg_limit = float(sys.argv[3]) if sys.argv[3] else None
knee_limit = float(sys.argv[4]) if sys.argv[4] else None
hip_pitch_roll_limit = float(sys.argv[5]) if sys.argv[5] else None
ankle_limit = float(sys.argv[6]) if sys.argv[6] else None
torque_multiplier = float(sys.argv[7])
waist_roll_limit = float(sys.argv[8]) if sys.argv[8] else None
dm3507_limit = float(sys.argv[9]) if sys.argv[9] else None
if not 1.0 <= torque_multiplier <= 1.5:
    raise SystemExit("startup torque multiplier must be in [1.0, 1.5]")
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
    if name in {f"{s}_{j}_motor" for s in ("left", "right") for j in ("shoulder_pitch", "shoulder_roll")}:
        limit = 3.0
    elif name in {f"{s}_{j}_motor" for s in ("left", "right") for j in ("shoulder_yaw", "elbow")}:
        limit = 3.0
    elif dm3507_limit is not None and name in {
        "head_motor_a",
        "head_motor_b",
        "head_yaw_motor",
        "left_wrist_pitch_motor",
        "left_wrist_roll_motor",
        "right_wrist_pitch_motor",
        "right_wrist_roll_motor",
    }:
        limit = dm3507_limit
    elif waist_roll_limit is not None and name == "waist_roll_motor":
        limit = waist_roll_limit
    elif ankle_limit is not None and name in {
        "left_ankle_motor_a",
        "left_ankle_motor_b",
        "right_ankle_motor_a",
        "right_ankle_motor_b",
    }:
        limit = ankle_limit
    elif knee_limit is not None and name in {
        "left_knee_motor",
        "right_knee_motor",
    }:
        limit = knee_limit
    elif hip_pitch_roll_limit is not None and name in {
        "left_hip_pitch_motor",
        "left_hip_roll_motor",
        "right_hip_pitch_motor",
        "right_hip_roll_motor",
    }:
        limit = hip_pitch_roll_limit
    else:
        limit = leg_limit if leg_limit is not None and name in leg_motors else default_limit
    admission_limit = limit * torque_multiplier
    if observed > admission_limit:
        violations.append(f"{name}={observed:.6f}>{admission_limit:.6f}Nm")
if violations:
    raise SystemExit(
        "startup command torque preview exceeds per-motor limits: "
        + ", ".join(violations)
    )
print(
    "STARTUP_COMMAND_TORQUE_PASSED "
    f"maximum={report['preview_maximum_abs_estimated_torque_nm']:.6f}Nm "
    f"default_limit={default_limit:.6f}Nm leg_limit={leg_limit} "
    f"knee_limit={knee_limit} "
    f"hip_pitch_roll_limit={hip_pitch_roll_limit} ankle_limit={ankle_limit}"
    f" waist_roll_limit={waist_roll_limit}"
    f" dm3507_limit={dm3507_limit}"
    f" torque_multiplier={torque_multiplier}"
)
PY
PREFLIGHT_LIMIT_GATE_ARGS=()
if [[ "$PREFLIGHT_ENFORCE_POLICY_SOFT_LIMITS" == "1" ]]; then
  PREFLIGHT_LIMIT_GATE_ARGS=(
    --fail-on-violation-joint left_ankle_pitch_joint
    --fail-on-violation-joint right_ankle_pitch_joint
    --fail-on-violation-joint left_ankle_roll_joint
    --fail-on-violation-joint right_ankle_roll_joint
  )
else
  echo "POLICY_SOFT_LIMIT_GATE advisory-only; mechanical command envelope remains enforced"
fi
PYTHONPATH="$ROOT/src" "$ROOT/.venv/bin/python" \
  "$ROOT/tools/analyze_policy_joint_limit_clamps.py" \
  --trace "$PREFLIGHT_TRACE" \
  --joint-limit-candidates "$JOINT_LIMITS" \
  --contract "$CANDIDATE/deploy/contract.json" \
  "${PREFLIGHT_LIMIT_GATE_ARGS[@]}" \
  --maximum-gated-overshoot-rad "$PREFLIGHT_MAXIMUM_GATED_OVERSHOOT_RAD" \
  --maximum-gated-violation-fraction "$PREFLIGHT_MAXIMUM_GATED_VIOLATION_FRACTION" \
  --maximum-gated-consecutive-violation-ticks "$PREFLIGHT_MAXIMUM_GATED_CONSECUTIVE_TICKS" \
  --ignore-initial-ticks 50 \
  --maximum-horizontal-gravity-norm 0.10 \
  --output "$PREFLIGHT_REPORT" >/dev/null
echo "STARTUP_READINESS_PASSED report=$PREFLIGHT_REPORT"


if [[ "$PREFLIGHT_ONLY" == "1" ]]; then
  echo "PREFLIGHT_ONLY_PASSED active configuration exported; no motor enable or nonzero command was attempted"
  exit 0
fi

JOINT_HASH="$(PYTHONPATH="$ROOT/src" "$ROOT/.venv/bin/python" -c \
  'import json,sys; from open_sprite_runtime.native_ipc import ordered_name_hash; d=json.load(open(sys.argv[1])); print(hex(ordered_name_hash(d["joint_names"])))' \
  "$CANDIDATE/deploy/contract.json")"

echo "ACTIVE HARDWARE CONTROL: suspended protected-policy admission"
if [[ ${#COMMAND_PROFILE_ARGS[@]} -gt 0 ]]; then
  echo "TIMELINE 0-10s stand; 10-13s ramp to ${PROFILE_FORWARD_SPEED}m/s; 13-27s forward; 27-30s ramp to zero; 30-40s stand then disable"
  echo "Zero velocity is not proof of stable standing. Fall arrest must catch the robot after disable. Faults exit immediately."
fi
echo "Fixed tier: name=${TIER} duration=${DURATION}s gain_scale=${GAIN_SCALE} non_hip_multiplier=${NON_HIP_GAIN_MULTIPLIER} waist_roll_multiplier=${WAIST_ROLL_GAIN_MULTIPLIER} leg_multiplier=${LEG_GAIN_MULTIPLIER} leg_damping_multiplier=${LEG_DAMPING_MULTIPLIER} ankle_multiplier=${ANKLE_GAIN_MULTIPLIER} hip_multiplier=${HIP_GAIN_MULTIPLIER} DM3507_multiplier=${DM3507_GAIN_MULTIPLIER} head_multiplier=${HEAD_GAIN_MULTIPLIER} vx=${COMMAND_VX} hold=${STARTUP_HOLD_SECONDS}s ramp=${STARTUP_RAMP_SECONDS}s"
if [[ "$HARDWARE_FEEDBACK_CAPS" == "1" ]]; then
  echo "Feedback anomaly caps: min(protocol TMAX, mechanical peak); command staircase remains independently enforced"
fi
if [[ "$HARDWARE_FEEDBACK_CAPS" == "1" ]]; then
  echo "Per-motor command caps: hip pitch/roll=${HIP_PITCH_ROLL_COMMAND_CAP_NM}Nm; knees=${KNEE_COMMAND_CAP_NM:-$LEG_COMMAND_CAP_NM}Nm; other legs=${LEG_COMMAND_CAP_NM}Nm; ankles=${ANKLE_COMMAND_CAP_NM}Nm; waist roll=${WAIST_ROLL_COMMAND_CAP_NM}Nm; other motors=min(10% rated, ${MAXIMUM_COMMAND_TORQUE_NM}Nm), checked after MIT quantization"
elif [[ -n "$LEG_COMMAND_CAP_NM" ]]; then
  if [[ -n "$HIP_PITCH_ROLL_COMMAND_CAP_NM" ]]; then
    echo "Per-motor command/feedback caps: hip pitch/roll=${HIP_PITCH_ROLL_COMMAND_CAP_NM}/${HIP_PITCH_ROLL_FEEDBACK_CAP_NM}Nm; other legs=${LEG_COMMAND_CAP_NM}/${LEG_FEEDBACK_CAP_NM}Nm; ankles=${ANKLE_COMMAND_CAP_NM:-$LEG_COMMAND_CAP_NM}/${ANKLE_FEEDBACK_CAP_NM:-$LEG_FEEDBACK_CAP_NM}Nm; other motors=min(10% rated, ${MAXIMUM_COMMAND_TORQUE_NM}Nm), checked after MIT quantization"
  else
    echo "Per-motor command cap: legs=${LEG_COMMAND_CAP_NM}Nm; other motors=min(10% rated, ${MAXIMUM_COMMAND_TORQUE_NM}Nm), checked after MIT quantization"
  fi
else
  echo "Per-motor command cap: min(10% of rated torque, ${MAXIMUM_COMMAND_TORQUE_NM} Nm), checked after MIT quantization"
fi
if [[ -n "$DM3507_COMMAND_CAP_NM" ]]; then
  echo "DM-J3507 motor command cap: ${DM3507_COMMAND_CAP_NM}Nm for head and wrist motors; feedback anomaly cap remains 3Nm"
fi
if [[ -n "$WAIST_ROLL_COMMAND_CAP_NM" && "$HARDWARE_FEEDBACK_CAPS" != "1" ]]; then
  echo "Waist-roll command/feedback caps: ${WAIST_ROLL_COMMAND_CAP_NM}/${WAIST_ROLL_FEEDBACK_CAP_NM}Nm"
fi
if [[ -n "$SHOULDER_PITCH_FEEDBACK_CAP_NM" ]]; then
  echo "Shoulder-pitch command/feedback caps: 1.4/${SHOULDER_PITCH_FEEDBACK_CAP_NM}Nm"
fi
echo "Native watchdogs cover target age, status, hard position, speed, torque, temperature, and timing"
echo "Any fault or SIGINT/SIGTERM performs whole-body disable and verifies all 31 disabled"
echo "No mode switch and no zero-position reset are implemented in this path"
echo "$SUPPORT_INSTRUCTION; independent power safety operator must be ready"
echo "NATIVE_REPORT $NATIVE_REPORT"
echo "POLICY_REPORT $POLICY_REPORT"

OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 taskset -c 4 env PYTHONPATH="$ROOT/src" \
  "$ROOT/.venv/bin/python" "$ROOT/tools/native_walk40_adapter.py" --prepare-only \
  --socket "$SOCKET" \
  --hardware-config "$ROOT/config/hardware.sprite0825.kp110_kd5.json" \
  --contract "$CANDIDATE/deploy/contract.json" \
  --imu-device /dev/sprite0825-imu \
  --imu-baud 115200 \
  "${IMU_ARGS[@]}" \
  --vx "$COMMAND_VX" --vy 0 --yaw-rate 0 \
  "${COMMAND_PROFILE_ARGS[@]}" \
  "${POLICY_REPLAY_ARGS[@]}" \
  --joint-limit-candidates "$JOINT_LIMITS" \
  --gain-scale "$GAIN_SCALE" \
  "${JOINT_GAIN_ARGS[@]}" \
  "${JOINT_DAMPING_ARGS[@]}" \
  "${CLAMP_WATCHDOG_ARGS[@]}" \
  --physical-startup-hold-seconds "$STARTUP_HOLD_SECONDS" \
  --physical-startup-ramp-seconds "$STARTUP_RAMP_SECONDS" \
  --output "$POLICY_REPORT" \
  --trace-output "$POLICY_TRACE"

"$ROOT/build/native/sprite_can_shadow_quantized_v1" \
  --config "${POLICY_REPORT}.native/motors.tsv" \
  --duration "$DURATION" \
  --cpu 5 \
  --realtime-priority 50 \
  --output "$NATIVE_REPORT" \
  --ipc-socket "$SOCKET" \
  --policy-joint-hash "$JOINT_HASH" \
  --kinematics-config "${POLICY_REPORT}.native/kinematics.tsv" \
  --joint-safety-config "${POLICY_REPORT}.native/joint_safety.tsv" \
  --policy-actuation \
  "${TORQUE_SATURATION_ARGS[@]}" \
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
  "$ROOT/.venv/bin/python" "$ROOT/tools/native_walk40_adapter.py" \
  --socket "$SOCKET" \
  --hardware-config "$ROOT/config/hardware.sprite0825.kp110_kd5.json" \
  --contract "$CANDIDATE/deploy/contract.json" \
  --imu-device /dev/sprite0825-imu \
  --imu-baud 115200 \
  "${IMU_ARGS[@]}" \
  --vx "$COMMAND_VX" --vy 0 --yaw-rate 0 \
  "${COMMAND_PROFILE_ARGS[@]}" \
  "${POLICY_REPLAY_ARGS[@]}" \
  --joint-limit-candidates "$JOINT_LIMITS" \
  --gain-scale "$GAIN_SCALE" \
  "${JOINT_GAIN_ARGS[@]}" \
  "${JOINT_DAMPING_ARGS[@]}" \
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

PYTHONPATH="$ROOT/src" "$ROOT/.venv/bin/python" "$ROOT/tools/check_native_powered_completion.py" \
  --report "$NATIVE_REPORT" --names "${POLICY_REPORT}.native/names.txt" \
  --actor-exit "$POLICY_STATUS" --controller-exit "$NATIVE_STATUS"
