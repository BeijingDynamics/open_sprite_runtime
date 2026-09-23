#!/usr/bin/env python3
"""Run frozen joint-space +/-5 degree motion on Sprite0825 right hip yaw."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

from open_sprite_runtime.damiao import endpoints_from_hardware_config
from open_sprite_runtime.single_joint_hold import (
    run_right_hip_yaw_low_gain_motion,
    run_right_hip_yaw_medium_gain_motion,
)
from open_sprite_runtime.socketcan import (
    SocketCanSingleMotorMitWriter,
    audit_socketcan_active_fd_snapshot,
)


ACKNOWLEDGEMENT = "ENABLE_RIGHT_HIP_YAW_5DEG_MOTION"
MEDIUM_ACKNOWLEDGEMENT = "ENABLE_RIGHT_HIP_YAW_5DEG_MEDIUM_GAIN_MOTION"


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--hardware", type=Path, required=True)
    parser.add_argument("--socketcan-snapshot", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--robot-suspended", action="store_true")
    parser.add_argument("--safety-operator-ready", action="store_true")
    parser.add_argument("--right-leg-clear", action="store_true")
    parser.add_argument("--confirm-hardware-tx", required=True)
    parser.add_argument("--medium-gain", action="store_true")
    args = parser.parse_args()
    expected_acknowledgement = (
        MEDIUM_ACKNOWLEDGEMENT if args.medium_gain else ACKNOWLEDGEMENT
    )
    if not (
        args.robot_suspended
        and args.safety_operator_ready
        and args.right_leg_clear
        and args.confirm_hardware_tx == expected_acknowledgement
    ):
        raise SystemExit("all physical safety confirmations and exact acknowledgement are required")

    hardware = json.loads(args.hardware.read_text(encoding="utf-8"))
    snapshot = json.loads(args.socketcan_snapshot.read_text(encoding="utf-8"))
    endpoints = endpoints_from_hardware_config(hardware, expected_motor_count=31)
    endpoint = next(item for item in endpoints if item.motor_name == "right_hip_yaw_motor")
    record = hardware["motor_map"][endpoint.motor_name]
    if (
        endpoint.interface != "kcan2"
        or endpoint.can_id != 3
        or endpoint.master_id != 0x13
        or record["model"] != "DM-J4340P-2EC V1.1 (48V)"
        or float(record["rated_torque_nm"]) != 14.0
        or float(record["peak_torque_nm"]) != 40.0
        or int(record["policy_to_motor_sign"]) != 1
    ):
        raise SystemExit("frozen right-hip-yaw endpoint/model/mapping invariant failed")
    preflight = audit_socketcan_active_fd_snapshot(snapshot, "kcan2")
    if not preflight.passed:
        raise SystemExit("kcan2 active CAN-FD preflight failed: " + "; ".join(preflight.errors))

    with SocketCanSingleMotorMitWriter.open(
        endpoint.interface, preflight, endpoint.motor_name, endpoint.can_id
    ) as writer:
        runner = (
            run_right_hip_yaw_medium_gain_motion
            if args.medium_gain
            else run_right_hip_yaw_low_gain_motion
        )
        motion = runner(
            writer, endpoint, soft_position_rad=tuple(record["soft_limit_rad"])
        )
    kp = 20.0 if args.medium_gain else 8.0
    kd = 0.5 if args.medium_gain else 0.3
    maximum_position_error_rad = 0.15 if args.medium_gain else 0.12
    maximum_velocity_rad_s = 0.8 if args.medium_gain else 0.6
    maximum_torque_nm = 2.5 if args.medium_gain else 1.0
    minimum_required_excursion_rad = math.radians(2.5)
    maximum_return_error_rad = math.radians(2.5)
    positive_excursion_rad = (
        motion.measured_maximum_position_rad - motion.initial_position_rad
    )
    negative_excursion_rad = (
        motion.initial_position_rad - motion.measured_minimum_position_rad
    )
    return_error_rad = abs(
        motion.final_measured_position_rad - motion.initial_position_rad
    )
    direction_gate = {
        "minimum_required_excursion_rad": minimum_required_excursion_rad,
        "positive_excursion_rad": positive_excursion_rad,
        "negative_excursion_rad": negative_excursion_rad,
        "maximum_return_error_rad": maximum_return_error_rad,
        "return_error_rad": return_error_rad,
        "positive_passed": positive_excursion_rad >= minimum_required_excursion_rad,
        "negative_passed": negative_excursion_rad >= minimum_required_excursion_rad,
        "return_passed": return_error_rad <= maximum_return_error_rad,
    }
    direction_gate["passed"] = all(
        direction_gate[key]
        for key in ("positive_passed", "negative_passed", "return_passed")
    )
    report = {
        "mode": "suspended_right_hip_yaw_direction_audit_medium_gain"
        if args.medium_gain
        else "suspended_right_hip_yaw_direction_audit_low_gain",
        "motor": "right_hip_yaw_motor",
        "policy_to_motor_sign": 1,
        "preflight": preflight.to_dict(),
        "fixed_trajectory": {
            "joint_sequence": "current -> current+5deg -> current-5deg -> current",
            "excursion_rad": math.radians(5.0),
            "interpolation": "quintic_smoothstep",
            "transition_s": 4.0,
            "dwell_s": 1.0,
            "duration_s": 15.0,
            "rate_hz": 50.0,
            "kp": kp,
            "kd": kd,
            "feedforward_torque_nm": 0.0,
            "maximum_position_error_rad": maximum_position_error_rad,
            "maximum_velocity_rad_s": maximum_velocity_rad_s,
            "maximum_estimated_torque_nm": maximum_torque_nm,
        },
        "motion": motion.to_dict(),
        "direction_gate": direction_gate,
        "automatic_mode_switch_attempts": 0,
        "automatic_zero_reset_attempts": 0,
        "passed": preflight.passed and motion.passed and direction_gate["passed"],
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    if not report["passed"]:
        raise SystemExit("right-hip-yaw five-degree motion failed closed")


if __name__ == "__main__":
    main()
