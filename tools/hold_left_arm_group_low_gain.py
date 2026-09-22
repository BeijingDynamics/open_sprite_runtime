#!/usr/bin/env python3
"""Hold the exact seven Sprite0825 left-arm motors at measured positions."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from open_sprite_runtime.damiao import endpoints_from_hardware_config
from open_sprite_runtime.motor_group_hold import (
    LEFT_ARM_GROUP,
    run_left_arm_group_low_gain_hold,
)
from open_sprite_runtime.socketcan import (
    SocketCanMotorGroupMitWriter,
    audit_socketcan_active_fd_snapshot,
)


ACKNOWLEDGEMENT = "ENABLE_LEFT_ARM_GROUP_LOW_GAIN_HOLD"


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--hardware", type=Path, required=True)
    parser.add_argument("--socketcan-snapshot", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--robot-supported", action="store_true")
    parser.add_argument("--safety-operator-ready", action="store_true")
    parser.add_argument("--left-arm-clear", action="store_true")
    parser.add_argument("--confirm-hardware-tx", required=True)
    args = parser.parse_args()
    if not (
        args.robot_supported
        and args.safety_operator_ready
        and args.left_arm_clear
        and args.confirm_hardware_tx == ACKNOWLEDGEMENT
    ):
        raise SystemExit("all physical safety confirmations and exact acknowledgement are required")

    hardware = json.loads(args.hardware.read_text(encoding="utf-8"))
    snapshot = json.loads(args.socketcan_snapshot.read_text(encoding="utf-8"))
    all_endpoints = endpoints_from_hardware_config(hardware, expected_motor_count=31)
    by_name = {item.motor_name: item for item in all_endpoints}
    names = tuple(item[0] for item in LEFT_ARM_GROUP)
    endpoints = tuple(by_name[name] for name in names)
    records = {name: hardware["motor_map"][name] for name in names}
    expected_models = (
        "DM-J4340P-2EC V1.1 (48V)",
        "DM-J4340P-2EC V1.1 (48V)",
        "DM-J4310P-2EC (48V)",
        "DM-J4310P-2EC (48V)",
        "DM-J4310P-2EC (48V)",
        "DM-J3507-2EC (48V)",
        "DM-J3507-2EC (48V)",
    )
    expected_signs = (1, 1, 1, 1, 1, -1, -1)
    for endpoint, expected, model, sign in zip(
        endpoints, LEFT_ARM_GROUP, expected_models, expected_signs, strict=True
    ):
        record = records[endpoint.motor_name]
        if (
            (endpoint.motor_name, endpoint.interface, endpoint.can_id, endpoint.master_id)
            != expected
            or record["model"] != model
            or int(record["policy_to_motor_sign"]) != sign
        ):
            raise SystemExit(f"frozen left-arm invariant failed for {endpoint.motor_name}")

    preflight = audit_socketcan_active_fd_snapshot(snapshot, "kcan3")
    if not preflight.passed:
        raise SystemExit("kcan3 active CAN-FD preflight failed: " + "; ".join(preflight.errors))
    motor_endpoints = tuple((item.motor_name, item.can_id) for item in endpoints)
    soft_limits = {
        name: tuple(float(value) for value in records[name]["soft_limit_rad"])
        for name in names
    }
    with SocketCanMotorGroupMitWriter.open(
        "kcan3", preflight, motor_endpoints
    ) as writer:
        hold = run_left_arm_group_low_gain_hold(
            writer, endpoints, soft_position_rad=soft_limits
        )
    report = {
        "mode": "first_powered_fixed_left_arm_current_position_hold",
        "group": "left_arm_seven_motors_without_head_yaw",
        "preflight": preflight.to_dict(),
        "fixed_safety_envelope": {
            "duration_s": 2.0,
            "rate_hz": 50.0,
            "kp": 0.2,
            "kd": 0.05,
            "feedforward_torque_nm": 0.0,
            "maximum_position_error_rad": 0.05,
            "maximum_velocity_rad_s": 0.2,
            "maximum_estimated_torque_nm_by_motor": {
                name: (0.5 if name in {
                    "left_shoulder_pitch_motor",
                    "left_shoulder_roll_motor",
                } else 0.1)
                for name in names
            },
        },
        "hold": hold.to_dict(),
        "automatic_mode_switch_attempts": 0,
        "automatic_zero_reset_attempts": 0,
        "passed": preflight.passed and hold.passed,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    if not report["passed"]:
        raise SystemExit("left-arm group low-gain hold failed closed")


if __name__ == "__main__":
    main()
