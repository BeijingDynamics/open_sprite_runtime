#!/usr/bin/env python3
"""Hold exactly one Sprite0825 hip/knee group at measured positions."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from open_sprite_runtime.damiao import endpoints_from_hardware_config
from open_sprite_runtime.motor_group_hold import (
    LEFT_PROXIMAL_LEG_GROUP,
    RIGHT_PROXIMAL_LEG_GROUP,
    run_proximal_leg_group_low_gain_hold,
)
from open_sprite_runtime.socketcan import (
    SocketCanMotorGroupMitWriter,
    audit_socketcan_active_fd_snapshot,
)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--side", choices=("left", "right"), required=True)
    parser.add_argument("--hardware", type=Path, required=True)
    parser.add_argument("--socketcan-snapshot", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--robot-supported", action="store_true")
    parser.add_argument("--safety-operator-ready", action="store_true")
    parser.add_argument("--leg-clear", action="store_true")
    parser.add_argument("--confirm-hardware-tx", required=True)
    args = parser.parse_args()
    acknowledgement = f"ENABLE_{args.side.upper()}_PROXIMAL_LEG_LOW_GAIN_HOLD"
    if not (
        args.robot_supported
        and args.safety_operator_ready
        and args.leg_clear
        and args.confirm_hardware_tx == acknowledgement
    ):
        raise SystemExit("all physical safety confirmations and exact acknowledgement are required")

    group, interface = (
        (LEFT_PROXIMAL_LEG_GROUP, "kcan1")
        if args.side == "left"
        else (RIGHT_PROXIMAL_LEG_GROUP, "kcan2")
    )
    hardware = json.loads(args.hardware.read_text(encoding="utf-8"))
    snapshot = json.loads(args.socketcan_snapshot.read_text(encoding="utf-8"))
    all_endpoints = endpoints_from_hardware_config(hardware, expected_motor_count=31)
    by_name = {item.motor_name: item for item in all_endpoints}
    names = tuple(item[0] for item in group)
    endpoints = tuple(by_name[name] for name in names)
    records = {name: hardware["motor_map"][name] for name in names}
    for endpoint, expected in zip(endpoints, group, strict=True):
        record = records[endpoint.motor_name]
        if (
            (endpoint.motor_name, endpoint.interface, endpoint.can_id, endpoint.master_id)
            != expected
            or record["model"] != "DM-J4340P-2EC V1.1 (48V)"
            or int(record["policy_to_motor_sign"]) != 1
        ):
            raise SystemExit(f"frozen proximal-leg invariant failed for {endpoint.motor_name}")

    preflight = audit_socketcan_active_fd_snapshot(snapshot, interface)
    if not preflight.passed:
        raise SystemExit(f"{interface} active CAN-FD preflight failed: " + "; ".join(preflight.errors))
    soft_limits = {
        name: tuple(float(value) for value in records[name]["soft_limit_rad"])
        for name in names
    }
    with SocketCanMotorGroupMitWriter.open(
        interface, preflight, tuple((item.motor_name, item.can_id) for item in endpoints)
    ) as writer:
        hold = run_proximal_leg_group_low_gain_hold(
            writer, endpoints, side=args.side, soft_position_rad=soft_limits
        )
    report = {
        "mode": "first_powered_fixed_proximal_leg_current_position_hold",
        "side": args.side,
        "group": "hip_pitch_roll_yaw_and_knee_only",
        "excluded": ["ankle_motor_a", "ankle_motor_b", "waist", "head"],
        "preflight": preflight.to_dict(),
        "fixed_safety_envelope": {
            "duration_s": 2.0,
            "rate_hz": 50.0,
            "kp": 0.2,
            "kd": 0.05,
            "feedforward_torque_nm": 0.0,
            "maximum_position_error_rad": 0.05,
            "maximum_velocity_rad_s": 0.2,
            "maximum_estimated_torque_nm_per_motor": 0.5,
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
        raise SystemExit(f"{args.side} proximal-leg low-gain hold failed closed")


if __name__ == "__main__":
    main()
