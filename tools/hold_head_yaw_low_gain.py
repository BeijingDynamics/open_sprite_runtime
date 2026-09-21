#!/usr/bin/env python3
"""Run the frozen first powered hold on Sprite0825 head yaw only."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from open_sprite_runtime.damiao import endpoints_from_hardware_config
from open_sprite_runtime.single_joint_hold import run_head_yaw_low_gain_hold
from open_sprite_runtime.socketcan import (
    SocketCanSingleMotorMitWriter,
    audit_socketcan_active_fd_snapshot,
)


ACKNOWLEDGEMENT = "ENABLE_HEAD_YAW_LOW_GAIN_HOLD"


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--hardware", type=Path, required=True)
    parser.add_argument("--socketcan-snapshot", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--robot-supported", action="store_true")
    parser.add_argument("--safety-operator-ready", action="store_true")
    parser.add_argument("--head-yaw-mechanically-free", action="store_true")
    parser.add_argument("--confirm-hardware-tx", required=True)
    args = parser.parse_args()
    if not (
        args.robot_supported
        and args.safety_operator_ready
        and args.head_yaw_mechanically_free
        and args.confirm_hardware_tx == ACKNOWLEDGEMENT
    ):
        raise SystemExit("all physical safety confirmations and exact acknowledgement are required")

    hardware = json.loads(args.hardware.read_text(encoding="utf-8"))
    snapshot = json.loads(args.socketcan_snapshot.read_text(encoding="utf-8"))
    endpoints = endpoints_from_hardware_config(hardware, expected_motor_count=31)
    endpoint = next(item for item in endpoints if item.motor_name == "head_yaw_motor")
    record = hardware["motor_map"][endpoint.motor_name]
    if (
        endpoint.interface != "kcan3"
        or endpoint.can_id != 8
        or endpoint.master_id != 0x18
        or record["model"] != "DM-J3507-2EC (48V)"
        or float(record["rated_torque_nm"]) != 0.8
        or float(record["peak_torque_nm"]) != 3.0
    ):
        raise SystemExit("frozen head-yaw endpoint/model/nameplate invariant failed")
    preflight = audit_socketcan_active_fd_snapshot(snapshot, "kcan3")
    if not preflight.passed:
        raise SystemExit("kcan3 active CAN-FD preflight failed: " + "; ".join(preflight.errors))

    with SocketCanSingleMotorMitWriter.open(
        endpoint.interface,
        preflight,
        endpoint.motor_name,
        endpoint.can_id,
    ) as writer:
        hold = run_head_yaw_low_gain_hold(
            writer,
            endpoint,
            soft_position_rad=tuple(record["soft_limit_rad"]),
        )
    report = {
        "mode": "first_powered_single_joint_current_position_hold",
        "manufacturer_manual": "DM-J3507-2EC Gear Motor User Manual V1.1 2026-04-16",
        "preflight": preflight.to_dict(),
        "fixed_safety_envelope": {
            "duration_s": 2.0,
            "rate_hz": 50.0,
            "kp": 0.2,
            "kd": 0.05,
            "feedforward_torque_nm": 0.0,
            "maximum_position_error_rad": 0.05,
            "maximum_velocity_rad_s": 0.2,
            "maximum_estimated_torque_nm": 0.1,
            "mos_temperature_limit_c": 100,
            "rotor_temperature_limit_c": 80,
        },
        "hold": hold.to_dict(),
        "automatic_mode_switch_attempts": 0,
        "passed": preflight.passed and hold.passed,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    if not report["passed"]:
        raise SystemExit("head-yaw low-gain hold failed closed")


if __name__ == "__main__":
    main()
