#!/usr/bin/env python3
"""Run the frozen visible ten-degree Sprite0825 head-yaw test."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

from open_sprite_runtime.damiao import endpoints_from_hardware_config
from open_sprite_runtime.single_joint_hold import run_head_yaw_low_gain_motion
from open_sprite_runtime.socketcan import (
    SocketCanSingleMotorMitWriter,
    audit_socketcan_active_fd_snapshot,
)


ACKNOWLEDGEMENT = "ENABLE_HEAD_YAW_VISIBLE_10DEG_MOTION"


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
        endpoint.interface, preflight, endpoint.motor_name, endpoint.can_id
    ) as writer:
        motion = run_head_yaw_low_gain_motion(
            writer,
            endpoint,
            soft_position_rad=tuple(record["soft_limit_rad"]),
            excursion_rad=math.radians(10.0),
            transition_s=4.0,
            dwell_s=1.0,
            kp=2.0,
            maximum_position_error_rad=0.08,
            maximum_velocity_rad_s=0.8,
            maximum_torque_nm=0.25,
        )
    report = {
        "mode": "third_powered_single_joint_visible_ten_degree_excursion",
        "manufacturer_manual": "DM-J3507-2EC Gear Motor User Manual V1.1 2026-04-16",
        "preflight": preflight.to_dict(),
        "fixed_trajectory": {
            "sequence": "current -> current+10deg -> current-10deg -> current",
            "interpolation": "quintic_smoothstep",
            "excursion_rad": math.radians(10.0),
            "transition_s": 4.0,
            "dwell_s": 1.0,
            "duration_s": 15.0,
            "rate_hz": 50.0,
            "kp": 2.0,
            "kd": 0.2,
            "feedforward_torque_nm": 0.0,
            "maximum_position_error_rad": 0.08,
            "maximum_velocity_rad_s": 0.8,
            "maximum_estimated_torque_nm": 0.25,
            "mos_temperature_limit_c": 100,
            "rotor_temperature_limit_c": 80,
        },
        "motion": motion.to_dict(),
        "automatic_mode_switch_attempts": 0,
        "passed": preflight.passed and motion.passed,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    if not report["passed"]:
        raise SystemExit("head-yaw visible ten-degree motion failed safety qualification")


if __name__ == "__main__":
    main()
