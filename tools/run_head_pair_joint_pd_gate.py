#!/usr/bin/env python3
"""Run the first powered low-PD gate for the head pitch/roll differential pair."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from open_sprite_runtime.ankle_pair_commissioning import HEAD_GROUP, run_head_pair_joint_pd_gate
from open_sprite_runtime.contracts import PolicyContract
from open_sprite_runtime.damiao import endpoints_from_hardware_config
from open_sprite_runtime.motor_mapping import motor_map_from_hardware_config
from open_sprite_runtime.socketcan import (
    SocketCanMotorGroupMitWriter,
    audit_socketcan_active_fd_snapshot,
)


ACKNOWLEDGEMENT = "ENABLE_HEAD_PAIR_LOW_JOINT_PD_500HZ"


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--hardware", type=Path, required=True)
    parser.add_argument("--contract", type=Path, required=True)
    parser.add_argument("--socketcan-snapshot", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--robot-supported", action="store_true")
    parser.add_argument("--safety-operator-ready", action="store_true")
    parser.add_argument("--head-clear", action="store_true")
    parser.add_argument("--confirm-hardware-tx", required=True)
    args = parser.parse_args()
    if not (
        args.robot_supported
        and args.safety_operator_ready
        and args.head_clear
        and args.confirm_hardware_tx == ACKNOWLEDGEMENT
    ):
        raise SystemExit("all physical safety confirmations and exact acknowledgement are required")

    hardware = json.loads(args.hardware.read_text(encoding="utf-8"))
    contract = PolicyContract.load(args.contract)
    mapping = motor_map_from_hardware_config(
        hardware, tuple(contract.data["joint_names"]), require_armable=False
    )
    pair = mapping.differentials["head"]
    all_endpoints = endpoints_from_hardware_config(hardware, expected_motor_count=31)
    by_name = {item.motor_name: item for item in all_endpoints}
    names = tuple(item[0] for item in HEAD_GROUP)
    endpoints = tuple(by_name[name] for name in names)
    records = {name: hardware["motor_map"][name] for name in names}
    pair_record = hardware["differentials"]["head"]
    if (
        pair_record.get("calibrated") is not True
        or tuple(pair_record["motor_names"]) != names
        or tuple(pair.motor_names) != names
    ):
        raise SystemExit("frozen calibrated head-pair invariant failed")
    for endpoint, identity in zip(endpoints, HEAD_GROUP, strict=True):
        record = records[endpoint.motor_name]
        if (
            (endpoint.motor_name, endpoint.interface, endpoint.can_id, endpoint.master_id)
            != identity
            or record["model"] != "DM-J3507-2EC (48V)"
        ):
            raise SystemExit(f"frozen head endpoint invariant failed for {endpoint.motor_name}")

    snapshot = json.loads(args.socketcan_snapshot.read_text(encoding="utf-8"))
    preflight = audit_socketcan_active_fd_snapshot(snapshot, "kcan2")
    if not preflight.passed:
        raise SystemExit("kcan2 active CAN-FD preflight failed: " + "; ".join(preflight.errors))
    soft_limits = {
        name: tuple(float(value) for value in records[name]["soft_limit_rad"])
        for name in names
    }
    with SocketCanMotorGroupMitWriter.open(
        "kcan2", preflight, tuple((item.motor_name, item.can_id) for item in endpoints)
    ) as writer:
        gate = run_head_pair_joint_pd_gate(
            writer, endpoints, pair, soft_position_rad=soft_limits
        )
    report = {
        "mode": "first_powered_head_pair_low_joint_pd_500hz_gate",
        "preflight": preflight.to_dict(),
        "calibration_source": pair_record.get("source"),
        "joint_to_motor_matrix": pair_record["joint_to_motor_matrix"],
        "motor_zero_rad": pair_record["motor_zero_rad"],
        "fixed_safety_envelope": {
            "duration_s": 2.0,
            "rate_hz_per_motor": 500.0,
            "embedded_kp": 0.0,
            "embedded_kd": 0.0,
            "joint_kp_nm_rad": 0.2,
            "joint_kd_nm_s_rad": 0.03,
            "maximum_joint_torque_nm": 0.05,
            "maximum_motor_torque_nm": 0.10,
            "maximum_motor_position_drift_rad": 0.05,
            "maximum_motor_velocity_rad_s": 0.2,
        },
        "gate": gate.to_dict(),
        "automatic_mode_switch_attempts": 0,
        "automatic_zero_reset_attempts": 0,
        "passed": preflight.passed and gate.passed,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    if not report["passed"]:
        raise SystemExit("head differential low-PD gate failed closed")


if __name__ == "__main__":
    main()
