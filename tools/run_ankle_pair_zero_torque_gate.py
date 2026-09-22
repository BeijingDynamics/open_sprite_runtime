#!/usr/bin/env python3
"""Run the first powered zero-torque transport gate for one ankle pair."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from open_sprite_runtime.ankle_pair_commissioning import (
    ANKLE_GROUPS,
    run_ankle_pair_zero_torque_gate,
)
from open_sprite_runtime.contracts import PolicyContract
from open_sprite_runtime.damiao import endpoints_from_hardware_config
from open_sprite_runtime.motor_mapping import motor_map_from_hardware_config
from open_sprite_runtime.socketcan import (
    SocketCanMotorGroupMitWriter,
    audit_socketcan_active_fd_snapshot,
)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--side", choices=("left", "right"), required=True)
    parser.add_argument("--hardware", type=Path, required=True)
    parser.add_argument("--contract", type=Path, required=True)
    parser.add_argument("--socketcan-snapshot", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--robot-supported", action="store_true")
    parser.add_argument("--safety-operator-ready", action="store_true")
    parser.add_argument("--ankle-clear", action="store_true")
    parser.add_argument("--confirm-hardware-tx", required=True)
    args = parser.parse_args()
    acknowledgement = f"ENABLE_{args.side.upper()}_ANKLE_ZERO_TORQUE_500HZ"
    if not (
        args.robot_supported
        and args.safety_operator_ready
        and args.ankle_clear
        and args.confirm_hardware_tx == acknowledgement
    ):
        raise SystemExit("all physical safety confirmations and exact acknowledgement are required")

    hardware = json.loads(args.hardware.read_text(encoding="utf-8"))
    contract = PolicyContract.load(args.contract)
    mapping = motor_map_from_hardware_config(
        hardware, tuple(contract.data["joint_names"]), require_armable=False
    )
    pair_name = f"{args.side}_ankle"
    pair = mapping.differentials[pair_name]
    expected = ANKLE_GROUPS[args.side]
    interface = expected[0][1]
    all_endpoints = endpoints_from_hardware_config(hardware, expected_motor_count=31)
    by_name = {item.motor_name: item for item in all_endpoints}
    names = tuple(item[0] for item in expected)
    endpoints = tuple(by_name[name] for name in names)
    records = {name: hardware["motor_map"][name] for name in names}
    pair_record = hardware["differentials"][pair_name]
    if (
        pair_record.get("calibrated") is not True
        or tuple(pair_record["motor_names"]) != names
        or tuple(pair.motor_names) != names
    ):
        raise SystemExit("frozen calibrated ankle-pair invariant failed")
    for endpoint, identity in zip(endpoints, expected, strict=True):
        record = records[endpoint.motor_name]
        if (
            (endpoint.motor_name, endpoint.interface, endpoint.can_id, endpoint.master_id)
            != identity
            or record["model"] != "DM-J4310P-2EC (48V)"
        ):
            raise SystemExit(f"frozen ankle endpoint invariant failed for {endpoint.motor_name}")

    snapshot = json.loads(args.socketcan_snapshot.read_text(encoding="utf-8"))
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
        gate = run_ankle_pair_zero_torque_gate(
            writer,
            endpoints,
            pair,
            side=args.side,
            soft_position_rad=soft_limits,
        )
    report = {
        "mode": "first_powered_ankle_pair_zero_torque_500hz_gate",
        "side": args.side,
        "preflight": preflight.to_dict(),
        "calibration_source": pair_record.get("source"),
        "joint_to_motor_matrix": pair_record["joint_to_motor_matrix"],
        "motor_zero_rad": pair_record["motor_zero_rad"],
        "fixed_safety_envelope": {
            "duration_s": 2.0,
            "rate_hz_per_motor": 500.0,
            "embedded_kp": 0.0,
            "embedded_kd": 0.0,
            "feedforward_torque_nm": 0.0,
            "maximum_motor_position_drift_rad": 0.05,
            "maximum_motor_velocity_rad_s": 0.2,
            "maximum_estimated_torque_nm": 0.1,
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
        raise SystemExit(f"{args.side} ankle zero-torque gate failed closed")


if __name__ == "__main__":
    main()
