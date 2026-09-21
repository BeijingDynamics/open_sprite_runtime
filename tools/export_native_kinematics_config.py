#!/usr/bin/env python3
"""Export the frozen 31-joint/31-motor affine map for native preview."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np

from open_sprite_runtime.contracts import PolicyContract
from open_sprite_runtime.motor_mapping import motor_map_from_hardware_config


def _native_motor_order(hardware: dict) -> tuple[str, ...]:
    interfaces = hardware["can_adapter"]["interfaces"]
    return tuple(
        name
        for name, _record in sorted(
            hardware["motor_map"].items(),
            key=lambda item: (
                interfaces[int(item[1]["can_channel"])],
                int(item[1]["can_id"]),
            ),
        )
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--hardware", type=Path, required=True)
    parser.add_argument("--contract", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    hardware = json.loads(args.hardware.read_text(encoding="utf-8"))
    contract = PolicyContract.load(args.contract)
    joint_names = tuple(contract.data["joint_names"])
    motor_names = _native_motor_order(hardware)
    mapping = motor_map_from_hardware_config(
        hardware, joint_names, require_armable=False
    )

    zero = np.zeros(31, dtype=np.float64)
    offset_map = mapping.joint_to_motor_positions(zero)
    offset = np.asarray([offset_map[name] for name in motor_names])
    joint_to_motor = np.zeros((31, 31), dtype=np.float64)
    for column in range(31):
        basis = zero.copy()
        basis[column] = 1.0
        values = mapping.joint_to_motor_positions(basis)
        joint_to_motor[:, column] = (
            np.asarray([values[name] for name in motor_names]) - offset
        )
    motor_to_joint = np.linalg.inv(joint_to_motor)
    if not np.allclose(motor_to_joint @ joint_to_motor, np.eye(31), atol=1.0e-12):
        raise SystemExit("native kinematics matrices are not inverse")

    coefficient_fields = tuple(f"c{index}" for index in range(31))
    fields = ("kind", "name", "offset_or_effort") + coefficient_fields
    effort = np.asarray(contract.data["effort_limit"], dtype=np.float64)
    rows = []
    for index, name in enumerate(motor_names):
        row = {
            "kind": "motor",
            "name": name,
            "offset_or_effort": float(offset[index]),
        }
        row.update(zip(coefficient_fields, map(float, joint_to_motor[index]), strict=True))
        rows.append(row)
    for index, name in enumerate(joint_names):
        row = {
            "kind": "joint",
            "name": name,
            "offset_or_effort": float(effort[index]),
        }
        row.update(zip(coefficient_fields, map(float, motor_to_joint[index]), strict=True))
        rows.append(row)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(
            stream, fieldnames=fields, dialect="excel-tab", lineterminator="\n"
        )
        writer.writeheader()
        writer.writerows(rows)
    print(
        f"EXPORTED motors={len(motor_names)} joints={len(joint_names)} "
        f"condition={np.linalg.cond(joint_to_motor):.6f} output={args.output}"
    )


if __name__ == "__main__":
    main()
