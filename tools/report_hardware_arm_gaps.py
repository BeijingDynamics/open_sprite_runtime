#!/usr/bin/env python3
"""Summarize the remaining Sprite0825 arming inputs without touching hardware."""

from __future__ import annotations

import argparse
from collections import defaultdict
import json
from pathlib import Path

from open_sprite_runtime.hardware import MOTOR_REQUIRED_FIELDS, validate_hardware_inventory


def build_report(hardware: dict, policy_joint_names: list[str]) -> dict:
    validation = validate_hardware_inventory(hardware, policy_joint_names)
    grouped: dict[str, dict] = {}
    by_model: dict[str, list[tuple[str, dict]]] = defaultdict(list)
    for motor_name, record in hardware.get("motor_map", {}).items():
        by_model[str(record.get("model") or "UNKNOWN")].append((motor_name, record))
    for model, rows in sorted(by_model.items()):
        missing_by_motor = {
            name: [field for field in MOTOR_REQUIRED_FIELDS if record.get(field) is None]
            for name, record in rows
        }
        bad_mit_source = [
            name
            for name, record in rows
            if record.get("mit_ranges", {}).get("source") != "motor_register_readback"
        ]
        grouped[model] = {
            "count": len(rows),
            "motor_names": [name for name, _record in rows],
            "missing_fields_by_motor": missing_by_motor,
            "missing_fields_all_motors": sorted(
                set.intersection(*(set(fields) for fields in missing_by_motor.values()))
            ),
            "missing_fields_any_motor": sorted(
                set.union(*(set(fields) for fields in missing_by_motor.values()))
            ),
            "mit_range_register_readback_required": bad_mit_source,
        }

    imu = hardware.get("imu", {})
    imu_blockers = []
    if imu.get("configured") is not True:
        imu_blockers.append("imu.configured is false")
    if not str(imu.get("device_path") or "").startswith("/dev/sprite-"):
        imu_blockers.append("dedicated stable udev symlink is not installed")
    if imu.get("measured_yaw_drift_deg_per_min") is None:
        imu_blockers.append("stationary yaw drift has not been measured")

    estop = hardware.get("estop", {})
    estop_blockers = []
    if estop.get("configured") is not True:
        estop_blockers.append("estop.configured is false")
    if not estop.get("hardware_chain"):
        estop_blockers.append("independent hardware chain is not described")

    return {
        "mode": "offline_hardware_arm_gap_report_no_hardware_io",
        "configured": hardware.get("configured") is True,
        "inventory_valid": validation.valid,
        "can_commissioning_shadow": hardware.get("can_adapter", {}).get(
            "commissioning_shadow"
        ),
        "motor_groups": grouped,
        "imu_blockers": imu_blockers,
        "estop_blockers": estop_blockers,
        "validation_errors": list(validation.errors),
    }


def markdown(report: dict) -> str:
    lines = [
        "# Sprite0825 hardware arming gaps",
        "",
        "Generated offline. This report performs no CAN, serial, or actuator I/O.",
        "",
        "## Motor groups",
        "",
    ]
    for model, group in report["motor_groups"].items():
        lines.append(f"- `{model}`: {group['count']} motors")
        common = group["missing_fields_all_motors"]
        partial = sorted(set(group["missing_fields_any_motor"]) - set(common))
        lines.append(f"  Missing on every motor: {', '.join(common) if common else 'none'}")
        lines.append(f"  Missing on some motors: {', '.join(partial) if partial else 'none'}")
        if group["mit_range_register_readback_required"]:
            lines.append(
                "  MIT register readback required for: "
                + ", ".join(group["mit_range_register_readback_required"])
            )
    lines.extend(["", "## IMU", ""])
    lines.extend(f"- {item}" for item in report["imu_blockers"])
    lines.extend(["", "## Independent e-stop", ""])
    lines.extend(f"- {item}" for item in report["estop_blockers"])
    lines.extend(["", "## Validator", ""])
    lines.append(f"- Remaining errors: {len(report['validation_errors'])}")
    lines.extend(f"- {item}" for item in report["validation_errors"])
    return "\n".join(lines) + "\n"


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--hardware", required=True)
    parser.add_argument("--contract", required=True)
    parser.add_argument("--json-output", required=True)
    parser.add_argument("--markdown-output", required=True)
    args = parser.parse_args()
    hardware = json.loads(Path(args.hardware).read_text(encoding="utf-8"))
    contract = json.loads(Path(args.contract).read_text(encoding="utf-8"))
    report = build_report(hardware, contract["joint_names"])
    Path(args.json_output).write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    Path(args.markdown_output).write_text(markdown(report), encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
