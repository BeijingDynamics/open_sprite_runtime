#!/usr/bin/env python3
"""Export the validated JSON inventory to a small native-runtime TSV."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path


FIELDS = (
    "motor_name",
    "interface",
    "can_id",
    "master_id",
    "position_min_rad",
    "position_max_rad",
    "velocity_min_rad_s",
    "velocity_max_rad_s",
    "torque_min_nm",
    "torque_max_nm",
    "soft_position_min_rad",
    "soft_position_max_rad",
    "hard_position_min_rad",
    "hard_position_max_rad",
    "deployment_velocity_max_rad_s",
    "mechanical_peak_torque_nm",
    "commissioning_torque_cap_nm",
    "feedback_torque_cap_nm",
    "mos_temperature_limit_c",
    "rotor_temperature_limit_c",
    "poll_rate_hz",
)


def _parse_motor_caps(values: list[str], option: str) -> dict[str, float]:
    caps: dict[str, float] = {}
    for value in values:
        try:
            name, raw_cap = value.rsplit("=", 1)
            cap = float(raw_cap)
        except ValueError as exc:
            raise SystemExit(f"{option} must use MOTOR_NAME=NM") from exc
        if not name or cap <= 0.0:
            raise SystemExit(f"{option} must use MOTOR_NAME=positive_NM")
        if name in caps:
            raise SystemExit(f"duplicate {option} for {name}")
        caps[name] = cap
    return caps


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--hardware", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--maximum-commissioning-torque-nm",
        type=float,
        help=(
            "Optional absolute per-motor command cap. The exported cap is the "
            "minimum of this value and 10%% of each motor's rated torque."
        ),
    )
    parser.add_argument(
        "--motor-command-cap",
        action="append",
        default=[],
        metavar="MOTOR_NAME=NM",
        help="Override the commissioning command cap for one named motor.",
    )
    parser.add_argument(
        "--motor-feedback-cap",
        action="append",
        default=[],
        metavar="MOTOR_NAME=NM",
        help="Override the independent feedback anomaly cap for one named motor.",
    )
    args = parser.parse_args()
    if (
        args.maximum_commissioning_torque_nm is not None
        and args.maximum_commissioning_torque_nm <= 0.0
    ):
        raise SystemExit("--maximum-commissioning-torque-nm must be positive")
    hardware = json.loads(args.hardware.read_text(encoding="utf-8"))
    command_caps = _parse_motor_caps(args.motor_command_cap, "--motor-command-cap")
    feedback_caps = _parse_motor_caps(args.motor_feedback_cap, "--motor-feedback-cap")
    motor_names = set(hardware["motor_map"])
    unknown = (set(command_caps) | set(feedback_caps)) - motor_names
    if unknown:
        raise SystemExit(f"unknown motor cap override(s): {', '.join(sorted(unknown))}")
    interfaces = hardware["can_adapter"]["interfaces"]
    model_specs = hardware["motor_model_specs"]
    rows = []
    for name, motor in hardware["motor_map"].items():
        ranges = motor["mit_ranges"]
        specs = model_specs[motor["model"]]
        mechanical_peak = float(motor["peak_torque_nm"])
        protocol_torque_cap = min(map(abs, map(float, ranges["torque_nm"])))
        command_cap = command_caps.get(
            name,
            min(
                0.1 * float(motor["rated_torque_nm"]),
                args.maximum_commissioning_torque_nm
                if args.maximum_commissioning_torque_nm is not None
                else float("inf"),
            ),
        )
        feedback_cap = feedback_caps.get(
            name, 0.1 * float(motor["rated_torque_nm"])
        )
        if command_cap > min(mechanical_peak, protocol_torque_cap):
            raise SystemExit(f"command cap exceeds hardware/protocol limit for {name}")
        if feedback_cap < command_cap:
            raise SystemExit(f"feedback cap is below command cap for {name}")
        if feedback_cap > mechanical_peak:
            raise SystemExit(f"feedback cap exceeds mechanical peak for {name}")
        configured_speed = motor.get("max_speed_rad_s")
        deployment_speed = min(
            float(ranges["velocity_rad_s"][1]),
            float(
                configured_speed
                if configured_speed is not None
                else specs["maximum_no_load_speed_rad_s_at_48v"]
            ),
        )
        rows.append(
            {
                "motor_name": name,
                "interface": interfaces[int(motor["can_channel"])],
                "can_id": int(motor["can_id"]),
                "master_id": int(motor["master_id"]),
                "position_min_rad": float(ranges["position_rad"][0]),
                "position_max_rad": float(ranges["position_rad"][1]),
                "velocity_min_rad_s": float(ranges["velocity_rad_s"][0]),
                "velocity_max_rad_s": float(ranges["velocity_rad_s"][1]),
                "torque_min_nm": float(ranges["torque_nm"][0]),
                "torque_max_nm": float(ranges["torque_nm"][1]),
                "soft_position_min_rad": float(motor["soft_limit_rad"][0]),
                "soft_position_max_rad": float(motor["soft_limit_rad"][1]),
                "hard_position_min_rad": float(motor["hard_limit_rad"][0]),
                "hard_position_max_rad": float(motor["hard_limit_rad"][1]),
                "deployment_velocity_max_rad_s": deployment_speed,
                "mechanical_peak_torque_nm": mechanical_peak,
                # Commissioning caps remain independent from protocol TMAX and
                # may only be raised for explicitly named, reviewed motors.
                "commissioning_torque_cap_nm": command_cap,
                # Feedback includes suspended static load and sensor offset. Keep
                # its independent, previously qualified anomaly threshold.
                "feedback_torque_cap_nm": feedback_cap,
                "mos_temperature_limit_c": float(
                    specs["drive_shutdown_temperature_c"]
                ),
                "rotor_temperature_limit_c": float(
                    specs["recommended_motor_temperature_limit_c"]
                ),
                "poll_rate_hz": 500 if "ankle_motor" in name else 50,
            }
        )
    rows.sort(key=lambda row: (row["interface"], row["can_id"]))
    if len(rows) != 31:
        raise SystemExit(f"expected 31 motors, found {len(rows)}")
    if len({(row["interface"], row["can_id"]) for row in rows}) != 31:
        raise SystemExit("duplicate command endpoint")
    if len({(row["interface"], row["master_id"]) for row in rows}) != 31:
        raise SystemExit("duplicate feedback endpoint")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(
            stream, fieldnames=FIELDS, dialect="excel-tab", lineterminator="\n"
        )
        writer.writeheader()
        writer.writerows(rows)
    print(f"EXPORTED motors={len(rows)} output={args.output}")


if __name__ == "__main__":
    main()
