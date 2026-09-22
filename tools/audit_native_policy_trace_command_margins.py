#!/usr/bin/env python3
"""Audit proposed physical-motor command margins from a no-actuation live trace."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from open_sprite_runtime.contracts import PolicyContract
from open_sprite_runtime.damiao import DamiaoMitCommand, DamiaoMitState
from open_sprite_runtime.motor_mapping import motor_map_from_hardware_config


ANKLES = ("left_ankle", "right_ankle")


def _minimum_symmetric_range(values: list[float]) -> float:
    return min(abs(float(value)) for value in values)


def audit(
    hardware: dict,
    contract: PolicyContract,
    trace_path: Path,
    *,
    gain_scale: float = 1.0,
    joint_limits: dict | None = None,
    commissioning_torque_fraction: float | None = None,
) -> dict:
    if not 0.0 < gain_scale <= 1.0:
        raise ValueError("gain_scale must be in (0, 1]")
    if commissioning_torque_fraction is not None and not (
        0.0 < commissioning_torque_fraction <= 1.0
    ):
        raise ValueError("commissioning_torque_fraction must be in (0, 1]")
    data = np.load(trace_path, allow_pickle=False)
    joint_names = tuple(map(str, data["joint_names"]))
    motor_names = tuple(map(str, data["motor_names"]))
    if joint_names != tuple(contract.data["joint_names"]):
        raise ValueError("trace joint order does not match contract")
    mapping = motor_map_from_hardware_config(
        hardware, joint_names, require_armable=False
    )
    if set(motor_names) != set(mapping.physical_motor_names) or len(motor_names) != 31:
        raise ValueError("trace motor order does not cover the physical motor map")

    motor_position = np.asarray(data["motor_position_rad"], dtype=np.float64)
    motor_velocity = np.asarray(data["motor_velocity_rad_s"], dtype=np.float64)
    joint_position = np.asarray(data["joint_position_rad"], dtype=np.float64)
    joint_velocity = np.asarray(data["joint_velocity_rad_s"], dtype=np.float64)
    exact_transmitted_target = all(
        name in data.files
        for name in (
            "startup_target_position_rad",
            "startup_target_velocity_rad_s",
            "startup_kp",
            "startup_kd",
            "startup_feedforward_torque_nm",
        )
    )
    target_position_key = (
        "startup_target_position_rad" if exact_transmitted_target else "target_position_rad"
    )
    target_position = np.asarray(data[target_position_key], dtype=np.float64)
    tick_count = target_position.shape[0]
    expected_shapes = {
        "motor_position_rad": (tick_count, 31),
        "motor_velocity_rad_s": (tick_count, 31),
        "joint_position_rad": (tick_count, 31),
        "joint_velocity_rad_s": (tick_count, 31),
        target_position_key: (tick_count, 31),
    }
    for name, shape in expected_shapes.items():
        if np.asarray(data[name]).shape != shape or not np.isfinite(data[name]).all():
            raise ValueError(f"trace {name} must have finite shape {shape}")

    kd_limit = float(hardware["controller"]["damiao_embedded_kd_max"])
    if exact_transmitted_target:
        target_velocity = np.asarray(
            data["startup_target_velocity_rad_s"], dtype=np.float64
        )
        kp = np.asarray(data["startup_kp"], dtype=np.float64)
        kd = np.asarray(data["startup_kd"], dtype=np.float64)
        feedforward = np.asarray(
            data["startup_feedforward_torque_nm"], dtype=np.float64
        )
        for name, value in (
            ("startup_target_velocity_rad_s", target_velocity),
            ("startup_kp", kp),
            ("startup_kd", kd),
            ("startup_feedforward_torque_nm", feedforward),
        ):
            if value.shape != (tick_count, 31) or not np.isfinite(value).all():
                raise ValueError(f"trace {name} must have finite shape {(tick_count, 31)}")
    else:
        target_velocity = np.zeros((tick_count, 31), dtype=np.float64)
        kp = np.broadcast_to(
            gain_scale * np.asarray(contract.data["stiffness"], dtype=np.float64),
            (tick_count, 31),
        )
        kd = np.broadcast_to(
            gain_scale
            * np.minimum(np.asarray(contract.data["damping"], dtype=np.float64), kd_limit),
            (tick_count, 31),
        )
        feedforward = np.zeros((tick_count, 31), dtype=np.float64)
    effort_limit = np.asarray(contract.data["effort_limit"], dtype=np.float64)
    indices = {name: index for index, name in enumerate(joint_names)}
    motor_index = {name: index for index, name in enumerate(motor_names)}
    records = hardware["motor_map"]
    accumulators = {
        name: {
            "initial_measured_position_rad": None,
            "initial_command_position_rad": None,
            "minimum_measured_position_rad": float("inf"),
            "maximum_measured_position_rad": float("-inf"),
            "minimum_command_position_rad": float("inf"),
            "maximum_command_position_rad": float("-inf"),
            "minimum_soft_position_margin_rad": float("inf"),
            "minimum_hard_position_margin_rad": float("inf"),
            "minimum_mit_position_margin_rad": float("inf"),
            "maximum_abs_target_velocity_rad_s": 0.0,
            "maximum_deployment_velocity_ratio": 0.0,
            "maximum_embedded_kp": 0.0,
            "maximum_embedded_kd": 0.0,
            "maximum_abs_feedforward_torque_nm": 0.0,
            "maximum_abs_estimated_output_torque_nm": 0.0,
            "commissioning_torque_cap_nm": None,
            "maximum_commissioning_torque_ratio": 0.0,
            "maximum_mechanical_peak_torque_ratio": 0.0,
            "maximum_mit_torque_ratio": 0.0,
            "violating_tick_count": 0,
            "first_violating_tick": None,
            "violation_counts": {
                "soft_position": 0,
                "hard_position": 0,
                "mit_position": 0,
                "mit_velocity": 0,
                "deployment_velocity": 0,
                "embedded_kd": 0,
                "feedforward_torque": 0,
                "estimated_mit_torque": 0,
                "estimated_commissioning_torque": 0,
                "estimated_mechanical_peak_torque": 0,
            },
        }
        for name in motor_names
    }
    maximum_joint_reconstruction_error = 0.0
    ankle_joint_saturation_count = 0
    ankle_joint_maximum_raw_torque = {name: [0.0, 0.0] for name in ANKLES}
    joint_target_clamp_count = {name: 0 for name in joint_names}

    for tick in range(tick_count):
        measured_states = {
            name: DamiaoMitState(
                motor_position[tick, motor_index[name]],
                motor_velocity[tick, motor_index[name]],
            )
            for name in motor_names
        }
        reconstructed = mapping.motor_to_joint_positions(
            {name: state.position_rad for name, state in measured_states.items()}
        )
        maximum_joint_reconstruction_error = max(
            maximum_joint_reconstruction_error,
            float(np.max(np.abs(reconstructed - joint_position[tick]))),
        )
        bounded_target = target_position[tick].copy()
        if joint_limits is not None:
            for joint_name, index in indices.items():
                low, high = map(
                    float, joint_limits[joint_name]["soft_limit_rad_candidate"]
                )
                limited = float(np.clip(bounded_target[index], low, high))
                joint_target_clamp_count[joint_name] += int(
                    limited != bounded_target[index]
                )
                bounded_target[index] = limited
        commands = mapping.joint_impedance_to_motor_commands(
            bounded_target,
            target_velocity[tick],
            joint_position[tick],
            joint_velocity[tick],
            kp[tick],
            kd[tick],
            feedforward[tick],
        )

        for pair_name in ANKLES:
            pair = mapping.differentials[pair_name]
            pair_indices = np.asarray([indices[name] for name in pair.joint_names])
            raw_torque = (
                kp[tick, pair_indices]
                * (bounded_target[pair_indices] - joint_position[tick, pair_indices])
                + kd[tick, pair_indices]
                * (target_velocity[tick, pair_indices] - joint_velocity[tick, pair_indices])
                + feedforward[tick, pair_indices]
            )
            limits = effort_limit[pair_indices]
            limited_torque = np.clip(raw_torque, -limits, limits)
            ankle_joint_saturation_count += int(np.count_nonzero(raw_torque != limited_torque))
            for offset in range(2):
                ankle_joint_maximum_raw_torque[pair_name][offset] = max(
                    ankle_joint_maximum_raw_torque[pair_name][offset],
                    abs(float(raw_torque[offset])),
                )
            motor_torque = pair.joint_to_drive_torque(limited_torque)
            for offset, motor_name in enumerate(pair.motor_names):
                state = measured_states[motor_name]
                commands[motor_name] = DamiaoMitCommand(
                    position_rad=state.position_rad,
                    velocity_rad_s=0.0,
                    kp=0.0,
                    kd=0.0,
                    feedforward_torque_nm=float(motor_torque[offset]),
                )

        for motor_name, command in commands.items():
            record = records[motor_name]
            state = measured_states[motor_name]
            estimated_torque = (
                command.kp * (command.position_rad - state.position_rad)
                + command.kd * (command.velocity_rad_s - state.velocity_rad_s)
                + command.feedforward_torque_nm
            )
            soft_low, soft_high = map(float, record["soft_limit_rad"])
            hard_low, hard_high = map(float, record["hard_limit_rad"])
            mit_low, mit_high = map(float, record["mit_ranges"]["position_rad"])
            protocol_velocity = _minimum_symmetric_range(record["mit_ranges"]["velocity_rad_s"])
            configured_velocity = record.get("max_speed_rad_s")
            deployment_velocity = min(
                protocol_velocity,
                float(
                    configured_velocity
                    if configured_velocity is not None
                    else hardware["motor_model_specs"][record["model"]][
                        "maximum_no_load_speed_rad_s_at_48v"
                    ]
                ),
            )
            protocol_torque = _minimum_symmetric_range(record["mit_ranges"]["torque_nm"])
            commissioning_torque_cap = (
                commissioning_torque_fraction * float(record["rated_torque_nm"])
                if commissioning_torque_fraction is not None
                else None
            )
            peak_torque = float(record["peak_torque_nm"])
            soft_margin = min(command.position_rad - soft_low, soft_high - command.position_rad)
            hard_margin = min(command.position_rad - hard_low, hard_high - command.position_rad)
            mit_margin = min(command.position_rad - mit_low, mit_high - command.position_rad)
            row = accumulators[motor_name]
            if row["initial_measured_position_rad"] is None:
                row["initial_measured_position_rad"] = float(state.position_rad)
                row["initial_command_position_rad"] = float(command.position_rad)
            row["minimum_measured_position_rad"] = min(
                row["minimum_measured_position_rad"], state.position_rad
            )
            row["maximum_measured_position_rad"] = max(
                row["maximum_measured_position_rad"], state.position_rad
            )
            row["minimum_command_position_rad"] = min(
                row["minimum_command_position_rad"], command.position_rad
            )
            row["maximum_command_position_rad"] = max(
                row["maximum_command_position_rad"], command.position_rad
            )
            row["minimum_soft_position_margin_rad"] = min(row["minimum_soft_position_margin_rad"], soft_margin)
            row["minimum_hard_position_margin_rad"] = min(row["minimum_hard_position_margin_rad"], hard_margin)
            row["minimum_mit_position_margin_rad"] = min(row["minimum_mit_position_margin_rad"], mit_margin)
            row["maximum_abs_target_velocity_rad_s"] = max(row["maximum_abs_target_velocity_rad_s"], abs(command.velocity_rad_s))
            row["maximum_deployment_velocity_ratio"] = max(
                row["maximum_deployment_velocity_ratio"],
                abs(command.velocity_rad_s) / deployment_velocity,
            )
            row["maximum_embedded_kp"] = max(row["maximum_embedded_kp"], command.kp)
            row["maximum_embedded_kd"] = max(row["maximum_embedded_kd"], command.kd)
            row["maximum_abs_feedforward_torque_nm"] = max(row["maximum_abs_feedforward_torque_nm"], abs(command.feedforward_torque_nm))
            row["maximum_abs_estimated_output_torque_nm"] = max(row["maximum_abs_estimated_output_torque_nm"], abs(estimated_torque))
            row["commissioning_torque_cap_nm"] = commissioning_torque_cap
            if commissioning_torque_cap is not None:
                row["maximum_commissioning_torque_ratio"] = max(
                    row["maximum_commissioning_torque_ratio"],
                    abs(estimated_torque) / commissioning_torque_cap,
                )
            row["maximum_mechanical_peak_torque_ratio"] = max(row["maximum_mechanical_peak_torque_ratio"], abs(estimated_torque) / peak_torque)
            row["maximum_mit_torque_ratio"] = max(row["maximum_mit_torque_ratio"], abs(estimated_torque) / protocol_torque)
            checks = {
                "soft_position": soft_margin < 0.0,
                "hard_position": hard_margin < 0.0,
                "mit_position": mit_margin < 0.0,
                "mit_velocity": abs(command.velocity_rad_s) > protocol_velocity,
                "deployment_velocity": abs(command.velocity_rad_s) > deployment_velocity,
                "embedded_kd": command.kd > kd_limit,
                "feedforward_torque": abs(command.feedforward_torque_nm) > protocol_torque,
                "estimated_mit_torque": abs(estimated_torque) > protocol_torque,
                "estimated_commissioning_torque": (
                    commissioning_torque_cap is not None
                    and abs(estimated_torque) > commissioning_torque_cap
                ),
                "estimated_mechanical_peak_torque": abs(estimated_torque) > peak_torque,
            }
            for category, failed in checks.items():
                row["violation_counts"][category] += int(failed)
            violation = any(checks.values())
            row["violating_tick_count"] += int(violation)
            if violation and row["first_violating_tick"] is None:
                row["first_violating_tick"] = tick

    for row in accumulators.values():
        for key, value in tuple(row.items()):
            if isinstance(value, float):
                row[key] = float(value)
    violating_motors = sorted(
        name for name, row in accumulators.items() if row["violating_tick_count"]
    )
    return {
        "mode": "offline_live_trace_physical_motor_command_margin_audit_no_hardware_io",
        "trace": str(trace_path.resolve()),
        "tick_count": tick_count,
        "gain_scale": gain_scale,
        "command_source": (
            "exact_final_startup_trace"
            if exact_transmitted_target
            else "legacy_reconstructed_from_contract"
        ),
        "joint_target_soft_limit_projection_enabled": joint_limits is not None,
        "joint_target_clamp_count": joint_target_clamp_count,
        "maximum_joint_reconstruction_error_rad": maximum_joint_reconstruction_error,
        "damiao_embedded_kd_limit": kd_limit,
        "commissioning_torque_fraction_of_rated": commissioning_torque_fraction,
        "ankle_joint_saturation_count": ankle_joint_saturation_count,
        "ankle_joint_maximum_raw_torque_nm": ankle_joint_maximum_raw_torque,
        "motors": accumulators,
        "violating_motors": violating_motors,
        "errors": [],
        "passed": (
            tick_count > 0
            and maximum_joint_reconstruction_error <= 1.0e-6
            and ankle_joint_saturation_count == 0
            and not violating_motors
        ),
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--hardware", required=True)
    parser.add_argument("--contract", required=True)
    parser.add_argument("--trace", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--gain-scale", type=float, default=1.0)
    parser.add_argument(
        "--commissioning-torque-fraction",
        type=float,
        help=(
            "Fail if estimated motor torque exceeds this fraction of each "
            "motor's rated torque"
        ),
    )
    parser.add_argument(
        "--joint-limit-candidates",
        help="URDF-derived limit report used to project targets into joint soft limits",
    )
    args = parser.parse_args()
    hardware = json.loads(Path(args.hardware).read_text(encoding="utf-8"))
    joint_limits = None
    if args.joint_limit_candidates:
        joint_limits = json.loads(
            Path(args.joint_limit_candidates).read_text(encoding="utf-8")
        )["joint_limits"]
    report = audit(
        hardware,
        PolicyContract.load(args.contract),
        Path(args.trace),
        gain_scale=args.gain_scale,
        joint_limits=joint_limits,
        commissioning_torque_fraction=args.commissioning_torque_fraction,
    )
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    if not report["passed"]:
        raise SystemExit("native policy trace command-margin audit failed")


if __name__ == "__main__":
    main()
