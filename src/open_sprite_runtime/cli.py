"""Command-line tools that do not transmit hardware commands."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from .ankle import DifferentialAnkle
from .contracts import PolicyContract, RuntimeTiming
from .safety import SafetyState


def load_json(path: str | Path) -> dict:
    return json.loads(Path(path).read_text(encoding="utf-8"))


def inspect(args: argparse.Namespace) -> None:
    runtime = load_json(args.runtime_config)
    hardware = load_json(args.hardware_config)
    timing = RuntimeTiming(
        policy_hz=int(runtime["policy_hz"]),
        state_hz=int(runtime["state_hz"]),
        motor_internal_hz=int(runtime["motor_internal_hz"]),
        policy_target_semantics=str(runtime.get("policy_target_semantics", "")),
    )
    contract = PolicyContract.load(args.contract)
    deployment_validation_error = None
    try:
        contract.validate_for_hardware(timing)
    except ValueError as exc:
        deployment_validation_error = str(exc)

    ankle_report = {}
    for side, ankle_config in hardware["ankles"].items():
        ankle = DifferentialAnkle(
            ankle_config["joint_to_motor_matrix"], ankle_config["motor_zero_rad"]
        )
        ankle_report[side] = {
            "calibrated": bool(ankle_config["calibrated"]),
            "determinant": float(np.linalg.det(ankle.joint_to_motor_matrix)),
            "ideal_equal_joint_kp_to_motor_kp": ankle.diagonal_motor_gains(
                [14.212230682373, 14.212230682373]
            ).tolist(),
            "ideal_equal_joint_kd_to_motor_kd": ankle.diagonal_motor_gains(
                [0.904778659344, 0.904778659344]
            ).tolist(),
        }

    safety = SafetyState(
        allow_hardware_tx=bool(runtime["allow_hardware_tx"]),
        hardware_configured=bool(hardware["configured"]),
        left_ankle_calibrated=bool(hardware["ankles"]["left"]["calibrated"]),
        right_ankle_calibrated=bool(hardware["ankles"]["right"]["calibrated"]),
        imu_valid=False,
        estop_healthy=False,
        state_fresh=False,
    )
    report = {
        "mode": "inspection_only_no_can_backend",
        "contract": contract.summary(),
        "deployment_contract_valid": deployment_validation_error is None,
        "deployment_validation_error": deployment_validation_error,
        "timing": {
            "policy_hz": timing.policy_hz,
            "state_hz": timing.state_hz,
            "motor_internal_hz": timing.motor_internal_hz,
            "state_updates_per_policy": timing.state_updates_per_policy,
            "policy_target_semantics": timing.policy_target_semantics,
        },
        "ankles": ankle_report,
        "hardware_arm_blockers": safety.blockers(),
    }
    print(json.dumps(report, indent=2))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(required=True)
    inspect_parser = subparsers.add_parser("inspect", help="validate contracts without CAN")
    inspect_parser.add_argument("--contract", required=True)
    inspect_parser.add_argument("--runtime-config", required=True)
    inspect_parser.add_argument("--hardware-config", required=True)
    inspect_parser.set_defaults(handler=inspect)
    args = parser.parse_args()
    args.handler(args)


if __name__ == "__main__":
    main()
