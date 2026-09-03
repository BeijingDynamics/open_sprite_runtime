"""Command-line tools that do not transmit hardware commands."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from .ankle import DifferentialAnkle
from .contracts import PolicyContract, RuntimeTiming
from .safety import (
    RuntimeMode,
    SafetyInputs,
    SafetyLimits,
    SafetyState,
    SafetySupervisor,
)
from .shadow import replay_mujoco_trace
from .timing import run_host_timing_probe


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


def timing_probe(args: argparse.Namespace) -> None:
    print(json.dumps(run_host_timing_probe(args.duration, args.state_hz), indent=2))


def safety_self_test(args: argparse.Namespace) -> None:
    runtime = load_json(args.runtime_config)
    limits = SafetyLimits(
        maximum_state_age_ms=float(runtime["maximum_state_age_ms"]),
        maximum_command_age_ms=float(runtime["maximum_command_age_ms"]),
        maximum_policy_overrun_ms=float(runtime["maximum_policy_overrun_ms"]),
    )
    now = 1_000_000_000

    def evaluate(supervisor: SafetySupervisor, **overrides: object) -> dict[str, object]:
        values: dict[str, object] = {
            "now_ns": now,
            "state_timestamp_ns": now - 2_000_000,
            "command_timestamp_ns": now - 20_000_000,
            "policy_overrun_ms": 0.2,
            "allow_hardware_tx": False,
            "hardware_configured": False,
            "left_ankle_calibrated": False,
            "right_ankle_calibrated": False,
            "imu_valid": True,
            "estop_healthy": True,
        }
        values.update(overrides)
        decision = supervisor.evaluate(SafetyInputs(**values))  # type: ignore[arg-type]
        return {
            "hardware_tx_permitted": decision.hardware_tx_permitted,
            "safe_hold_required": decision.safe_hold_required,
            "blockers": decision.blockers,
            "latched_faults": decision.latched_faults,
        }

    fresh = lambda: SafetySupervisor(RuntimeMode.SHADOW, limits)
    latch_probe = fresh()
    stale_latched = evaluate(latch_probe, state_timestamp_ns=now - 20_000_000)
    recovered_but_latched = evaluate(latch_probe)
    latch_probe.clear_latched_faults(("state_stale",))
    explicitly_cleared = evaluate(latch_probe)
    report = {
        "mode": "deterministic_safety_self_test_no_hardware_tx",
        "healthy_shadow": evaluate(fresh()),
        "stale_state": evaluate(fresh(), state_timestamp_ns=now - 20_000_000),
        "stale_command": evaluate(fresh(), command_timestamp_ns=now - 200_000_000),
        "estop_open": evaluate(fresh(), estop_healthy=False),
        "policy_overrun": evaluate(fresh(), policy_overrun_ms=3.0),
        "latch_sequence": {
            "stale": stale_latched,
            "healthy_input_still_latched": recovered_but_latched,
            "after_explicit_clear": explicitly_cleared,
        },
    }
    independent_cases = (
        report["healthy_shadow"],
        report["stale_state"],
        report["stale_command"],
        report["estop_open"],
        report["policy_overrun"],
    )
    if any(case["hardware_tx_permitted"] for case in independent_cases):
        raise RuntimeError("shadow self-test unexpectedly permitted hardware TX")
    print(json.dumps(report, indent=2))


def replay_trace(args: argparse.Namespace) -> None:
    report = replay_mujoco_trace(args.contract, args.trace, args.sample_stride)
    print(json.dumps(report, indent=2))
    if not report["passed"]:
        raise SystemExit("shadow replay parity failed")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(required=True)
    inspect_parser = subparsers.add_parser("inspect", help="validate contracts without CAN")
    inspect_parser.add_argument("--contract", required=True)
    inspect_parser.add_argument("--runtime-config", required=True)
    inspect_parser.add_argument("--hardware-config", required=True)
    inspect_parser.set_defaults(handler=inspect)
    timing_parser = subparsers.add_parser(
        "timing-probe", help="measure host scheduling without hardware transmission"
    )
    timing_parser.add_argument("--duration", type=float, default=5.0)
    timing_parser.add_argument("--state-hz", type=int, default=500)
    timing_parser.set_defaults(handler=timing_probe)
    safety_parser = subparsers.add_parser(
        "safety-self-test", help="exercise stale-data, timeout, overrun, and e-stop gates"
    )
    safety_parser.add_argument("--runtime-config", required=True)
    safety_parser.set_defaults(handler=safety_self_test)
    replay_parser = subparsers.add_parser(
        "replay-trace", help="replay recorded observations through ONNX without CAN"
    )
    replay_parser.add_argument("--contract", required=True)
    replay_parser.add_argument("--trace", required=True)
    replay_parser.add_argument("--sample-stride", type=int, default=10)
    replay_parser.set_defaults(handler=replay_trace)
    args = parser.parse_args()
    args.handler(args)


if __name__ == "__main__":
    main()
