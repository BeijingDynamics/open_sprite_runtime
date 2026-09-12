"""Command-line tools that do not transmit hardware commands."""

from __future__ import annotations

import argparse
from contextlib import ExitStack
import csv
import json
import math
from pathlib import Path

import numpy as np

from .ankle import DifferentialAnkle, fit_differential_ankle
from .contracts import PolicyContract, RuntimeTiming
from .damiao import (
    DamiaoRxAudit,
    collect_receive_only_audit,
    endpoints_from_hardware_config,
)
from .heading import HeadingCommandController, HeadingControllerConfig
from .hardware import make_hardware_template, validate_hardware_inventory
from .safety import (
    RuntimeMode,
    SafetyInputs,
    SafetyLimits,
    SafetyState,
    SafetySupervisor,
)
from .socketcan import SocketCanReceiver, audit_socketcan_rx_snapshot
from .shadow import replay_mujoco_trace, replay_multirate_mujoco_trace
from .timing import run_host_timing_probe
from .telemetry import (
    MotorTelemetry,
    evaluate_motor_bank,
    limits_from_hardware_record,
)


def load_json(path: str | Path) -> dict:
    return json.loads(Path(path).read_text(encoding="utf-8"))


def load_heading_config(runtime: dict) -> HeadingControllerConfig:
    raw = runtime["heading_controller"]
    if raw.get("actor_receives_global_yaw") is not False:
        raise ValueError("global yaw must remain outside the actor")
    if raw.get("stand_resets_heading") is not True:
        raise ValueError("standing must reset the integrated-yaw heading reference")
    config = HeadingControllerConfig(
        stiffness=float(raw["stiffness"]),
        yaw_rate_limit_rad_s=float(raw["yaw_rate_limit_rad_s"]),
        manual_yaw_deadband_rad_s=float(raw["manual_yaw_deadband_rad_s"]),
    )
    config.validate()
    return config


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
    hardware_inventory = validate_hardware_inventory(
        hardware, contract.data["joint_names"]
    )
    deployment_validation_error = None
    try:
        contract.validate_for_hardware(timing)
        heading = load_heading_config(runtime)
    except (KeyError, TypeError, ValueError) as exc:
        deployment_validation_error = str(exc)
        heading = None

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
        motor_telemetry_healthy=False,
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
        "heading_controller": {
            "valid": heading is not None,
            "stiffness": heading.stiffness if heading is not None else None,
            "yaw_rate_limit_rad_s": (
                heading.yaw_rate_limit_rad_s if heading is not None else None
            ),
            "actor_receives_global_yaw": False,
        },
        "ankles": ankle_report,
        "hardware_inventory": hardware_inventory.to_dict(),
        "hardware_arm_blockers": safety.blockers(),
    }
    print(json.dumps(report, indent=2))


def timing_probe(args: argparse.Namespace) -> None:
    print(json.dumps(run_host_timing_probe(args.duration, args.state_hz), indent=2))


def heading_self_test(args: argparse.Namespace) -> None:
    runtime = load_json(args.runtime_config)
    config = load_heading_config(runtime)
    controller = HeadingCommandController(config)
    rows = []
    for name, yaw, requested, standing in (
        ("stand_zero", 1.0, 0.0, True),
        ("hold_small_error", 1.2, 0.0, False),
        ("hold_saturated", 2.0, 0.0, False),
        ("manual_right", 1.4, 0.15, False),
        ("release_and_hold", 1.4, 0.0, False),
    ):
        command = controller.update(yaw, requested, standing=standing)
        rows.append(
            {
                "case": name,
                "measured_yaw_rad": yaw,
                "operator_yaw_rate_rad_s": requested,
                "actor_yaw_rate_command_rad_s": command,
                "hold_target_yaw_rad": controller.target_yaw,
            }
        )
    expected = [0.0, -0.1, -0.2, 0.15, 0.0]
    mismatches = (
        abs(row["actor_yaw_rate_command_rad_s"] - target) > 1.0e-12
        for row, target in zip(rows, expected, strict=True)
    )
    if any(mismatches):
        raise RuntimeError("heading controller self-test mismatch")
    print(
        json.dumps(
            {
                "mode": "heading_self_test_no_hardware_tx",
                "actor_receives_global_yaw": False,
                "formula": "clip(stiffness * wrap(target_yaw - imu_yaw), yaw_rate_limit)",
                "rows": rows,
                "passed": True,
            },
            indent=2,
        )
    )


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
            "motor_telemetry_healthy": True,
            "command_envelope_healthy": True,
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
        "motor_limit": evaluate(fresh(), motor_telemetry_healthy=False),
        "command_limit": evaluate(fresh(), command_envelope_healthy=False),
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
        report["motor_limit"],
        report["command_limit"],
    )
    if any(case["hardware_tx_permitted"] for case in independent_cases):
        raise RuntimeError("shadow self-test unexpectedly permitted hardware TX")
    print(json.dumps(report, indent=2))


def telemetry_self_test(args: argparse.Namespace) -> None:
    hardware = load_json(args.hardware_config)
    motor_map = hardware.get("motor_map", {})
    if not isinstance(motor_map, dict) or not motor_map:
        raise ValueError("hardware motor_map is missing or empty")
    limits = {
        name: limits_from_hardware_record(record)
        for name, record in motor_map.items()
    }
    samples = {
        name: MotorTelemetry(
            position_rad=0.5 * sum(limit.hard_position_rad),
            velocity_rad_s=0.0,
            torque_nm=0.0,
            current_a=0.0,
            temperature_c=20.0,
        )
        for name, limit in limits.items()
    }
    healthy = evaluate_motor_bank(samples, limits, motor_map)
    first = next(iter(samples))
    bad_samples = dict(samples)
    bad_samples[first] = MotorTelemetry(
        position_rad=samples[first].position_rad,
        velocity_rad_s=0.0,
        torque_nm=limits[first].peak_torque_nm + 1.0,
        current_a=0.0,
        temperature_c=20.0,
    )
    over_torque = evaluate_motor_bank(bad_samples, limits, motor_map)
    missing = evaluate_motor_bank(
        {name: sample for name, sample in samples.items() if name != first},
        limits,
        motor_map,
    )
    passed = bool(healthy["healthy"] and not over_torque["healthy"] and not missing["healthy"])
    report = {
        "mode": "motor_telemetry_self_test_no_hardware_tx",
        "motor_count": len(motor_map),
        "healthy_bank_passes": healthy["healthy"],
        "over_torque_motor": first,
        "over_torque_blocked": not over_torque["healthy"],
        "missing_motor_blocked": not missing["healthy"],
        "passed": passed,
    }
    print(json.dumps(report, indent=2))
    if not passed:
        raise RuntimeError("motor telemetry self-test failed")


def ankle_calibrate(args: argparse.Namespace) -> None:
    with Path(args.samples).open(encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    required = ("pitch_rad", "roll_rad", "motor_a_rad", "motor_b_rad")
    missing = [name for name in required if not rows or name not in rows[0]]
    if missing:
        raise ValueError(f"ankle calibration CSV missing columns: {', '.join(missing)}")
    joints = np.asarray(
        [[row["pitch_rad"], row["roll_rad"]] for row in rows], dtype=float
    )
    motors = np.asarray(
        [[row["motor_a_rad"], row["motor_b_rad"]] for row in rows], dtype=float
    )
    result = fit_differential_ankle(
        joints,
        motors,
        maximum_rms_residual_rad=args.maximum_rms_residual_rad,
        maximum_condition_number=args.maximum_condition_number,
    )
    report = {
        "mode": "unloaded_differential_ankle_calibration_no_hardware_tx",
        "side": args.side,
        "source": str(Path(args.samples).resolve()),
        **result,
    }
    output = json.dumps(report, indent=2) + "\n"
    if args.output:
        Path(args.output).write_text(output, encoding="utf-8")
    print(output, end="")
    if not report["passed"]:
        raise SystemExit("ankle calibration quality gates failed")


def replay_trace(args: argparse.Namespace) -> None:
    report = replay_mujoco_trace(args.contract, args.trace, args.sample_stride)
    print(json.dumps(report, indent=2))
    if not report["passed"]:
        raise SystemExit("shadow replay parity failed")


def replay_multirate_trace(args: argparse.Namespace) -> None:
    runtime = load_json(args.runtime_config)
    report = replay_multirate_mujoco_trace(
        args.contract,
        args.trace,
        policy_hz=int(runtime["policy_hz"]),
        state_hz=int(runtime["state_hz"]),
        maximum_state_age_ms=float(runtime["maximum_state_age_ms"]),
        maximum_command_age_ms=float(runtime["maximum_command_age_ms"]),
        maximum_policy_overrun_ms=float(runtime["maximum_policy_overrun_ms"]),
    )
    print(json.dumps(report, indent=2))
    if not report["passed"]:
        raise SystemExit("multirate shadow replay failed")


def hardware_template(args: argparse.Namespace) -> None:
    output = Path(args.output)
    if output.exists() and not args.force:
        raise FileExistsError(f"refusing to overwrite {output}; pass --force explicitly")
    contract = PolicyContract.load(args.contract)
    template = make_hardware_template(contract.data["joint_names"])
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(template, indent=2) + "\n", encoding="utf-8")
    print(
        json.dumps(
            {
                "mode": "hardware_measurement_template_no_hardware_tx",
                "output": str(output.resolve()),
                "physical_motor_count": len(template["motor_map"]),
                "configured": False,
            },
            indent=2,
        )
    )


def socketcan_rx_preflight(args: argparse.Namespace) -> None:
    snapshot = json.loads(Path(args.snapshot).read_text(encoding="utf-8"))
    report = audit_socketcan_rx_snapshot(snapshot, args.interfaces)
    print(json.dumps({"mode": "offline_socketcan_rx_preflight_no_hardware_tx", **report.to_dict()}, indent=2))
    if not report.passed:
        raise SystemExit("SocketCAN receive-only preflight failed")


def damiao_rx_audit(args: argparse.Namespace) -> None:
    hardware = load_json(args.hardware_config)
    interfaces = hardware.get("can_adapter", {}).get("interfaces")
    endpoints = endpoints_from_hardware_config(hardware)
    snapshot = load_json(args.snapshot)
    preflight = audit_socketcan_rx_snapshot(snapshot, interfaces)
    if not preflight.passed:
        raise SystemExit(
            "SocketCAN receive-only preflight failed: " + "; ".join(preflight.errors)
        )
    minimum_samples = math.ceil(
        args.duration * args.minimum_feedback_hz * args.minimum_sample_coverage
    )
    audit = DamiaoRxAudit(
        endpoints,
        minimum_samples_per_motor=minimum_samples,
        minimum_feedback_hz=args.minimum_feedback_hz,
        maximum_hardware_gap_ms=args.maximum_hardware_gap_ms,
        maximum_userspace_queue_age_p99_ms=args.maximum_queue_age_p99_ms,
    )
    with ExitStack() as stack:
        receivers = {
            interface: stack.enter_context(SocketCanReceiver.open(interface, preflight))
            for interface in interfaces
        }
        report = collect_receive_only_audit(receivers, audit, args.duration)
    result = {
        "mode": "live_damiao_socketcan_receive_only_no_hardware_tx",
        "duration_s": args.duration,
        "preflight": preflight.to_dict(),
        **report.to_dict(),
    }
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2))
    if not report.passed:
        raise SystemExit("Damiao receive-only audit failed")


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
    heading_parser = subparsers.add_parser(
        "heading-self-test", help="exercise IMU-yaw outer command control without CAN"
    )
    heading_parser.add_argument("--runtime-config", required=True)
    heading_parser.set_defaults(handler=heading_self_test)
    safety_parser = subparsers.add_parser(
        "safety-self-test",
        help="exercise timing, e-stop, telemetry, and command-envelope gates",
    )
    safety_parser.add_argument("--runtime-config", required=True)
    safety_parser.set_defaults(handler=safety_self_test)
    telemetry_parser = subparsers.add_parser(
        "telemetry-self-test",
        help="exercise physical motor limit and exact-inventory gates without CAN",
    )
    telemetry_parser.add_argument("--hardware-config", required=True)
    telemetry_parser.set_defaults(handler=telemetry_self_test)
    ankle_parser = subparsers.add_parser(
        "ankle-calibrate",
        help="fit one unloaded differential-ankle map from measured CSV samples",
    )
    ankle_parser.add_argument("--side", required=True, choices=("left", "right"))
    ankle_parser.add_argument("--samples", required=True)
    ankle_parser.add_argument("--output")
    ankle_parser.add_argument("--maximum-rms-residual-rad", type=float, default=0.01)
    ankle_parser.add_argument("--maximum-condition-number", type=float, default=100.0)
    ankle_parser.set_defaults(handler=ankle_calibrate)
    replay_parser = subparsers.add_parser(
        "replay-trace", help="replay recorded observations through ONNX without CAN"
    )
    replay_parser.add_argument("--contract", required=True)
    replay_parser.add_argument("--trace", required=True)
    replay_parser.add_argument("--sample-stride", type=int, default=10)
    replay_parser.set_defaults(handler=replay_trace)
    multirate_parser = subparsers.add_parser(
        "multirate-replay",
        help="replay every policy tick through a synthetic 500 Hz no-TX safety loop",
    )
    multirate_parser.add_argument("--contract", required=True)
    multirate_parser.add_argument("--runtime-config", required=True)
    multirate_parser.add_argument("--trace", required=True)
    multirate_parser.set_defaults(handler=replay_multirate_trace)
    template_parser = subparsers.add_parser(
        "hardware-template", help="generate a non-armable 31-motor measurement worksheet"
    )
    template_parser.add_argument("--contract", required=True)
    template_parser.add_argument("--output", required=True)
    template_parser.add_argument("--force", action="store_true")
    template_parser.set_defaults(handler=hardware_template)
    can_parser = subparsers.add_parser(
        "socketcan-rx-preflight",
        help="audit a saved ip-link JSON snapshot without opening or transmitting on CAN",
    )
    can_parser.add_argument("--snapshot", required=True)
    can_parser.add_argument(
        "--interfaces", nargs=4, default=("can0", "can1", "can2", "can3")
    )
    can_parser.set_defaults(handler=socketcan_rx_preflight)
    damiao_parser = subparsers.add_parser(
        "damiao-rx-audit",
        help="collect a finite four-bus Damiao shadow trace without CAN transmission",
    )
    damiao_parser.add_argument("--hardware-config", required=True)
    damiao_parser.add_argument("--snapshot", required=True)
    damiao_parser.add_argument("--output", required=True)
    damiao_parser.add_argument("--duration", type=float, default=10.0)
    damiao_parser.add_argument("--minimum-feedback-hz", type=float, default=475.0)
    damiao_parser.add_argument("--minimum-sample-coverage", type=float, default=0.95)
    damiao_parser.add_argument("--maximum-hardware-gap-ms", type=float, default=6.0)
    damiao_parser.add_argument("--maximum-queue-age-p99-ms", type=float, default=6.0)
    damiao_parser.set_defaults(handler=damiao_rx_audit)
    args = parser.parse_args()
    args.handler(args)


if __name__ == "__main__":
    main()
