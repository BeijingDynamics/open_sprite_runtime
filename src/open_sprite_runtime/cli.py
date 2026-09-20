"""Runtime inspection tools and explicitly gated hardware commissioning commands."""

from __future__ import annotations

import argparse
from contextlib import ExitStack
import csv
import json
import math
import os
from pathlib import Path
import time

import numpy as np

from .ankle import DifferentialPair, fit_differential_pair
from .contracts import PolicyContract, RuntimeTiming
from .damiao import (
    DamiaoRxAudit,
    collect_receive_only_audit,
    collect_zero_gain_group_position_echo,
    collect_zero_gain_position_echo,
    endpoints_from_hardware_config,
)
from .heading import HeadingCommandController, HeadingControllerConfig
from .hardware import make_hardware_template, validate_hardware_inventory
from .imu import ImuMount, quaternion_wxyz_to_matrix
from .imu_serial import (
    HARDWARE_TX_CONFIRMATION,
    capture_serial_read_only,
    configure_report_rate,
)
from .safety import (
    RuntimeMode,
    SafetyInputs,
    SafetyLimits,
    SafetyState,
    SafetySupervisor,
)
from .socketcan import (
    SocketCanReceiver,
    SocketCanZeroGainPoller,
    audit_socketcan_active_fd_snapshot,
    audit_socketcan_rx_snapshot,
)
from .shadow import replay_mujoco_trace, replay_multirate_mujoco_trace
from .timing import run_host_timing_probe
from .telemetry import (
    MotorTelemetry,
    evaluate_motor_bank,
    limits_from_hardware_record,
)


def imu_mount_self_test(args) -> None:
    mount = ImuMount.sprite0825_rear_pelvis()
    sensor_to_body = mount.sensor_to_body_matrix
    body_to_sensor_quaternion = mount.body_to_sensor_quaternion_wxyz
    reconstructed = quaternion_wxyz_to_matrix(tuple(body_to_sensor_quaternion))
    result = {
        "mode": "sprite0825_rear_pelvis_imu_mount_self_test_no_hardware",
        "body_axes": {"x": "forward", "y": "left", "z": "up"},
        "physical_mount": {
            "sensor_positive_x": "down",
            "sensor_positive_y": "right",
            "sensor_positive_z": "backward",
        },
        "sensor_to_body_vector_formula": "[x_b,y_b,z_b]=[-z_s,-y_s,-x_s]",
        "sensor_to_body_matrix": sensor_to_body.tolist(),
        "body_to_sensor_quaternion_wxyz": body_to_sensor_quaternion.tolist(),
        "determinant": float(np.linalg.det(sensor_to_body)),
        "orthogonality_error": float(
            np.max(np.abs(sensor_to_body.T @ sensor_to_body - np.eye(3)))
        ),
        "quaternion_matrix_error": float(
            np.max(np.abs(reconstructed - sensor_to_body.T))
        ),
    }
    result["passed"] = (
        abs(result["determinant"] - 1.0) < 1.0e-9
        and result["orthogonality_error"] < 1.0e-9
        and result["quaternion_matrix_error"] < 1.0e-9
    )
    print(json.dumps(result, indent=2))
    if not result["passed"]:
        raise SystemExit("IMU mount self-test failed")


def imu_serial_capture(args) -> None:
    report = capture_serial_read_only(
        device=args.device,
        baud_rate=args.baud,
        duration_s=args.duration,
        raw_output=args.raw_output,
    )
    result = report.to_dict()
    output = Path(args.report).expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2))
    if not report.passed:
        raise SystemExit("no serial bytes received; IMU capture failed closed")


def imu_set_report_rate(args) -> None:
    report = configure_report_rate(
        device=args.device,
        baud_rate=args.baud,
        rate_hz=args.rate,
        verification_duration_s=args.verification_duration,
        confirmation=args.confirm_hardware_tx,
    )
    result = report.to_dict()
    output = Path(args.report).expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2))
    if not report.passed:
        raise SystemExit("IMU report-rate verification failed closed")


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

    differential_report = {}
    for name, pair_config in hardware["differentials"].items():
        coupling = DifferentialPair(
            pair_config["joint_to_motor_matrix"], pair_config["motor_zero_rad"]
        )
        motor_kp = coupling.motor_impedance_matrix([14.212230682373, 14.212230682373])
        motor_kd = coupling.motor_impedance_matrix([0.904778659344, 0.904778659344])
        kp_cross = motor_kp - np.diag(np.diag(motor_kp))
        kd_cross = motor_kd - np.diag(np.diag(motor_kd))
        differential_report[name] = {
            "calibrated": bool(pair_config["calibrated"]),
            "determinant": float(np.linalg.det(coupling.joint_to_motor_matrix)),
            "motor_kp_matrix": motor_kp.tolist(),
            "motor_kd_matrix": motor_kd.tolist(),
            "embedded_diagonal_motor_kp": np.diag(motor_kp).tolist(),
            "embedded_diagonal_motor_kd": np.diag(motor_kd).tolist(),
            "host_cross_coupling_required": bool(
                np.max(np.abs(kp_cross)) > 1.0e-9
                or np.max(np.abs(kd_cross)) > 1.0e-9
            ),
        }

    safety = SafetyState(
        allow_hardware_tx=bool(runtime["allow_hardware_tx"]),
        hardware_configured=bool(hardware["configured"]),
        left_ankle_calibrated=bool(hardware["differentials"]["left_ankle"]["calibrated"]),
        right_ankle_calibrated=bool(hardware["differentials"]["right_ankle"]["calibrated"]),
        head_differential_calibrated=bool(hardware["differentials"]["head"]["calibrated"]),
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
        "differentials": differential_report,
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
            "head_differential_calibrated": False,
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
        raise ValueError(f"differential calibration CSV missing columns: {', '.join(missing)}")
    joints = np.asarray(
        [[row["pitch_rad"], row["roll_rad"]] for row in rows], dtype=float
    )
    motors = np.asarray(
        [[row["motor_a_rad"], row["motor_b_rad"]] for row in rows], dtype=float
    )
    result = fit_differential_pair(
        joints,
        motors,
        maximum_rms_residual_rad=args.maximum_rms_residual_rad,
        maximum_condition_number=args.maximum_condition_number,
    )
    report = {
        "mode": "unloaded_differential_pair_calibration_no_hardware_tx",
        "pair": getattr(args, "pair", f"{args.side}_ankle"),
        "source": str(Path(args.samples).resolve()),
        **result,
    }
    output = json.dumps(report, indent=2) + "\n"
    if args.output:
        Path(args.output).write_text(output, encoding="utf-8")
    print(output, end="")
    if not report["passed"]:
        raise SystemExit("differential calibration quality gates failed")


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
    range_sources = sorted({endpoint.ranges.source for endpoint in endpoints})
    ranges_verified = all(endpoint.ranges.register_readback_verified for endpoint in endpoints)
    if not ranges_verified and not args.viewer:
        raise SystemExit(
            "Damiao receive-only qualification requires per-drive register readback ranges"
        )
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
        viewer = None
        if args.viewer:
            if not args.contract or not args.mjcf:
                raise SystemExit("--viewer requires both --contract and --mjcf")
            if args.display:
                os.environ["DISPLAY"] = args.display
            from .mujoco_rx_viewer import MujocoRxViewer

            contract = PolicyContract.load(args.contract)
            viewer = stack.enter_context(
                MujocoRxViewer(
                    hardware,
                    contract.data["joint_names"],
                    args.mjcf,
                    root_height_m=args.root_height,
                    refresh_hz=args.viewer_hz,
                )
            )
            if viewer.frozen_joints:
                print(
                    "VIEWER frozen uncalibrated joints: "
                    + ", ".join(viewer.frozen_joints)
                )
        receivers = {
            interface: stack.enter_context(SocketCanReceiver.open(interface, preflight))
            for interface in interfaces
        }
        report = collect_receive_only_audit(
            receivers,
            audit,
            args.duration,
            on_feedback=None if viewer is None else viewer.update,
            keep_running=None if viewer is None else viewer.is_running,
        )
    result = {
        "mode": "live_damiao_socketcan_receive_only_no_hardware_tx",
        "duration_s": args.duration,
        "preflight": preflight.to_dict(),
        "protocol_range_sources": range_sources,
        "protocol_ranges_register_readback_verified": ranges_verified,
        "qualification_passed": bool(report.passed and ranges_verified),
        **report.to_dict(),
    }
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2))
    if not report.passed:
        raise SystemExit("Damiao receive-only audit failed")


def damiao_zero_gain_probe(args: argparse.Namespace) -> None:
    if args.acknowledge_hardware_tx != "ZERO_GAIN_POSITION_ECHO":
        raise SystemExit(
            "refusing hardware TX: pass "
            "--acknowledge-hardware-tx ZERO_GAIN_POSITION_ECHO"
        )
    if not args.mit_mode_confirmed:
        raise SystemExit("refusing hardware TX: --mit-mode-confirmed is required")
    if not args.supported_unloaded:
        raise SystemExit("refusing hardware TX: --supported-unloaded is required")
    maximum_duration_s = 120.0 if args.viewer else 10.0
    if not 0.0 < args.duration <= maximum_duration_s:
        raise SystemExit(
            f"--duration must be in (0, {maximum_duration_s:g}] seconds"
        )
    if not 0.0 < args.rate_hz <= 100.0:
        raise SystemExit("--rate-hz must be in (0, 100] Hz")
    if not 0.0 < args.print_hz <= 50.0:
        raise SystemExit("--print-hz must be in (0, 50] Hz")

    hardware = load_json(args.hardware_config)
    endpoints = endpoints_from_hardware_config(hardware)
    matches = [item for item in endpoints if item.motor_name == args.motor]
    if len(matches) != 1:
        names = ", ".join(sorted(item.motor_name for item in endpoints))
        raise SystemExit(f"unknown motor {args.motor!r}; configured motors: {names}")
    endpoint = matches[0]
    preflight = audit_socketcan_active_fd_snapshot(
        load_json(args.snapshot),
        endpoint.interface,
        arbitration_bitrate=1_000_000,
        data_bitrate=5_000_000,
    )
    if not preflight.passed:
        raise SystemExit("active CAN-FD preflight failed: " + "; ".join(preflight.errors))

    print(
        "TX_ARMED "
        f"motor={endpoint.motor_name} interface={endpoint.interface} "
        f"can_id={endpoint.can_id:#x} master_id={endpoint.master_id:#x} "
        f"duration={args.duration:.3f}s rate={args.rate_hz:.1f}Hz "
        "payload=position_echo,v=0,Kp=0,Kd=0,tau=0 CANFD_BRS=on"
    )
    with ExitStack() as stack:
        viewer = None
        if args.viewer:
            if not args.contract or not args.mjcf:
                raise SystemExit("--viewer requires both --contract and --mjcf")
            if args.display:
                os.environ["DISPLAY"] = args.display
            from .mujoco_rx_viewer import MujocoRxViewer

            contract = PolicyContract.load(args.contract)
            viewer = stack.enter_context(
                MujocoRxViewer(
                    hardware,
                    contract.data["joint_names"],
                    args.mjcf,
                    root_height_m=args.root_height,
                    refresh_hz=args.viewer_hz,
                )
            )
        last_print = -math.inf

        def show_feedback(feedback) -> None:
            nonlocal last_print
            now = time.monotonic()
            if now - last_print >= 1.0 / args.print_hz:
                print(
                    "RX "
                    f"motor={feedback.motor_name} "
                    f"position_rad={feedback.position_rad:+.6f} "
                    f"velocity_rad_s={feedback.velocity_rad_s:+.6f} "
                    f"estimated_torque_nm={feedback.estimated_output_torque_nm:+.6f} "
                    f"mos_c={feedback.mos_temperature_c} "
                    f"rotor_c={feedback.rotor_temperature_c} "
                    f"status={feedback.status_name}",
                    flush=True,
                )
                last_print = now
            if viewer is not None:
                viewer.update(feedback)

        poller = stack.enter_context(
            SocketCanZeroGainPoller.open(endpoint.interface, preflight)
        )
        report = collect_zero_gain_position_echo(
            poller,
            endpoint,
            endpoints,
            args.duration,
            args.rate_hz,
            feedback_timeout_s=args.feedback_timeout,
            on_feedback=show_feedback,
            keep_running=None if viewer is None else viewer.is_running,
            maximum_duration_s=maximum_duration_s,
        )
    result = {
        "mode": "single_motor_zero_gain_position_echo_hardware_tx",
        "preflight": preflight.to_dict(),
        "automatic_enable_attempts": 0,
        "automatic_mode_switch_attempts": 0,
        **report.to_dict(),
    }
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2))
    if not report.passed:
        raise SystemExit("Damiao zero-gain position-echo probe failed")


def damiao_zero_gain_group_probe(args: argparse.Namespace) -> None:
    if args.acknowledge_hardware_tx != "ZERO_GAIN_GROUP_POSITION_ECHO":
        raise SystemExit(
            "refusing hardware TX: pass "
            "--acknowledge-hardware-tx ZERO_GAIN_GROUP_POSITION_ECHO"
        )
    if not args.mit_mode_confirmed:
        raise SystemExit("refusing hardware TX: --mit-mode-confirmed is required")
    if not args.supported_unloaded:
        raise SystemExit("refusing hardware TX: --supported-unloaded is required")
    maximum_duration_s = 120.0 if args.viewer else 10.0
    if not 0.0 < args.duration <= maximum_duration_s:
        raise SystemExit(
            f"--duration must be in (0, {maximum_duration_s:g}] seconds"
        )
    if not 0.0 < args.rate_hz <= 500.0:
        raise SystemExit("--rate-hz must be in (0, 500] Hz")
    if not 0.0 < args.print_hz <= 10.0:
        raise SystemExit("--print-hz must be in (0, 10] Hz")

    hardware = load_json(args.hardware_config)
    all_endpoints = endpoints_from_hardware_config(hardware)
    names = tuple(args.motors)
    if not 1 <= len(names) <= 8 or len(set(names)) != len(names):
        raise SystemExit("--motors must contain between one and eight unique motor names")
    by_name = {item.motor_name: item for item in all_endpoints}
    missing = sorted(set(names) - set(by_name))
    if missing:
        raise SystemExit("unknown motors: " + ", ".join(missing))
    selected = tuple(by_name[name] for name in names)
    if {item.interface for item in selected} != {args.interface}:
        raise SystemExit("all selected motors must belong to --interface")
    preflight = audit_socketcan_active_fd_snapshot(
        load_json(args.snapshot),
        args.interface,
        arbitration_bitrate=1_000_000,
        data_bitrate=5_000_000,
    )
    if not preflight.passed:
        raise SystemExit("active CAN-FD preflight failed: " + "; ".join(preflight.errors))

    print(
        "TX_ARMED_GROUP "
        f"interface={args.interface} motors={len(selected)} "
        f"duration={args.duration:.3f}s rate_per_motor={args.rate_hz:.1f}Hz "
        f"total_nominal_tx_rate={len(selected) * args.rate_hz:.1f}Hz "
        "payload=position_echo,v=0,Kp=0,Kd=0,tau=0 CANFD_BRS=on"
    )
    with ExitStack() as stack:
        viewer = None
        if args.viewer:
            if not args.contract or not args.mjcf:
                raise SystemExit("--viewer requires both --contract and --mjcf")
            if args.display:
                os.environ["DISPLAY"] = args.display
            from .mujoco_rx_viewer import MujocoRxViewer

            contract = PolicyContract.load(args.contract)
            viewer = stack.enter_context(
                MujocoRxViewer(
                    hardware,
                    contract.data["joint_names"],
                    args.mjcf,
                    root_height_m=args.root_height,
                    refresh_hz=args.viewer_hz,
                )
            )
        last_print = {item.motor_name: -math.inf for item in selected}

        def show_feedback(feedback) -> None:
            now = time.monotonic()
            if now - last_print[feedback.motor_name] >= 1.0 / args.print_hz:
                print(
                    "RX "
                    f"motor={feedback.motor_name} "
                    f"position_rad={feedback.position_rad:+.6f} "
                    f"status={feedback.status_name}",
                    flush=True,
                )
                last_print[feedback.motor_name] = now
            if viewer is not None:
                viewer.update(feedback)

        poller = stack.enter_context(
            SocketCanZeroGainPoller.open(args.interface, preflight)
        )
        report = collect_zero_gain_group_position_echo(
            poller,
            selected,
            all_endpoints,
            args.duration,
            args.rate_hz,
            feedback_timeout_s=args.feedback_timeout,
            on_feedback=show_feedback,
            keep_running=None if viewer is None else viewer.is_running,
            maximum_duration_s=maximum_duration_s,
            minimum_sample_coverage=args.minimum_sample_coverage,
        )
    result = {
        "mode": "motor_group_zero_gain_position_echo_hardware_tx",
        "preflight": preflight.to_dict(),
        "automatic_enable_attempts": 0,
        "automatic_mode_switch_attempts": 0,
        **report.to_dict(),
    }
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2))
    if not report.passed:
        raise SystemExit("Damiao zero-gain group position-echo probe failed")


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
    imu_mount_parser = subparsers.add_parser(
        "imu-mount-self-test",
        help="validate the fixed Sprite0825 rear-pelvis IMU transform without hardware",
    )
    imu_mount_parser.set_defaults(handler=imu_mount_self_test)
    imu_capture_parser = subparsers.add_parser(
        "imu-serial-capture",
        help="capture raw IMU serial bytes without transmitting protocol data",
    )
    imu_capture_parser.add_argument("--device", required=True)
    imu_capture_parser.add_argument("--baud", required=True, type=int)
    imu_capture_parser.add_argument("--duration", type=float, default=10.0)
    imu_capture_parser.add_argument("--raw-output", required=True)
    imu_capture_parser.add_argument("--report", required=True)
    imu_capture_parser.set_defaults(handler=imu_serial_capture)
    imu_rate_parser = subparsers.add_parser(
        "imu-set-report-rate",
        help="explicitly configure and verify Yahboom IMU serial report rate",
    )
    imu_rate_parser.add_argument("--device", required=True)
    imu_rate_parser.add_argument("--baud", type=int, default=115200)
    imu_rate_parser.add_argument("--rate", required=True, type=int)
    imu_rate_parser.add_argument("--verification-duration", type=float, default=3.0)
    imu_rate_parser.add_argument("--report", required=True)
    imu_rate_parser.add_argument(
        "--confirm-hardware-tx",
        required=True,
        metavar=HARDWARE_TX_CONFIRMATION,
        help=f"must equal {HARDWARE_TX_CONFIRMATION}",
    )
    imu_rate_parser.set_defaults(handler=imu_set_report_rate)
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
    differential_parser = subparsers.add_parser(
        "differential-calibrate",
        help="fit an unloaded ankle or head differential map from measured CSV samples",
    )
    differential_parser.add_argument(
        "--pair", required=True, choices=("left_ankle", "right_ankle", "head")
    )
    differential_parser.add_argument("--samples", required=True)
    differential_parser.add_argument("--output")
    differential_parser.add_argument("--maximum-rms-residual-rad", type=float, default=0.01)
    differential_parser.add_argument("--maximum-condition-number", type=float, default=100.0)
    differential_parser.set_defaults(handler=ankle_calibrate, side=None)
    replay_parser = subparsers.add_parser(
        "replay-trace", help="replay recorded observations through the frozen actor without CAN"
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
    damiao_parser.add_argument(
        "--viewer", action="store_true", help="show decoded joint state in local MuJoCo"
    )
    damiao_parser.add_argument("--contract")
    damiao_parser.add_argument("--mjcf")
    damiao_parser.add_argument("--display", default=None)
    damiao_parser.add_argument("--root-height", type=float, default=0.52)
    damiao_parser.add_argument("--viewer-hz", type=float, default=50.0)
    damiao_parser.set_defaults(handler=damiao_rx_audit)
    zero_gain_parser = subparsers.add_parser(
        "damiao-zero-gain-probe",
        help="send finite single-motor CAN-FD position-echo frames with v/Kp/Kd/tau zero",
    )
    zero_gain_parser.add_argument("--hardware-config", required=True)
    zero_gain_parser.add_argument("--snapshot", required=True)
    zero_gain_parser.add_argument("--motor", required=True)
    zero_gain_parser.add_argument("--output", required=True)
    zero_gain_parser.add_argument("--duration", type=float, default=2.0)
    zero_gain_parser.add_argument("--rate-hz", type=float, default=50.0)
    zero_gain_parser.add_argument("--print-hz", type=float, default=10.0)
    zero_gain_parser.add_argument("--feedback-timeout", type=float, default=0.2)
    zero_gain_parser.add_argument("--mit-mode-confirmed", action="store_true")
    zero_gain_parser.add_argument("--supported-unloaded", action="store_true")
    zero_gain_parser.add_argument("--acknowledge-hardware-tx")
    zero_gain_parser.add_argument("--viewer", action="store_true")
    zero_gain_parser.add_argument("--contract")
    zero_gain_parser.add_argument("--mjcf")
    zero_gain_parser.add_argument("--display", default=None)
    zero_gain_parser.add_argument("--root-height", type=float, default=0.52)
    zero_gain_parser.add_argument("--viewer-hz", type=float, default=50.0)
    zero_gain_parser.set_defaults(handler=damiao_zero_gain_probe)
    group_parser = subparsers.add_parser(
        "damiao-zero-gain-group-probe",
        help="poll one to eight motors on one CAN-FD bus with zero-gain position echo",
    )
    group_parser.add_argument("--hardware-config", required=True)
    group_parser.add_argument("--snapshot", required=True)
    group_parser.add_argument("--interface", required=True)
    group_parser.add_argument("--motors", nargs="+", required=True)
    group_parser.add_argument("--output", required=True)
    group_parser.add_argument("--duration", type=float, default=10.0)
    group_parser.add_argument("--rate-hz", type=float, default=50.0)
    group_parser.add_argument("--print-hz", type=float, default=2.0)
    group_parser.add_argument("--feedback-timeout", type=float, default=0.2)
    group_parser.add_argument("--minimum-sample-coverage", type=float, default=0.95)
    group_parser.add_argument("--mit-mode-confirmed", action="store_true")
    group_parser.add_argument("--supported-unloaded", action="store_true")
    group_parser.add_argument("--acknowledge-hardware-tx")
    group_parser.add_argument("--viewer", action="store_true")
    group_parser.add_argument("--contract")
    group_parser.add_argument("--mjcf")
    group_parser.add_argument("--display", default=None)
    group_parser.add_argument("--root-height", type=float, default=0.52)
    group_parser.add_argument("--viewer-hz", type=float, default=50.0)
    group_parser.set_defaults(handler=damiao_zero_gain_group_probe)
    args = parser.parse_args()
    args.handler(args)


if __name__ == "__main__":
    main()
