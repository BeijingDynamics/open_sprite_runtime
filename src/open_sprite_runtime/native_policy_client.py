"""50 Hz ONNX/IMU client for the native zero-gain policy IPC shadow."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import selectors
import socket
import time

import numpy as np

from .contracts import PolicyContract
from .damiao import DamiaoMitState
from .motor_mapping import motor_map_from_hardware_config
from .native_ipc import NativeStatePacket, PolicyTargetPacket, ordered_name_hash
from .physical_startup import PhysicalStartupRamp
from .policy_shadow import LivePolicyShadow
from .target_projection import (
    ConsecutiveClampWatchdog,
    ProtectedTargetProjector,
    parse_joint_gain_multiplier_overrides,
)
from .yahboom_imu import YahboomQuaternion, YahboomRawImu, YahboomStreamDecoder


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


def _percentile(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, math.ceil(fraction * len(ordered)) - 1)]


def run(args: argparse.Namespace) -> dict:
    hardware = json.loads(Path(args.hardware_config).read_text(encoding="utf-8"))
    contract = PolicyContract.load(args.contract)
    mapping = motor_map_from_hardware_config(
        hardware, contract.data["joint_names"], require_armable=False
    )
    shadow = LivePolicyShadow(contract, hardware, mapping, (args.vx, args.vy, args.yaw_rate))
    projector = None
    if args.joint_limit_candidates:
        joint_names = tuple(contract.data["joint_names"])
        projector = ProtectedTargetProjector.from_limit_report(
            joint_names,
            args.joint_limit_candidates,
            gain_scale=args.gain_scale,
            maximum_embedded_kd=float(
                hardware["controller"]["damiao_embedded_kd_max"]
            ),
            joint_gain_multipliers=parse_joint_gain_multiplier_overrides(
                joint_names, args.joint_gain_multiplier
            ),
        )
    elif args.gain_scale != 1.0:
        raise ValueError("--gain-scale requires --joint-limit-candidates")
    clamp_watchdog = None
    if args.fail_on_consecutive_clamp_joint:
        if projector is None:
            raise ValueError("clamp watchdog requires --joint-limit-candidates")
        clamp_watchdog = ConsecutiveClampWatchdog(
            tuple(contract.data["joint_names"]),
            tuple(args.fail_on_consecutive_clamp_joint),
            args.clamp_watchdog_minimum_overshoot_rad,
            args.clamp_watchdog_maximum_consecutive_ticks,
        )
    startup = None
    if args.physical_startup_hold_seconds or args.physical_startup_ramp_seconds:
        if projector is None:
            raise ValueError("physical startup ramp requires protected target projection")
        startup = PhysicalStartupRamp(
            policy_hz=contract.policy_hz,
            hold_seconds=args.physical_startup_hold_seconds,
            ramp_seconds=args.physical_startup_ramp_seconds,
        )
    motor_names = _native_motor_order(hardware)
    motor_hash = ordered_name_hash(motor_names)
    joint_hash = ordered_name_hash(contract.data["joint_names"])
    decoder = YahboomStreamDecoder()
    state_count = 0
    target_count = 0
    target_sequence = 0
    raw_imu_count = 0
    quaternion_count = 0
    last_state_sequence = 0
    inference_ms: list[float] = []
    errors: list[str] = []
    trace: dict[str, list] | None = None
    if args.trace_output:
        trace = {
            "state_sequence": [],
            "state_monotonic_ns": [],
            "target_monotonic_ns": [],
            "motor_position_rad": [],
            "motor_velocity_rad_s": [],
            "joint_position_rad": [],
            "joint_velocity_rad_s": [],
            "base_angular_velocity_rad_s": [],
            "projected_gravity": [],
            "observation": [],
            "raw_action": [],
            "handoff_action": [],
            "target_position_rad": [],
            "projected_target_position_rad": [],
            "projected_target_velocity_rad_s": [],
            "projected_kp": [],
            "projected_kd": [],
            "projected_feedforward_torque_nm": [],
            "startup_target_position_rad": [],
            "startup_target_velocity_rad_s": [],
            "startup_kp": [],
            "startup_kd": [],
            "startup_feedforward_torque_nm": [],
            "startup_alpha": [],
        }

    try:
        import serial
    except ImportError as exc:
        raise RuntimeError("pyserial is required for native policy IPC shadow") from exc

    with socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET) as connection, serial.Serial(
        port=args.imu_device,
        baudrate=args.imu_baud,
        timeout=0,
        write_timeout=0,
        exclusive=True,
    ) as imu_port:
        connection.connect(args.socket)
        connection.setblocking(False)
        imu_port.dtr = False
        imu_port.rts = False
        with selectors.DefaultSelector() as selector:
            selector.register(connection, selectors.EVENT_READ, "state")
            selector.register(imu_port, selectors.EVENT_READ, "imu")
            running = True
            while running:
                for key, _mask in selector.select(timeout=0.1):
                    if key.data == "imu":
                        payload = imu_port.read(max(imu_port.in_waiting, 1))
                        for packet in decoder.feed(payload):
                            if isinstance(packet, YahboomRawImu):
                                raw_imu_count += 1
                                shadow.update_imu(packet)
                            elif isinstance(packet, YahboomQuaternion):
                                quaternion_count += 1
                                shadow.update_imu(packet)
                        continue

                    try:
                        payload = connection.recv(4096)
                    except ConnectionResetError:
                        if state_count == 0:
                            raise RuntimeError(
                                "native IPC disconnected before publishing a state"
                            ) from None
                        running = False
                        break
                    if not payload:
                        running = False
                        break
                    state = NativeStatePacket.unpack(payload)
                    if state.motor_order_hash != motor_hash:
                        raise RuntimeError("native state motor-order hash mismatch")
                    if state.sequence <= last_state_sequence:
                        raise RuntimeError("native state sequence did not increase")
                    last_state_sequence = state.sequence
                    state_count += 1
                    shadow.motor_states = {
                        name: DamiaoMitState(position, velocity)
                        for name, position, velocity in zip(
                            motor_names, state.position_rad, state.velocity_rad_s, strict=True
                        )
                    }
                    if not shadow.ready:
                        continue
                    started_ns = time.perf_counter_ns()
                    target = shadow.infer_policy_target(state.monotonic_ns)
                    inference_ms.append((time.perf_counter_ns() - started_ns) / 1.0e6)
                    policy_trace = shadow.last_policy_trace
                    if policy_trace is None:
                        raise RuntimeError("policy trace was not captured")
                    raw_target = target
                    if projector is not None:
                        target = projector.project(raw_target)
                    projected_target = target
                    if clamp_watchdog is not None:
                        clamp_watchdog.update(
                            raw_target.position_rad, projected_target.position_rad
                        )
                    if startup is not None:
                        target = startup.apply(
                            projected_target,
                            projector.require_position_within_limits(
                                policy_trace.joint_position_rad,
                                label="physical startup measured pose",
                            ),
                        )
                    target_sequence += 1
                    packet = PolicyTargetPacket(
                        sequence=target_sequence,
                        monotonic_ns=time.monotonic_ns(),
                        joint_order_hash=joint_hash,
                        source_state_sequence=state.sequence,
                        position_rad=tuple(map(float, target.position_rad)),
                        velocity_rad_s=tuple(map(float, target.velocity_rad_s)),
                        kp=tuple(map(float, target.kp)),
                        kd=tuple(map(float, target.kd)),
                        feedforward_torque_nm=tuple(map(float, target.feedforward_torque_nm)),
                    )
                    if trace is not None:
                        trace["state_sequence"].append(state.sequence)
                        trace["state_monotonic_ns"].append(state.monotonic_ns)
                        trace["target_monotonic_ns"].append(packet.monotonic_ns)
                        trace["motor_position_rad"].append(state.position_rad)
                        trace["motor_velocity_rad_s"].append(state.velocity_rad_s)
                        for field in (
                            "joint_position_rad",
                            "joint_velocity_rad_s",
                            "base_angular_velocity_rad_s",
                            "projected_gravity",
                            "observation",
                            "raw_action",
                            "handoff_action",
                            "target_position_rad",
                        ):
                            trace[field].append(getattr(policy_trace, field))
                        trace["projected_target_position_rad"].append(
                            projected_target.position_rad
                        )
                        trace["projected_target_velocity_rad_s"].append(
                            projected_target.velocity_rad_s
                        )
                        trace["projected_kp"].append(projected_target.kp)
                        trace["projected_kd"].append(projected_target.kd)
                        trace["projected_feedforward_torque_nm"].append(
                            projected_target.feedforward_torque_nm
                        )
                        trace["startup_target_position_rad"].append(target.position_rad)
                        trace["startup_target_velocity_rad_s"].append(
                            target.velocity_rad_s
                        )
                        trace["startup_kp"].append(target.kp)
                        trace["startup_kd"].append(target.kd)
                        trace["startup_feedforward_torque_nm"].append(
                            target.feedforward_torque_nm
                        )
                        trace["startup_alpha"].append(
                            startup.last_alpha if startup is not None else 1.0
                        )
                    try:
                        connection.sendall(packet.pack())
                    except BrokenPipeError:
                        if target_count == 0:
                            raise RuntimeError(
                                "native IPC disconnected before accepting a target"
                            ) from None
                        running = False
                        break
                    target_count += 1

    coverage = target_count / state_count if state_count else 0.0
    trace_sha256 = None
    if trace is not None:
        trace_path = Path(args.trace_output)
        trace_path.parent.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(
            trace_path,
            motor_names=np.asarray(motor_names),
            joint_names=np.asarray(contract.data["joint_names"]),
            **{name: np.asarray(values) for name, values in trace.items()},
        )
        trace_sha256 = hashlib.sha256(trace_path.read_bytes()).hexdigest()

    report = {
        "mode": "python_onnx_native_ipc_policy_shadow_no_actuation",
        "state_count": state_count,
        "target_count": target_count,
        "target_coverage": coverage,
        "raw_imu_count": raw_imu_count,
        "quaternion_count": quaternion_count,
        "imu_rejected_frame_count": decoder.rejected_frame_count,
        "inference_mean_ms": float(np.mean(inference_ms)) if inference_ms else None,
        "inference_p99_ms": _percentile(inference_ms, 0.99),
        "inference_max_ms": max(inference_ms) if inference_ms else None,
        "nonzero_can_tx_attempts": 0,
        "protected_target_projection": (
            projector.report() if projector is not None else {"enabled": False}
        ),
        "consecutive_clamp_watchdog": (
            clamp_watchdog.report()
            if clamp_watchdog is not None
            else {"enabled": False}
        ),
        "physical_startup_ramp": (
            startup.report() if startup is not None else {"enabled": False}
        ),
        "trace_output": str(Path(args.trace_output).resolve()) if args.trace_output else None,
        "trace_sha256": trace_sha256,
        "trace_tick_count": len(trace["state_sequence"]) if trace is not None else 0,
        "errors": errors,
        "passed": (
            not errors
            and state_count > 0
            and coverage >= 0.9
            and raw_imu_count > 0
            and quaternion_count > 0
            and bool(inference_ms)
            and max(inference_ms) <= 20.0
        ),
    }
    Path(args.output).parent.mkdir(parents=True, exist_ok=True)
    Path(args.output).write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    return report


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--socket", required=True)
    parser.add_argument("--hardware-config", required=True)
    parser.add_argument("--contract", required=True)
    parser.add_argument("--imu-device", default="/dev/ttyCH341USB0")
    parser.add_argument("--imu-baud", type=int, default=115200)
    parser.add_argument("--vx", type=float, default=0.0)
    parser.add_argument("--vy", type=float, default=0.0)
    parser.add_argument("--yaw-rate", type=float, default=0.0)
    parser.add_argument("--output", required=True)
    parser.add_argument("--trace-output")
    parser.add_argument("--joint-limit-candidates")
    parser.add_argument("--gain-scale", type=float, default=1.0)
    parser.add_argument(
        "--joint-gain-multiplier",
        action="append",
        default=[],
        metavar="JOINT=FACTOR",
    )
    parser.add_argument(
        "--fail-on-consecutive-clamp-joint",
        action="append",
        default=[],
        metavar="JOINT",
    )
    parser.add_argument(
        "--clamp-watchdog-minimum-overshoot-rad", type=float, default=0.05
    )
    parser.add_argument(
        "--clamp-watchdog-maximum-consecutive-ticks", type=int, default=5
    )
    parser.add_argument("--physical-startup-hold-seconds", type=float, default=0.0)
    parser.add_argument("--physical-startup-ramp-seconds", type=float, default=0.0)
    args = parser.parse_args()
    report = run(args)
    print(json.dumps(report, indent=2))
    if not report["passed"]:
        raise SystemExit("native policy IPC client shadow failed")


if __name__ == "__main__":
    main()
