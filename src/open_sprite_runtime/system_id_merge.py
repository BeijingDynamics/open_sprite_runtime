"""Align raw CAN-FD traffic with the 50 Hz policy trace."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
from typing import Any, Mapping

import numpy as np


def _decode_uint(value: np.ndarray, limits: tuple[float, float], bits: int) -> np.ndarray:
    low, high = limits
    return value.astype(np.float64) * (high - low) / ((1 << bits) - 1) + low


def _motor_records(hardware: Mapping[str, Any]) -> list[tuple[str, Mapping[str, Any]]]:
    interfaces = tuple(hardware["can_adapter"]["interfaces"])
    records = list(hardware["motor_map"].items())
    records.sort(key=lambda item: (int(item[1]["can_channel"]), int(item[1]["can_id"])))
    for name, record in records:
        channel = int(record["can_channel"])
        if not 0 <= channel < len(interfaces):
            raise ValueError(f"{name}: invalid can_channel")
        if "mit_ranges" not in record:
            raise ValueError(f"{name}: missing mit_ranges")
    return records


def _latest_indices(sample_ns: np.ndarray, target_ns: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    indices = np.searchsorted(sample_ns, target_ns, side="right") - 1
    valid = indices >= 0
    clipped = np.maximum(indices, 0)
    age_ms = np.full(target_ns.shape, np.nan, dtype=np.float64)
    age_ms[valid] = (target_ns[valid] - sample_ns[clipped[valid]]) / 1.0e6
    return clipped, age_ms


def merge_system_id_traces(
    can_trace: Mapping[str, np.ndarray],
    policy_trace: Mapping[str, np.ndarray],
    hardware: Mapping[str, Any],
) -> tuple[dict[str, np.ndarray], dict[str, Any]]:
    policy_ns = np.asarray(policy_trace["state_monotonic_ns"], dtype=np.int64)
    if policy_ns.ndim != 1 or policy_ns.size == 0 or np.any(np.diff(policy_ns) <= 0):
        raise ValueError("policy state_monotonic_ns must be a non-empty increasing vector")
    interfaces = tuple(str(value) for value in can_trace["interface_names"])
    expected_interfaces = tuple(hardware["can_adapter"]["interfaces"])
    if interfaces != expected_interfaces:
        raise ValueError("CAN trace interface order does not match hardware contract")

    frame_ns = np.asarray(can_trace["monotonic_receive_ns"], dtype=np.int64)
    interface_index = np.asarray(can_trace["interface_index"], dtype=np.int64)
    can_id = np.asarray(can_trace["can_id"], dtype=np.int64)
    payload_length = np.asarray(can_trace["payload_length"], dtype=np.int64)
    payload = np.asarray(can_trace["payload"], dtype=np.uint8)
    if payload.shape != (frame_ns.size, 64):
        raise ValueError("CAN trace payload must have shape (frames, 64)")

    motors = _motor_records(hardware)
    ticks = policy_ns.size
    motor_count = len(motors)
    feedback_position = np.full((ticks, motor_count), np.nan)
    feedback_velocity = np.full((ticks, motor_count), np.nan)
    feedback_torque = np.full((ticks, motor_count), np.nan)
    feedback_mos = np.full((ticks, motor_count), np.nan)
    feedback_rotor = np.full((ticks, motor_count), np.nan)
    feedback_age_ms = np.full((ticks, motor_count), np.nan)
    command_position = np.full((ticks, motor_count), np.nan)
    command_velocity = np.full((ticks, motor_count), np.nan)
    command_kp = np.full((ticks, motor_count), np.nan)
    command_kd = np.full((ticks, motor_count), np.nan)
    command_torque = np.full((ticks, motor_count), np.nan)
    command_age_ms = np.full((ticks, motor_count), np.nan)
    feedback_counts: dict[str, int] = {}
    command_counts: dict[str, int] = {}

    for column, (name, record) in enumerate(motors):
        channel = int(record["can_channel"])
        ranges = record["mit_ranges"]
        feedback_mask = (
            (interface_index == channel)
            & (can_id == int(record["master_id"]))
            & (payload_length == 8)
        )
        feedback_rows = np.flatnonzero(feedback_mask)
        feedback_counts[name] = int(feedback_rows.size)
        if feedback_rows.size:
            data = payload[feedback_rows]
            order = np.argsort(frame_ns[feedback_rows], kind="stable")
            rows = feedback_rows[order]
            data = data[order]
            sample_ns = frame_ns[rows]
            raw_position = (data[:, 1].astype(np.uint16) << 8) | data[:, 2]
            raw_velocity = (data[:, 3].astype(np.uint16) << 4) | (data[:, 4] >> 4)
            raw_torque = ((data[:, 4].astype(np.uint16) & 0x0F) << 8) | data[:, 5]
            latest, age = _latest_indices(sample_ns, policy_ns)
            feedback_position[:, column] = _decode_uint(
                raw_position[latest], tuple(ranges["position_rad"]), 16
            )
            feedback_velocity[:, column] = _decode_uint(
                raw_velocity[latest], tuple(ranges["velocity_rad_s"]), 12
            )
            feedback_torque[:, column] = _decode_uint(
                raw_torque[latest], tuple(ranges["torque_nm"]), 12
            )
            feedback_mos[:, column] = data[latest, 6]
            feedback_rotor[:, column] = data[latest, 7]
            feedback_age_ms[:, column] = age

        command_mask = (
            (interface_index == channel)
            & (can_id == int(record["can_id"]))
            & (payload_length == 8)
        )
        command_rows = np.flatnonzero(command_mask)
        if command_rows.size:
            command_data = payload[command_rows]
            special = np.all(command_data[:, :7] == 0xFF, axis=1) & (
                command_data[:, 7] >= 0xFC
            )
            command_rows = command_rows[~special]
        command_counts[name] = int(command_rows.size)
        if command_rows.size:
            data = payload[command_rows]
            order = np.argsort(frame_ns[command_rows], kind="stable")
            rows = command_rows[order]
            data = data[order]
            sample_ns = frame_ns[rows]
            raw_position = (data[:, 0].astype(np.uint16) << 8) | data[:, 1]
            raw_velocity = (data[:, 2].astype(np.uint16) << 4) | (data[:, 3] >> 4)
            raw_kp = ((data[:, 3].astype(np.uint16) & 0x0F) << 8) | data[:, 4]
            raw_kd = (data[:, 5].astype(np.uint16) << 4) | (data[:, 6] >> 4)
            raw_torque = ((data[:, 6].astype(np.uint16) & 0x0F) << 8) | data[:, 7]
            latest, age = _latest_indices(sample_ns, policy_ns)
            command_position[:, column] = _decode_uint(
                raw_position[latest], tuple(ranges["position_rad"]), 16
            )
            command_velocity[:, column] = _decode_uint(
                raw_velocity[latest], tuple(ranges["velocity_rad_s"]), 12
            )
            command_kp[:, column] = _decode_uint(raw_kp[latest], (0.0, 500.0), 12)
            command_kd[:, column] = _decode_uint(raw_kd[latest], (0.0, 5.0), 12)
            command_torque[:, column] = _decode_uint(
                raw_torque[latest], tuple(ranges["torque_nm"]), 12
            )
            command_age_ms[:, column] = age

    feedback_coverage = np.mean(np.isfinite(feedback_age_ms), axis=0)
    command_coverage = np.mean(np.isfinite(command_age_ms), axis=0)
    feedback_p99_age = np.asarray(
        [
            np.nanpercentile(feedback_age_ms[:, index], 99)
            if np.any(np.isfinite(feedback_age_ms[:, index]))
            else np.nan
            for index in range(motor_count)
        ]
    )
    errors = []
    if np.any(feedback_coverage < 0.98):
        errors.append("one or more motors have less than 98% feedback coverage")
    if np.any(~np.isfinite(feedback_p99_age)) or np.nanmax(feedback_p99_age) > 60.0:
        errors.append("one or more motors exceed the 60 ms p99 feedback-age gate")
    if np.any(command_coverage < 0.90):
        errors.append("one or more motors have less than 90% observed command coverage")

    names = np.asarray([name for name, _record in motors])
    merged = {
        "policy_monotonic_ns": policy_ns,
        "motor_names": names,
        "feedback_position_rad": feedback_position,
        "feedback_velocity_rad_s": feedback_velocity,
        "feedback_estimated_torque_nm": feedback_torque,
        "feedback_mos_temperature_c": feedback_mos,
        "feedback_rotor_temperature_c": feedback_rotor,
        "feedback_age_ms": feedback_age_ms,
        "command_position_rad": command_position,
        "command_velocity_rad_s": command_velocity,
        "command_kp": command_kp,
        "command_kd": command_kd,
        "command_feedforward_torque_nm": command_torque,
        "command_age_ms": command_age_ms,
    }
    report = {
        "mode": "offline_can_policy_system_id_alignment",
        "policy_tick_count": int(ticks),
        "can_frame_count": int(frame_ns.size),
        "motor_count": motor_count,
        "feedback_count_by_motor": feedback_counts,
        "command_count_by_motor": command_counts,
        "feedback_coverage_by_motor": dict(zip(names.tolist(), feedback_coverage.tolist())),
        "command_coverage_by_motor": dict(zip(names.tolist(), command_coverage.tolist())),
        "feedback_p99_age_ms_by_motor": dict(zip(names.tolist(), feedback_p99_age.tolist())),
        "errors": errors,
        "passed": not errors,
    }
    return merged, report


def merge_system_id_files(
    can_trace_path: str | Path,
    policy_trace_path: str | Path,
    hardware_path: str | Path,
    output_path: str | Path,
    report_path: str | Path,
) -> dict[str, Any]:
    can_trace_path = Path(can_trace_path)
    policy_trace_path = Path(policy_trace_path)
    hardware_path = Path(hardware_path)
    with np.load(can_trace_path, allow_pickle=False) as data:
        can_trace = {name: data[name] for name in data.files}
    with np.load(policy_trace_path, allow_pickle=False) as data:
        policy_trace = {name: data[name] for name in data.files}
    hardware = json.loads(hardware_path.read_text(encoding="utf-8"))
    merged, report = merge_system_id_traces(can_trace, policy_trace, hardware)
    output_path = Path(output_path)
    report_path = Path(report_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(output_path, **merged)
    report.update(
        {
            "can_trace_sha256": hashlib.sha256(can_trace_path.read_bytes()).hexdigest(),
            "policy_trace_sha256": hashlib.sha256(policy_trace_path.read_bytes()).hexdigest(),
            "hardware_sha256": hashlib.sha256(hardware_path.read_bytes()).hexdigest(),
            "output": str(output_path.resolve()),
        }
    )
    report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    return report
