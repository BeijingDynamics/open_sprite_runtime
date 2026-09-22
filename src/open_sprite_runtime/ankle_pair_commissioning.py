"""Fail-closed first-powered transport gate for one differential ankle pair."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math
import time
from typing import Any, Callable, Mapping, Sequence

from .damiao import DamiaoFeedback, DamiaoFeedbackEndpoint, decode_damiao_feedback, encode_zero_gain_position_echo
from .motor_mapping import DifferentialPairDriveMap


ANKLE_GROUPS = {
    "left": (
        ("left_ankle_motor_a", "kcan1", 5, 0x15),
        ("left_ankle_motor_b", "kcan1", 6, 0x16),
    ),
    "right": (
        ("right_ankle_motor_a", "kcan2", 5, 0x15),
        ("right_ankle_motor_b", "kcan2", 6, 0x16),
    ),
}


@dataclass(frozen=True)
class AnkleZeroTorqueReport:
    side: str
    interface: str
    motor_names: tuple[str, str]
    initial_motor_position_rad: dict[str, float]
    initial_joint_position_rad: dict[str, float]
    duration_s: float
    rate_hz_per_motor: float
    command_count: dict[str, int]
    feedback_count: dict[str, int]
    enable_attempts: dict[str, int]
    disable_attempts: dict[str, int]
    maximum_abs_motor_position_drift_rad: dict[str, float]
    maximum_abs_motor_velocity_rad_s: dict[str, float]
    maximum_abs_estimated_torque_nm: dict[str, float]
    final_status: dict[str, str | None]
    errors: tuple[str, ...]

    @property
    def passed(self) -> bool:
        minimum_count = math.floor(self.duration_s * self.rate_hz_per_motor * 0.95)
        return (
            not self.errors
            and all(value >= minimum_count for value in self.command_count.values())
            and all(value >= minimum_count for value in self.feedback_count.values())
            and all(value == 1 for value in self.enable_attempts.values())
            and all(value >= 3 for value in self.disable_attempts.values())
            and all(value == "disabled" for value in self.final_status.values())
        )

    def to_dict(self) -> dict[str, Any]:
        return {**asdict(self), "passed": self.passed}


def _receive(
    writer: Any,
    endpoint: DamiaoFeedbackEndpoint,
    timeout_s: float,
    *,
    monotonic: Callable[[], float],
    sleep: Callable[[float], None],
) -> DamiaoFeedback:
    deadline = monotonic() + timeout_s
    while monotonic() < deadline:
        try:
            frame = writer.receive()
        except BlockingIOError:
            sleep(0.0001)
            continue
        if frame.interface == endpoint.interface and frame.can_id == endpoint.master_id:
            return decode_damiao_feedback(frame, endpoint)
    raise RuntimeError(f"{endpoint.motor_name} feedback watchdog expired")


def run_ankle_pair_zero_torque_gate(
    writer: Any,
    endpoints: Sequence[DamiaoFeedbackEndpoint],
    pair: DifferentialPairDriveMap,
    *,
    side: str,
    soft_position_rad: Mapping[str, tuple[float, float]],
    monotonic: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
) -> AnkleZeroTorqueReport:
    """Enable one ankle pair at 500 Hz with zero embedded gains and zero torque."""
    if side not in ANKLE_GROUPS:
        raise ValueError("ankle side must be left or right")
    expected = ANKLE_GROUPS[side]
    selected = tuple(endpoints)
    identity = tuple(
        (item.motor_name, item.interface, item.can_id, item.master_id) for item in selected
    )
    if identity != expected or tuple(pair.motor_names) != tuple(item[0] for item in expected):
        raise ValueError("ankle gate does not match the frozen ordered differential pair")
    names = tuple(item.motor_name for item in selected)
    interface = expected[0][1]
    if writer.interface != interface or set(soft_position_rad) != set(names):
        raise ValueError("ankle writer or soft-limit coverage mismatch")

    duration_s = 2.0
    rate_hz = 500.0
    period = 1.0 / rate_hz
    feedback_timeout_s = 0.02
    maximum_position_drift_rad = 0.05
    maximum_velocity_rad_s = 0.2
    maximum_torque_nm = 0.1

    errors: list[str] = []
    initial: dict[str, float] = {}
    latest: dict[str, DamiaoFeedback] = {}
    command_count = {name: 0 for name in names}
    feedback_count = {name: 0 for name in names}
    max_drift = {name: 0.0 for name in names}
    max_velocity = {name: 0.0 for name in names}
    max_torque = {name: 0.0 for name in names}
    final_status: dict[str, str | None] = {name: None for name in names}
    initial_joint: dict[str, float] = {}

    try:
        for endpoint in selected:
            writer.send_command(encode_zero_gain_position_echo(endpoint, 0.0))
            feedback = _receive(
                writer, endpoint, feedback_timeout_s, monotonic=monotonic, sleep=sleep
            )
            if feedback.status_name != "disabled":
                raise RuntimeError(f"{endpoint.motor_name} must start disabled")
            low, high = soft_position_rad[endpoint.motor_name]
            if not low + 0.05 <= feedback.position_rad <= high - 0.05:
                raise RuntimeError(f"{endpoint.motor_name} lacks soft-limit margin")
            initial[endpoint.motor_name] = feedback.position_rad
            latest[endpoint.motor_name] = feedback

        joint_values = pair.drive_to_joint_position([initial[name] for name in names])
        if len(joint_values) != 2 or not all(math.isfinite(float(value)) for value in joint_values):
            raise RuntimeError("ankle differential reconstruction is not finite")
        initial_joint = dict(zip(pair.joint_names, map(float, joint_values), strict=True))

        for endpoint in selected:
            writer.send_command(
                encode_zero_gain_position_echo(endpoint, initial[endpoint.motor_name])
            )
            prepared = _receive(
                writer, endpoint, feedback_timeout_s, monotonic=monotonic, sleep=sleep
            )
            if prepared.status_name != "disabled":
                raise RuntimeError(f"{endpoint.motor_name} changed status during preparation")
            latest[endpoint.motor_name] = prepared
        for endpoint in selected:
            writer.send_enable(endpoint.motor_name)

        start = monotonic()
        next_cycle = start
        while monotonic() - start < duration_s:
            now = monotonic()
            if now < next_cycle:
                sleep(min(next_cycle - now, 0.0001))
                continue
            for endpoint in selected:
                name = endpoint.motor_name
                writer.send_command(encode_zero_gain_position_echo(endpoint, latest[name].position_rad))
                command_count[name] += 1
                feedback = _receive(
                    writer, endpoint, feedback_timeout_s, monotonic=monotonic, sleep=sleep
                )
                feedback_count[name] += 1
                if feedback.status_name != "enabled":
                    raise RuntimeError(f"{name} status must be enabled")
                drift = abs(feedback.position_rad - initial[name])
                max_drift[name] = max(max_drift[name], drift)
                max_velocity[name] = max(max_velocity[name], abs(feedback.velocity_rad_s))
                max_torque[name] = max(max_torque[name], abs(feedback.estimated_output_torque_nm))
                if drift > maximum_position_drift_rad:
                    raise RuntimeError(f"{name} position-drift guard tripped")
                if abs(feedback.velocity_rad_s) > maximum_velocity_rad_s:
                    raise RuntimeError(f"{name} velocity guard tripped")
                if abs(feedback.estimated_output_torque_nm) > maximum_torque_nm:
                    raise RuntimeError(f"{name} torque guard tripped")
                latest[name] = feedback
            next_cycle += period
            if next_cycle <= monotonic():
                next_cycle = monotonic() + period
    except BaseException as exc:
        errors.append(str(exc))
    finally:
        for _ in range(3):
            for endpoint in selected:
                try:
                    writer.send_disable(endpoint.motor_name)
                except BaseException as exc:
                    errors.append(f"{endpoint.motor_name} disable failed: {exc}")
            sleep(0.01)
        for endpoint in selected:
            name = endpoint.motor_name
            deadline = monotonic() + 0.5
            last_error: str | None = None
            while final_status[name] != "disabled" and monotonic() < deadline:
                try:
                    writer.send_disable(name)
                    writer.send_command(
                        encode_zero_gain_position_echo(endpoint, initial.get(name, 0.0))
                    )
                    feedback = _receive(
                        writer, endpoint, feedback_timeout_s, monotonic=monotonic, sleep=sleep
                    )
                    final_status[name] = feedback.status_name
                except BaseException as exc:
                    last_error = str(exc)
            if final_status[name] != "disabled":
                detail = f": {last_error}" if last_error else ""
                errors.append(f"{name} final disabled verification failed{detail}")

    return AnkleZeroTorqueReport(
        side=side,
        interface=interface,
        motor_names=names,
        initial_motor_position_rad=initial,
        initial_joint_position_rad=initial_joint,
        duration_s=duration_s,
        rate_hz_per_motor=rate_hz,
        command_count=command_count,
        feedback_count=feedback_count,
        enable_attempts=dict(writer.enable_attempts),
        disable_attempts=dict(writer.disable_attempts),
        maximum_abs_motor_position_drift_rad=max_drift,
        maximum_abs_motor_velocity_rad_s=max_velocity,
        maximum_abs_estimated_torque_nm=max_torque,
        final_status=final_status,
        errors=tuple(errors),
    )
