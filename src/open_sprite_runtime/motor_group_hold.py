"""Fail-closed current-position holds for fixed same-bus motor groups."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import time
from typing import Any, Callable, Mapping, Sequence

from .damiao import (
    DamiaoFeedback,
    DamiaoFeedbackEndpoint,
    DamiaoMitCommand,
    DamiaoMitCommandEnvelope,
    DamiaoMitState,
    decode_damiao_feedback,
    encode_damiao_mit_command,
    encode_zero_gain_position_echo,
)


RIGHT_WRIST_GROUP = (
    ("right_wrist_yaw_motor", "kcan4", 5, 0x15),
    ("right_wrist_pitch_motor", "kcan4", 6, 0x16),
    ("right_wrist_roll_motor", "kcan4", 7, 0x17),
)

RIGHT_ARM_GROUP = (
    ("right_shoulder_pitch_motor", "kcan4", 1, 0x11),
    ("right_shoulder_roll_motor", "kcan4", 2, 0x12),
    ("right_shoulder_yaw_motor", "kcan4", 3, 0x13),
    ("right_elbow_motor", "kcan4", 4, 0x14),
    *RIGHT_WRIST_GROUP,
)

LEFT_ARM_GROUP = (
    ("left_shoulder_pitch_motor", "kcan3", 1, 0x11),
    ("left_shoulder_roll_motor", "kcan3", 2, 0x12),
    ("left_shoulder_yaw_motor", "kcan3", 3, 0x13),
    ("left_elbow_motor", "kcan3", 4, 0x14),
    ("left_wrist_yaw_motor", "kcan3", 5, 0x15),
    ("left_wrist_pitch_motor", "kcan3", 6, 0x16),
    ("left_wrist_roll_motor", "kcan3", 7, 0x17),
)

LEFT_PROXIMAL_LEG_GROUP = (
    ("left_hip_pitch_motor", "kcan1", 1, 0x11),
    ("left_hip_roll_motor", "kcan1", 2, 0x12),
    ("left_hip_yaw_motor", "kcan1", 3, 0x13),
    ("left_knee_motor", "kcan1", 4, 0x14),
)

RIGHT_PROXIMAL_LEG_GROUP = (
    ("right_hip_pitch_motor", "kcan2", 1, 0x11),
    ("right_hip_roll_motor", "kcan2", 2, 0x12),
    ("right_hip_yaw_motor", "kcan2", 3, 0x13),
    ("right_knee_motor", "kcan2", 4, 0x14),
)


@dataclass(frozen=True)
class MotorGroupHoldReport:
    interface: str
    motor_names: tuple[str, ...]
    initial_position_rad: dict[str, float]
    target_position_rad: dict[str, float]
    duration_s: float
    rate_hz: float
    command_count: dict[str, int]
    feedback_count: dict[str, int]
    enable_attempts: dict[str, int]
    disable_attempts: dict[str, int]
    maximum_abs_position_error_rad: dict[str, float]
    maximum_abs_velocity_rad_s: dict[str, float]
    maximum_abs_estimated_torque_nm: dict[str, float]
    maximum_mos_temperature_c: dict[str, int]
    maximum_rotor_temperature_c: dict[str, int]
    final_status: dict[str, str | None]
    errors: tuple[str, ...]

    @property
    def passed(self) -> bool:
        return (
            not self.errors
            and all(value > 0 for value in self.command_count.values())
            and all(value > 0 for value in self.feedback_count.values())
            and all(value == 1 for value in self.enable_attempts.values())
            and all(value >= 3 for value in self.disable_attempts.values())
            and all(value == "disabled" for value in self.final_status.values())
        )

    def to_dict(self) -> dict[str, Any]:
        return {**asdict(self), "passed": self.passed}


def _receive_endpoint(
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
            sleep(0.001)
            continue
        if frame.interface == endpoint.interface and frame.can_id == endpoint.master_id:
            return decode_damiao_feedback(frame, endpoint)
    raise RuntimeError(f"{endpoint.motor_name} feedback watchdog expired")


def _run_fixed_group_low_gain_hold(
    writer: Any,
    endpoints: Sequence[DamiaoFeedbackEndpoint],
    *,
    soft_position_rad: Mapping[str, tuple[float, float]],
    maximum_torque_nm: Mapping[str, float],
    expected_identity: tuple[tuple[str, str, int, int], ...],
    expected_interface: str,
    monotonic: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
) -> MotorGroupHoldReport:
    """Hold one exact same-bus motor group at its measured positions."""
    selected = tuple(endpoints)
    identity = tuple(
        (item.motor_name, item.interface, item.can_id, item.master_id) for item in selected
    )
    if identity != expected_identity:
        raise ValueError("powered group hold does not match the frozen ordered group")
    names = tuple(item.motor_name for item in selected)
    if set(soft_position_rad) != set(names):
        raise ValueError("soft limits must exactly cover the frozen motor group")
    if set(maximum_torque_nm) != set(names) or any(
        not 0.0 < value <= 0.5 for value in maximum_torque_nm.values()
    ):
        raise ValueError("torque guards must exactly cover the group and be in (0, 0.5] Nm")
    if writer.interface != expected_interface:
        raise ValueError(f"motor-group writer must use {expected_interface}")

    duration_s = 2.0
    rate_hz = 50.0
    kp = 0.2
    kd = 0.05
    maximum_position_error_rad = 0.05
    maximum_velocity_rad_s = 0.2
    feedback_timeout_s = 0.1
    mos_temperature_limit_c = 100
    rotor_temperature_limit_c = 80
    period = 1.0 / rate_hz

    errors: list[str] = []
    initial_positions: dict[str, float] = {}
    targets: dict[str, float] = {}
    latest: dict[str, DamiaoFeedback] = {}
    command_count = {name: 0 for name in names}
    feedback_count = {name: 0 for name in names}
    max_error = {name: 0.0 for name in names}
    max_velocity = {name: 0.0 for name in names}
    max_torque = {name: 0.0 for name in names}
    max_mos = {name: 0 for name in names}
    max_rotor = {name: 0 for name in names}
    final_status: dict[str, str | None] = {name: None for name in names}

    def accept(feedback: DamiaoFeedback) -> None:
        name = feedback.motor_name
        feedback_count[name] += 1
        if feedback.status_name != "enabled":
            raise RuntimeError(f"{name} status must be enabled, observed {feedback.status_name}")
        error = targets[name] - feedback.position_rad
        max_error[name] = max(max_error[name], abs(error))
        max_velocity[name] = max(max_velocity[name], abs(feedback.velocity_rad_s))
        max_torque[name] = max(max_torque[name], abs(feedback.estimated_output_torque_nm))
        max_mos[name] = max(max_mos[name], feedback.mos_temperature_c)
        max_rotor[name] = max(max_rotor[name], feedback.rotor_temperature_c)
        if abs(error) > maximum_position_error_rad:
            raise RuntimeError(f"{name} position-error guard tripped")
        if abs(feedback.velocity_rad_s) > maximum_velocity_rad_s:
            raise RuntimeError(f"{name} velocity guard tripped")
        if abs(feedback.estimated_output_torque_nm) > maximum_torque_nm[name]:
            raise RuntimeError(f"{name} torque guard tripped")
        if feedback.mos_temperature_c >= mos_temperature_limit_c:
            raise RuntimeError(f"{name} MOS-temperature guard tripped")
        if feedback.rotor_temperature_c >= rotor_temperature_limit_c:
            raise RuntimeError(f"{name} rotor-temperature guard tripped")

    try:
        for endpoint in selected:
            writer.send_command(encode_zero_gain_position_echo(endpoint, 0.0))
            feedback = _receive_endpoint(
                writer, endpoint, feedback_timeout_s, monotonic=monotonic, sleep=sleep
            )
            if feedback.status_name != "disabled":
                raise RuntimeError(f"{endpoint.motor_name} must start disabled")
            initial_positions[endpoint.motor_name] = feedback.position_rad
            low, high = soft_position_rad[endpoint.motor_name]
            if not low + 0.05 <= feedback.position_rad <= high - 0.05:
                raise RuntimeError(f"{endpoint.motor_name} lacks soft-limit margin")
            targets[endpoint.motor_name] = feedback.position_rad
            latest[endpoint.motor_name] = feedback

        for endpoint in selected:
            writer.send_command(
                encode_zero_gain_position_echo(endpoint, targets[endpoint.motor_name])
            )
            prepared = _receive_endpoint(
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
                sleep(min(next_cycle - now, 0.001))
                continue
            for endpoint in selected:
                name = endpoint.motor_name
                low, high = soft_position_rad[name]
                envelope = DamiaoMitCommandEnvelope(
                    position_rad=(low, high),
                    maximum_velocity_rad_s=maximum_velocity_rad_s,
                    maximum_feedforward_torque_nm=maximum_torque_nm[name],
                    maximum_output_torque_nm=maximum_torque_nm[name],
                )
                measured = DamiaoMitState(
                    latest[name].position_rad, latest[name].velocity_rad_s
                )
                command = encode_damiao_mit_command(
                    endpoint,
                    DamiaoMitCommand(targets[name], 0.0, kp, kd, 0.0),
                    measured,
                    envelope,
                )
                writer.send_command(command)
                command_count[name] += 1
                feedback = _receive_endpoint(
                    writer, endpoint, feedback_timeout_s, monotonic=monotonic, sleep=sleep
                )
                accept(feedback)
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
                        encode_zero_gain_position_echo(endpoint, targets.get(name, 0.0))
                    )
                    feedback = _receive_endpoint(
                        writer,
                        endpoint,
                        feedback_timeout_s,
                        monotonic=monotonic,
                        sleep=sleep,
                    )
                    final_status[name] = feedback.status_name
                except BaseException as exc:
                    last_error = str(exc)
            if final_status[name] != "disabled":
                detail = f": {last_error}" if last_error else ""
                errors.append(f"{name} final disabled verification failed{detail}")

    return MotorGroupHoldReport(
        interface=writer.interface,
        motor_names=names,
        initial_position_rad=initial_positions,
        target_position_rad=targets,
        duration_s=duration_s,
        rate_hz=rate_hz,
        command_count=command_count,
        feedback_count=feedback_count,
        enable_attempts=dict(writer.enable_attempts),
        disable_attempts=dict(writer.disable_attempts),
        maximum_abs_position_error_rad=max_error,
        maximum_abs_velocity_rad_s=max_velocity,
        maximum_abs_estimated_torque_nm=max_torque,
        maximum_mos_temperature_c=max_mos,
        maximum_rotor_temperature_c=max_rotor,
        final_status=final_status,
        errors=tuple(errors),
    )


def run_right_wrist_group_low_gain_hold(
    writer: Any,
    endpoints: Sequence[DamiaoFeedbackEndpoint],
    *,
    soft_position_rad: Mapping[str, tuple[float, float]],
    monotonic: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
) -> MotorGroupHoldReport:
    """Hold the exact three right-wrist motors at their measured positions."""
    return _run_fixed_group_low_gain_hold(
        writer,
        endpoints,
        soft_position_rad=soft_position_rad,
        maximum_torque_nm={item.motor_name: 0.1 for item in endpoints},
        expected_identity=RIGHT_WRIST_GROUP,
        expected_interface="kcan4",
        monotonic=monotonic,
        sleep=sleep,
    )


def run_right_arm_group_low_gain_hold(
    writer: Any,
    endpoints: Sequence[DamiaoFeedbackEndpoint],
    *,
    soft_position_rad: Mapping[str, tuple[float, float]],
    monotonic: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
) -> MotorGroupHoldReport:
    """Hold the exact seven right-arm motors at their measured positions."""
    return _run_fixed_group_low_gain_hold(
        writer,
        endpoints,
        soft_position_rad=soft_position_rad,
        maximum_torque_nm={
            item.motor_name: (
                0.5 if item.motor_name in {
                    "right_shoulder_pitch_motor",
                    "right_shoulder_roll_motor",
                } else 0.1
            )
            for item in endpoints
        },
        expected_identity=RIGHT_ARM_GROUP,
        expected_interface="kcan4",
        monotonic=monotonic,
        sleep=sleep,
    )


def run_left_arm_group_low_gain_hold(
    writer: Any,
    endpoints: Sequence[DamiaoFeedbackEndpoint],
    *,
    soft_position_rad: Mapping[str, tuple[float, float]],
    monotonic: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
) -> MotorGroupHoldReport:
    """Hold the exact seven left-arm motors at their measured positions."""
    return _run_fixed_group_low_gain_hold(
        writer,
        endpoints,
        soft_position_rad=soft_position_rad,
        maximum_torque_nm={
            item.motor_name: (
                0.5 if item.motor_name in {
                    "left_shoulder_pitch_motor",
                    "left_shoulder_roll_motor",
                } else 0.1
            )
            for item in endpoints
        },
        expected_identity=LEFT_ARM_GROUP,
        expected_interface="kcan3",
        monotonic=monotonic,
        sleep=sleep,
    )


def run_proximal_leg_group_low_gain_hold(
    writer: Any,
    endpoints: Sequence[DamiaoFeedbackEndpoint],
    *,
    side: str,
    soft_position_rad: Mapping[str, tuple[float, float]],
    monotonic: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
) -> MotorGroupHoldReport:
    """Hold one exact hip/knee group; ankle and torso/head endpoints are excluded."""
    groups = {
        "left": (LEFT_PROXIMAL_LEG_GROUP, "kcan1"),
        "right": (RIGHT_PROXIMAL_LEG_GROUP, "kcan2"),
    }
    if side not in groups:
        raise ValueError("proximal leg side must be left or right")
    expected_identity, expected_interface = groups[side]
    return _run_fixed_group_low_gain_hold(
        writer,
        endpoints,
        soft_position_rad=soft_position_rad,
        maximum_torque_nm={item.motor_name: 0.5 for item in endpoints},
        expected_identity=expected_identity,
        expected_interface=expected_interface,
        monotonic=monotonic,
        sleep=sleep,
    )
