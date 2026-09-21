"""Fail-closed first powered hold for the independent Sprite head-yaw motor."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import time
from typing import Any, Callable

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


@dataclass(frozen=True)
class SingleJointHoldReport:
    motor_name: str
    target_position_rad: float | None
    duration_s: float
    rate_hz: float
    kp: float
    kd: float
    command_count: int
    feedback_count: int
    enable_attempts: int
    disable_attempts: int
    maximum_abs_position_error_rad: float
    maximum_abs_velocity_rad_s: float
    maximum_abs_estimated_torque_nm: float
    maximum_mos_temperature_c: int
    maximum_rotor_temperature_c: int
    final_status: str | None
    errors: tuple[str, ...]

    @property
    def passed(self) -> bool:
        return (
            not self.errors
            and self.command_count > 0
            and self.feedback_count > 0
            and self.final_status == "disabled"
            and self.enable_attempts == 1
            and self.disable_attempts >= 3
        )

    def to_dict(self) -> dict[str, Any]:
        return {**asdict(self), "passed": self.passed}


@dataclass(frozen=True)
class SingleJointMotionReport:
    motor_name: str
    initial_position_rad: float | None
    requested_minimum_position_rad: float | None
    requested_maximum_position_rad: float | None
    duration_s: float
    rate_hz: float
    kp: float
    kd: float
    command_count: int
    feedback_count: int
    enable_attempts: int
    disable_attempts: int
    maximum_abs_position_error_rad: float
    maximum_abs_velocity_rad_s: float
    maximum_abs_estimated_torque_nm: float
    maximum_mos_temperature_c: int
    maximum_rotor_temperature_c: int
    final_status: str | None
    errors: tuple[str, ...]

    @property
    def passed(self) -> bool:
        return (
            not self.errors
            and self.command_count > 0
            and self.feedback_count > 0
            and self.final_status == "disabled"
            and self.enable_attempts == 1
            and self.disable_attempts >= 3
        )

    def to_dict(self) -> dict[str, Any]:
        return {**asdict(self), "passed": self.passed}


def _receive_selected(
    writer: Any,
    endpoint: DamiaoFeedbackEndpoint,
    timeout_s: float,
    *,
    monotonic: Callable[[], float],
    sleep: Callable[[float], None] = time.sleep,
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
    raise RuntimeError("head-yaw feedback watchdog expired")


def run_head_yaw_low_gain_hold(
    writer: Any,
    endpoint: DamiaoFeedbackEndpoint,
    *,
    soft_position_rad: tuple[float, float],
    duration_s: float = 2.0,
    rate_hz: float = 50.0,
    kp: float = 0.2,
    kd: float = 0.05,
    maximum_position_error_rad: float = 0.05,
    maximum_velocity_rad_s: float = 0.2,
    maximum_torque_nm: float = 0.1,
    mos_temperature_limit_c: int = 100,
    rotor_temperature_limit_c: int = 80,
    feedback_timeout_s: float = 0.1,
    monotonic: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
) -> SingleJointHoldReport:
    """Hold measured head yaw; always disable three times before returning."""
    if (
        endpoint.motor_name != "head_yaw_motor"
        or endpoint.interface != "kcan3"
        or endpoint.can_id != 8
    ):
        raise ValueError("first powered hold is restricted to kcan3 ID 0x08 head_yaw_motor")
    if (duration_s, rate_hz, kp, kd) != (2.0, 50.0, 0.2, 0.05):
        raise ValueError("first powered hold duration/rate/gains are frozen")
    if maximum_torque_nm > 0.1 or maximum_velocity_rad_s > 0.2:
        raise ValueError("first powered hold safety envelope cannot be widened")
    soft_low, soft_high = map(float, soft_position_rad)
    if soft_low >= soft_high:
        raise ValueError("head-yaw soft position range is invalid")

    errors: list[str] = []
    target_position: float | None = None
    feedback_count = 0
    command_count = 0
    max_error = 0.0
    max_velocity = 0.0
    max_torque = 0.0
    max_mos = 0
    max_rotor = 0
    final_status: str | None = None
    period = 1.0 / rate_hz

    def accept(feedback: DamiaoFeedback, *, require_status: str) -> None:
        nonlocal feedback_count, max_error, max_velocity, max_torque, max_mos, max_rotor
        feedback_count += 1
        if feedback.status_name != require_status:
            raise RuntimeError(
                f"head-yaw status must be {require_status}, observed {feedback.status_name}"
            )
        error = 0.0 if target_position is None else target_position - feedback.position_rad
        max_error = max(max_error, abs(error))
        max_velocity = max(max_velocity, abs(feedback.velocity_rad_s))
        max_torque = max(max_torque, abs(feedback.estimated_output_torque_nm))
        max_mos = max(max_mos, feedback.mos_temperature_c)
        max_rotor = max(max_rotor, feedback.rotor_temperature_c)
        if abs(error) > maximum_position_error_rad:
            raise RuntimeError("head-yaw position-error guard tripped")
        if abs(feedback.velocity_rad_s) > maximum_velocity_rad_s:
            raise RuntimeError("head-yaw velocity guard tripped")
        if abs(feedback.estimated_output_torque_nm) > maximum_torque_nm:
            raise RuntimeError("head-yaw torque guard tripped")
        if feedback.mos_temperature_c >= mos_temperature_limit_c:
            raise RuntimeError("head-yaw MOS-temperature guard tripped")
        if feedback.rotor_temperature_c >= rotor_temperature_limit_c:
            raise RuntimeError("head-yaw rotor-temperature guard tripped")

    try:
        writer.send_command(encode_zero_gain_position_echo(endpoint, 0.0))
        initial = _receive_selected(
            writer, endpoint, feedback_timeout_s, monotonic=monotonic, sleep=sleep
        )
        if initial.status_name != "disabled":
            raise RuntimeError("head-yaw must be disabled before first powered hold")
        target_position = initial.position_rad
        if not soft_low + 0.05 <= target_position <= soft_high - 0.05:
            raise RuntimeError("head-yaw measured position lacks 0.05 rad soft-limit margin")
        writer.send_command(encode_zero_gain_position_echo(endpoint, target_position))
        prepared = _receive_selected(
            writer, endpoint, feedback_timeout_s, monotonic=monotonic, sleep=sleep
        )
        if prepared.status_name != "disabled":
            raise RuntimeError("head-yaw changed status during zero-gain preparation")

        envelope = DamiaoMitCommandEnvelope(
            position_rad=(soft_low, soft_high),
            maximum_velocity_rad_s=maximum_velocity_rad_s,
            maximum_feedforward_torque_nm=maximum_torque_nm,
            maximum_output_torque_nm=maximum_torque_nm,
        )
        writer.send_enable()
        start = monotonic()
        next_send = start
        while monotonic() - start < duration_s:
            now = monotonic()
            if now < next_send:
                sleep(min(next_send - now, 0.001))
                continue
            measured = DamiaoMitState(
                prepared.position_rad, prepared.velocity_rad_s
            )
            encoded = encode_damiao_mit_command(
                endpoint,
                DamiaoMitCommand(target_position, 0.0, kp, kd, 0.0),
                measured,
                envelope,
            )
            writer.send_command(encoded)
            command_count += 1
            prepared = _receive_selected(
                writer, endpoint, feedback_timeout_s, monotonic=monotonic, sleep=sleep
            )
            accept(prepared, require_status="enabled")
            next_send += period
    except BaseException as exc:
        errors.append(str(exc))
    finally:
        # A control reply already in the RX queue can still report enabled after
        # the first disable. Keep every verification poll zero-gain and require
        # a fresh disabled reply instead of classifying one stale frame.
        poll_position = target_position if target_position is not None else 0.0
        shutdown_deadline = monotonic() + 0.5
        shutdown_error: str | None = None
        shutdown_attempts = 0
        while shutdown_attempts < 3 or (
            final_status != "disabled" and monotonic() < shutdown_deadline
        ):
            shutdown_attempts += 1
            try:
                writer.send_disable()
                sleep(0.01)
                writer.send_command(encode_zero_gain_position_echo(endpoint, poll_position))
                final = _receive_selected(
                    writer,
                    endpoint,
                    feedback_timeout_s,
                    monotonic=monotonic,
                    sleep=sleep,
                )
                final_status = final.status_name
                if writer.disable_attempts >= 3 and final_status == "disabled":
                    break
            except BaseException as exc:
                shutdown_error = str(exc)
        if final_status != "disabled":
            detail = f": {shutdown_error}" if shutdown_error else ""
            errors.append(
                f"final disabled verification timed out at status {final_status}{detail}"
            )

    return SingleJointHoldReport(
        motor_name=endpoint.motor_name,
        target_position_rad=target_position,
        duration_s=duration_s,
        rate_hz=rate_hz,
        kp=kp,
        kd=kd,
        command_count=command_count,
        feedback_count=feedback_count,
        enable_attempts=writer.enable_attempts,
        disable_attempts=writer.disable_attempts,
        maximum_abs_position_error_rad=max_error,
        maximum_abs_velocity_rad_s=max_velocity,
        maximum_abs_estimated_torque_nm=max_torque,
        maximum_mos_temperature_c=max_mos,
        maximum_rotor_temperature_c=max_rotor,
        final_status=final_status,
        errors=tuple(errors),
    )


def _quintic_smoothstep(fraction: float) -> tuple[float, float]:
    """Return position fraction and derivative for a unit-duration move."""
    x = min(max(float(fraction), 0.0), 1.0)
    return 6.0 * x**5 - 15.0 * x**4 + 10.0 * x**3, 30.0 * x**4 - 60.0 * x**3 + 30.0 * x**2


def run_head_yaw_low_gain_motion(
    writer: Any,
    endpoint: DamiaoFeedbackEndpoint,
    *,
    soft_position_rad: tuple[float, float],
    excursion_rad: float = 0.02,
    transition_s: float = 1.0,
    dwell_s: float = 0.5,
    rate_hz: float = 50.0,
    kp: float = 1.0,
    kd: float = 0.2,
    maximum_position_error_rad: float = 0.05,
    maximum_velocity_rad_s: float = 0.2,
    maximum_torque_nm: float = 0.1,
    mos_temperature_limit_c: int = 100,
    rotor_temperature_limit_c: int = 80,
    feedback_timeout_s: float = 0.1,
    monotonic: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
) -> SingleJointMotionReport:
    """Run the frozen unloaded head-yaw +/-0.02 rad trajectory, then disable."""
    if (
        endpoint.motor_name != "head_yaw_motor"
        or endpoint.interface != "kcan3"
        or endpoint.can_id != 8
    ):
        raise ValueError("powered motion is restricted to kcan3 ID 0x08 head_yaw_motor")
    if (excursion_rad, transition_s, dwell_s, rate_hz, kp, kd) != (
        0.02, 1.0, 0.5, 50.0, 1.0, 0.2
    ):
        raise ValueError("head-yaw powered-motion trajectory and gains are frozen")
    if maximum_torque_nm > 0.1 or maximum_velocity_rad_s > 0.2:
        raise ValueError("head-yaw powered-motion safety envelope cannot be widened")
    soft_low, soft_high = map(float, soft_position_rad)
    if soft_low >= soft_high:
        raise ValueError("head-yaw soft position range is invalid")

    duration_s = 3.0 * transition_s + 3.0 * dwell_s
    errors: list[str] = []
    initial_position: float | None = None
    requested_minimum: float | None = None
    requested_maximum: float | None = None
    feedback_count = 0
    command_count = 0
    max_error = 0.0
    max_velocity = 0.0
    max_torque = 0.0
    max_mos = 0
    max_rotor = 0
    final_status: str | None = None
    period = 1.0 / rate_hz

    def target_at(elapsed: float) -> tuple[float, float]:
        assert initial_position is not None
        phases = (
            (transition_s, initial_position, initial_position + excursion_rad),
            (dwell_s, initial_position + excursion_rad, initial_position + excursion_rad),
            (transition_s, initial_position + excursion_rad, initial_position - excursion_rad),
            (dwell_s, initial_position - excursion_rad, initial_position - excursion_rad),
            (transition_s, initial_position - excursion_rad, initial_position),
            (dwell_s, initial_position, initial_position),
        )
        remaining = min(max(elapsed, 0.0), duration_s)
        for phase_duration, start_position, end_position in phases:
            if remaining <= phase_duration:
                if start_position == end_position:
                    return start_position, 0.0
                fraction, derivative = _quintic_smoothstep(remaining / phase_duration)
                delta = end_position - start_position
                return start_position + delta * fraction, delta * derivative / phase_duration
            remaining -= phase_duration
        return initial_position, 0.0

    def accept(feedback: DamiaoFeedback, target_position: float) -> None:
        nonlocal feedback_count, max_error, max_velocity, max_torque, max_mos, max_rotor
        feedback_count += 1
        if feedback.status_name != "enabled":
            raise RuntimeError(f"head-yaw status must be enabled, observed {feedback.status_name}")
        error = target_position - feedback.position_rad
        max_error = max(max_error, abs(error))
        max_velocity = max(max_velocity, abs(feedback.velocity_rad_s))
        max_torque = max(max_torque, abs(feedback.estimated_output_torque_nm))
        max_mos = max(max_mos, feedback.mos_temperature_c)
        max_rotor = max(max_rotor, feedback.rotor_temperature_c)
        if abs(error) > maximum_position_error_rad:
            raise RuntimeError("head-yaw position-error guard tripped")
        if abs(feedback.velocity_rad_s) > maximum_velocity_rad_s:
            raise RuntimeError("head-yaw velocity guard tripped")
        if abs(feedback.estimated_output_torque_nm) > maximum_torque_nm:
            raise RuntimeError("head-yaw torque guard tripped")
        if feedback.mos_temperature_c >= mos_temperature_limit_c:
            raise RuntimeError("head-yaw MOS-temperature guard tripped")
        if feedback.rotor_temperature_c >= rotor_temperature_limit_c:
            raise RuntimeError("head-yaw rotor-temperature guard tripped")

    try:
        writer.send_command(encode_zero_gain_position_echo(endpoint, 0.0))
        prepared = _receive_selected(
            writer, endpoint, feedback_timeout_s, monotonic=monotonic, sleep=sleep
        )
        if prepared.status_name != "disabled":
            raise RuntimeError("head-yaw must be disabled before powered motion")
        initial_position = prepared.position_rad
        requested_minimum = initial_position - excursion_rad
        requested_maximum = initial_position + excursion_rad
        if not soft_low + maximum_position_error_rad <= requested_minimum:
            raise RuntimeError("head-yaw initial position lacks lower soft-limit margin")
        if not requested_maximum <= soft_high - maximum_position_error_rad:
            raise RuntimeError("head-yaw initial position lacks upper soft-limit margin")
        writer.send_command(encode_zero_gain_position_echo(endpoint, initial_position))
        prepared = _receive_selected(
            writer, endpoint, feedback_timeout_s, monotonic=monotonic, sleep=sleep
        )
        if prepared.status_name != "disabled":
            raise RuntimeError("head-yaw changed status during zero-gain preparation")

        envelope = DamiaoMitCommandEnvelope(
            position_rad=(soft_low, soft_high),
            maximum_velocity_rad_s=maximum_velocity_rad_s,
            maximum_feedforward_torque_nm=maximum_torque_nm,
            maximum_output_torque_nm=maximum_torque_nm,
        )
        writer.send_enable()
        start = monotonic()
        next_send = start
        while monotonic() - start < duration_s:
            now = monotonic()
            if now < next_send:
                sleep(min(next_send - now, 0.001))
                continue
            target_position, target_velocity = target_at(now - start)
            measured = DamiaoMitState(prepared.position_rad, prepared.velocity_rad_s)
            encoded = encode_damiao_mit_command(
                endpoint,
                DamiaoMitCommand(target_position, target_velocity, kp, kd, 0.0),
                measured,
                envelope,
            )
            writer.send_command(encoded)
            command_count += 1
            prepared = _receive_selected(
                writer, endpoint, feedback_timeout_s, monotonic=monotonic, sleep=sleep
            )
            accept(prepared, target_position)
            next_send += period
    except BaseException as exc:
        errors.append(str(exc))
    finally:
        poll_position = initial_position if initial_position is not None else 0.0
        shutdown_deadline = monotonic() + 0.5
        shutdown_error: str | None = None
        shutdown_attempts = 0
        while shutdown_attempts < 3 or (
            final_status != "disabled" and monotonic() < shutdown_deadline
        ):
            shutdown_attempts += 1
            try:
                writer.send_disable()
                sleep(0.01)
                writer.send_command(encode_zero_gain_position_echo(endpoint, poll_position))
                final = _receive_selected(
                    writer, endpoint, feedback_timeout_s, monotonic=monotonic, sleep=sleep
                )
                final_status = final.status_name
                if writer.disable_attempts >= 3 and final_status == "disabled":
                    break
            except BaseException as exc:
                shutdown_error = str(exc)
        if final_status != "disabled":
            detail = f": {shutdown_error}" if shutdown_error else ""
            errors.append(f"final disabled verification timed out at status {final_status}{detail}")

    return SingleJointMotionReport(
        motor_name=endpoint.motor_name,
        initial_position_rad=initial_position,
        requested_minimum_position_rad=requested_minimum,
        requested_maximum_position_rad=requested_maximum,
        duration_s=duration_s,
        rate_hz=rate_hz,
        kp=kp,
        kd=kd,
        command_count=command_count,
        feedback_count=feedback_count,
        enable_attempts=writer.enable_attempts,
        disable_attempts=writer.disable_attempts,
        maximum_abs_position_error_rad=max_error,
        maximum_abs_velocity_rad_s=max_velocity,
        maximum_abs_estimated_torque_nm=max_torque,
        maximum_mos_temperature_c=max_mos,
        maximum_rotor_temperature_c=max_rotor,
        final_status=final_status,
        errors=tuple(errors),
    )
