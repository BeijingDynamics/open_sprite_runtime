"""Pure Damiao MIT frame codecs and receive-only feedback auditing."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math
import selectors
import time
from typing import Any, Callable, Iterable, Mapping

from .hardware import DAMIAO_PROJECT_MAX_EMBEDDED_KD, validate_hardware_inventory
from .socketcan import ReceivedCanFrame, SocketCanTimestampError
from .telemetry import MotorTelemetryLimits, limits_from_hardware_record


STATUS_NAMES = {
    0x0: "disabled",
    0x1: "enabled",
    0x8: "over_voltage",
    0x9: "under_voltage",
    0xA: "over_current",
    0xB: "mos_over_temperature",
    0xC: "motor_coil_over_temperature",
    0xD: "communication_lost",
    0xE: "overload",
}


def _validate_range(name: str, value: tuple[float, float]) -> tuple[float, float]:
    if len(value) != 2:
        raise ValueError(f"{name} must contain exactly two values")
    low, high = float(value[0]), float(value[1])
    if not math.isfinite(low) or not math.isfinite(high) or low >= high:
        raise ValueError(f"{name} must have finite lower < upper")
    return low, high


@dataclass(frozen=True)
class DamiaoMitRanges:
    """The PMAX/VMAX/TMAX mapping currently stored in one motor."""

    position_rad: tuple[float, float]
    velocity_rad_s: tuple[float, float]
    torque_nm: tuple[float, float]
    source: str

    def __post_init__(self) -> None:
        object.__setattr__(self, "position_rad", _validate_range("position_rad", self.position_rad))
        object.__setattr__(
            self, "velocity_rad_s", _validate_range("velocity_rad_s", self.velocity_rad_s)
        )
        object.__setattr__(self, "torque_nm", _validate_range("torque_nm", self.torque_nm))
        if self.source not in {
            "motor_register_readback",
            "operator_confirmed_drive_configuration",
        }:
            raise ValueError(
                "MIT ranges must use motor_register_readback or an explicitly "
                "operator_confirmed_drive_configuration source"
            )

    @property
    def register_readback_verified(self) -> bool:
        return self.source == "motor_register_readback"


@dataclass(frozen=True)
class DamiaoFeedbackEndpoint:
    motor_name: str
    interface: str
    can_id: int
    master_id: int
    ranges: DamiaoMitRanges

    def __post_init__(self) -> None:
        if not self.motor_name or not self.interface:
            raise ValueError("motor_name and interface are required")
        for name, value in (("can_id", self.can_id), ("master_id", self.master_id)):
            if not isinstance(value, int) or not 0 <= value <= 0x7FF:
                raise ValueError(f"{name} must be an 11-bit CAN identifier")
        # The official V1.4 frame stores the controller ID in D[0]'s low nibble.
        if self.can_id > 0xF:
            raise ValueError("can_id must fit the feedback frame's four-bit controller ID field")


@dataclass(frozen=True)
class DamiaoMitCommand:
    """One MIT impedance command before protocol quantization."""

    position_rad: float
    velocity_rad_s: float
    kp: float
    kd: float
    feedforward_torque_nm: float

    def __post_init__(self) -> None:
        for name, value in asdict(self).items():
            if not math.isfinite(value):
                raise ValueError(f"{name} must be finite")


@dataclass(frozen=True)
class DamiaoMitState:
    """Measured joint state used to gate the complete MIT torque request."""

    position_rad: float
    velocity_rad_s: float

    def __post_init__(self) -> None:
        for name, value in asdict(self).items():
            if not math.isfinite(value):
                raise ValueError(f"measured {name} must be finite")


@dataclass(frozen=True)
class DamiaoMitCommandEnvelope:
    """Project limits inside the motor's register-readback MIT ranges."""

    position_rad: tuple[float, float]
    maximum_velocity_rad_s: float
    maximum_feedforward_torque_nm: float
    maximum_output_torque_nm: float

    def __post_init__(self) -> None:
        object.__setattr__(
            self, "position_rad", _validate_range("position_rad", self.position_rad)
        )
        for name in (
            "maximum_velocity_rad_s",
            "maximum_feedforward_torque_nm",
            "maximum_output_torque_nm",
        ):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")


@dataclass(frozen=True)
class EncodedDamiaoMitCommand:
    """A standard-CAN MIT payload; this object has no transport capability."""

    motor_name: str
    interface: str
    can_id: int
    data: bytes


@dataclass(frozen=True)
class DamiaoMitCommandProfile:
    """Validated physical-motor endpoint and its dynamic command limits."""

    endpoint: DamiaoFeedbackEndpoint
    soft_position_rad: tuple[float, float]
    telemetry_limits: MotorTelemetryLimits
    maximum_embedded_kd: float = DAMIAO_PROJECT_MAX_EMBEDDED_KD

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "soft_position_rad",
            _validate_range("soft_position_rad", self.soft_position_rad),
        )
        self.telemetry_limits.validate()
        if (
            not math.isfinite(self.maximum_embedded_kd)
            or self.maximum_embedded_kd <= 0.0
            or self.maximum_embedded_kd > 5.0
        ):
            raise ValueError("maximum_embedded_kd must be finite and in (0, 5]")
        low, high = self.soft_position_rad
        protocol_low, protocol_high = self.endpoint.ranges.position_rad
        if low < protocol_low or high > protocol_high:
            raise ValueError("motor soft limits exceed the register-readback MIT position range")

    def envelope_for_state(
        self, measured_state: DamiaoMitState
    ) -> DamiaoMitCommandEnvelope:
        """Build a torque-speed-aware envelope for one fresh physical-motor state."""
        protocol_velocity = min(abs(value) for value in self.endpoint.ranges.velocity_rad_s)
        maximum_velocity = min(
            self.telemetry_limits.maximum_speed_rad_s,
            protocol_velocity,
        )
        protocol_torque = min(abs(value) for value in self.endpoint.ranges.torque_nm)
        maximum_torque = min(
            self.telemetry_limits.torque_limit_at_speed(measured_state.velocity_rad_s),
            protocol_torque,
        )
        if maximum_velocity <= 0.0 or maximum_torque <= 0.0:
            raise ValueError("motor state leaves no positive MIT command envelope")
        return DamiaoMitCommandEnvelope(
            position_rad=self.soft_position_rad,
            maximum_velocity_rad_s=maximum_velocity,
            maximum_feedforward_torque_nm=maximum_torque,
            maximum_output_torque_nm=maximum_torque,
        )

    def encode(
        self,
        command: DamiaoMitCommand,
        measured_state: DamiaoMitState,
    ) -> EncodedDamiaoMitCommand:
        if command.kd > self.maximum_embedded_kd:
            raise ValueError(
                f"kd={command.kd} exceeds the qualified Damiao embedded Kd limit "
                f"{self.maximum_embedded_kd} for {self.endpoint.motor_name}"
            )
        return encode_damiao_mit_command(
            self.endpoint,
            command,
            measured_state,
            self.envelope_for_state(measured_state),
        )


def _float_to_uint_strict(
    name: str,
    value: float,
    limits: tuple[float, float],
    bits: int,
) -> int:
    low, high = limits
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite")
    if not low <= value <= high:
        raise ValueError(f"{name}={value} is outside [{low}, {high}]")
    # Match the official SDK's truncating conversion for in-range values. Unlike
    # the SDK, reject out-of-range values instead of clipping or wrapping them.
    return int((value - low) / (high - low) * ((1 << bits) - 1))


def encode_damiao_mit_command(
    endpoint: DamiaoFeedbackEndpoint,
    command: DamiaoMitCommand,
    measured_state: DamiaoMitState,
    envelope: DamiaoMitCommandEnvelope,
) -> EncodedDamiaoMitCommand:
    """Encode the official eight-byte MIT command without opening or writing CAN."""
    protocol = endpoint.ranges
    if (
        envelope.position_rad[0] < protocol.position_rad[0]
        or envelope.position_rad[1] > protocol.position_rad[1]
    ):
        raise ValueError("position envelope exceeds the motor register-readback MIT range")
    protocol_velocity = max(abs(value) for value in protocol.velocity_rad_s)
    if envelope.maximum_velocity_rad_s > protocol_velocity:
        raise ValueError("velocity envelope exceeds the motor register-readback MIT range")
    protocol_torque = max(abs(value) for value in protocol.torque_nm)
    if envelope.maximum_feedforward_torque_nm > protocol_torque:
        raise ValueError("torque envelope exceeds the motor register-readback MIT range")
    if envelope.maximum_output_torque_nm > protocol_torque:
        raise ValueError("output torque envelope exceeds the motor register-readback MIT range")

    _float_to_uint_strict("position_rad", command.position_rad, envelope.position_rad, 16)
    if abs(command.velocity_rad_s) > envelope.maximum_velocity_rad_s:
        raise ValueError("velocity_rad_s exceeds the project command envelope")
    if abs(command.feedforward_torque_nm) > envelope.maximum_feedforward_torque_nm:
        raise ValueError("feedforward_torque_nm exceeds the project command envelope")
    estimated_output_torque_nm = (
        command.kp * (command.position_rad - measured_state.position_rad)
        + command.kd * (command.velocity_rad_s - measured_state.velocity_rad_s)
        + command.feedforward_torque_nm
    )
    if not math.isfinite(estimated_output_torque_nm):
        raise ValueError("estimated MIT output torque must be finite")
    if abs(estimated_output_torque_nm) > envelope.maximum_output_torque_nm:
        raise ValueError("estimated MIT output torque exceeds the project command envelope")

    position = _float_to_uint_strict("position_rad", command.position_rad, protocol.position_rad, 16)
    velocity = _float_to_uint_strict(
        "velocity_rad_s", command.velocity_rad_s, protocol.velocity_rad_s, 12
    )
    kp = _float_to_uint_strict("kp", command.kp, (0.0, 500.0), 12)
    kd = _float_to_uint_strict("kd", command.kd, (0.0, 5.0), 12)
    torque = _float_to_uint_strict(
        "feedforward_torque_nm", command.feedforward_torque_nm, protocol.torque_nm, 12
    )
    data = bytes(
        (
            (position >> 8) & 0xFF,
            position & 0xFF,
            (velocity >> 4) & 0xFF,
            ((velocity & 0xF) << 4) | ((kp >> 8) & 0xF),
            kp & 0xFF,
            (kd >> 4) & 0xFF,
            ((kd & 0xF) << 4) | ((torque >> 8) & 0xF),
            torque & 0xFF,
        )
    )
    # Official SDK commit 0b2ede4 defines MIT_MODE as 0x000.
    return EncodedDamiaoMitCommand(
        motor_name=endpoint.motor_name,
        interface=endpoint.interface,
        can_id=endpoint.can_id,
        data=data,
    )


def encode_zero_gain_position_echo(
    endpoint: DamiaoFeedbackEndpoint,
    position_rad: float,
) -> EncodedDamiaoMitCommand:
    """Encode position plus v/Kp/Kd/tau=0; nominal MIT output torque is zero."""
    protocol = endpoint.ranges
    maximum_velocity = min(abs(value) for value in protocol.velocity_rad_s)
    maximum_torque = min(abs(value) for value in protocol.torque_nm)
    state = DamiaoMitState(position_rad=position_rad, velocity_rad_s=0.0)
    return encode_damiao_mit_command(
        endpoint,
        DamiaoMitCommand(
            position_rad=position_rad,
            velocity_rad_s=0.0,
            kp=0.0,
            kd=0.0,
            feedforward_torque_nm=0.0,
        ),
        state,
        DamiaoMitCommandEnvelope(
            position_rad=protocol.position_rad,
            maximum_velocity_rad_s=maximum_velocity,
            maximum_feedforward_torque_nm=maximum_torque,
            maximum_output_torque_nm=maximum_torque,
        ),
    )


def command_profiles_from_hardware_config(
    hardware: Mapping[str, Any],
    policy_joint_names: Iterable[str],
    *,
    expected_motor_count: int = 31,
) -> tuple[DamiaoMitCommandProfile, ...]:
    """Build exact physical-motor profiles only from a complete armable inventory."""
    hardware_dict = dict(hardware)
    policy_order = tuple(policy_joint_names)
    if hardware_dict.get("configured") is not True:
        raise ValueError("hardware configured=true is required for command profiles")
    report = validate_hardware_inventory(hardware_dict, policy_order)
    if not report.valid:
        raise ValueError("hardware inventory is invalid: " + "; ".join(report.errors))
    endpoints = endpoints_from_hardware_config(
        hardware_dict,
        expected_motor_count=expected_motor_count,
    )
    motor_map = hardware_dict["motor_map"]
    maximum_embedded_kd = float(
        hardware_dict["controller"]["damiao_embedded_kd_max"]
    )
    profiles = []
    for endpoint in endpoints:
        record = motor_map[endpoint.motor_name]
        profile = DamiaoMitCommandProfile(
            endpoint=endpoint,
            soft_position_rad=tuple(float(value) for value in record["soft_limit_rad"]),
            telemetry_limits=limits_from_hardware_record(record),
            maximum_embedded_kd=maximum_embedded_kd,
        )
        # Exercise the dynamic limits at rest so incompatible measured ranges fail now.
        profile.envelope_for_state(DamiaoMitState(0.0, 0.0))
        profiles.append(profile)
    if len(profiles) != expected_motor_count:
        raise ValueError(
            f"built {len(profiles)} command profiles; expected {expected_motor_count}"
        )
    return tuple(profiles)


def encode_damiao_mit_command_bank(
    profiles: Iterable[DamiaoMitCommandProfile],
    commands: Mapping[str, DamiaoMitCommand],
    measured_states: Mapping[str, DamiaoMitState],
) -> tuple[EncodedDamiaoMitCommand, ...]:
    """Encode an exact all-or-nothing physical-motor bank without transmission."""
    profile_list = tuple(profiles)
    profile_map = {profile.endpoint.motor_name: profile for profile in profile_list}
    if len(profile_map) != len(profile_list):
        raise ValueError("duplicate physical motor profile")
    expected = set(profile_map)
    if set(commands) != expected:
        raise ValueError("MIT commands must exactly cover the physical motor profiles")
    if set(measured_states) != expected:
        raise ValueError("measured states must exactly cover the physical motor profiles")
    return tuple(
        profile.encode(commands[name], measured_states[name])
        for name, profile in profile_map.items()
    )


@dataclass(frozen=True)
class DamiaoFeedback:
    motor_name: str
    interface: str
    can_id: int
    master_id: int
    status_code: int
    status_name: str
    position_rad: float
    velocity_rad_s: float
    estimated_output_torque_nm: float
    mos_temperature_c: int
    rotor_temperature_c: int
    software_timestamp_ns: int
    hardware_timestamp_ns: int
    userspace_receive_timestamp_ns: int

    @property
    def userspace_queue_age_ns(self) -> int:
        return self.userspace_receive_timestamp_ns - self.software_timestamp_ns

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def _uint_to_float(value: int, limits: tuple[float, float], bits: int) -> float:
    low, high = limits
    return float(value) * (high - low) / float((1 << bits) - 1) + low


def decode_damiao_feedback(
    frame: ReceivedCanFrame, endpoint: DamiaoFeedbackEndpoint
) -> DamiaoFeedback:
    """Decode one official eight-byte feedback frame without any hardware I/O."""
    if frame.interface != endpoint.interface:
        raise ValueError("frame interface does not match the configured motor endpoint")
    if frame.can_id != endpoint.master_id:
        raise ValueError("frame CAN ID does not match the configured Damiao Master ID")
    if frame.is_extended or frame.is_remote or frame.is_error:
        raise ValueError("Damiao feedback must be a standard CAN data frame")
    if len(frame.data) != 8:
        raise ValueError("Damiao feedback must contain exactly eight data bytes")

    data = frame.data
    controller_id = data[0] & 0x0F
    if controller_id != endpoint.can_id:
        raise ValueError("feedback payload controller ID does not match configured CAN ID")
    status = (data[0] >> 4) & 0x0F
    position_raw = (data[1] << 8) | data[2]
    velocity_raw = (data[3] << 4) | (data[4] >> 4)
    torque_raw = ((data[4] & 0x0F) << 8) | data[5]
    return DamiaoFeedback(
        motor_name=endpoint.motor_name,
        interface=frame.interface,
        can_id=endpoint.can_id,
        master_id=frame.can_id,
        status_code=status,
        status_name=STATUS_NAMES.get(status, "reserved_or_unknown"),
        position_rad=_uint_to_float(position_raw, endpoint.ranges.position_rad, 16),
        velocity_rad_s=_uint_to_float(velocity_raw, endpoint.ranges.velocity_rad_s, 12),
        estimated_output_torque_nm=_uint_to_float(
            torque_raw, endpoint.ranges.torque_nm, 12
        ),
        mos_temperature_c=data[6],
        rotor_temperature_c=data[7],
        software_timestamp_ns=frame.software_timestamp_ns,
        hardware_timestamp_ns=frame.hardware_timestamp_ns,
        userspace_receive_timestamp_ns=frame.userspace_receive_timestamp_ns,
    )


class DamiaoFeedbackDecoder:
    """Map feedback by (interface, Master ID); CAN enumeration order is never used."""

    def __init__(self, endpoints: Iterable[DamiaoFeedbackEndpoint]):
        self._endpoints: dict[tuple[str, int], DamiaoFeedbackEndpoint] = {}
        for endpoint in endpoints:
            key = (endpoint.interface, endpoint.master_id)
            if key in self._endpoints:
                raise ValueError(f"duplicate Damiao feedback endpoint: {key[0]}:{key[1]}")
            self._endpoints[key] = endpoint
        if not self._endpoints:
            raise ValueError("at least one Damiao feedback endpoint is required")

    def decode(self, frame: ReceivedCanFrame) -> DamiaoFeedback:
        endpoint = self._endpoints.get((frame.interface, frame.can_id))
        if endpoint is None:
            raise ValueError(
                f"unconfigured Damiao feedback endpoint: {frame.interface}:{frame.can_id}"
            )
        return decode_damiao_feedback(frame, endpoint)


@dataclass(frozen=True)
class DamiaoZeroGainProbeReport:
    motor_name: str
    interface: str
    can_id: int
    master_id: int
    requested_duration_s: float
    requested_rate_hz: float
    elapsed_s: float
    tx_count: int
    rx_count: int
    first_position_rad: float | None
    last_position_rad: float | None
    minimum_position_rad: float | None
    maximum_position_rad: float | None
    status_codes: tuple[int, ...]
    discarded_timestamp_frames: int
    errors: tuple[str, ...]

    @property
    def passed(self) -> bool:
        return not self.errors and self.tx_count > 0 and self.rx_count > 0

    def to_dict(self) -> dict[str, Any]:
        return {**asdict(self), "passed": self.passed}


@dataclass(frozen=True)
class DamiaoZeroGainGroupProbeReport:
    interface: str
    motor_names: tuple[str, ...]
    requested_duration_s: float
    requested_rate_hz_per_motor: float
    elapsed_s: float
    tx_count: int
    minimum_sample_coverage: float
    rx_count_by_motor: dict[str, int]
    sample_coverage_by_motor: dict[str, float]
    first_position_rad_by_motor: dict[str, float | None]
    last_position_rad_by_motor: dict[str, float | None]
    status_codes_by_motor: dict[str, tuple[int, ...]]
    discarded_timestamp_frames: int
    errors: tuple[str, ...]

    @property
    def passed(self) -> bool:
        return (
            not self.errors
            and self.tx_count > 0
            and all(
                self.sample_coverage_by_motor.get(name, 0.0)
                >= self.minimum_sample_coverage
                for name in self.motor_names
            )
        )

    def to_dict(self) -> dict[str, Any]:
        return {**asdict(self), "passed": self.passed}


def collect_zero_gain_position_echo(
    poller: Any,
    endpoint: DamiaoFeedbackEndpoint,
    configured_endpoints: Iterable[DamiaoFeedbackEndpoint],
    duration_s: float,
    rate_hz: float,
    *,
    feedback_timeout_s: float = 0.2,
    selector_factory: Callable[[], Any] = selectors.DefaultSelector,
    monotonic: Callable[[], float] = time.monotonic,
    on_feedback: Callable[[DamiaoFeedback], None] | None = None,
    keep_running: Callable[[], bool] | None = None,
    maximum_duration_s: float = 10.0,
) -> DamiaoZeroGainProbeReport:
    """Run a finite single-motor position-echo probe through a restricted poller."""
    if not math.isfinite(maximum_duration_s) or not 0.0 < maximum_duration_s <= 120.0:
        raise ValueError("maximum_duration_s must be finite and in (0, 120]")
    if not math.isfinite(duration_s) or not 0.0 < duration_s <= maximum_duration_s:
        raise ValueError(
            f"duration_s must be finite and in (0, {maximum_duration_s:g}]"
        )
    if not math.isfinite(rate_hz) or not 0.0 < rate_hz <= 100.0:
        raise ValueError("rate_hz must be finite and in (0, 100]")
    if not math.isfinite(feedback_timeout_s) or not 0.0 < feedback_timeout_s <= 1.0:
        raise ValueError("feedback_timeout_s must be finite and in (0, 1]")
    if poller.interface != endpoint.interface:
        raise ValueError("poller interface does not match selected endpoint")
    if not hasattr(poller, "send_zero_gain_poll"):
        raise ValueError("restricted zero-gain poller is required")

    endpoint_list = tuple(configured_endpoints)
    decoder = DamiaoFeedbackDecoder(endpoint_list)
    known_command_keys = {(item.interface, item.can_id) for item in endpoint_list}
    known_feedback_keys = {(item.interface, item.master_id) for item in endpoint_list}
    period_s = 1.0 / rate_hz
    start = monotonic()
    next_send = start
    last_selected_feedback = start
    target_position = 0.0
    feedback_values: list[DamiaoFeedback] = []
    discarded_timestamp_frames = 0
    errors: list[str] = []

    with selector_factory() as selector:
        selector.register(poller, selectors.EVENT_READ, poller)
        while True:
            now = monotonic()
            if keep_running is not None and not keep_running():
                break
            if now - start >= duration_s:
                break
            if now >= next_send:
                command = encode_zero_gain_position_echo(endpoint, target_position)
                poller.send_zero_gain_poll(command.can_id, command.data)
                next_send += period_s
                if next_send <= now:
                    next_send = now + period_s
            timeout = min(max(0.0, next_send - monotonic()), 0.02)
            for key, _mask in selector.select(timeout=timeout):
                try:
                    frame = key.data.receive()
                except SocketCanTimestampError:
                    discarded_timestamp_frames += 1
                    continue
                frame_key = (frame.interface, frame.can_id)
                if frame_key in known_command_keys:
                    continue
                if frame_key not in known_feedback_keys:
                    errors.append(f"unexpected CAN endpoint: {frame.interface}:{frame.can_id:#x}")
                    continue
                feedback = decoder.decode(frame)
                if feedback.motor_name != endpoint.motor_name:
                    continue
                if not frame.is_fd or not frame.bit_rate_switch:
                    errors.append("selected feedback was not CAN-FD with bit-rate switching")
                    break
                feedback_values.append(feedback)
                target_position = feedback.position_rad
                last_selected_feedback = monotonic()
                if feedback.status_code not in (0x0, 0x1):
                    errors.append(f"motor fault: {feedback.status_name}")
                    break
                if on_feedback is not None:
                    on_feedback(feedback)
            if errors:
                break
            if monotonic() - last_selected_feedback > feedback_timeout_s:
                errors.append(
                    f"no selected-motor feedback within {feedback_timeout_s:.3f} s"
                )
                break

    elapsed = monotonic() - start
    positions = [item.position_rad for item in feedback_values]
    return DamiaoZeroGainProbeReport(
        motor_name=endpoint.motor_name,
        interface=endpoint.interface,
        can_id=endpoint.can_id,
        master_id=endpoint.master_id,
        requested_duration_s=duration_s,
        requested_rate_hz=rate_hz,
        elapsed_s=elapsed,
        tx_count=int(poller.hardware_tx_attempts),
        rx_count=len(feedback_values),
        first_position_rad=positions[0] if positions else None,
        last_position_rad=positions[-1] if positions else None,
        minimum_position_rad=min(positions) if positions else None,
        maximum_position_rad=max(positions) if positions else None,
        status_codes=tuple(sorted({item.status_code for item in feedback_values})),
        discarded_timestamp_frames=discarded_timestamp_frames,
        errors=tuple(errors),
    )


def collect_zero_gain_group_position_echo(
    poller: Any,
    endpoints: Iterable[DamiaoFeedbackEndpoint],
    configured_endpoints: Iterable[DamiaoFeedbackEndpoint],
    duration_s: float,
    rate_hz_per_motor: float,
    *,
    feedback_timeout_s: float = 0.2,
    selector_factory: Callable[[], Any] = selectors.DefaultSelector,
    monotonic: Callable[[], float] = time.monotonic,
    on_feedback: Callable[[DamiaoFeedback], None] | None = None,
    keep_running: Callable[[], bool] | None = None,
    maximum_duration_s: float = 10.0,
    minimum_sample_coverage: float = 0.95,
) -> DamiaoZeroGainGroupProbeReport:
    """Poll up to eight motors on one CAN-FD bus using only zero-gain MIT frames."""
    selected = tuple(endpoints)
    if not 1 <= len(selected) <= 8:
        raise ValueError("zero-gain group must contain between one and eight motors")
    if len({item.motor_name for item in selected}) != len(selected):
        raise ValueError("zero-gain group contains duplicate motor names")
    if len({item.can_id for item in selected}) != len(selected):
        raise ValueError("zero-gain group contains duplicate command CAN IDs")
    interfaces = {item.interface for item in selected}
    if interfaces != {poller.interface}:
        raise ValueError("all selected motors must use the poller interface")
    if not hasattr(poller, "send_zero_gain_poll"):
        raise ValueError("restricted zero-gain poller is required")
    if not math.isfinite(maximum_duration_s) or not 0.0 < maximum_duration_s <= 120.0:
        raise ValueError("maximum_duration_s must be finite and in (0, 120]")
    if not math.isfinite(duration_s) or not 0.0 < duration_s <= maximum_duration_s:
        raise ValueError(
            f"duration_s must be finite and in (0, {maximum_duration_s:g}]"
        )
    if not math.isfinite(rate_hz_per_motor) or not 0.0 < rate_hz_per_motor <= 500.0:
        raise ValueError("rate_hz_per_motor must be finite and in (0, 500]")
    if not math.isfinite(feedback_timeout_s) or not 0.0 < feedback_timeout_s <= 1.0:
        raise ValueError("feedback_timeout_s must be finite and in (0, 1]")
    if (
        not math.isfinite(minimum_sample_coverage)
        or not 0.0 < minimum_sample_coverage <= 1.0
    ):
        raise ValueError("minimum_sample_coverage must be finite and in (0, 1]")

    configured = tuple(configured_endpoints)
    decoder = DamiaoFeedbackDecoder(configured)
    known_command_keys = {(item.interface, item.can_id) for item in configured}
    known_feedback_keys = {(item.interface, item.master_id) for item in configured}
    selected_names = {item.motor_name for item in selected}
    # Spread one cycle across the bus instead of bursting every motor at the
    # same instant. At 8 x 500 Hz this emits one frame every 250 us and avoids
    # overflowing the small SocketCAN TX queue when the viewer briefly stalls.
    send_slot_s = 1.0 / (rate_hz_per_motor * len(selected))
    start = monotonic()
    next_send = start
    next_motor_index = 0
    last_feedback = {item.motor_name: start for item in selected}
    targets = {item.motor_name: 0.0 for item in selected}
    feedback_values: dict[str, list[DamiaoFeedback]] = {
        item.motor_name: [] for item in selected
    }
    discarded_timestamp_frames = 0
    errors: list[str] = []

    with selector_factory() as selector:
        selector.register(poller, selectors.EVENT_READ, poller)
        while True:
            now = monotonic()
            if keep_running is not None and not keep_running():
                break
            if now - start >= duration_s:
                break
            if now >= next_send:
                endpoint = selected[next_motor_index]
                command = encode_zero_gain_position_echo(
                    endpoint, targets[endpoint.motor_name]
                )
                try:
                    poller.send_zero_gain_poll(command.can_id, command.data)
                except OSError as exc:
                    errors.append(
                        f"SocketCAN transmit failed for {endpoint.motor_name}: {exc}"
                    )
                    break
                next_motor_index = (next_motor_index + 1) % len(selected)
                next_send += send_slot_s
                if next_send <= now:
                    # Never replay a backlog into a real-time actuator bus.
                    next_send = now + send_slot_s
            if errors:
                break
            timeout = min(max(0.0, next_send - monotonic()), 0.02)
            for key, _mask in selector.select(timeout=timeout):
                try:
                    frame = key.data.receive()
                except SocketCanTimestampError:
                    discarded_timestamp_frames += 1
                    continue
                frame_key = (frame.interface, frame.can_id)
                if frame_key in known_command_keys:
                    continue
                if frame_key not in known_feedback_keys:
                    errors.append(
                        f"unexpected CAN endpoint: {frame.interface}:{frame.can_id:#x}"
                    )
                    continue
                feedback = decoder.decode(frame)
                if feedback.motor_name not in selected_names:
                    continue
                if not frame.is_fd or not frame.bit_rate_switch:
                    errors.append(
                        f"{feedback.motor_name} feedback was not CAN-FD with bit-rate switching"
                    )
                    break
                feedback_values[feedback.motor_name].append(feedback)
                targets[feedback.motor_name] = feedback.position_rad
                last_feedback[feedback.motor_name] = monotonic()
                if feedback.status_code not in (0x0, 0x1):
                    errors.append(
                        f"{feedback.motor_name} motor fault: {feedback.status_name}"
                    )
                    break
                if on_feedback is not None:
                    on_feedback(feedback)
            if errors:
                break
            now = monotonic()
            stale = [
                name
                for name, last_seen in last_feedback.items()
                if now - last_seen > feedback_timeout_s
            ]
            if stale:
                errors.append(
                    "no feedback within "
                    f"{feedback_timeout_s:.3f} s from: {', '.join(sorted(stale))}"
                )
                break

    elapsed = monotonic() - start
    expected_samples_per_motor = max(1, int(elapsed * rate_hz_per_motor))
    rx_counts = {name: len(values) for name, values in feedback_values.items()}
    coverage = {
        name: min(1.0, count / expected_samples_per_motor)
        for name, count in rx_counts.items()
    }
    return DamiaoZeroGainGroupProbeReport(
        interface=poller.interface,
        motor_names=tuple(item.motor_name for item in selected),
        requested_duration_s=duration_s,
        requested_rate_hz_per_motor=rate_hz_per_motor,
        elapsed_s=elapsed,
        tx_count=int(poller.hardware_tx_attempts),
        minimum_sample_coverage=minimum_sample_coverage,
        rx_count_by_motor=rx_counts,
        sample_coverage_by_motor=coverage,
        first_position_rad_by_motor={
            name: values[0].position_rad if values else None
            for name, values in feedback_values.items()
        },
        last_position_rad_by_motor={
            name: values[-1].position_rad if values else None
            for name, values in feedback_values.items()
        },
        status_codes_by_motor={
            name: tuple(sorted({item.status_code for item in values}))
            for name, values in feedback_values.items()
        },
        discarded_timestamp_frames=discarded_timestamp_frames,
        errors=tuple(errors),
    )


def endpoints_from_hardware_config(
    hardware: Mapping[str, Any],
    *,
    expected_motor_count: int = 31,
) -> tuple[DamiaoFeedbackEndpoint, ...]:
    """Build explicit decoder endpoints from a measured hardware contract."""
    adapter = hardware.get("can_adapter")
    if not isinstance(adapter, Mapping):
        raise ValueError("can_adapter configuration is missing")
    interfaces = adapter.get("interfaces")
    if not isinstance(interfaces, list) or len(interfaces) != 4:
        raise ValueError("can_adapter.interfaces must contain four entries")
    if not all(isinstance(name, str) and name for name in interfaces):
        raise ValueError("all SocketCAN interface names must be measured")

    motor_map = hardware.get("motor_map")
    if not isinstance(motor_map, Mapping) or not motor_map:
        raise ValueError("motor_map is missing or empty")
    if len(motor_map) != expected_motor_count:
        raise ValueError(
            f"motor_map has {len(motor_map)} motors; expected {expected_motor_count}"
        )
    endpoints: list[DamiaoFeedbackEndpoint] = []
    for motor_name, raw in motor_map.items():
        if not isinstance(raw, Mapping):
            raise ValueError(f"motor_map.{motor_name} must be an object")
        try:
            channel = int(raw["can_channel"])
            can_id = raw["can_id"]
            master_id = raw["master_id"]
            mit = raw["mit_ranges"]
            ranges = DamiaoMitRanges(
                position_rad=tuple(mit["position_rad"]),
                velocity_rad_s=tuple(mit["velocity_rad_s"]),
                torque_nm=tuple(mit["torque_nm"]),
                source=mit["source"],
            )
            interface = interfaces[channel]
        except (KeyError, IndexError, TypeError, ValueError) as exc:
            raise ValueError(f"motor_map.{motor_name} feedback mapping is incomplete") from exc
        endpoints.append(
            DamiaoFeedbackEndpoint(
                motor_name=str(motor_name),
                interface=interface,
                can_id=can_id,
                master_id=master_id,
                ranges=ranges,
            )
        )
    DamiaoFeedbackDecoder(endpoints)
    command_keys = [(item.interface, item.can_id) for item in endpoints]
    feedback_keys = [(item.interface, item.master_id) for item in endpoints]
    if len(set(command_keys)) != len(command_keys):
        raise ValueError("duplicate Damiao command endpoint")
    overlap = sorted(set(command_keys) & set(feedback_keys))
    if overlap:
        raise ValueError(f"Damiao command and feedback endpoints overlap: {overlap}")
    return tuple(endpoints)


def _percentile_99(values: list[float]) -> float:
    if not values:
        return math.inf
    ordered = sorted(values)
    return ordered[max(0, math.ceil(0.99 * len(ordered)) - 1)]


@dataclass(frozen=True)
class DamiaoMotorRxStats:
    samples: int
    feedback_hz: float
    maximum_hardware_gap_ms: float
    userspace_queue_age_p99_ms: float
    maximum_mos_temperature_c: int | None
    maximum_rotor_temperature_c: int | None


@dataclass(frozen=True)
class DamiaoRxAuditReport:
    expected_motor_count: int
    seen_motor_count: int
    missing_motors: tuple[str, ...]
    motor_stats: dict[str, DamiaoMotorRxStats]
    errors: tuple[str, ...]
    observed_control_frame_count: int
    decoded_feedback_frame_count: int
    unexpected_frame_count: int
    hardware_tx_attempts: int = 0

    @property
    def passed(self) -> bool:
        return not self.errors

    def to_dict(self) -> dict[str, Any]:
        return {
            "expected_motor_count": self.expected_motor_count,
            "seen_motor_count": self.seen_motor_count,
            "missing_motors": self.missing_motors,
            "motor_stats": {
                name: asdict(stats) for name, stats in self.motor_stats.items()
            },
            "errors": self.errors,
            "observed_control_frame_count": self.observed_control_frame_count,
            "decoded_feedback_frame_count": self.decoded_feedback_frame_count,
            "unexpected_frame_count": self.unexpected_frame_count,
            "hardware_tx_attempts": self.hardware_tx_attempts,
            "passed": self.passed,
        }


class DamiaoRxAudit:
    """Aggregate a finite receive-only trace into fail-closed evidence."""

    def __init__(
        self,
        endpoints: Iterable[DamiaoFeedbackEndpoint],
        *,
        minimum_samples_per_motor: int,
        minimum_feedback_hz: float = 475.0,
        maximum_hardware_gap_ms: float = 6.0,
        maximum_userspace_queue_age_p99_ms: float = 6.0,
    ):
        endpoint_list = tuple(endpoints)
        self._decoder = DamiaoFeedbackDecoder(endpoint_list)
        self._expected = {endpoint.motor_name for endpoint in endpoint_list}
        self._feedback_keys = {
            (endpoint.interface, endpoint.master_id) for endpoint in endpoint_list
        }
        self._command_keys = {
            (endpoint.interface, endpoint.can_id) for endpoint in endpoint_list
        }
        if self._feedback_keys & self._command_keys:
            raise ValueError("Damiao command and feedback endpoints overlap")
        if minimum_samples_per_motor <= 0:
            raise ValueError("minimum_samples_per_motor must be positive")
        for name, value in (
            ("minimum_feedback_hz", minimum_feedback_hz),
            ("maximum_hardware_gap_ms", maximum_hardware_gap_ms),
            ("maximum_userspace_queue_age_p99_ms", maximum_userspace_queue_age_p99_ms),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
        self.minimum_samples_per_motor = minimum_samples_per_motor
        self.minimum_feedback_hz = minimum_feedback_hz
        self.maximum_hardware_gap_ms = maximum_hardware_gap_ms
        self.maximum_userspace_queue_age_p99_ms = maximum_userspace_queue_age_p99_ms
        self._samples: dict[str, list[DamiaoFeedback]] = {
            name: [] for name in self._expected
        }
        self._errors: list[str] = []
        self._unexpected_keys: set[tuple[str, int]] = set()
        self._observed_control_frames = 0
        self._decoded_feedback_frames = 0

    def ingest(self, frame: ReceivedCanFrame) -> DamiaoFeedback:
        feedback = self._decoder.decode(frame)
        samples = self._samples[feedback.motor_name]
        if samples and feedback.hardware_timestamp_ns <= samples[-1].hardware_timestamp_ns:
            self._errors.append(f"{feedback.motor_name}: raw hardware timestamp regressed")
        if feedback.status_code not in (0x0, 0x1):
            self._errors.append(
                f"{feedback.motor_name}: motor fault {feedback.status_name}"
            )
        samples.append(feedback)
        self._decoded_feedback_frames += 1
        return feedback

    def ingest_bus_frame(self, frame: ReceivedCanFrame) -> DamiaoFeedback | None:
        """Classify one bus frame without treating observed controller TX as ours."""
        key = (frame.interface, frame.can_id)
        if key in self._feedback_keys:
            return self.ingest(frame)
        if key in self._command_keys:
            self._observed_control_frames += 1
            return None
        self._unexpected_keys.add(key)
        return None

    def report(self) -> DamiaoRxAuditReport:
        errors = list(self._errors)
        for interface, can_id in sorted(self._unexpected_keys):
            errors.append(f"unexpected CAN endpoint: {interface}:{can_id:#x}")
        stats: dict[str, DamiaoMotorRxStats] = {}
        missing = tuple(sorted(name for name, values in self._samples.items() if not values))
        for name in sorted(self._samples):
            samples = self._samples[name]
            if not samples:
                continue
            if len(samples) < self.minimum_samples_per_motor:
                errors.append(
                    f"{name}: received {len(samples)} samples; "
                    f"minimum is {self.minimum_samples_per_motor}"
                )
            gaps_ns = [
                current.hardware_timestamp_ns - previous.hardware_timestamp_ns
                for previous, current in zip(samples, samples[1:])
                if current.hardware_timestamp_ns > previous.hardware_timestamp_ns
            ]
            elapsed_ns = samples[-1].hardware_timestamp_ns - samples[0].hardware_timestamp_ns
            feedback_hz = (
                (len(samples) - 1) * 1_000_000_000.0 / elapsed_ns
                if len(samples) > 1 and elapsed_ns > 0
                else 0.0
            )
            maximum_gap_ms = max(gaps_ns, default=0) / 1_000_000.0
            queue_ages_ms = [sample.userspace_queue_age_ns / 1_000_000.0 for sample in samples]
            queue_p99_ms = _percentile_99(queue_ages_ms)
            if feedback_hz < self.minimum_feedback_hz:
                errors.append(f"{name}: feedback rate {feedback_hz:.3f} Hz is too low")
            if maximum_gap_ms > self.maximum_hardware_gap_ms:
                errors.append(f"{name}: hardware timestamp gap {maximum_gap_ms:.3f} ms is too large")
            if queue_p99_ms > self.maximum_userspace_queue_age_p99_ms:
                errors.append(f"{name}: userspace queue age P99 {queue_p99_ms:.3f} ms is too large")
            stats[name] = DamiaoMotorRxStats(
                samples=len(samples),
                feedback_hz=feedback_hz,
                maximum_hardware_gap_ms=maximum_gap_ms,
                userspace_queue_age_p99_ms=queue_p99_ms,
                maximum_mos_temperature_c=max(sample.mos_temperature_c for sample in samples),
                maximum_rotor_temperature_c=max(
                    sample.rotor_temperature_c for sample in samples
                ),
            )
        if missing:
            errors.append("missing motor feedback: " + ", ".join(missing))
        return DamiaoRxAuditReport(
            expected_motor_count=len(self._expected),
            seen_motor_count=len(self._expected) - len(missing),
            missing_motors=missing,
            motor_stats=stats,
            errors=tuple(errors),
            observed_control_frame_count=self._observed_control_frames,
            decoded_feedback_frame_count=self._decoded_feedback_frames,
            unexpected_frame_count=len(self._unexpected_keys),
        )


def collect_receive_only_audit(
    receivers: Mapping[str, Any],
    audit: DamiaoRxAudit,
    duration_s: float,
    *,
    selector_factory: Callable[[], Any] = selectors.DefaultSelector,
    monotonic: Callable[[], float] = time.monotonic,
    on_feedback: Callable[[DamiaoFeedback], None] | None = None,
    keep_running: Callable[[], bool] | None = None,
) -> DamiaoRxAuditReport:
    """Collect from already-open receive-only sockets for a finite duration."""
    if not math.isfinite(duration_s) or duration_s <= 0.0:
        raise ValueError("duration_s must be finite and positive")
    if not receivers:
        raise ValueError("at least one receive-only socket is required")
    start = monotonic()
    with selector_factory() as selector:
        for interface, receiver in receivers.items():
            if receiver.interface != interface:
                raise ValueError("receiver mapping key does not match its interface")
            if hasattr(receiver, "send") or hasattr(receiver, "enable"):
                raise ValueError("collector accepts receive-only interfaces only")
            selector.register(receiver, selectors.EVENT_READ, receiver)
        while True:
            if keep_running is not None and not keep_running():
                break
            remaining = duration_s - (monotonic() - start)
            if remaining <= 0.0:
                break
            for key, _mask in selector.select(timeout=min(remaining, 0.1)):
                feedback = audit.ingest_bus_frame(key.data.receive())
                if feedback is not None and on_feedback is not None:
                    on_feedback(feedback)
    return audit.report()
