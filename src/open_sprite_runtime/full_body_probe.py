"""Finite four-bus motor and IMU commissioning probe with zero actuator gains."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math
import selectors
import time
from typing import Any, Callable, Iterable, Mapping

from .damiao import (
    DamiaoFeedback,
    DamiaoFeedbackDecoder,
    DamiaoFeedbackEndpoint,
    encode_zero_gain_position_echo,
)
from .socketcan import SocketCanTimestampError
from .yahboom_imu import (
    YahboomPacket,
    YahboomQuaternion,
    YahboomRawImu,
    YahboomStreamDecoder,
)


@dataclass(frozen=True)
class FullBodyShadowReport:
    requested_duration_s: float
    requested_rate_hz_per_motor: float
    elapsed_s: float
    motor_names: tuple[str, ...]
    tx_count_by_interface: dict[str, int]
    rx_count_by_motor: dict[str, int]
    sample_coverage_by_motor: dict[str, float]
    first_position_rad_by_motor: dict[str, float | None]
    last_position_rad_by_motor: dict[str, float | None]
    status_codes_by_motor: dict[str, tuple[int, ...]]
    discarded_timestamp_frames: int
    imu_raw_count: int
    imu_quaternion_count: int
    imu_raw_rate_hz: float
    imu_quaternion_rate_hz: float
    imu_rejected_frame_count: int
    minimum_motor_sample_coverage: float
    minimum_imu_rate_hz: float
    serial_write_count: int
    automatic_enable_attempts: int
    automatic_mode_switch_attempts: int
    errors: tuple[str, ...]

    @property
    def passed(self) -> bool:
        return (
            not self.errors
            and all(
                self.sample_coverage_by_motor.get(name, 0.0)
                >= self.minimum_motor_sample_coverage
                for name in self.motor_names
            )
            and self.imu_raw_rate_hz >= self.minimum_imu_rate_hz
            and self.imu_quaternion_rate_hz >= self.minimum_imu_rate_hz
            and self.imu_rejected_frame_count == 0
            and all(count > 0 for count in self.tx_count_by_interface.values())
            and self.serial_write_count == 0
            and self.automatic_enable_attempts == 0
            and self.automatic_mode_switch_attempts == 0
        )

    def to_dict(self) -> dict[str, Any]:
        return {**asdict(self), "passed": self.passed}


def collect_full_body_shadow(
    pollers: Mapping[str, Any],
    endpoints: Iterable[DamiaoFeedbackEndpoint],
    imu_port: Any,
    duration_s: float,
    rate_hz_per_motor: float,
    *,
    feedback_timeout_s: float = 0.2,
    minimum_motor_sample_coverage: float = 0.9,
    minimum_imu_rate_hz: float = 80.0,
    selector_factory: Callable[[], Any] = selectors.DefaultSelector,
    monotonic: Callable[[], float] = time.monotonic,
    on_feedback: Callable[[DamiaoFeedback], None] | None = None,
    on_imu_packet: Callable[[YahboomPacket], None] | None = None,
    keep_running: Callable[[], bool] | None = None,
    on_loop: Callable[[int], None] | None = None,
) -> FullBodyShadowReport:
    """Poll all four buses and read the IMU without enabling any motor."""
    selected = tuple(endpoints)
    if len(selected) != 31:
        raise ValueError(f"full-body probe requires 31 motors, got {len(selected)}")
    if len({item.motor_name for item in selected}) != len(selected):
        raise ValueError("full-body probe contains duplicate motor names")
    if not math.isfinite(duration_s) or not 0.0 < duration_s <= 120.0:
        raise ValueError("duration_s must be finite and in (0, 120]")
    if not math.isfinite(rate_hz_per_motor) or not 0.0 < rate_hz_per_motor <= 100.0:
        raise ValueError("rate_hz_per_motor must be finite and in (0, 100]")
    if not math.isfinite(feedback_timeout_s) or not 0.0 < feedback_timeout_s <= 1.0:
        raise ValueError("feedback_timeout_s must be finite and in (0, 1]")
    if not 0.0 < minimum_motor_sample_coverage <= 1.0:
        raise ValueError("minimum_motor_sample_coverage must be in (0, 1]")
    if not math.isfinite(minimum_imu_rate_hz) or minimum_imu_rate_hz <= 0.0:
        raise ValueError("minimum_imu_rate_hz must be finite and positive")
    if not hasattr(imu_port, "read") or not hasattr(imu_port, "fileno"):
        raise ValueError("a readable IMU serial port is required")

    by_interface: dict[str, tuple[DamiaoFeedbackEndpoint, ...]] = {}
    for interface in pollers:
        items = tuple(item for item in selected if item.interface == interface)
        if not 1 <= len(items) <= 8:
            raise ValueError(f"{interface} must contain between one and eight motors")
        if pollers[interface].interface != interface:
            raise ValueError(f"poller interface mismatch for {interface}")
        if not hasattr(pollers[interface], "send_zero_gain_poll"):
            raise ValueError(f"{interface} requires a restricted zero-gain poller")
        by_interface[interface] = items
    endpoint_interfaces = {item.interface for item in selected}
    if set(pollers) != endpoint_interfaces or len(pollers) != 4:
        raise ValueError("full-body probe requires exactly the four configured interfaces")

    decoder = DamiaoFeedbackDecoder(selected)
    imu_decoder = YahboomStreamDecoder()
    known_commands = {(item.interface, item.can_id) for item in selected}
    known_feedback = {(item.interface, item.master_id) for item in selected}
    selected_names = {item.motor_name for item in selected}
    targets = {item.motor_name: 0.0 for item in selected}
    feedback_values: dict[str, list[DamiaoFeedback]] = {
        item.motor_name: [] for item in selected
    }
    status_codes: dict[str, set[int]] = {item.motor_name: set() for item in selected}
    start = monotonic()
    last_feedback = {item.motor_name: start for item in selected}
    last_imu = start
    next_send = {interface: start for interface in pollers}
    next_index = {interface: 0 for interface in pollers}
    slot_s = {
        interface: 1.0 / (rate_hz_per_motor * len(items))
        for interface, items in by_interface.items()
    }
    discarded_timestamp_frames = 0
    imu_raw_count = 0
    imu_quaternion_count = 0
    errors: list[str] = []

    with selector_factory() as selector:
        for interface, poller in pollers.items():
            selector.register(poller, selectors.EVENT_READ, ("can", interface, poller))
        selector.register(imu_port, selectors.EVENT_READ, ("imu", None, imu_port))
        while True:
            now = monotonic()
            if keep_running is not None and not keep_running():
                break
            if now - start >= duration_s:
                break

            for interface, poller in pollers.items():
                if now < next_send[interface]:
                    continue
                items = by_interface[interface]
                endpoint = items[next_index[interface]]
                command = encode_zero_gain_position_echo(
                    endpoint, targets[endpoint.motor_name]
                )
                try:
                    poller.send_zero_gain_poll(command.can_id, command.data)
                except OSError as exc:
                    errors.append(
                        f"SocketCAN transmit failed on {interface} for "
                        f"{endpoint.motor_name}: {exc}"
                    )
                    break
                next_index[interface] = (next_index[interface] + 1) % len(items)
                next_send[interface] += slot_s[interface]
                if next_send[interface] <= now:
                    next_send[interface] = now + slot_s[interface]
            if errors:
                break

            timeout = min(
                max(0.0, min(next_send.values()) - monotonic()),
                0.005,
            )
            for key, _mask in selector.select(timeout=timeout):
                kind, interface, source = key.data
                if kind == "imu":
                    payload = source.read(max(source.in_waiting, 1))
                    for packet in imu_decoder.feed(payload):
                        if isinstance(packet, YahboomRawImu):
                            imu_raw_count += 1
                        elif isinstance(packet, YahboomQuaternion):
                            imu_quaternion_count += 1
                        last_imu = monotonic()
                        if on_imu_packet is not None:
                            on_imu_packet(packet)
                    continue
                try:
                    frame = source.receive()
                except SocketCanTimestampError:
                    discarded_timestamp_frames += 1
                    continue
                frame_key = (frame.interface, frame.can_id)
                if frame_key in known_commands:
                    continue
                if frame_key not in known_feedback:
                    errors.append(
                        f"unexpected CAN endpoint: {frame.interface}:{frame.can_id:#x}"
                    )
                    continue
                feedback = decoder.decode(frame)
                if feedback.motor_name not in selected_names:
                    continue
                if not frame.is_fd or not frame.bit_rate_switch:
                    errors.append(
                        f"{feedback.motor_name} feedback was not CAN-FD with BRS"
                    )
                    break
                feedback_values[feedback.motor_name].append(feedback)
                status_codes[feedback.motor_name].add(feedback.status_code)
                targets[feedback.motor_name] = feedback.position_rad
                last_feedback[feedback.motor_name] = monotonic()
                if feedback.status_code != 0x0:
                    errors.append(
                        f"{feedback.motor_name} is not disabled: {feedback.status_name}"
                    )
                    break
                if on_feedback is not None:
                    on_feedback(feedback)
            if errors:
                break

            if on_loop is not None:
                on_loop(time.perf_counter_ns())

            now = monotonic()
            stale_motors = sorted(
                name
                for name, last_seen in last_feedback.items()
                if now - last_seen > feedback_timeout_s
            )
            if stale_motors:
                errors.append(
                    f"no feedback within {feedback_timeout_s:.3f}s from: "
                    + ", ".join(stale_motors)
                )
                break
            if now - last_imu > feedback_timeout_s:
                errors.append(f"no IMU packet within {feedback_timeout_s:.3f}s")
                break

    elapsed = monotonic() - start
    expected = max(1, int(elapsed * rate_hz_per_motor))
    counts = {name: len(values) for name, values in feedback_values.items()}
    coverage = {name: min(1.0, count / expected) for name, count in counts.items()}
    return FullBodyShadowReport(
        requested_duration_s=duration_s,
        requested_rate_hz_per_motor=rate_hz_per_motor,
        elapsed_s=elapsed,
        motor_names=tuple(item.motor_name for item in selected),
        tx_count_by_interface={
            interface: int(poller.hardware_tx_attempts)
            for interface, poller in pollers.items()
        },
        rx_count_by_motor=counts,
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
            name: tuple(sorted(values)) for name, values in status_codes.items()
        },
        discarded_timestamp_frames=discarded_timestamp_frames,
        imu_raw_count=imu_raw_count,
        imu_quaternion_count=imu_quaternion_count,
        imu_raw_rate_hz=imu_raw_count / elapsed,
        imu_quaternion_rate_hz=imu_quaternion_count / elapsed,
        imu_rejected_frame_count=imu_decoder.rejected_frame_count,
        minimum_motor_sample_coverage=minimum_motor_sample_coverage,
        minimum_imu_rate_hz=minimum_imu_rate_hz,
        serial_write_count=0,
        automatic_enable_attempts=0,
        automatic_mode_switch_attempts=0,
        errors=tuple(errors),
    )
