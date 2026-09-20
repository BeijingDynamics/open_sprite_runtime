"""Read-only raw serial capture for commissioning the Yahboom IMU."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
import time

from .yahboom_imu import YahboomRawImu, YahboomStreamDecoder, build_report_rate_command


HARDWARE_TX_CONFIRMATION = "WRITE_IMU_CONFIGURATION"


@dataclass(frozen=True)
class SerialCaptureReport:
    device: str
    baud_rate: int
    requested_duration_s: float
    elapsed_s: float
    byte_count: int
    read_count: int
    first_byte_monotonic_ns: int | None
    last_byte_monotonic_ns: int | None
    raw_output: str

    @property
    def passed(self) -> bool:
        return self.byte_count > 0 and self.read_count > 0

    def to_dict(self) -> dict:
        return {
            "mode": "read_only_imu_serial_capture_no_tx",
            "device": self.device,
            "baud_rate": self.baud_rate,
            "requested_duration_s": self.requested_duration_s,
            "elapsed_s": self.elapsed_s,
            "byte_count": self.byte_count,
            "read_count": self.read_count,
            "first_byte_monotonic_ns": self.first_byte_monotonic_ns,
            "last_byte_monotonic_ns": self.last_byte_monotonic_ns,
            "raw_output": self.raw_output,
            "passed": self.passed,
        }


@dataclass(frozen=True)
class ReportRateConfigurationReport:
    device: str
    baud_rate: int
    requested_rate_hz: int
    command_hex: str
    baseline_rate_hz: float
    measured_rate_hz: float
    verification_duration_s: float
    protocol_ack_available: bool = False
    persistence_after_power_cycle_verified: bool = False

    @property
    def passed(self) -> bool:
        tolerance_hz = max(2.0, self.requested_rate_hz * 0.05)
        return abs(self.measured_rate_hz - self.requested_rate_hz) <= tolerance_hz

    def to_dict(self) -> dict:
        return {
            "mode": "yahboom_imu_report_rate_hardware_configuration",
            "device": self.device,
            "baud_rate": self.baud_rate,
            "requested_rate_hz": self.requested_rate_hz,
            "command_hex": self.command_hex,
            "baseline_rate_hz": self.baseline_rate_hz,
            "measured_rate_hz": self.measured_rate_hz,
            "verification_duration_s": self.verification_duration_s,
            "protocol_ack_available": self.protocol_ack_available,
            "persistence_after_power_cycle_verified": self.persistence_after_power_cycle_verified,
            "passed": self.passed,
        }


def _measure_raw_imu_rate(port, duration_s: float) -> float:
    decoder = YahboomStreamDecoder()
    count = 0
    start_ns = time.monotonic_ns()
    deadline_ns = start_ns + int(duration_s * 1.0e9)
    while time.monotonic_ns() < deadline_ns:
        payload = port.read(max(port.in_waiting, 1))
        if payload:
            count += sum(isinstance(packet, YahboomRawImu) for packet in decoder.feed(payload))
    elapsed_s = (time.monotonic_ns() - start_ns) / 1.0e9
    return count / elapsed_s


def configure_report_rate(
    device: str,
    baud_rate: int,
    rate_hz: int,
    verification_duration_s: float,
    confirmation: str,
) -> ReportRateConfigurationReport:
    """Write one report-rate command and verify the resulting stream rate."""
    if confirmation != HARDWARE_TX_CONFIRMATION:
        raise ValueError(
            f"hardware TX is not armed; pass confirmation {HARDWARE_TX_CONFIRMATION!r}"
        )
    if not device.startswith("/dev/"):
        raise ValueError("serial device must be an absolute /dev path")
    if baud_rate <= 0:
        raise ValueError("baud rate must be positive")
    if (
        not math.isfinite(verification_duration_s)
        or not 1.0 <= verification_duration_s <= 10.0
    ):
        raise ValueError("verification duration must be finite and in [1, 10] seconds")
    command = build_report_rate_command(rate_hz)
    try:
        import serial
    except ImportError as exc:
        raise RuntimeError("pyserial is required for IMU serial configuration") from exc

    with serial.Serial(
        port=device,
        baudrate=baud_rate,
        timeout=0.02,
        write_timeout=1.0,
        exclusive=True,
    ) as port:
        port.dtr = False
        port.rts = False
        port.reset_input_buffer()
        baseline_rate_hz = _measure_raw_imu_rate(port, verification_duration_s)
        written = port.write(command)
        port.flush()
        if written != len(command):
            raise RuntimeError(
                f"short IMU configuration write: {written}/{len(command)} bytes"
            )
        time.sleep(0.25)
        port.reset_input_buffer()
        measured_rate_hz = _measure_raw_imu_rate(port, verification_duration_s)

    return ReportRateConfigurationReport(
        device=device,
        baud_rate=baud_rate,
        requested_rate_hz=rate_hz,
        command_hex=command.hex(" "),
        baseline_rate_hz=baseline_rate_hz,
        measured_rate_hz=measured_rate_hz,
        verification_duration_s=verification_duration_s,
    )


def capture_serial_read_only(
    device: str,
    baud_rate: int,
    duration_s: float,
    raw_output: str | Path,
) -> SerialCaptureReport:
    if not device.startswith("/dev/"):
        raise ValueError("serial device must be an absolute /dev path")
    if baud_rate <= 0:
        raise ValueError("baud rate must be positive")
    if not math.isfinite(duration_s) or not 0.0 < duration_s <= 120.0:
        raise ValueError("duration must be finite and in (0, 120] seconds")
    try:
        import serial
    except ImportError as exc:
        raise RuntimeError("pyserial is required for IMU serial capture") from exc

    output = Path(raw_output).expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    start_ns = time.monotonic_ns()
    deadline_ns = start_ns + int(duration_s * 1.0e9)
    first_ns: int | None = None
    last_ns: int | None = None
    byte_count = 0
    read_count = 0
    with serial.Serial(
        port=device,
        baudrate=baud_rate,
        timeout=0.02,
        write_timeout=0,
        exclusive=True,
    ) as port, output.open("wb") as raw:
        # Opening a USB UART can change modem-control lines. Keep both inactive;
        # this command never calls write() and sends no protocol bytes.
        port.dtr = False
        port.rts = False
        while time.monotonic_ns() < deadline_ns:
            payload = port.read(max(port.in_waiting, 1))
            if not payload:
                continue
            timestamp_ns = time.monotonic_ns()
            first_ns = timestamp_ns if first_ns is None else first_ns
            last_ns = timestamp_ns
            raw.write(payload)
            byte_count += len(payload)
            read_count += 1
    elapsed_s = (time.monotonic_ns() - start_ns) / 1.0e9
    return SerialCaptureReport(
        device=device,
        baud_rate=baud_rate,
        requested_duration_s=duration_s,
        elapsed_s=elapsed_s,
        byte_count=byte_count,
        read_count=read_count,
        first_byte_monotonic_ns=first_ns,
        last_byte_monotonic_ns=last_ns,
        raw_output=str(output),
    )
