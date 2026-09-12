"""Hard gates for future hardware transmission."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Iterable


class RuntimeMode(str, Enum):
    DRY_RUN = "dry_run"
    SHADOW = "shadow"
    ARMED = "armed"


@dataclass(frozen=True)
class SafetyState:
    allow_hardware_tx: bool = False
    hardware_configured: bool = False
    left_ankle_calibrated: bool = False
    right_ankle_calibrated: bool = False
    imu_valid: bool = False
    estop_healthy: bool = False
    state_fresh: bool = False
    motor_telemetry_healthy: bool = False

    def blockers(self) -> list[str]:
        checks = {
            "allow_hardware_tx": self.allow_hardware_tx,
            "hardware_configured": self.hardware_configured,
            "left_ankle_calibrated": self.left_ankle_calibrated,
            "right_ankle_calibrated": self.right_ankle_calibrated,
            "imu_valid": self.imu_valid,
            "estop_healthy": self.estop_healthy,
            "state_fresh": self.state_fresh,
            "motor_telemetry_healthy": self.motor_telemetry_healthy,
        }
        return [name for name, ready in checks.items() if not ready]

    def require_armed(self) -> None:
        blockers = self.blockers()
        if blockers:
            raise RuntimeError(f"hardware arm blocked by: {', '.join(blockers)}")


@dataclass(frozen=True)
class SafetyLimits:
    maximum_state_age_ms: float = 6.0
    maximum_command_age_ms: float = 100.0
    maximum_policy_overrun_ms: float = 2.0

    def validate(self) -> None:
        for name, value in (
            ("maximum_state_age_ms", self.maximum_state_age_ms),
            ("maximum_command_age_ms", self.maximum_command_age_ms),
            ("maximum_policy_overrun_ms", self.maximum_policy_overrun_ms),
        ):
            if value <= 0.0:
                raise ValueError(f"{name} must be positive")


@dataclass(frozen=True)
class SafetyInputs:
    now_ns: int
    state_timestamp_ns: int
    command_timestamp_ns: int
    policy_overrun_ms: float
    allow_hardware_tx: bool
    hardware_configured: bool
    left_ankle_calibrated: bool
    right_ankle_calibrated: bool
    imu_valid: bool
    estop_healthy: bool
    motor_telemetry_healthy: bool = False
    command_envelope_healthy: bool = False


@dataclass(frozen=True)
class SafetyDecision:
    mode: RuntimeMode
    hardware_tx_permitted: bool
    safe_hold_required: bool
    blockers: tuple[str, ...]
    latched_faults: tuple[str, ...]
    state_age_ms: float
    command_age_ms: float


class SafetySupervisor:
    """Fail-closed runtime gate independent of the future CAN backend."""

    _LATCHING_FAULTS = frozenset(
        (
            "estop_unhealthy",
            "state_stale",
            "policy_overrun",
            "motor_limit",
            "command_limit",
        )
    )

    def __init__(self, mode: RuntimeMode, limits: SafetyLimits):
        limits.validate()
        self.mode = mode
        self.limits = limits
        self._latched_faults: set[str] = set()

    @property
    def latched_faults(self) -> tuple[str, ...]:
        return tuple(sorted(self._latched_faults))

    def clear_latched_faults(self, healthy_faults: Iterable[str] = ()) -> None:
        """Clear only faults explicitly demonstrated healthy by the caller."""
        self._latched_faults.difference_update(set(healthy_faults) & self._LATCHING_FAULTS)

    def evaluate(self, inputs: SafetyInputs) -> SafetyDecision:
        if inputs.now_ns < inputs.state_timestamp_ns or inputs.now_ns < inputs.command_timestamp_ns:
            raise ValueError("timestamps cannot be in the future")
        state_age_ms = (inputs.now_ns - inputs.state_timestamp_ns) / 1.0e6
        command_age_ms = (inputs.now_ns - inputs.command_timestamp_ns) / 1.0e6
        dynamic: list[str] = []
        checks = (
            ("allow_hardware_tx", inputs.allow_hardware_tx),
            ("hardware_configured", inputs.hardware_configured),
            ("left_ankle_calibrated", inputs.left_ankle_calibrated),
            ("right_ankle_calibrated", inputs.right_ankle_calibrated),
            ("imu_invalid", inputs.imu_valid),
            ("estop_unhealthy", inputs.estop_healthy),
            ("motor_limit", inputs.motor_telemetry_healthy),
            ("command_limit", inputs.command_envelope_healthy),
            ("state_stale", state_age_ms <= self.limits.maximum_state_age_ms),
            ("command_stale", command_age_ms <= self.limits.maximum_command_age_ms),
            (
                "policy_overrun",
                inputs.policy_overrun_ms <= self.limits.maximum_policy_overrun_ms,
            ),
        )
        dynamic.extend(name for name, ready in checks if not ready)
        self._latched_faults.update(set(dynamic) & self._LATCHING_FAULTS)
        blockers = set(dynamic) | self._latched_faults
        if self.mode is not RuntimeMode.ARMED:
            blockers.add(f"mode_{self.mode.value}_no_tx")
        permitted = self.mode is RuntimeMode.ARMED and not blockers
        return SafetyDecision(
            mode=self.mode,
            hardware_tx_permitted=permitted,
            safe_hold_required=not permitted,
            blockers=tuple(sorted(blockers)),
            latched_faults=self.latched_faults,
            state_age_ms=state_age_ms,
            command_age_ms=command_age_ms,
        )
