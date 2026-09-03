"""PM01-style outer heading controller for the deployable velocity command."""

from __future__ import annotations

from dataclasses import dataclass
import math


def wrap_to_pi(angle: float) -> float:
    """Wrap an angle to the same [-pi, pi) interval used by Isaac Lab."""
    if not math.isfinite(angle):
        raise ValueError("heading angle must be finite")
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


@dataclass(frozen=True)
class HeadingControllerConfig:
    stiffness: float = 0.5
    yaw_rate_limit_rad_s: float = 0.2
    manual_yaw_deadband_rad_s: float = 0.01

    def validate(self) -> None:
        values = (
            self.stiffness,
            self.yaw_rate_limit_rad_s,
            self.manual_yaw_deadband_rad_s,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("heading controller values must be finite")
        if self.stiffness <= 0.0:
            raise ValueError("heading stiffness must be positive")
        if self.yaw_rate_limit_rad_s <= 0.0:
            raise ValueError("yaw-rate limit must be positive")
        if not 0.0 <= self.manual_yaw_deadband_rad_s < self.yaw_rate_limit_rad_s:
            raise ValueError("manual yaw deadband must be non-negative and below the limit")


class HeadingCommandController:
    """Convert integrated IMU yaw and operator input to the actor yaw-rate command.

    Global yaw stays outside the actor. During a manual turn, the requested yaw
    rate is passed through and the hold target follows the measured heading. When
    the operator releases the turn command, the most recent heading is held.
    """

    def __init__(self, config: HeadingControllerConfig | None = None):
        self.config = config or HeadingControllerConfig()
        self.config.validate()
        self._target_yaw: float | None = None

    @property
    def target_yaw(self) -> float | None:
        return self._target_yaw

    def reset(self, current_yaw: float) -> None:
        self._target_yaw = wrap_to_pi(current_yaw)

    def update(
        self,
        current_yaw: float,
        requested_yaw_rate_rad_s: float = 0.0,
        *,
        standing: bool = False,
    ) -> float:
        current_yaw = wrap_to_pi(current_yaw)
        if not math.isfinite(requested_yaw_rate_rad_s):
            raise ValueError("requested yaw rate must be finite")
        if self._target_yaw is None:
            self.reset(current_yaw)

        if standing:
            self.reset(current_yaw)
            return 0.0

        limit = self.config.yaw_rate_limit_rad_s
        if abs(requested_yaw_rate_rad_s) > self.config.manual_yaw_deadband_rad_s:
            self.reset(current_yaw)
            return max(-limit, min(limit, requested_yaw_rate_rad_s))

        error = wrap_to_pi(float(self._target_yaw) - current_yaw)
        correction = self.config.stiffness * error
        return max(-limit, min(limit, correction))
