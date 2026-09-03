"""Hard gates for future hardware transmission."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


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

    def blockers(self) -> list[str]:
        checks = {
            "allow_hardware_tx": self.allow_hardware_tx,
            "hardware_configured": self.hardware_configured,
            "left_ankle_calibrated": self.left_ankle_calibrated,
            "right_ankle_calibrated": self.right_ankle_calibrated,
            "imu_valid": self.imu_valid,
            "estop_healthy": self.estop_healthy,
            "state_fresh": self.state_fresh,
        }
        return [name for name, ready in checks.items() if not ready]

    def require_armed(self) -> None:
        blockers = self.blockers()
        if blockers:
            raise RuntimeError(f"hardware arm blocked by: {', '.join(blockers)}")

