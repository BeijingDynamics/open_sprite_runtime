"""Fixed commissioning command schedule, independent of hardware transport."""
import math

PROFILE = "stand-walk-stop-40s"
PROFILE_010 = "stand-walk010-stop-40s"
PROFILE_SPEEDS = {PROFILE: 0.05, PROFILE_010: 0.10}


def sample_stand_walk_stop(elapsed_s: float, profile: str = PROFILE) -> tuple[float, str]:
    if profile not in PROFILE_SPEEDS:
        raise ValueError("unsupported command profile")
    speed = PROFILE_SPEEDS[profile]
    if not math.isfinite(elapsed_s) or elapsed_s < 0:
        raise ValueError("elapsed time must be finite and nonnegative")
    if elapsed_s < 10.0:
        return 0.0, "stand"
    if elapsed_s < 13.0:
        x = (elapsed_s - 10.0) / 3.0
        return speed * x * x * (3.0 - 2.0 * x), "ramp_forward"
    if elapsed_s < 27.0:
        return speed, "forward"
    if elapsed_s < 30.0:
        x = (elapsed_s - 27.0) / 3.0
        return speed * (1.0 - x * x * (3.0 - 2.0 * x)), "ramp_stop"
    return 0.0, "final_stand"


def countdown(elapsed_s: float) -> int | None:
    if 35.0 <= elapsed_s < 40.0:
        return math.ceil(40.0 - elapsed_s)
    return None
