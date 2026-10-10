"""Session-relative gyro heading telemetry; never feeds the actor or commands."""

import math

import numpy as np


class SessionHeading:
    def __init__(self):
        self.reset()

    def reset(self):
        self.yaw_rad = 0.0
        self.origin_ns = None
        self.previous_ns = None
        self.previous_rate = None
        self.valid = True

    def update(self, omega_body, gravity_body, timestamp_ns):
        omega = np.asarray(omega_body, dtype=float)
        gravity = np.asarray(gravity_body, dtype=float)
        if (omega.shape != (3,) or gravity.shape != (3,)
                or not np.all(np.isfinite(omega))
                or not np.all(np.isfinite(gravity)) or timestamp_ns <= 0):
            self.valid = False
            return self.yaw_rad
        norm = np.linalg.norm(gravity)
        if norm < 1e-6:
            self.valid = False
            return self.yaw_rad
        down = gravity / norm
        cos_pitch = math.hypot(down[1], down[2])
        if cos_pitch < 0.2:
            self.valid = False
            return self.yaw_rad
        # Euler heading derivative, not simply body gyro Z when tilted.
        rate = (-down[1] * omega[1] - down[2] * omega[2]) / cos_pitch**2
        if self.previous_ns is not None:
            dt = (timestamp_ns - self.previous_ns) / 1e9
            if dt <= 0 or dt > 0.05:
                self.valid = False
                return self.yaw_rad
            if self.valid:
                self.yaw_rad += 0.5 * (self.previous_rate + rate) * dt
        else:
            self.origin_ns = timestamp_ns
        self.previous_ns = timestamp_ns
        self.previous_rate = rate
        return self.yaw_rad

    def report(self):
        return {"reset_before_first_policy_inference": True,
                "origin_monotonic_ns": self.origin_ns,
                "relative_yaw_rad": self.yaw_rad, "valid": self.valid,
                "integration": "policy_tick_gyro_trapezoid_tilt_corrected",
                "gyro_bias_subtracted": False, "affects_policy_or_commands": False}
