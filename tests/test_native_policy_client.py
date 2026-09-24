import unittest

import numpy as np

from open_sprite_runtime.multirate_control import JointImpedanceTarget
from open_sprite_runtime.native_policy_client import (
    _capture_runtime_errors,
    _startup_measured_position,
)
from open_sprite_runtime.physical_startup import PhysicalStartupRamp


class RuntimeErrorCaptureTests(unittest.TestCase):
    def test_runtime_error_is_recorded_without_escaping(self) -> None:
        errors: list[str] = []

        with _capture_runtime_errors(errors):
            raise RuntimeError("watchdog test trip")

        self.assertEqual(errors, ["RuntimeError: watchdog test trip"])

    def test_clean_exit_does_not_add_an_error(self) -> None:
        errors: list[str] = []

        with _capture_runtime_errors(errors):
            pass

        self.assertEqual(errors, [])


class StartupMeasuredPoseTests(unittest.TestCase):
    def test_soft_limit_gate_runs_only_for_initial_pose_capture(self) -> None:
        class Projector:
            calls = 0

            def require_position_within_limits(self, position, *, label):
                self.calls += 1
                self.assert_label = label
                return np.asarray(position, dtype=np.float64).copy()

        projector = Projector()
        startup = PhysicalStartupRamp(50.0, 1.0, 4.0)
        initial = np.zeros(31)
        captured = _startup_measured_position(startup, projector, initial)
        startup.apply(
            JointImpedanceTarget(
                position_rad=np.zeros(31),
                velocity_rad_s=np.zeros(31),
                kp=np.zeros(31),
                kd=np.zeros(31),
                feedforward_torque_nm=np.zeros(31),
            ),
            captured,
        )

        disturbed = np.full(31, 10.0)
        returned = _startup_measured_position(startup, projector, disturbed)

        self.assertEqual(projector.calls, 1)
        self.assertEqual(projector.assert_label, "physical startup measured pose")
        self.assertIs(returned, disturbed)


if __name__ == "__main__":
    unittest.main()
