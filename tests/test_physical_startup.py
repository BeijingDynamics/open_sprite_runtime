import unittest

import numpy as np

from open_sprite_runtime.multirate_control import JointImpedanceTarget
from open_sprite_runtime.physical_startup import PhysicalStartupRamp


def target(position: float = 1.0) -> JointImpedanceTarget:
    return JointImpedanceTarget(
        position_rad=np.full(31, position),
        velocity_rad_s=np.full(31, 2.0),
        kp=np.full(31, 10.0),
        kd=np.full(31, 1.0),
        feedforward_torque_nm=np.full(31, 3.0),
    )


class PhysicalStartupRampTests(unittest.TestCase):
    def test_holds_captured_pose_then_smoothly_admits_target_and_gains(self) -> None:
        ramp = PhysicalStartupRamp(policy_hz=2.0, hold_seconds=1.0, ramp_seconds=1.0)
        measured = np.full(31, 0.25)

        first = ramp.apply(target(), measured)
        second = ramp.apply(target(), measured + 0.5)
        third = ramp.apply(target(), measured + 0.5)
        fourth = ramp.apply(target(), measured + 0.5)

        np.testing.assert_allclose(first.position_rad, 0.25)
        np.testing.assert_allclose(second.position_rad, 0.25)
        np.testing.assert_allclose(third.position_rad, 0.625)
        np.testing.assert_allclose(third.kp, 5.0)
        np.testing.assert_allclose(fourth.position_rad, 1.0)
        np.testing.assert_allclose(fourth.kd, 1.0)
        self.assertTrue(ramp.complete)
        self.assertEqual(ramp.report()["last_alpha"], 1.0)

    def test_rejects_invalid_timing_and_nonfinite_measurement(self) -> None:
        with self.assertRaisesRegex(ValueError, "ramp_seconds"):
            PhysicalStartupRamp(50.0, 1.0, 0.0)
        ramp = PhysicalStartupRamp(50.0, 0.0, 1.0)
        measured = np.zeros(31)
        measured[0] = np.nan
        with self.assertRaisesRegex(ValueError, "measured_position_rad"):
            ramp.apply(target(), measured)


if __name__ == "__main__":
    unittest.main()
