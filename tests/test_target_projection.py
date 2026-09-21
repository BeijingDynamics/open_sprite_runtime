import unittest

import numpy as np

from open_sprite_runtime.multirate_control import JointImpedanceTarget
from open_sprite_runtime.target_projection import ProtectedTargetProjector


NAMES = tuple(f"joint_{index}" for index in range(31))


def limit_report() -> dict:
    return {
        "joint_limits": {
            name: {"soft_limit_rad_candidate": [-0.5, 0.5]} for name in NAMES
        }
    }


def target(position: np.ndarray, *, kd: float = 2.0) -> JointImpedanceTarget:
    return JointImpedanceTarget(
        position_rad=position,
        velocity_rad_s=np.zeros(31),
        kp=np.full(31, 100.0),
        kd=np.full(31, kd),
        feedforward_torque_nm=np.full(31, 4.0),
    )


class ProtectedTargetProjectorTests(unittest.TestCase):
    def test_projects_positions_and_scales_impedance(self) -> None:
        projector = ProtectedTargetProjector.from_limit_report(
            NAMES, limit_report(), gain_scale=0.1
        )
        position = np.zeros(31)
        position[2] = 0.8
        position[7] = -0.9

        result = projector.project(target(position))

        self.assertEqual(result.position_rad[2], 0.5)
        self.assertEqual(result.position_rad[7], -0.5)
        np.testing.assert_allclose(result.kp, 10.0)
        np.testing.assert_allclose(result.kd, 0.2)
        np.testing.assert_allclose(result.feedforward_torque_nm, 0.4)
        self.assertEqual(projector.clamp_count_by_joint["joint_2"], 1)
        self.assertEqual(projector.clamp_count_by_joint["joint_7"], 1)
        self.assertAlmostEqual(projector.maximum_raw_position_overshoot_rad, 0.4)

    def test_rejects_incomplete_limits_and_excessive_projected_kd(self) -> None:
        incomplete = limit_report()
        del incomplete["joint_limits"]["joint_4"]
        with self.assertRaisesRegex(ValueError, "exactly cover"):
            ProtectedTargetProjector.from_limit_report(
                NAMES, incomplete, gain_scale=0.1
            )
        projector = ProtectedTargetProjector.from_limit_report(
            NAMES, limit_report(), gain_scale=1.0, maximum_embedded_kd=3.0
        )
        with self.assertRaisesRegex(ValueError, "Kd"):
            projector.project(target(np.zeros(31), kd=3.01))

    def test_rejects_invalid_gain_scale(self) -> None:
        with self.assertRaisesRegex(ValueError, "gain_scale"):
            ProtectedTargetProjector.from_limit_report(
                NAMES, limit_report(), gain_scale=0.0
            )


if __name__ == "__main__":
    unittest.main()
