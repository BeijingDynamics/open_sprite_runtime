import unittest

import numpy as np

from open_sprite_runtime.multirate_control import JointImpedanceTarget
from open_sprite_runtime.target_projection import (
    ConsecutiveClampWatchdog,
    ProtectedTargetProjector,
    parse_joint_gain_multiplier_overrides,
)


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

    def test_identifies_measured_pose_outside_soft_limits(self) -> None:
        projector = ProtectedTargetProjector.from_limit_report(
            NAMES, limit_report(), gain_scale=0.1
        )
        position = np.zeros(31)
        position[3] = 0.6
        position[9] = -0.7

        with self.assertRaisesRegex(
            ValueError,
            r"physical startup measured pose.*joint_3=\+0\.600000.*joint_9=-0\.700000",
        ):
            projector.require_position_within_limits(
                position, label="physical startup measured pose"
            )

        np.testing.assert_allclose(
            projector.require_position_within_limits(np.zeros(31)), 0.0
        )

    def test_applies_explicit_per_joint_gain_multipliers(self) -> None:
        multipliers = parse_joint_gain_multiplier_overrides(
            NAMES, ["joint_3=0.1", "joint_8=0.25"]
        )
        projector = ProtectedTargetProjector.from_limit_report(
            NAMES,
            limit_report(),
            gain_scale=0.2,
            joint_gain_multipliers=multipliers,
        )

        result = projector.project(target(np.zeros(31)))

        self.assertAlmostEqual(result.kp[0], 20.0)
        self.assertAlmostEqual(result.kp[3], 2.0)
        self.assertAlmostEqual(result.kp[8], 5.0)
        self.assertEqual(
            projector.report()["joint_gain_multiplier_overrides"],
            {"joint_3": 0.1, "joint_8": 0.25},
        )
        with self.assertRaisesRegex(ValueError, "duplicate"):
            parse_joint_gain_multiplier_overrides(
                NAMES, ["joint_3=0.1", "joint_3=0.2"]
            )

    def test_consecutive_clamp_watchdog_resets_and_trips(self) -> None:
        watchdog = ConsecutiveClampWatchdog(
            NAMES, ("joint_2", "joint_7"), 0.05, 3
        )
        raw = np.zeros(31)
        projected = np.zeros(31)
        raw[2] = 0.08

        watchdog.update(raw, projected)
        watchdog.update(raw, projected)
        raw[2] = 0.04
        watchdog.update(raw, projected)
        self.assertEqual(
            watchdog.report()["maximum_observed_consecutive_ticks_by_joint"]["joint_2"],
            2,
        )

        raw[7] = -0.08
        for _ in range(2):
            watchdog.update(raw, projected)
        with self.assertRaisesRegex(RuntimeError, "joint_7.*3 consecutive"):
            watchdog.update(raw, projected)

    def test_consecutive_clamp_watchdog_rejects_unknown_joint(self) -> None:
        with self.assertRaisesRegex(ValueError, "unknown"):
            ConsecutiveClampWatchdog(NAMES, ("missing",), 0.05, 5)


if __name__ == "__main__":
    unittest.main()
