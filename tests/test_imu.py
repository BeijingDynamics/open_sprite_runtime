import math
import unittest

import numpy as np

from open_sprite_runtime.imu import (
    ImuMount,
    ImuValidityGate,
    ImuValidityLimits,
    IntegratedYawEstimator,
    RawImuSample,
    matrix_to_quaternion_wxyz,
    quaternion_wxyz_to_matrix,
    transform_pelvis_sample,
)


class ImuRuntimeTests(unittest.TestCase):
    def setUp(self):
        self.mount = ImuMount.sprite0825_rear_pelvis()

    def upright_sample(self, timestamp_ns=1_000_000_000):
        # For an upright body, world_from_sensor equals body_from_sensor.
        orientation = matrix_to_quaternion_wxyz(self.mount.sensor_to_body_matrix)
        return RawImuSample(
            monotonic_ns=timestamp_ns,
            angular_velocity_sensor_rad_s=(1.0, 2.0, 3.0),
            linear_acceleration_sensor_m_s2=(0.0, 0.0, 9.80665),
            orientation_sensor_to_world_wxyz=tuple(orientation),
            magnetic_field_sensor=(4.0, 5.0, 6.0),
        )

    def test_mount_matches_declared_physical_axes(self):
        np.testing.assert_allclose(
            self.mount.sensor_to_body_matrix,
            [[0.0, 0.0, -1.0], [0.0, -1.0, 0.0], [-1.0, 0.0, 0.0]],
        )
        np.testing.assert_allclose(
            self.mount.vector_sensor_to_body((1.0, 2.0, 3.0)),
            [-3.0, -2.0, -1.0],
        )
        self.assertAlmostEqual(np.linalg.det(self.mount.sensor_to_body_matrix), 1.0)

    def test_quaternion_round_trip(self):
        quaternion = self.mount.body_to_sensor_quaternion_wxyz
        np.testing.assert_allclose(
            quaternion_wxyz_to_matrix(tuple(quaternion)),
            self.mount.sensor_to_body_matrix.T,
            atol=1.0e-9,
        )

    def test_policy_facing_upright_state(self):
        transformed = transform_pelvis_sample(self.upright_sample(), self.mount)
        np.testing.assert_allclose(transformed.angular_velocity_body_rad_s, [-3.0, -2.0, -1.0])
        np.testing.assert_allclose(
            transformed.projected_gravity_body, [0.0, 0.0, -1.0], atol=1.0e-12
        )
        np.testing.assert_allclose(
            quaternion_wxyz_to_matrix(tuple(transformed.orientation_body_to_world_wxyz)),
            np.eye(3),
            atol=1.0e-9,
        )
        np.testing.assert_allclose(transformed.magnetic_field_body, [-6.0, -5.0, -4.0])

    def test_validity_gate_fails_closed_on_stale_gap_and_bad_quaternion(self):
        gate = ImuValidityGate(ImuValidityLimits(maximum_age_ms=20.0, maximum_gap_ms=20.0))
        sample = self.upright_sample()
        valid, errors = gate.check(sample, now_ns=sample.monotonic_ns + 10_000_000)
        self.assertTrue(valid)
        self.assertFalse(errors)

        late = self.upright_sample(sample.monotonic_ns + 30_000_000)
        valid, errors = gate.check(late, now_ns=late.monotonic_ns + 30_000_000)
        self.assertFalse(valid)
        self.assertIn("stale_sample", errors)
        self.assertIn("sample_gap", errors)

        bad = RawImuSample(
            monotonic_ns=late.monotonic_ns + 10_000_000,
            angular_velocity_sensor_rad_s=(0.0, 0.0, 0.0),
            linear_acceleration_sensor_m_s2=(0.0, 0.0, 9.8),
            orientation_sensor_to_world_wxyz=(2.0, 0.0, 0.0, 0.0),
        )
        valid, errors = gate.check(bad, now_ns=bad.monotonic_ns)
        self.assertFalse(valid)
        self.assertIn("quaternion_norm", errors)

    def test_integrated_yaw_resets_while_standing(self):
        estimator = IntegratedYawEstimator()
        self.assertEqual(estimator.update(1.0, 1_000_000_000, standing=True), 0.0)
        self.assertAlmostEqual(estimator.update(1.0, 1_010_000_000, standing=False), 0.01)
        self.assertAlmostEqual(estimator.update(1.0, 1_020_000_000, standing=False), 0.02)
        self.assertEqual(estimator.update(1.0, 1_030_000_000, standing=True), 0.0)
        with self.assertRaises(ValueError):
            estimator.update(1.0, 1_100_000_000, standing=False)


if __name__ == "__main__":
    unittest.main()
