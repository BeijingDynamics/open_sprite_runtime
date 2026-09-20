import math
import unittest

import numpy as np

from open_sprite_runtime.imu import ImuMount, matrix_to_quaternion_wxyz
from open_sprite_runtime.imu_commissioning import (
    body_orientation_matrix,
    relative_body_orientation,
)


class ImuCommissioningTests(unittest.TestCase):
    def test_mount_conversion_recovers_upright_body(self):
        mount = ImuMount.sprite0825_rear_pelvis()
        sensor_to_world = mount.sensor_to_body_matrix
        quaternion = matrix_to_quaternion_wxyz(sensor_to_world)
        body_to_world = body_orientation_matrix(tuple(quaternion), mount)
        np.testing.assert_allclose(body_to_world, np.eye(3), atol=1.0e-9)

    def test_flat_sensor_does_not_hide_incorrect_mount(self):
        mount = ImuMount.sprite0825_rear_pelvis()
        body_to_world = body_orientation_matrix((1.0, 0.0, 0.0, 0.0), mount)
        np.testing.assert_allclose(
            body_to_world, mount.sensor_to_body_matrix.T, atol=1.0e-9
        )
        np.testing.assert_allclose(
            body_to_world @ np.array([1.0, 0.0, 0.0]),
            np.array([0.0, 0.0, -1.0]),
            atol=1.0e-9,
        )

    def test_relative_orientation_starts_at_identity_and_tracks_yaw(self):
        mount = ImuMount.sprite0825_rear_pelvis()
        initial = np.eye(3)
        angle = math.pi / 2.0
        yaw = np.array(
            [
                [math.cos(angle), -math.sin(angle), 0.0],
                [math.sin(angle), math.cos(angle), 0.0],
                [0.0, 0.0, 1.0],
            ]
        )
        sensor_to_world = yaw @ mount.sensor_to_body_matrix
        quaternion = matrix_to_quaternion_wxyz(sensor_to_world)
        current = body_orientation_matrix(tuple(quaternion), mount)
        np.testing.assert_allclose(
            relative_body_orientation(initial, initial), np.eye(3), atol=1.0e-9
        )
        np.testing.assert_allclose(
            relative_body_orientation(initial, current), yaw, atol=1.0e-9
        )


if __name__ == "__main__":
    unittest.main()
