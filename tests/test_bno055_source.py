import time
import unittest
from dataclasses import replace
from unittest.mock import patch
import numpy as np

from open_sprite_runtime.bno055 import Sample
from open_sprite_runtime.bno055_source import Bno055Source, sample_packets
from open_sprite_runtime.imu import ImuMount, quaternion_wxyz_to_matrix


def sample(stamp):
    return Sample(stamp, stamp+1000, (1., 2., 3.), (.1, .2, .3),
                  (1., 0., 0., 0.), (0., 0., 9.81), 25, {}, '')


class SourceTests(unittest.TestCase):
    def test_adapter_matches_viewer_frame(self):
        s = sample(1)
        raw, q = sample_packets(s)
        m = ImuMount.sprite0825_rear_pelvis().sensor_to_body_matrix
        r = quaternion_wxyz_to_matrix(q.wxyz) @ m.T
        np.testing.assert_allclose(m @ raw.angular_velocity_rad_s,
            s.body_state('p0-output-axes')['angular_velocity_body_rad_s'])
        np.testing.assert_allclose(r.T @ [0, 0, -1],
            s.body_state('p0-output-axes')['projected_gravity'])

    def test_latest_only_original_timestamp_no_refresh(self):
        source = Bno055Source()
        source.latest = sample(1_000_000_000)
        got = []
        with patch('time.monotonic_ns', return_value=1_010_000_000):
            source.drain(lambda p, t: got.append((p, t)))
            source.drain(lambda p, t: got.append((p, t)))
        self.assertEqual(len(got), 2)
        self.assertEqual(got[0][1], 1_000_000_000)
        with patch('time.monotonic_ns', return_value=1_031_000_000):
            with self.assertRaisesRegex(RuntimeError, 'stale'):
                source.drain(lambda p, t: None)

    def test_worker_fault_never_delivers_cached_data(self):
        source = Bno055Source()
        source.latest = sample(time.monotonic_ns())
        source.error = 'disconnected'
        with self.assertRaisesRegex(RuntimeError, 'disconnected'):
            source.drain(lambda p, t: self.fail('must not deliver'))

    def test_no_implicit_configuration(self):
        class IO:
            closed = False
            def close(self): self.closed = True
        io = IO()
        with patch('open_sprite_runtime.bno055_source.BNO055') as sensor:
            sensor.return_value.inspect.return_value = dict(matches=False, system_error=0, system_status=0)
            with self.assertRaises(RuntimeError):
                with Bno055Source(io_factory=lambda *a: io): pass
            sensor.return_value.configure.assert_not_called()
            self.assertTrue(io.closed)


if __name__ == '__main__':
    unittest.main()
