import math
import unittest

from open_sprite_runtime.session_heading import SessionHeading


class SessionHeadingTests(unittest.TestCase):
    def test_start_and_restart_zero(self):
        h = SessionHeading()
        h.update([0, 0, 1], [0, 0, -1], 1_000_000_000)
        self.assertEqual(h.yaw_rad, 0)
        h.update([0, 0, 1], [0, 0, -1], 1_020_000_000)
        self.assertAlmostEqual(h.yaw_rad, .02)
        h.reset()
        h.update([0, 0, 1], [0, 0, -1], 9_000_000_000)
        self.assertEqual(h.yaw_rad, 0)

    def test_tilt_correction_and_negative_direction(self):
        h = SessionHeading()
        r, p = .3, .2
        # Body angular velocity for pure world heading rate -1 rad/s.
        down = [math.sin(p), -math.sin(r)*math.cos(p), -math.cos(r)*math.cos(p)]
        h.update(down, down, 1_000_000_000)
        h.update(down, down, 1_020_000_000)
        self.assertAlmostEqual(h.yaw_rad, -.02)

    def test_gap_and_bad_input_invalid_not_silent_reset(self):
        for stamp in (1_000_000_000, 900_000_000, 1_100_000_000):
            h = SessionHeading()
            h.update([0, 0, 1], [0, 0, -1], 1_000_000_000)
            h.update([0, 0, 1], [0, 0, -1], stamp)
            self.assertFalse(h.valid)
        h = SessionHeading()
        h.update([float('nan'), 0, 0], [0, 0, -1], 1)
        self.assertFalse(h.valid)

    def test_no_command_effect_and_distinct_sessions(self):
        self.assertFalse(SessionHeading().report()['affects_policy_or_commands'])
        self.assertIsNone(SessionHeading().origin_ns)


if __name__ == '__main__':
    unittest.main()
