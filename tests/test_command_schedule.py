import math
import unittest
from open_sprite_runtime.command_schedule import PROFILE_010, countdown, sample_stand_walk_stop


class CommandScheduleTests(unittest.TestCase):
    def test_010_profile_only_doubles_speed(self):
        for i in range(2501):
            t = i / 50
            old, stage = sample_stand_walk_stop(t)
            new, new_stage = sample_stand_walk_stop(t, PROFILE_010)
            self.assertEqual(new, 2 * old)
            self.assertEqual(stage, new_stage)
        with self.assertRaises(ValueError):
            sample_stand_walk_stop(0, 'unknown')

    def test_stages_and_boundaries(self):
        for t, vx, stage in [(0, 0, 'stand'), (9.99, 0, 'stand'),
                             (10, 0, 'ramp_forward'), (13, .05, 'forward'),
                             (26.99, .05, 'forward'), (27, .05, 'ramp_stop'),
                             (30, 0, 'final_stand'), (40, 0, 'final_stand'),
                             (100, 0, 'final_stand')]:
            self.assertEqual(sample_stand_walk_stop(t), (vx, stage))

    def test_bounds_and_final_ten_seconds(self):
        for i in range(2501):
            t = i / 50
            vx, _ = sample_stand_walk_stop(t)
            self.assertGreaterEqual(vx, 0)
            self.assertLessEqual(vx, .05)
            if t >= 30:
                self.assertEqual(vx, 0)

    def test_ramps_continuous_monotonic(self):
        up = [sample_stand_walk_stop(10 + i / 50)[0] for i in range(151)]
        down = [sample_stand_walk_stop(27 + i / 50)[0] for i in range(151)]
        self.assertEqual(up, sorted(up))
        self.assertEqual(down, sorted(down, reverse=True))
        self.assertAlmostEqual(up[75], .025)
        self.assertAlmostEqual(down[75], .025)

    def test_invalid_and_countdown(self):
        for t in (-1, math.nan, math.inf):
            with self.assertRaises(ValueError):
                sample_stand_walk_stop(t)
        self.assertIsNone(countdown(34.99))
        self.assertEqual(countdown(35), 5)
        self.assertEqual(countdown(39.1), 1)
        self.assertIsNone(countdown(40))


if __name__ == '__main__':
    unittest.main()
