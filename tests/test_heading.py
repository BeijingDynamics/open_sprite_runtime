import math
import unittest

from open_sprite_runtime.cli import load_heading_config
from open_sprite_runtime.heading import (
    HeadingCommandController,
    HeadingControllerConfig,
    wrap_to_pi,
)


class HeadingControllerTests(unittest.TestCase):
    def test_wrap_to_pi(self):
        self.assertAlmostEqual(wrap_to_pi(2.0 * math.pi + 0.2), 0.2)
        self.assertAlmostEqual(wrap_to_pi(-2.0 * math.pi - 0.2), -0.2)

    def test_stand_resets_heading(self):
        controller = HeadingCommandController()
        self.assertEqual(controller.update(1.2, standing=True), 0.0)
        self.assertAlmostEqual(float(controller.target_yaw), 1.2)
        self.assertAlmostEqual(controller.update(1.3), -0.05)

    def test_heading_correction_matches_isaac_and_clips(self):
        controller = HeadingCommandController()
        controller.reset(0.0)
        self.assertAlmostEqual(controller.update(-0.2), 0.1)
        self.assertAlmostEqual(controller.update(1.0), -0.2)

    def test_manual_turn_is_limited_and_reacquires_current_heading(self):
        controller = HeadingCommandController()
        controller.reset(0.0)
        self.assertAlmostEqual(controller.update(0.4, 0.15), 0.15)
        self.assertAlmostEqual(float(controller.target_yaw), 0.4)
        self.assertAlmostEqual(controller.update(0.4), 0.0)
        self.assertAlmostEqual(controller.update(0.5), -0.05)
        self.assertAlmostEqual(controller.update(0.5, -0.5), -0.2)

    def test_invalid_values_fail_closed(self):
        with self.assertRaises(ValueError):
            HeadingControllerConfig(stiffness=0.0).validate()
        controller = HeadingCommandController()
        with self.assertRaises(ValueError):
            controller.update(float("nan"))
        with self.assertRaises(ValueError):
            controller.update(0.0, float("inf"))

    def test_runtime_contract_rejects_global_yaw_or_missing_stand_reset(self):
        valid = {
            "heading_controller": {
                "stiffness": 0.5,
                "yaw_rate_limit_rad_s": 0.2,
                "manual_yaw_deadband_rad_s": 0.01,
                "stand_resets_heading": True,
                "actor_receives_global_yaw": False,
            }
        }
        self.assertEqual(load_heading_config(valid).stiffness, 0.5)
        valid["heading_controller"]["actor_receives_global_yaw"] = True
        with self.assertRaises(ValueError):
            load_heading_config(valid)
        valid["heading_controller"]["actor_receives_global_yaw"] = False
        valid["heading_controller"]["stand_resets_heading"] = False
        with self.assertRaises(ValueError):
            load_heading_config(valid)


if __name__ == "__main__":
    unittest.main()
