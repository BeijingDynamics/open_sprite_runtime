import unittest

import numpy as np

from open_sprite_runtime.shadow import expand_zero_order_hold


class MultirateShadowTests(unittest.TestCase):
    def test_zero_order_hold_expands_each_policy_target_ten_times(self) -> None:
        actions = np.asarray([[1.0, 2.0], [3.0, 4.0]])
        expanded = expand_zero_order_hold(actions, 10)
        self.assertEqual(expanded.shape, (20, 2))
        np.testing.assert_array_equal(expanded[:10], np.tile(actions[0], (10, 1)))
        np.testing.assert_array_equal(expanded[10:], np.tile(actions[1], (10, 1)))

    def test_invalid_shape_and_rate_are_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "2-D"):
            expand_zero_order_hold(np.zeros(3), 10)
        with self.assertRaisesRegex(ValueError, "positive"):
            expand_zero_order_hold(np.zeros((1, 3)), 0)


if __name__ == "__main__":
    unittest.main()
