import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from open_sprite_runtime.action_trace_replay import CenteredActionTraceReplay


class CenteredActionTraceReplayTests(unittest.TestCase):
    def test_loads_window_and_centers_position_delta(self) -> None:
        records = [{"action": [float(i)] * 31} for i in range(8)]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "trace.json"
            path.write_text(json.dumps(records), encoding="utf-8")
            replay = CenteredActionTraceReplay.load(
                path,
                np.full(31, 0.5),
                source_hz=2.0,
                start_seconds=1.0,
                duration_seconds=2.0,
                amplitude_scale=0.2,
            )

        self.assertEqual(replay.sample_count, 4)
        np.testing.assert_allclose(replay.position_delta(0), np.full(31, -0.15))
        np.testing.assert_allclose(replay.position_delta(3), np.full(31, 0.15))
        np.testing.assert_allclose(replay.position_delta(99), np.full(31, 0.15))

    def test_rejects_non_finite_or_invalid_scale(self) -> None:
        actions = np.zeros((2, 31))
        with self.assertRaises(ValueError):
            CenteredActionTraceReplay(actions, np.ones(31), 0.0)
        actions[0, 0] = np.nan
        with self.assertRaises(ValueError):
            CenteredActionTraceReplay(actions, np.ones(31), 0.2)


if __name__ == "__main__":
    unittest.main()
