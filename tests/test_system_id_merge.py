import unittest

import numpy as np

from open_sprite_runtime.system_id_merge import merge_system_id_traces


def encode(value, low, high, bits):
    return int((value - low) / (high - low) * ((1 << bits) - 1))


class SystemIdMergeTests(unittest.TestCase):
    def test_aligns_feedback_and_exact_transmitted_command(self) -> None:
        p = encode(0.5, -12.5, 12.5, 16)
        v = encode(1.0, -20.0, 20.0, 12)
        tau = encode(2.0, -28.0, 28.0, 12)
        feedback = bytes((0x11, p >> 8, p & 0xFF, v >> 4, ((v & 0xF) << 4) | (tau >> 8), tau & 0xFF, 31, 30))
        kp = encode(10.0, 0.0, 500.0, 12)
        kd = encode(1.0, 0.0, 5.0, 12)
        command = bytes((p >> 8, p & 0xFF, v >> 4, ((v & 0xF) << 4) | (kp >> 8), kp & 0xFF, kd >> 4, ((kd & 0xF) << 4) | (tau >> 8), tau & 0xFF))
        payload = np.zeros((4, 64), dtype=np.uint8)
        payload[0, :8] = np.frombuffer(command, dtype=np.uint8)
        payload[1, :8] = np.frombuffer(feedback, dtype=np.uint8)
        payload[2, :8] = np.frombuffer(command, dtype=np.uint8)
        payload[3, :8] = np.frombuffer(feedback, dtype=np.uint8)
        can_trace = {
            "interface_names": np.asarray(["kcan1"]),
            "monotonic_receive_ns": np.asarray([90, 95, 190, 195], dtype=np.int64),
            "interface_index": np.zeros(4, dtype=np.uint8),
            "can_id": np.asarray([1, 17, 1, 17]),
            "payload_length": np.full(4, 8),
            "payload": payload,
        }
        policy_trace = {"state_monotonic_ns": np.asarray([100, 200], dtype=np.int64)}
        hardware = {
            "can_adapter": {"interfaces": ["kcan1"]},
            "motor_map": {
                "motor": {
                    "can_channel": 0,
                    "can_id": 1,
                    "master_id": 17,
                    "mit_ranges": {
                        "position_rad": [-12.5, 12.5],
                        "velocity_rad_s": [-20.0, 20.0],
                        "torque_nm": [-28.0, 28.0],
                    },
                }
            },
        }
        merged, report = merge_system_id_traces(can_trace, policy_trace, hardware)
        self.assertTrue(report["passed"], report["errors"])
        self.assertAlmostEqual(merged["feedback_position_rad"][1, 0], 0.5, places=3)
        self.assertAlmostEqual(merged["feedback_estimated_torque_nm"][0, 0], 2.0, places=1)
        self.assertAlmostEqual(merged["command_kp"][0, 0], 10.0, delta=500.0 / 4095.0)
        self.assertAlmostEqual(merged["command_kd"][1, 0], 1.0, places=2)
        self.assertEqual(merged["feedback_mos_temperature_c"][0, 0], 31)
