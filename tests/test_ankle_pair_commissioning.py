import unittest

import numpy as np

from open_sprite_runtime.ankle_pair_commissioning import (
    ANKLE_GROUPS,
    HEAD_GROUP,
    run_ankle_pair_joint_pd_gate,
    run_ankle_pitch_direction_gate,
    run_ankle_pair_zero_torque_gate,
    run_head_pair_joint_pd_gate,
)
from open_sprite_runtime.damiao import DamiaoFeedbackEndpoint, DamiaoMitRanges
from open_sprite_runtime.motor_mapping import DifferentialPairDriveMap
from open_sprite_runtime.ankle import DifferentialPair
from open_sprite_runtime.socketcan import ReceivedCanFrame


class Clock:
    def __init__(self):
        self.value = 0.0

    def __call__(self):
        self.value += 0.00001
        return self.value

    def sleep(self, seconds):
        self.value += seconds


class Writer:
    def __init__(self, endpoints):
        self.interface = endpoints[0].interface
        self.endpoints = {item.motor_name: item for item in endpoints}
        self.status = {name: 0 for name in self.endpoints}
        self.last_name = endpoints[0].motor_name
        self.command_tx_attempts = {name: 0 for name in self.endpoints}
        self.enable_attempts = {name: 0 for name in self.endpoints}
        self.disable_attempts = {name: 0 for name in self.endpoints}

    def send_command(self, command):
        self.last_name = command.motor_name
        self.command_tx_attempts[self.last_name] += 1

    def send_enable(self, name):
        self.status[name] = 1
        self.enable_attempts[name] += 1

    def send_disable(self, name):
        self.status[name] = 0
        self.disable_attempts[name] += 1

    def receive(self):
        endpoint = self.endpoints[self.last_name]
        data = bytes(((self.status[self.last_name] << 4) | endpoint.can_id,
                      0x80, 0x00, 0x80, 0x08, 0x00, 30, 29))
        return ReceivedCanFrame(
            interface=self.interface, can_id=endpoint.master_id, data=data,
            is_extended=False, is_remote=False, is_error=False, is_fd=True,
            bit_rate_switch=True, error_state_indicator=False,
            software_timestamp_ns=1, hardware_timestamp_ns=1,
            userspace_receive_timestamp_ns=2,
        )


def fixture(side="left"):
    identity = ANKLE_GROUPS[side]
    ranges = DamiaoMitRanges(
        (-12.5, 12.5), (-50.0, 50.0), (-10.0, 10.0), "motor_register_readback"
    )
    endpoints = tuple(
        DamiaoFeedbackEndpoint(name, interface, can_id, master_id, ranges)
        for name, interface, can_id, master_id in identity
    )
    prefix = f"{side}_ankle"
    pair = DifferentialPairDriveMap(
        name=prefix,
        joint_names=(f"{prefix}_pitch_joint", f"{prefix}_roll_joint"),
        motor_names=tuple(item.motor_name for item in endpoints),
        drive_zero_rad=np.zeros(2),
        encoder_sign=np.ones(2),
        coupling=DifferentialPair(np.asarray([[1.3, -1.0], [-1.3, -1.0]]), np.zeros(2)),
    )
    return endpoints, pair


def head_fixture():
    ranges = DamiaoMitRanges(
        (-12.566, 12.566), (-50.0, 50.0), (-5.0, 5.0), "motor_register_readback"
    )
    endpoints = tuple(
        DamiaoFeedbackEndpoint(name, interface, can_id, master_id, ranges)
        for name, interface, can_id, master_id in HEAD_GROUP
    )
    pair = DifferentialPairDriveMap(
        name="head",
        joint_names=("head_pitch_joint", "head_roll_joint"),
        motor_names=tuple(item.motor_name for item in endpoints),
        drive_zero_rad=np.zeros(2),
        encoder_sign=np.ones(2),
        coupling=DifferentialPair(
            np.asarray([[1.0047, 0.6148], [-1.0047, 0.6148]]), np.zeros(2)
        ),
    )
    return endpoints, pair


class AnklePairCommissioningTests(unittest.TestCase):
    def test_zero_torque_gate_runs_500hz_and_disables_pair(self):
        endpoints, pair = fixture()
        writer = Writer(endpoints)
        clock = Clock()
        report = run_ankle_pair_zero_torque_gate(
            writer, endpoints, pair, side="left",
            soft_position_rad={item.motor_name: (-1.0, 1.0) for item in endpoints},
            monotonic=clock, sleep=clock.sleep,
        )
        self.assertTrue(report.passed, report.errors)
        self.assertTrue(all(value >= 950 for value in report.command_count.values()))
        self.assertEqual(set(report.initial_joint_position_rad), set(pair.joint_names))
        self.assertTrue(all(value == "disabled" for value in report.final_status.values()))

    def test_wrong_pair_or_soft_limit_blocks_enable(self):
        endpoints, pair = fixture()
        writer = Writer(endpoints)
        clock = Clock()
        limits = {item.motor_name: (-1.0, 1.0) for item in endpoints}
        limits[endpoints[0].motor_name] = (0.1, 1.0)
        report = run_ankle_pair_zero_torque_gate(
            writer, endpoints, pair, side="left", soft_position_rad=limits,
            monotonic=clock, sleep=clock.sleep,
        )
        self.assertFalse(report.passed)
        self.assertTrue(all(value == 0 for value in report.enable_attempts.values()))
        with self.assertRaisesRegex(ValueError, "frozen ordered differential"):
            run_ankle_pair_zero_torque_gate(
                Writer(tuple(reversed(endpoints))), tuple(reversed(endpoints)), pair,
                side="left", soft_position_rad=limits,
            )

    def test_joint_pd_gate_runs_with_host_torque_mapping_and_disables(self):
        endpoints, pair = fixture()
        writer = Writer(endpoints)
        clock = Clock()
        report = run_ankle_pair_joint_pd_gate(
            writer, endpoints, pair, side="left",
            soft_position_rad={item.motor_name: (-1.0, 1.0) for item in endpoints},
            monotonic=clock, sleep=clock.sleep,
        )
        self.assertTrue(report.passed, report.errors)
        self.assertTrue(all(value >= 950 for value in report.command_count.values()))
        self.assertEqual(report.joint_kp_nm_rad, 0.5)
        self.assertEqual(report.maximum_joint_torque_nm, 0.15)
        self.assertTrue(all(value == "disabled" for value in report.final_status.values()))

    def test_head_pair_uses_lower_torque_envelope_and_disables(self):
        endpoints, pair = head_fixture()
        writer = Writer(endpoints)
        clock = Clock()
        report = run_head_pair_joint_pd_gate(
            writer,
            endpoints,
            pair,
            soft_position_rad={item.motor_name: (-1.0, 1.0) for item in endpoints},
            monotonic=clock,
            sleep=clock.sleep,
        )
        self.assertTrue(report.passed, report.errors)
        self.assertEqual(report.side, "head")
        self.assertEqual(report.joint_kp_nm_rad, 0.2)
        self.assertEqual(report.maximum_joint_torque_nm, 0.05)
        self.assertTrue(all(value == "disabled" for value in report.final_status.values()))

    def test_pitch_direction_gate_fails_closed_without_bidirectional_response(self):
        endpoints, pair = fixture("right")
        writer = Writer(endpoints)
        clock = Clock()
        report = run_ankle_pitch_direction_gate(
            writer,
            endpoints,
            pair,
            side="right",
            soft_position_rad={item.motor_name: (-1.0, 1.0) for item in endpoints},
            monotonic=clock,
            sleep=clock.sleep,
        )
        self.assertFalse(report.passed)
        self.assertIn(
            "positive pitch command produced no qualified positive response",
            report.errors,
        )
        self.assertIn(
            "negative pitch command produced no qualified negative response",
            report.errors,
        )
        self.assertTrue(all(value == "disabled" for value in report.final_status.values()))

    def test_pitch_direction_gate_rejects_wrong_endpoint_order(self):
        endpoints, pair = fixture("right")
        with self.assertRaisesRegex(ValueError, "frozen ordered pair"):
            run_ankle_pitch_direction_gate(
                Writer(tuple(reversed(endpoints))),
                tuple(reversed(endpoints)),
                pair,
                side="right",
                soft_position_rad={item.motor_name: (-1.0, 1.0) for item in endpoints},
            )

    def test_observable_pitch_tier_uses_bounded_higher_excitation(self):
        endpoints, pair = fixture("right")
        writer = Writer(endpoints)
        clock = Clock()
        report = run_ankle_pitch_direction_gate(
            writer,
            endpoints,
            pair,
            side="right",
            soft_position_rad={item.motor_name: (-1.0, 1.0) for item in endpoints},
            excitation_tier="observable",
            monotonic=clock,
            sleep=clock.sleep,
        )
        self.assertFalse(report.passed)
        self.assertEqual(report.joint_kp_nm_rad, 16.0)
        self.assertEqual(report.maximum_joint_torque_nm, 0.5)
        self.assertLessEqual(max(report.maximum_abs_motor_torque_command_nm.values()), 0.3)
        self.assertTrue(all(value == "disabled" for value in report.final_status.values()))

    def test_pitch_direction_gate_rejects_unknown_excitation_tier(self):
        endpoints, pair = fixture("right")
        with self.assertRaisesRegex(ValueError, "unknown ankle pitch-direction"):
            run_ankle_pitch_direction_gate(
                Writer(endpoints),
                endpoints,
                pair,
                side="right",
                soft_position_rad={item.motor_name: (-1.0, 1.0) for item in endpoints},
                excitation_tier="unbounded",
            )

    def test_negative_slow_pitch_tier_requires_only_negative_response(self):
        endpoints, pair = fixture("right")
        writer = Writer(endpoints)
        clock = Clock()
        report = run_ankle_pitch_direction_gate(
            writer,
            endpoints,
            pair,
            side="right",
            soft_position_rad={item.motor_name: (-1.0, 1.0) for item in endpoints},
            excitation_tier="negative-slow",
            monotonic=clock,
            sleep=clock.sleep,
        )
        self.assertFalse(report.passed)
        self.assertEqual(report.required_response_directions, ("negative",))
        self.assertEqual(report.duration_s, 5.0)
        self.assertNotIn(
            "positive pitch command produced no qualified positive response",
            report.errors,
        )
        self.assertIn(
            "negative pitch command produced no qualified negative response",
            report.errors,
        )
        self.assertLessEqual(max(report.maximum_abs_motor_torque_command_nm.values()), 0.3)
        self.assertTrue(all(value == "disabled" for value in report.final_status.values()))


if __name__ == "__main__":
    unittest.main()
