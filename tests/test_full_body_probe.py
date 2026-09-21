import struct
import unittest
from types import SimpleNamespace

from open_sprite_runtime.damiao import DamiaoFeedbackEndpoint, DamiaoMitRanges
from open_sprite_runtime.full_body_probe import collect_full_body_shadow
from open_sprite_runtime.socketcan import ReceivedCanFrame


def yahboom_frame(function: int, payload: bytes) -> bytes:
    frame = bytearray((0x7E, 0x23, len(payload) + 5, function))
    frame.extend(payload)
    frame.append(sum(frame) & 0xFF)
    return bytes(frame)


class FullBodyProbeTests(unittest.TestCase):
    def test_collects_four_buses_and_imu_without_nonzero_control(self) -> None:
        counts = {"kcan1": 8, "kcan2": 8, "kcan3": 8, "kcan4": 7}
        ranges = DamiaoMitRanges(
            (-12.5, 12.5),
            (-20.0, 20.0),
            (-28.0, 28.0),
            "motor_register_readback",
        )
        endpoints = tuple(
            DamiaoFeedbackEndpoint(
                motor_name=f"{interface}_motor_{can_id}",
                interface=interface,
                can_id=can_id,
                master_id=can_id + 0x10,
                ranges=ranges,
            )
            for interface, count in counts.items()
            for can_id in range(1, count + 1)
        )

        def feedback_frame(endpoint):
            return ReceivedCanFrame(
                interface=endpoint.interface,
                can_id=endpoint.master_id,
                data=bytes((endpoint.can_id, 0x80, 0, 0x80, 0x08, 0, 30, 31)),
                is_extended=False,
                is_remote=False,
                is_error=False,
                is_fd=True,
                bit_rate_switch=True,
                error_state_indicator=False,
                software_timestamp_ns=100,
                hardware_timestamp_ns=90,
                userspace_receive_timestamp_ns=110,
            )

        class Poller:
            def __init__(self, interface):
                self.interface = interface
                self.hardware_tx_attempts = 0
                self.frames = [
                    feedback_frame(item)
                    for item in endpoints
                    if item.interface == interface
                ]
                self.sent_payloads = []

            def fileno(self):
                return 1

            def send_zero_gain_poll(self, _can_id, data):
                self.hardware_tx_attempts += 1
                self.sent_payloads.append(data)

            def receive(self):
                return self.frames.pop(0)

        raw = yahboom_frame(0x04, struct.pack("<9h", *([0] * 9)))
        quaternion = yahboom_frame(0x16, struct.pack("<4f", 1.0, 0.0, 0.0, 0.0))

        class ImuPort:
            def __init__(self):
                self.payload = raw + quaternion

            @property
            def in_waiting(self):
                return len(self.payload)

            def fileno(self):
                return 2

            def read(self, _size):
                payload, self.payload = self.payload, b""
                return payload

        pollers = {interface: Poller(interface) for interface in counts}
        imu = ImuPort()

        class Selector:
            def __init__(self):
                self.items = []

            def __enter__(self):
                return self

            def __exit__(self, *_args):
                return None

            def register(self, _fileobj, _events, data):
                self.items.append(data)

            def select(self, timeout):
                ready = []
                for data in self.items:
                    kind, _interface, source = data
                    if (kind == "can" and source.frames) or (
                        kind == "imu" and source.in_waiting
                    ):
                        ready.append((SimpleNamespace(data=data), 1))
                return ready

        class Clock:
            value = 0.0

            def __call__(self):
                self.value += 0.00005
                return self.value

        report = collect_full_body_shadow(
            pollers,
            endpoints,
            imu,
            duration_s=0.01,
            rate_hz_per_motor=50.0,
            minimum_motor_sample_coverage=0.9,
            minimum_imu_rate_hz=50.0,
            selector_factory=Selector,
            monotonic=Clock(),
        )
        self.assertTrue(report.passed, report.errors)
        self.assertEqual(sum(report.rx_count_by_motor.values()), 31)
        self.assertEqual(report.imu_raw_count, 1)
        self.assertEqual(report.imu_quaternion_count, 1)
        self.assertEqual(report.serial_write_count, 0)
        for poller in pollers.values():
            self.assertGreater(poller.hardware_tx_attempts, 0)
            for payload in poller.sent_payloads:
                kp_raw = ((payload[3] & 0x0F) << 8) | payload[4]
                kd_raw = (payload[5] << 4) | (payload[6] >> 4)
                torque_raw = ((payload[6] & 0x0F) << 8) | payload[7]
                self.assertEqual(kp_raw, 0)
                self.assertEqual(kd_raw, 0)
                self.assertIn(torque_raw, (2047, 2048))

    def test_requires_exactly_31_motors(self) -> None:
        with self.assertRaisesRegex(ValueError, "requires 31 motors"):
            collect_full_body_shadow({}, (), SimpleNamespace(), 1.0, 50.0)


if __name__ == "__main__":
    unittest.main()
