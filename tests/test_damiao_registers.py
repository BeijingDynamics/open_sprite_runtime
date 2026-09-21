import struct
import unittest

from open_sprite_runtime.damiao import DamiaoFeedbackEndpoint, DamiaoMitRanges
from open_sprite_runtime.damiao_registers import (
    build_read_request,
    collect_mit_range_registers,
    decode_read_response,
)
from open_sprite_runtime.socketcan import ReceivedCanFrame


def response(master_id: int, register_id: int, value: float) -> ReceivedCanFrame:
    return ReceivedCanFrame(
        interface="kcan1",
        can_id=master_id,
        data=bytes((master_id, 0, 0x33, register_id)) + struct.pack("<f", value),
        is_extended=False,
        is_remote=False,
        is_error=False,
        is_fd=True,
        bit_rate_switch=True,
        error_state_indicator=False,
        software_timestamp_ns=1,
        hardware_timestamp_ns=1,
        userspace_receive_timestamp_ns=2,
    )


class DamiaoRegisterTests(unittest.TestCase):
    def test_builds_vendor_read_frame_only_for_mit_ranges(self) -> None:
        self.assertEqual(build_read_request(8, 23), bytes((8, 0, 0x33, 23, 0, 0, 0, 0)))
        with self.assertRaisesRegex(ValueError, "21/22/23"):
            build_read_request(8, 10)

    def test_decodes_little_endian_float_with_identity_checks(self) -> None:
        decoded = decode_read_response(
            response(0x18, 22, 20.0), expected_master_id=0x18, expected_register_id=22
        )
        self.assertEqual(decoded.register_name, "VMAX")
        self.assertEqual(decoded.value, 20.0)
        with self.assertRaisesRegex(ValueError, "RID mismatch"):
            decode_read_response(
                response(0x18, 21, 12.5),
                expected_master_id=0x18,
                expected_register_id=23,
            )

    def test_collects_exactly_three_read_only_registers(self) -> None:
        endpoint = DamiaoFeedbackEndpoint(
            motor_name="motor",
            interface="kcan1",
            can_id=1,
            master_id=0x11,
            ranges=DamiaoMitRanges(
                position_rad=(-12.5, 12.5),
                velocity_rad_s=(-20.0, 20.0),
                torque_nm=(-28.0, 28.0),
                source="operator_confirmed_drive_configuration",
            ),
        )

        class Reader:
            hardware_tx_attempts = 0

            def __init__(self) -> None:
                self.frames = []

            def send_read_request(self, _can_id: int, register_id: int) -> None:
                self.hardware_tx_attempts += 1
                self.frames.append(response(0x11, register_id, {21: 12.5, 22: 20.0, 23: 28.0}[register_id]))

            def receive(self):
                return self.frames.pop(0)

        reader = Reader()
        report = collect_mit_range_registers({"kcan1": reader}, (endpoint,))
        self.assertEqual(report["tx_count"], 3)
        self.assertEqual(report["write_register_attempts"], 0)
        self.assertEqual(report["motors"]["motor"], {"PMAX": 12.5, "VMAX": 20.0, "TMAX": 28.0})


if __name__ == "__main__":
    unittest.main()
