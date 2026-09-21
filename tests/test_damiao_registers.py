import struct
import unittest

from open_sprite_runtime.damiao import DamiaoFeedbackEndpoint, DamiaoMitRanges
from open_sprite_runtime.damiao_registers import (
    build_read_request,
    apply_mit_range_readback,
    collect_commissioning_registers,
    collect_mit_range_registers,
    decode_commissioning_read_response,
    decode_read_response,
)
from open_sprite_runtime.socketcan import ReceivedCanFrame


def response(
    master_id: int, register_id: int, value: float, *, command_can_id: int | None = None
) -> ReceivedCanFrame:
    payload_id = master_id if command_can_id is None else command_can_id
    return ReceivedCanFrame(
        interface="kcan1",
        can_id=master_id,
        data=bytes((payload_id, 0, 0x33, register_id)) + struct.pack("<f", value),
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
    def test_decodes_commissioning_float_and_integer_registers(self) -> None:
        temperature = decode_commissioning_read_response(
            response(0x11, 2, 80.0, command_can_id=1),
            expected_command_can_id=1,
            expected_master_id=0x11,
            expected_register_id=2,
        )
        self.assertEqual(temperature.register_name, "OT_Value")
        self.assertEqual(temperature.value, 80.0)
        version_frame = response(0x11, 13, 0.0, command_can_id=1)
        version_frame = ReceivedCanFrame(
            **{**version_frame.__dict__, "data": bytes((1, 0, 0x33, 13)) + struct.pack("<I", 42)}
        )
        version = decode_commissioning_read_response(
            version_frame,
            expected_command_can_id=1,
            expected_master_id=0x11,
            expected_register_id=13,
        )
        self.assertEqual(version.register_name, "hw_ver")
        self.assertEqual(version.value, 42)

    def test_builds_vendor_read_frame_only_for_mit_ranges(self) -> None:
        self.assertEqual(build_read_request(8, 23), bytes((8, 0, 0x33, 23, 0, 0, 0, 0)))
        with self.assertRaisesRegex(ValueError, "21/22/23"):
            build_read_request(8, 10)

    def test_decodes_little_endian_float_with_identity_checks(self) -> None:
        decoded = decode_read_response(
            response(0x18, 22, 20.0, command_can_id=8),
            expected_command_can_id=8,
            expected_master_id=0x18,
            expected_register_id=22,
        )
        self.assertEqual(decoded.register_name, "VMAX")
        self.assertEqual(decoded.value, 20.0)
        with self.assertRaisesRegex(ValueError, "RID mismatch"):
            decode_read_response(
                response(0x18, 21, 12.5, command_can_id=8),
                expected_command_can_id=8,
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
                self.frames.append(
                    response(
                        0x11,
                        register_id,
                        {21: 12.5, 22: 20.0, 23: 28.0}[register_id],
                        command_can_id=1,
                    )
                )

            def receive(self):
                return self.frames.pop(0)

        reader = Reader()
        report = collect_mit_range_registers({"kcan1": reader}, (endpoint,))
        self.assertEqual(report["tx_count"], 3)
        self.assertEqual(report["write_register_attempts"], 0)
        self.assertEqual(report["motors"]["motor"], {"PMAX": 12.5, "VMAX": 20.0, "TMAX": 28.0})

    def test_collects_only_six_commissioning_registers(self) -> None:
        endpoint = DamiaoFeedbackEndpoint(
            motor_name="motor", interface="kcan1", can_id=1, master_id=0x11,
            ranges=DamiaoMitRanges(
                position_rad=(-12.5, 12.5), velocity_rad_s=(-20.0, 20.0),
                torque_nm=(-28.0, 28.0), source="motor_register_readback",
            ),
        )
        raw_values = {2: 80.0, 3: 20.0, 6: 20.0, 13: 1, 14: 2, 36: 3}

        class Reader:
            hardware_tx_attempts = 0

            def __init__(self) -> None:
                self.frames = []

            def send_read_request(self, _can_id: int, register_id: int) -> None:
                self.hardware_tx_attempts += 1
                frame = response(0x11, register_id, float(raw_values[register_id]), command_can_id=1)
                if register_id in (13, 14, 36):
                    frame = ReceivedCanFrame(
                        **{**frame.__dict__, "data": bytes((1, 0, 0x33, register_id)) + struct.pack("<I", raw_values[register_id])}
                    )
                self.frames.append(frame)

            def receive(self):
                return self.frames.pop(0)

        report = collect_commissioning_registers({"kcan1": Reader()}, (endpoint,))
        self.assertEqual(report["tx_count"], 6)
        self.assertEqual(report["allowed_register_ids"], [2, 3, 6, 13, 14, 36])
        self.assertEqual(report["write_register_attempts"], 0)
        self.assertEqual(report["motors"]["motor"]["sub_ver"], 3)

    def test_applies_readback_without_changing_nameplate_torque(self) -> None:
        hardware = {
            "motor_map": {
                "motor": {
                    "peak_torque_nm": 40.0,
                    "mit_ranges": {"source": "operator_confirmed_drive_configuration"},
                }
            }
        }
        report = {
            "mode": "read_only_damiao_mit_range_register_audit",
            "passed": True,
            "write_register_attempts": 0,
            "enable_attempts": 0,
            "mode_switch_attempts": 0,
            "motors": {"motor": {"PMAX": 12.5, "VMAX": 20.0, "TMAX": 28.0}},
        }
        self.assertEqual(apply_mit_range_readback(hardware, report, "report.json"), 1)
        motor = hardware["motor_map"]["motor"]
        self.assertEqual(motor["peak_torque_nm"], 40.0)
        self.assertEqual(motor["mit_ranges"]["torque_nm"], [-28.0, 28.0])
        self.assertEqual(motor["mit_ranges"]["source"], "motor_register_readback")


if __name__ == "__main__":
    unittest.main()
