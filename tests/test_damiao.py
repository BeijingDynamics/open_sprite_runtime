import unittest

from open_sprite_runtime.damiao import (
    DamiaoFeedbackDecoder,
    DamiaoFeedbackEndpoint,
    DamiaoMitRanges,
    DamiaoRxAudit,
    decode_damiao_feedback,
    endpoints_from_hardware_config,
)
from open_sprite_runtime.socketcan import ReceivedCanFrame


def ranges() -> DamiaoMitRanges:
    return DamiaoMitRanges(
        position_rad=(-12.5, 12.5),
        velocity_rad_s=(-20.0, 20.0),
        torque_nm=(-28.0, 28.0),
        source="motor_register_readback",
    )


def endpoint(**changes) -> DamiaoFeedbackEndpoint:
    values = {
        "motor_name": "left_shoulder_pitch_motor",
        "interface": "can2",
        "can_id": 0x03,
        "master_id": 0x13,
        "ranges": ranges(),
    }
    values.update(changes)
    return DamiaoFeedbackEndpoint(**values)


def frame(data: bytes, **changes) -> ReceivedCanFrame:
    values = {
        "interface": "can2",
        "can_id": 0x13,
        "data": data,
        "is_extended": False,
        "is_remote": False,
        "is_error": False,
        "is_fd": False,
        "bit_rate_switch": False,
        "error_state_indicator": False,
        "software_timestamp_ns": 123456700,
        "hardware_timestamp_ns": 123456789,
        "userspace_receive_timestamp_ns": 123456800,
    }
    values.update(changes)
    return ReceivedCanFrame(**values)


class DamiaoFeedbackTests(unittest.TestCase):
    def test_decodes_official_v14_layout_and_temperatures(self) -> None:
        # enabled + CAN ID 3, zero position/velocity/torque, 42 C MOS, 37 C rotor
        data = bytes([0x13, 0x80, 0x00, 0x80, 0x08, 0x00, 42, 37])
        state = decode_damiao_feedback(frame(data), endpoint())
        self.assertEqual(state.status_name, "enabled")
        self.assertAlmostEqual(state.position_rad, 12.5 / 65535.0, places=6)
        self.assertAlmostEqual(state.velocity_rad_s, 20.0 / 4095.0, places=6)
        self.assertAlmostEqual(state.estimated_output_torque_nm, 28.0 / 4095.0, places=6)
        self.assertEqual(state.mos_temperature_c, 42)
        self.assertEqual(state.rotor_temperature_c, 37)
        self.assertEqual(state.hardware_timestamp_ns, 123456789)

    def test_fault_code_is_preserved(self) -> None:
        state = decode_damiao_feedback(
            frame(bytes([0xC3, 0, 0, 0, 0, 0, 90, 99])), endpoint()
        )
        self.assertEqual(state.status_code, 0xC)
        self.assertEqual(state.status_name, "motor_coil_over_temperature")

    def test_rejects_wrong_master_id_controller_id_and_short_frame(self) -> None:
        with self.assertRaisesRegex(ValueError, "Master ID"):
            decode_damiao_feedback(frame(bytes(8), can_id=0x14), endpoint())
        with self.assertRaisesRegex(ValueError, "controller ID"):
            decode_damiao_feedback(frame(bytes([0x14]) + bytes(7)), endpoint())
        with self.assertRaisesRegex(ValueError, "eight"):
            decode_damiao_feedback(frame(bytes(6)), endpoint())

    def test_rejects_unverified_ranges(self) -> None:
        with self.assertRaisesRegex(ValueError, "motor_register_readback"):
            DamiaoMitRanges((-12.5, 12.5), (-20, 20), (-28, 28), "sdk_default")

    def test_decoder_maps_by_interface_and_master_id(self) -> None:
        decoder = DamiaoFeedbackDecoder([endpoint()])
        state = decoder.decode(frame(bytes([0x03]) + bytes(7)))
        self.assertEqual(state.motor_name, "left_shoulder_pitch_motor")
        with self.assertRaisesRegex(ValueError, "unconfigured"):
            decoder.decode(frame(bytes(8), interface="can1"))

    def test_decoder_has_no_transmit_method(self) -> None:
        decoder = DamiaoFeedbackDecoder([endpoint()])
        self.assertFalse(hasattr(decoder, "send"))
        self.assertFalse(hasattr(decoder, "enable"))

    def test_builds_endpoints_from_explicit_hardware_mapping(self) -> None:
        hardware = {
            "can_adapter": {"interfaces": ["can0", "can1", "can2", "can3"]},
            "motor_map": {
                "left_shoulder_pitch_motor": {
                    "can_channel": 2,
                    "can_id": 3,
                    "master_id": 0x13,
                    "mit_ranges": {
                        "position_rad": [-12.5, 12.5],
                        "velocity_rad_s": [-20, 20],
                        "torque_nm": [-28, 28],
                        "source": "motor_register_readback",
                    },
                }
            },
        }
        built = endpoints_from_hardware_config(hardware)
        self.assertEqual(built, (endpoint(),))

    def test_rx_audit_passes_complete_500hz_trace(self) -> None:
        audit = DamiaoRxAudit([endpoint()], minimum_samples_per_motor=3)
        for index in range(3):
            timestamp = 1_000_000_000 + index * 2_000_000
            audit.ingest(
                frame(
                    bytes([0x03, 0x80, 0, 0x80, 0x08, 0, 30, 31]),
                    hardware_timestamp_ns=timestamp,
                    software_timestamp_ns=2_000_000_000 + index * 2_000_000,
                    userspace_receive_timestamp_ns=2_000_200_000 + index * 2_000_000,
                )
            )
        report = audit.report()
        self.assertTrue(report.passed, report.errors)
        self.assertEqual(report.hardware_tx_attempts, 0)
        self.assertAlmostEqual(
            report.motor_stats["left_shoulder_pitch_motor"].feedback_hz, 500.0
        )

    def test_rx_audit_rejects_fault_gap_queue_delay_and_missing_motor(self) -> None:
        second = endpoint(
            motor_name="right_shoulder_pitch_motor",
            interface="can3",
            can_id=4,
            master_id=0x14,
        )
        audit = DamiaoRxAudit(
            [endpoint(), second], minimum_samples_per_motor=2
        )
        audit.ingest(frame(bytes([0xC3]) + bytes(7)))
        audit.ingest(
            frame(
                bytes([0x03]) + bytes(7),
                hardware_timestamp_ns=133456789,
                userspace_receive_timestamp_ns=140456800,
            )
        )
        report = audit.report()
        self.assertFalse(report.passed)
        self.assertEqual(report.missing_motors, ("right_shoulder_pitch_motor",))
        self.assertTrue(any("motor fault" in error for error in report.errors))
        self.assertTrue(any("hardware timestamp gap" in error for error in report.errors))
        self.assertTrue(any("queue age P99" in error for error in report.errors))


if __name__ == "__main__":
    unittest.main()
