import unittest
from types import SimpleNamespace

from open_sprite_runtime.damiao import (
    DamiaoMitCommand,
    DamiaoMitCommandEnvelope,
    DamiaoMitState,
    DamiaoFeedbackDecoder,
    DamiaoFeedbackEndpoint,
    DamiaoMitRanges,
    DamiaoRxAudit,
    collect_receive_only_audit,
    collect_zero_gain_group_position_echo,
    collect_zero_gain_position_echo,
    decode_damiao_feedback,
    endpoints_from_hardware_config,
    encode_damiao_mit_command,
    encode_zero_gain_position_echo,
)
from open_sprite_runtime.socketcan import ReceivedCanFrame, SocketCanTimestampError


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
    def test_zero_gain_group_requires_one_bus_and_at_most_eight_motors(self) -> None:
        poller = SimpleNamespace(
            interface="can2",
            send_zero_gain_poll=lambda *_args: None,
        )
        configured = [
            endpoint(
                motor_name=f"motor_{index}",
                can_id=index + 1,
                master_id=index + 0x11,
            )
            for index in range(9)
        ]
        with self.assertRaisesRegex(ValueError, "between one and eight"):
            collect_zero_gain_group_position_echo(
                poller, configured, configured, 1.0, 50.0
            )

        mixed = (
            configured[0],
            endpoint(
                motor_name="other_bus",
                interface="can3",
                can_id=2,
                master_id=0x12,
            ),
        )
        with self.assertRaisesRegex(ValueError, "poller interface"):
            collect_zero_gain_group_position_echo(
                poller, mixed, mixed, 1.0, 50.0
            )

    def test_zero_gain_group_allows_500hz_only_with_valid_remaining_limits(self) -> None:
        selected = endpoint()
        poller = SimpleNamespace(
            interface=selected.interface,
            send_zero_gain_poll=lambda *_args: None,
        )
        with self.assertRaisesRegex(ValueError, "feedback_timeout_s"):
            collect_zero_gain_group_position_echo(
                poller,
                [selected],
                [selected],
                1.0,
                500.0,
                feedback_timeout_s=2.0,
            )
        with self.assertRaisesRegex(ValueError, r"\(0, 500\]"):
            collect_zero_gain_group_position_echo(
                poller, [selected], [selected], 1.0, 500.1
            )

    def test_zero_gain_group_discards_one_untrusted_timestamp_frame(self) -> None:
        selected = endpoint()

        class Poller:
            interface = selected.interface
            hardware_tx_attempts = 0

            def __init__(self):
                self.items = [
                    SocketCanTimestampError("kernel software RX timestamp is missing"),
                    frame(
                        bytes([0x03, 0x80, 0, 0x80, 0x08, 0, 30, 31]),
                        is_fd=True,
                        bit_rate_switch=True,
                    ),
                ]

            def send_zero_gain_poll(self, *_args):
                self.hardware_tx_attempts += 1

            def receive(self):
                item = self.items.pop(0)
                if isinstance(item, BaseException):
                    raise item
                return item

        class Selector:
            def __enter__(self):
                return self

            def __exit__(self, *_args):
                return None

            def register(self, _fileobj, _events, data):
                self.data = data

            def select(self, timeout):
                if self.data.items:
                    return [(SimpleNamespace(data=self.data), 1)]
                return []

        class Clock:
            value = 0.0

            def __call__(self):
                self.value += 0.001
                return self.value

        report = collect_zero_gain_group_position_echo(
            Poller(),
            [selected],
            [selected],
            0.02,
            50.0,
            selector_factory=Selector,
            monotonic=Clock(),
        )
        self.assertTrue(report.passed, report.errors)
        self.assertEqual(report.discarded_timestamp_frames, 1)
        self.assertEqual(report.rx_count_by_motor[selected.motor_name], 1)

    def test_zero_gain_probe_extended_duration_requires_explicit_limit(self) -> None:
        poller = SimpleNamespace(
            interface="wrong",
            send_zero_gain_poll=lambda *_args: None,
        )
        with self.assertRaisesRegex(ValueError, r"\(0, 10\]"):
            collect_zero_gain_position_echo(
                poller, endpoint(), [endpoint()], 120.0, 50.0
            )
        with self.assertRaisesRegex(ValueError, "poller interface"):
            collect_zero_gain_position_echo(
                poller,
                endpoint(),
                [endpoint()],
                120.0,
                50.0,
                maximum_duration_s=120.0,
            )

    def test_zero_gain_position_echo_keeps_only_position_nonzero(self) -> None:
        encoded = encode_zero_gain_position_echo(endpoint(), 1.25)
        velocity_raw = (encoded.data[2] << 4) | (encoded.data[3] >> 4)
        kp_raw = ((encoded.data[3] & 0x0F) << 8) | encoded.data[4]
        kd_raw = (encoded.data[5] << 4) | (encoded.data[6] >> 4)
        torque_raw = ((encoded.data[6] & 0x0F) << 8) | encoded.data[7]
        self.assertEqual(velocity_raw, 2047)
        self.assertEqual(kp_raw, 0)
        self.assertEqual(kd_raw, 0)
        self.assertEqual(torque_raw, 2047)
        self.assertNotEqual(encoded.data[:2], bytes.fromhex("7fff"))

    def test_encodes_official_mit_zero_vector_without_transport(self) -> None:
        encoded = encode_damiao_mit_command(
            endpoint(),
            DamiaoMitCommand(0.0, 0.0, 0.0, 0.0, 0.0),
            DamiaoMitState(0.0, 0.0),
            DamiaoMitCommandEnvelope((-2.0, 2.0), 9.3, 14.0, 14.0),
        )
        self.assertEqual(encoded.can_id, 0x03)  # MIT_MODE is 0x000.
        self.assertEqual(encoded.data, bytes.fromhex("7fff7ff0000007ff"))
        self.assertFalse(hasattr(encoded, "send"))

    def test_encodes_official_mit_protocol_maximum_vector(self) -> None:
        encoded = encode_damiao_mit_command(
            endpoint(),
            DamiaoMitCommand(12.5, 20.0, 500.0, 5.0, 28.0),
            DamiaoMitState(12.5, 20.0),
            DamiaoMitCommandEnvelope((-12.5, 12.5), 20.0, 28.0, 28.0),
        )
        self.assertEqual(encoded.data, bytes.fromhex("ffffffffffffffff"))

    def test_encoder_rejects_nonfinite_and_all_command_overruns(self) -> None:
        envelope = DamiaoMitCommandEnvelope((-2.0, 2.0), 9.3, 14.0, 14.0)
        invalid = (
            DamiaoMitCommand(2.01, 0.0, 0.0, 0.0, 0.0),
            DamiaoMitCommand(0.0, 9.31, 0.0, 0.0, 0.0),
            DamiaoMitCommand(0.0, 0.0, 500.01, 0.0, 0.0),
            DamiaoMitCommand(0.0, 0.0, 0.0, 5.01, 0.0),
            DamiaoMitCommand(0.0, 0.0, 0.0, 0.0, 14.01),
        )
        for command in invalid:
            with self.subTest(command=command), self.assertRaises(ValueError):
                encode_damiao_mit_command(
                    endpoint(), command, DamiaoMitState(0.0, 0.0), envelope
                )
        with self.assertRaisesRegex(ValueError, "finite"):
            DamiaoMitCommand(float("nan"), 0.0, 0.0, 0.0, 0.0)

    def test_encoder_rejects_envelopes_outside_register_readback_ranges(self) -> None:
        command = DamiaoMitCommand(0.0, 0.0, 0.0, 0.0, 0.0)
        for envelope in (
            DamiaoMitCommandEnvelope((-13.0, 12.5), 9.3, 14.0, 14.0),
            DamiaoMitCommandEnvelope((-2.0, 2.0), 20.1, 14.0, 14.0),
            DamiaoMitCommandEnvelope((-2.0, 2.0), 9.3, 28.1, 14.0),
            DamiaoMitCommandEnvelope((-2.0, 2.0), 9.3, 14.0, 28.1),
        ):
            with self.subTest(envelope=envelope), self.assertRaises(ValueError):
                encode_damiao_mit_command(
                    endpoint(), command, DamiaoMitState(0.0, 0.0), envelope
                )

    def test_encoder_gates_complete_mit_torque_request_from_measured_state(self) -> None:
        command = DamiaoMitCommand(1.0, 0.0, 20.0, 1.0, 0.0)
        envelope = DamiaoMitCommandEnvelope((-2.0, 2.0), 9.3, 14.0, 14.0)
        with self.assertRaisesRegex(ValueError, "output torque"):
            encode_damiao_mit_command(
                endpoint(), command, DamiaoMitState(0.0, 0.0), envelope
            )
        encoded = encode_damiao_mit_command(
            endpoint(), command, DamiaoMitState(0.5, -1.0), envelope
        )
        self.assertEqual(len(encoded.data), 8)

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

    def test_operator_confirmed_ranges_are_decode_only(self) -> None:
        declared = DamiaoMitRanges(
            (-12.5, 12.5),
            (-20, 20),
            (-28, 28),
            "operator_confirmed_drive_configuration",
        )
        self.assertFalse(declared.register_readback_verified)
        self.assertTrue(ranges().register_readback_verified)

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
        built = endpoints_from_hardware_config(hardware, expected_motor_count=1)
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

    def test_bus_classifier_accepts_observed_control_but_rejects_unknown_id(self) -> None:
        audit = DamiaoRxAudit([endpoint()], minimum_samples_per_motor=1)
        self.assertIsNone(audit.ingest_bus_frame(frame(bytes(8), can_id=0x03)))
        self.assertIsNone(audit.ingest_bus_frame(frame(bytes(8), can_id=0x55)))
        report = audit.report()
        self.assertEqual(report.observed_control_frame_count, 1)
        self.assertEqual(report.unexpected_frame_count, 1)
        self.assertFalse(report.passed)

    def test_finite_collector_uses_only_receive_api(self) -> None:
        class Receiver:
            interface = "can2"

            def __init__(self):
                self.frames = [
                    frame(
                        bytes([0x03]) + bytes(7),
                        hardware_timestamp_ns=1_000_000_000 + index * 2_000_000,
                        software_timestamp_ns=2_000_000_000 + index * 2_000_000,
                        userspace_receive_timestamp_ns=2_000_100_000 + index * 2_000_000,
                    )
                    for index in range(3)
                ]

            def receive(self):
                return self.frames.pop(0)

        receiver = Receiver()

        class Selector:
            def __enter__(self):
                return self

            def __exit__(self, *_args):
                return None

            def register(self, _fileobj, _events, data):
                self.data = data

            def select(self, timeout):
                if self.data.frames:
                    return [(SimpleNamespace(data=self.data), 1)]
                return []

        ticks = iter((0.0, 0.001, 0.002, 0.003, 0.2))
        audit = DamiaoRxAudit([endpoint()], minimum_samples_per_motor=3)
        report = collect_receive_only_audit(
            {"can2": receiver},
            audit,
            0.1,
            selector_factory=Selector,
            monotonic=lambda: next(ticks),
        )
        self.assertTrue(report.passed, report.errors)
        self.assertEqual(report.decoded_feedback_frame_count, 3)
        self.assertEqual(report.hardware_tx_attempts, 0)


if __name__ == "__main__":
    unittest.main()
