import unittest

from open_sprite_runtime.socketcan import ReceivedCanFrame
from open_sprite_runtime.system_id_capture import ActiveCanTraceBuffer


class ActiveCanTraceBufferTests(unittest.TestCase):
    def test_records_raw_frame_and_monotonic_alignment_timestamp(self) -> None:
        trace = ActiveCanTraceBuffer(("kcan1", "kcan2"))
        trace.append(
            ReceivedCanFrame(
                interface="kcan2",
                can_id=0x18,
                data=b"12345678",
                is_extended=False,
                is_remote=False,
                is_error=False,
                is_fd=True,
                bit_rate_switch=True,
                error_state_indicator=False,
                software_timestamp_ns=100,
                hardware_timestamp_ns=90,
                userspace_receive_timestamp_ns=110,
            ),
            monotonic_receive_ns=200,
        )
        arrays = trace.arrays()
        self.assertEqual(arrays["payload"].shape, (1, 64))
        self.assertEqual(arrays["interface_index"].tolist(), [1])
        self.assertEqual(arrays["can_id"].tolist(), [0x18])
        self.assertEqual(arrays["payload"][0, :8].tobytes(), b"12345678")
        self.assertEqual(arrays["monotonic_receive_ns"].tolist(), [200])

    def test_rejects_unknown_interface(self) -> None:
        trace = ActiveCanTraceBuffer(("kcan1",))
        frame = ReceivedCanFrame(
            "kcan2", 1, b"", False, False, False, True, True, False, 1, 1, 1
        )
        with self.assertRaisesRegex(ValueError, "unexpected CAN interface"):
            trace.append(frame, 1)
