import unittest
import socket
import struct

from open_sprite_runtime.socketcan import (
    CANFD_FRAME,
    SO_TIMESTAMPING_LINUX_64,
    SOF_TIMESTAMPING_RX_SOFTWARE,
    SocketCanReceiver,
    audit_socketcan_rx_snapshot,
)


def entry(name: str, *, up: bool = True, listen_only: bool = True) -> dict:
    modes = ["FD"]
    if listen_only:
        modes.append("LISTEN-ONLY")
    return {
        "ifname": name,
        "flags": ["NOARP", "UP"] if up else ["NOARP"],
        "link_type": "can",
        "linkinfo": {"info_kind": "can", "info_data": {"ctrlmode": modes}},
    }


class SocketCanRxPreflightTests(unittest.TestCase):
    def test_four_up_listen_only_interfaces_pass_without_tx(self) -> None:
        report = audit_socketcan_rx_snapshot(
            [entry(f"can{index}") for index in range(4)],
            [f"can{index}" for index in range(4)],
        )
        self.assertTrue(report.passed, report.errors)
        self.assertEqual(report.to_dict()["hardware_tx_attempts"], 0)

    def test_missing_down_or_active_interface_fails_closed(self) -> None:
        report = audit_socketcan_rx_snapshot(
            [entry("can0"), entry("can1", up=False), entry("can2", listen_only=False)],
            ["can0", "can1", "can2", "can3"],
        )
        self.assertFalse(report.passed)
        self.assertTrue(any("can1: interface is not UP" in error for error in report.errors))
        self.assertTrue(any("can2: kernel LISTEN-ONLY" in error for error in report.errors))
        self.assertTrue(any("can3: interface is missing" in error for error in report.errors))

    def test_non_can_interface_fails_closed(self) -> None:
        snapshot = [entry(f"can{index}") for index in range(4)]
        snapshot[2]["link_type"] = "ether"
        snapshot[2]["linkinfo"]["info_kind"] = "ether"
        report = audit_socketcan_rx_snapshot(snapshot, [f"can{index}" for index in range(4)])
        self.assertFalse(report.passed)
        self.assertTrue(any("can2: interface is not CAN" in error for error in report.errors))


class FakeSocket:
    def __init__(self, frame: bytes = b"", ancillary: list | None = None):
        self.frame = frame
        self.ancillary = ancillary or []
        self.options = []
        self.address = None
        self.closed = False

    def setsockopt(self, level, name, value):
        self.options.append((level, name, value))

    def bind(self, address):
        self.address = address

    def recvmsg(self, *_args):
        return self.frame, self.ancillary, 0, ("can0",)

    def close(self):
        self.closed = True


class SocketCanReceiverTests(unittest.TestCase):
    def setUp(self) -> None:
        self.preflight = audit_socketcan_rx_snapshot(
            [entry(f"can{index}") for index in range(4)],
            [f"can{index}" for index in range(4)],
        )

    def test_open_requires_passed_listen_only_evidence(self) -> None:
        failed = audit_socketcan_rx_snapshot([], [f"can{index}" for index in range(4)])
        with self.assertRaisesRegex(RuntimeError, "preflight did not pass"):
            SocketCanReceiver.open("can0", failed, socket_factory=lambda *_args: FakeSocket())

    def test_open_enables_fd_and_hardware_timestamping_before_bind(self) -> None:
        fake = FakeSocket()
        receiver = SocketCanReceiver.open("can0", self.preflight, socket_factory=lambda *_: fake)
        self.assertEqual(fake.address, ("can0",))
        self.assertFalse(hasattr(receiver, "send"))
        self.assertIn((socket.SOL_CAN_RAW, socket.CAN_RAW_FD_FRAMES, 1), fake.options)
        self.assertTrue(any(option[1] == SO_TIMESTAMPING_LINUX_64 for option in fake.options))
        timestamp_value = next(
            option[2] for option in fake.options if option[1] == SO_TIMESTAMPING_LINUX_64
        )
        self.assertTrue(timestamp_value & SOF_TIMESTAMPING_RX_SOFTWARE)
        receiver.close()
        self.assertTrue(fake.closed)

    def test_receive_decodes_fd_frame_and_raw_hardware_timestamp(self) -> None:
        frame = CANFD_FRAME.pack(0x123, 4, 0x01, 0, 0, b"abcd".ljust(64, b"\0"))
        timestamps = struct.pack("=6q", 1, 2, 3, 4, 5, 6)
        fake = FakeSocket(frame, [(socket.SOL_SOCKET, SO_TIMESTAMPING_LINUX_64, timestamps)])
        received = SocketCanReceiver(
            "can0", fake, clock_ns=lambda: 1_000_000_102
        ).receive()
        self.assertEqual(received.can_id, 0x123)
        self.assertEqual(received.data, b"abcd")
        self.assertTrue(received.is_fd)
        self.assertTrue(received.bit_rate_switch)
        self.assertEqual(received.software_timestamp_ns, 1_000_000_002)
        self.assertEqual(received.hardware_timestamp_ns, 5_000_000_006)
        self.assertEqual(received.userspace_queue_age_ns, 100)

    def test_receive_rejects_missing_hardware_timestamp(self) -> None:
        frame = CANFD_FRAME.pack(0x123, 1, 0, 0, 0, b"x".ljust(64, b"\0"))
        with self.assertRaisesRegex(RuntimeError, "SCM_TIMESTAMPING evidence is missing"):
            SocketCanReceiver("can0", FakeSocket(frame)).receive()

    def test_receive_rejects_missing_raw_hardware_timestamp(self) -> None:
        frame = CANFD_FRAME.pack(0x123, 1, 0, 0, 0, b"x".ljust(64, b"\0"))
        timestamps = struct.pack("=6q", 1, 2, 0, 0, 0, 0)
        ancillary = [(socket.SOL_SOCKET, SO_TIMESTAMPING_LINUX_64, timestamps)]
        with self.assertRaisesRegex(RuntimeError, "software fallback is forbidden"):
            SocketCanReceiver(
                "can0", FakeSocket(frame, ancillary), clock_ns=lambda: 2_000_000_000
            ).receive()

    def test_receive_rejects_missing_kernel_software_timestamp(self) -> None:
        frame = CANFD_FRAME.pack(0x123, 1, 0, 0, 0, b"x".ljust(64, b"\0"))
        timestamps = struct.pack("=6q", 0, 0, 0, 0, 5, 6)
        ancillary = [(socket.SOL_SOCKET, SO_TIMESTAMPING_LINUX_64, timestamps)]
        with self.assertRaisesRegex(RuntimeError, "software RX timestamp is missing"):
            SocketCanReceiver(
                "can0", FakeSocket(frame, ancillary), clock_ns=lambda: 6_000_000_000
            ).receive()


if __name__ == "__main__":
    unittest.main()
