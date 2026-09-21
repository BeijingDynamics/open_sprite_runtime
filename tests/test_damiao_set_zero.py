import unittest

from open_sprite_runtime.socketcan import CANFD_FRAME, SocketCanSetZeroWriter


class FakeSocket:
    def __init__(self):
        self.frames = []

    def send(self, frame):
        self.frames.append(frame)
        return len(frame)

    def close(self):
        return None


class DamiaoSetZeroWriterTests(unittest.TestCase):
    def test_sends_only_exact_fd_brs_set_zero_frame_to_allowlist(self) -> None:
        raw = FakeSocket()
        writer = SocketCanSetZeroWriter("kcan2", raw, (0x07, 0x08))
        writer.send_set_zero(0x07)
        self.assertEqual(writer.hardware_tx_attempts, 1)
        can_id, length, flags, reserved0, reserved1, payload = CANFD_FRAME.unpack(
            raw.frames[0]
        )
        self.assertEqual(can_id, 0x07)
        self.assertEqual(length, 8)
        self.assertEqual(flags, 0x01)
        self.assertEqual((reserved0, reserved1), (0, 0))
        self.assertEqual(payload[:8], bytes.fromhex("ff ff ff ff ff ff ff fe"))
        self.assertEqual(payload[8:], bytes(56))

    def test_rejects_every_can_id_outside_allowlist(self) -> None:
        writer = SocketCanSetZeroWriter("kcan2", FakeSocket(), (0x07,))
        with self.assertRaisesRegex(ValueError, "not in the set-zero allowlist"):
            writer.send_set_zero(0x08)


if __name__ == "__main__":
    unittest.main()
