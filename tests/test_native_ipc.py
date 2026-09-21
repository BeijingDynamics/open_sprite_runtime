import math
import struct
import unittest

from open_sprite_runtime.native_ipc import (
    MAGIC,
    NativeStatePacket,
    PolicyTargetPacket,
    STATE_PACKET_SIZE,
    TARGET_PACKET_SIZE,
    ordered_name_hash,
)


NAMES = tuple(f"joint_{index}" for index in range(31))


class NativeIpcTests(unittest.TestCase):
    def test_name_hash_is_order_sensitive_and_stable(self) -> None:
        self.assertEqual(ordered_name_hash(NAMES), 0xF8295D8299BD2E68)
        self.assertNotEqual(ordered_name_hash(NAMES), ordered_name_hash(tuple(reversed(NAMES))))

    def test_state_packet_round_trip(self) -> None:
        packet = NativeStatePacket(
            sequence=7,
            monotonic_ns=123456,
            motor_order_hash=ordered_name_hash(NAMES),
            position_rad=tuple(index / 10 for index in range(31)),
            velocity_rad_s=tuple(-index / 20 for index in range(31)),
        )
        payload = packet.pack()
        self.assertEqual(len(payload), STATE_PACKET_SIZE)
        self.assertEqual(payload[:4], MAGIC)
        self.assertEqual(NativeStatePacket.unpack(payload), packet)

    def test_target_packet_round_trip(self) -> None:
        packet = PolicyTargetPacket(
            sequence=8,
            monotonic_ns=234567,
            joint_order_hash=ordered_name_hash(NAMES),
            source_state_sequence=7,
            position_rad=(0.1,) * 31,
            velocity_rad_s=(0.0,) * 31,
            kp=(20.0,) * 31,
            kd=(1.5,) * 31,
            feedforward_torque_nm=(0.0,) * 31,
        )
        payload = packet.pack()
        self.assertEqual(len(payload), TARGET_PACKET_SIZE)
        self.assertEqual(PolicyTargetPacket.unpack(payload), packet)

    def test_packets_fail_closed_on_bad_shape_header_and_values(self) -> None:
        with self.assertRaisesRegex(ValueError, "31 finite"):
            NativeStatePacket(1, 2, 3, (0.0,) * 30, (0.0,) * 31).pack()
        with self.assertRaisesRegex(ValueError, "finite"):
            NativeStatePacket(1, 2, 3, (math.nan,) * 31, (0.0,) * 31).pack()
        packet = bytearray(
            NativeStatePacket(1, 2, 3, (0.0,) * 31, (0.0,) * 31).pack()
        )
        packet[0] ^= 0xFF
        with self.assertRaisesRegex(ValueError, "header"):
            NativeStatePacket.unpack(bytes(packet))
        invalid = bytearray(
            NativeStatePacket(1, 2, 3, (0.0,) * 31, (0.0,) * 31).pack()
        )
        # A malformed binary floating-point field must fail rather than reach policy code.
        invalid[32:40] = struct.pack("<d", math.nan)
        with self.assertRaisesRegex(ValueError, "finite"):
            NativeStatePacket.unpack(bytes(invalid))
        with self.assertRaisesRegex(ValueError, "Kp/Kd"):
            PolicyTargetPacket(
                1,
                2,
                3,
                1,
                (0.0,) * 31,
                (0.0,) * 31,
                (-1.0,) * 31,
                (0.0,) * 31,
                (0.0,) * 31,
            ).pack()


if __name__ == "__main__":
    unittest.main()
