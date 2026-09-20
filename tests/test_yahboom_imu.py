import math
import struct
import unittest

from open_sprite_runtime.yahboom_imu import (
    YahboomEuler,
    YahboomQuaternion,
    YahboomRawImu,
    YahboomStreamDecoder,
    decode_frame,
)


def frame(function: int, payload: bytes) -> bytes:
    result = bytearray(b"\x7e\x23")
    result.extend((len(payload) + 5, function))
    result.extend(payload)
    result.append(sum(result) & 0xFF)
    return bytes(result)


class YahboomImuTests(unittest.TestCase):
    def test_decodes_observed_raw_frame_with_documented_units(self):
        packet = decode_frame(
            bytes.fromhex(
                "7e 23 17 04 c2 f9 9c ff ea fa 00 00 00 00 00 00 ad fc d9 f5 80 04 f1"
            )
        )
        self.assertIsInstance(packet, YahboomRawImu)
        self.assertAlmostEqual(packet.acceleration_m_s2[0], -7.652102, places=6)
        self.assertEqual(packet.angular_velocity_rad_s, (0.0, 0.0, 0.0))
        self.assertAlmostEqual(packet.magnetic_field_ut[0], -20.777, places=3)

    def test_decodes_observed_quaternion_and_euler_frames(self):
        quaternion = decode_frame(
            bytes.fromhex(
                "7e 23 15 16 2f aa c2 3e ec fc ee be 80 3c 45 3f 32 51 4e 3e 88"
            )
        )
        euler = decode_frame(
            bytes.fromhex("7e 23 11 26 26 80 44 c0 50 93 62 3f fc dd 00 c0 9f")
        )
        self.assertIsInstance(quaternion, YahboomQuaternion)
        self.assertAlmostEqual(quaternion.norm, 0.998312, places=6)
        self.assertIsInstance(euler, YahboomEuler)
        self.assertAlmostEqual(euler.roll_pitch_yaw_rad[0], -3.070322, places=6)

    def test_stream_decoder_handles_chunks_noise_and_bad_checksum(self):
        quaternion = frame(0x16, struct.pack("<4f", 1.0, 0.0, 0.0, 0.0))
        euler = frame(0x26, struct.pack("<3f", 0.1, 0.2, 0.3))
        bad = bytearray(quaternion)
        bad[-1] ^= 0x01
        decoder = YahboomStreamDecoder()
        self.assertEqual(decoder.feed(b"noise" + bytes(bad) + quaternion[:7]), [])
        packets = decoder.feed(quaternion[7:] + euler)
        self.assertEqual(len(packets), 2)
        self.assertIsInstance(packets[0], YahboomQuaternion)
        self.assertTrue(math.isclose(packets[0].norm, 1.0))
        self.assertIsInstance(packets[1], YahboomEuler)
        self.assertEqual(decoder.rejected_frame_count, 1)


if __name__ == "__main__":
    unittest.main()
