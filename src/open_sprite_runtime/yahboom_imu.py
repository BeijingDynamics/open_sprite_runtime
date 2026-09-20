"""Streaming decoder for the Yahboom IMU serial protocol."""

from __future__ import annotations

from dataclasses import dataclass
import math
import struct


FRAME_HEADER = b"\x7e\x23"
FUNCTION_RAW_IMU = 0x04
FUNCTION_QUATERNION = 0x16
FUNCTION_EULER = 0x26

STANDARD_GRAVITY_M_S2 = 9.80665
ACCELERATION_SCALE_G = 16.0 / 32767.0
ANGULAR_VELOCITY_SCALE_RAD_S = (2000.0 / 32767.0) * (math.pi / 180.0)
MAGNETIC_FIELD_SCALE_UT = 800.0 / 32767.0


Vector3 = tuple[float, float, float]
QuaternionWxyz = tuple[float, float, float, float]


@dataclass(frozen=True)
class YahboomRawImu:
    acceleration_m_s2: Vector3
    angular_velocity_rad_s: Vector3
    magnetic_field_ut: Vector3


@dataclass(frozen=True)
class YahboomQuaternion:
    wxyz: QuaternionWxyz

    @property
    def norm(self) -> float:
        return math.sqrt(sum(value * value for value in self.wxyz))


@dataclass(frozen=True)
class YahboomEuler:
    roll_pitch_yaw_rad: Vector3


YahboomPacket = YahboomRawImu | YahboomQuaternion | YahboomEuler


def decode_frame(frame: bytes) -> YahboomPacket:
    if len(frame) < 5 or frame[:2] != FRAME_HEADER:
        raise ValueError("invalid Yahboom frame header")
    if frame[2] != len(frame):
        raise ValueError("Yahboom frame length mismatch")
    if sum(frame[:-1]) & 0xFF != frame[-1]:
        raise ValueError("Yahboom frame checksum mismatch")

    function = frame[3]
    payload = frame[4:-1]
    if function == FUNCTION_RAW_IMU and len(payload) == 18:
        raw = struct.unpack("<9h", payload)
        acceleration = tuple(
            value * ACCELERATION_SCALE_G * STANDARD_GRAVITY_M_S2 for value in raw[:3]
        )
        angular_velocity = tuple(
            value * ANGULAR_VELOCITY_SCALE_RAD_S for value in raw[3:6]
        )
        magnetic_field = tuple(value * MAGNETIC_FIELD_SCALE_UT for value in raw[6:9])
        return YahboomRawImu(acceleration, angular_velocity, magnetic_field)
    if function == FUNCTION_QUATERNION and len(payload) == 16:
        return YahboomQuaternion(struct.unpack("<4f", payload))
    if function == FUNCTION_EULER and len(payload) == 12:
        return YahboomEuler(struct.unpack("<3f", payload))
    raise ValueError(
        f"unsupported Yahboom frame function 0x{function:02x} with {len(payload)} payload bytes"
    )


class YahboomStreamDecoder:
    """Resynchronizing decoder for arbitrarily chunked serial input."""

    def __init__(self, maximum_frame_length: int = 64):
        if maximum_frame_length < 5 or maximum_frame_length > 255:
            raise ValueError("maximum_frame_length must be in [5, 255]")
        self.maximum_frame_length = maximum_frame_length
        self.buffer = bytearray()
        self.rejected_frame_count = 0

    def feed(self, data: bytes) -> list[YahboomPacket]:
        self.buffer.extend(data)
        packets: list[YahboomPacket] = []
        while True:
            start = self.buffer.find(FRAME_HEADER)
            if start < 0:
                if self.buffer[-1:] == FRAME_HEADER[:1]:
                    self.buffer[:] = self.buffer[-1:]
                else:
                    self.buffer.clear()
                break
            if start:
                del self.buffer[:start]
            if len(self.buffer) < 3:
                break
            length = self.buffer[2]
            if length < 5 or length > self.maximum_frame_length:
                self.rejected_frame_count += 1
                del self.buffer[0]
                continue
            if len(self.buffer) < length:
                break
            frame = bytes(self.buffer[:length])
            try:
                packets.append(decode_frame(frame))
            except ValueError:
                self.rejected_frame_count += 1
                del self.buffer[0]
                continue
            del self.buffer[:length]
        return packets
