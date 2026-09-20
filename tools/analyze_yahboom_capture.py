#!/usr/bin/env python3
"""Inspect length-prefixed Yahboom IMU captures without writing to hardware."""

from __future__ import annotations

import argparse
import math
import struct
from collections import Counter
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class Frame:
    offset: int
    data: bytes

    @property
    def packet_type(self) -> int:
        return self.data[3]

    @property
    def payload(self) -> bytes:
        return self.data[4:-1]

    @property
    def checksum_valid(self) -> bool:
        return sum(self.data[:-1]) & 0xFF == self.data[-1]


def parse_frames(data: bytes, header: bytes, maximum_length: int) -> tuple[list[Frame], int]:
    frames: list[Frame] = []
    rejected = 0
    offset = 0
    while offset <= len(data) - 3:
        start = data.find(header, offset)
        if start < 0:
            break
        length = data[start + 2]
        if length < 5 or length > maximum_length or start + length > len(data):
            rejected += 1
            offset = start + 1
            continue
        frame = Frame(start, data[start : start + length])
        if not frame.checksum_valid:
            rejected += 1
            offset = start + 1
            continue
        frames.append(frame)
        offset = start + length
    return frames, rejected


def candidate_decode(frame: Frame) -> str:
    """Decode observed layouts, explicitly labelled as unverified candidates."""
    payload = frame.payload
    if frame.packet_type == 0x04 and len(payload) == 18:
        return f"candidate_i16x9={struct.unpack('<9h', payload)}"
    if frame.packet_type == 0x16 and len(payload) == 16:
        values = struct.unpack("<4f", payload)
        norm = math.sqrt(sum(value * value for value in values))
        return f"candidate_f32x4={tuple(round(v, 6) for v in values)} norm={norm:.6f}"
    if frame.packet_type == 0x26 and len(payload) == 12:
        values = struct.unpack("<3f", payload)
        return f"candidate_f32x3={tuple(round(v, 6) for v in values)}"
    return f"payload={payload.hex(' ')}"


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("capture", type=Path)
    parser.add_argument("--header", default="7e23")
    parser.add_argument("--maximum-frame-length", type=int, default=128)
    parser.add_argument("--duration", type=float, help="Capture duration in seconds")
    parser.add_argument("--show", type=int, default=10)
    args = parser.parse_args()

    data = args.capture.read_bytes()
    header = bytes.fromhex(args.header)
    frames, rejected = parse_frames(data, header, args.maximum_frame_length)
    offsets = [frame.offset for frame in frames]
    deltas = [right - left for left, right in zip(offsets, offsets[1:])]
    signatures = Counter((len(frame.data), frame.packet_type) for frame in frames)
    triplets = sum(
        1
        for index in range(len(frames) - 2)
        if tuple(frame.packet_type for frame in frames[index : index + 3]) == (0x04, 0x16, 0x26)
    )

    print(f"capture={args.capture}")
    print(f"bytes={len(data)} valid_frames={len(frames)} rejected_candidates={rejected}")
    print(f"first_offsets={offsets[:20]}")
    print(f"delta_histogram={dict(Counter(deltas).most_common(10))}")
    print(
        "frame_signatures="
        + repr({f"length={length},type=0x{packet_type:02x}": count for (length, packet_type), count in signatures.items()})
    )
    print(f"complete_04_16_26_groups={triplets}")
    if args.duration is not None:
        if not math.isfinite(args.duration) or args.duration <= 0:
            parser.error("--duration must be finite and positive")
        print(f"candidate_group_rate_hz={triplets / args.duration:.3f}")

    for index, frame in enumerate(frames[: args.show]):
        print(
            f"frame={index} offset={frame.offset} length={len(frame.data)} "
            f"type=0x{frame.packet_type:02x} checksum=ok "
            f"{candidate_decode(frame)}"
        )


if __name__ == "__main__":
    main()
