"""Deliberately violate one native IPC invariant for fail-closed testing."""

from __future__ import annotations

import argparse
import socket
import time

import numpy as np

from .native_ipc import NativeStatePacket, PolicyTargetPacket


def _target(state: NativeStatePacket, joint_hash: int, mode: str) -> PolicyTargetPacket:
    zeros = tuple(np.zeros(31, dtype=np.float64))
    kp = tuple(np.ones(31, dtype=np.float64))
    kd = list(np.ones(31, dtype=np.float64))
    sequence = 1
    timestamp = time.monotonic_ns()
    source_sequence = state.sequence
    if mode == "wrong_hash":
        joint_hash ^= 1
    elif mode == "future_source":
        source_sequence += 1
    elif mode == "stale_timestamp":
        timestamp -= 1_000_000_000
    elif mode == "excessive_kd":
        kd[0] = 3.01
    else:
        raise ValueError(f"unsupported packet fault mode: {mode}")
    return PolicyTargetPacket(
        sequence=sequence,
        monotonic_ns=timestamp,
        joint_order_hash=joint_hash,
        source_state_sequence=source_sequence,
        position_rad=zeros,
        velocity_rad_s=zeros,
        kp=kp,
        kd=tuple(kd),
        feedforward_torque_nm=zeros,
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--socket", required=True)
    parser.add_argument("--joint-hash", required=True, type=lambda value: int(value, 0))
    parser.add_argument(
        "--mode",
        required=True,
        choices=("wrong_hash", "future_source", "stale_timestamp", "excessive_kd", "silence"),
    )
    args = parser.parse_args()
    with socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET) as connection:
        connection.connect(args.socket)
        state = NativeStatePacket.unpack(connection.recv(4096))
        if args.mode != "silence":
            connection.sendall(_target(state, args.joint_hash, args.mode).pack())
        try:
            while connection.recv(4096):
                pass
        except (ConnectionResetError, BrokenPipeError):
            pass


if __name__ == "__main__":
    main()
