#!/usr/bin/env python3
"""Capture four active Sprite CAN-FD buses without transmitting."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import subprocess

from open_sprite_runtime.socketcan import (
    SocketCanReceiver,
    audit_socketcan_active_fd_snapshot,
)
from open_sprite_runtime.system_id_capture import (
    capture_active_can_trace,
    write_active_can_trace,
)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--interface", action="append", default=[])
    parser.add_argument("--duration", type=float, required=True)
    parser.add_argument("--trace", required=True)
    parser.add_argument("--report", required=True)
    args = parser.parse_args()
    interfaces = tuple(args.interface or ("kcan1", "kcan2", "kcan3", "kcan4"))
    if len(interfaces) != 4 or len(set(interfaces)) != 4:
        raise ValueError("exactly four unique --interface values are required")

    snapshot = json.loads(
        subprocess.run(
            ["ip", "-j", "-d", "link", "show"],
            check=True,
            capture_output=True,
            text=True,
        ).stdout
    )
    receivers = {}
    try:
        for interface in interfaces:
            preflight = audit_socketcan_active_fd_snapshot(snapshot, interface)
            if not preflight.passed:
                raise RuntimeError(
                    f"{interface}: active observer preflight failed: "
                    + "; ".join(preflight.errors)
                )
            receivers[interface] = SocketCanReceiver.open_active_observer(
                interface, preflight
            )
        trace, report = capture_active_can_trace(receivers, args.duration)
        report = {
            **report,
            "interfaces": list(interfaces),
            "trace": str(Path(args.trace).resolve()),
        }
        write_active_can_trace(trace, report, args.trace, args.report)
        print(json.dumps(report, indent=2))
        if not report["passed"]:
            raise SystemExit("active CAN system-identification capture failed")
    finally:
        for receiver in receivers.values():
            receiver.close()


if __name__ == "__main__":
    main()
