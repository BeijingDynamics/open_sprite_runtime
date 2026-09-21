#!/usr/bin/env python3
"""Read six non-mutating Damiao commissioning registers from disabled drives."""

from __future__ import annotations

import argparse
from contextlib import ExitStack
import json
from pathlib import Path

from open_sprite_runtime.damiao import endpoints_from_hardware_config
from open_sprite_runtime.damiao_registers import collect_commissioning_registers
from open_sprite_runtime.socketcan import (
    SocketCanDamiaoCommissioningRegisterReader,
    audit_socketcan_active_fd_snapshot,
)


CONFIRMATION = "READ_ONLY_DAMIAO_COMMISSIONING_REGISTERS"


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--hardware", required=True)
    parser.add_argument("--snapshot", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--timeout", type=float, default=0.1)
    parser.add_argument("--retries", type=int, default=3)
    parser.add_argument("--supported-unloaded", action="store_true")
    parser.add_argument("--all-motors-disabled-confirmed", action="store_true")
    parser.add_argument("--acknowledge-hardware-tx", required=True)
    args = parser.parse_args()
    if args.acknowledge_hardware_tx != CONFIRMATION:
        raise SystemExit(f"--acknowledge-hardware-tx must equal {CONFIRMATION}")
    if not args.supported_unloaded or not args.all_motors_disabled_confirmed:
        raise SystemExit("supported/unloaded and all-motors-disabled confirmations are required")

    hardware = json.loads(Path(args.hardware).read_text(encoding="utf-8"))
    snapshot = json.loads(Path(args.snapshot).read_text(encoding="utf-8"))
    endpoints = endpoints_from_hardware_config(hardware)
    interfaces = tuple(hardware["can_adapter"]["interfaces"])
    preflights = {
        interface: audit_socketcan_active_fd_snapshot(
            snapshot, interface, arbitration_bitrate=1_000_000, data_bitrate=5_000_000
        )
        for interface in interfaces
    }
    failed = {name: report.errors for name, report in preflights.items() if not report.passed}
    if failed:
        raise SystemExit(f"active CAN-FD preflight failed: {failed}")
    by_interface = {
        interface: tuple(endpoint for endpoint in endpoints if endpoint.interface == interface)
        for interface in interfaces
    }
    with ExitStack() as stack:
        readers = {
            interface: stack.enter_context(
                SocketCanDamiaoCommissioningRegisterReader.open(
                    interface, preflights[interface],
                    (endpoint.can_id for endpoint in by_interface[interface]),
                )
            )
            for interface in interfaces
        }
        report = collect_commissioning_registers(
            readers, endpoints, timeout_s=args.timeout, retries=args.retries
        )
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
