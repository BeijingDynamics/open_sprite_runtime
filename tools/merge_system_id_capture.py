#!/usr/bin/env python3
"""Align a passive CAN capture with an existing policy trace."""

from __future__ import annotations

import argparse
import json

from open_sprite_runtime.system_id_merge import merge_system_id_files


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--can-trace", required=True)
    parser.add_argument("--policy-trace", required=True)
    parser.add_argument("--hardware-config", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--report", required=True)
    args = parser.parse_args()
    report = merge_system_id_files(
        args.can_trace,
        args.policy_trace,
        args.hardware_config,
        args.output,
        args.report,
    )
    print(json.dumps(report, indent=2))
    if not report["passed"]:
        raise SystemExit("system-identification capture quality gates failed")


if __name__ == "__main__":
    main()
