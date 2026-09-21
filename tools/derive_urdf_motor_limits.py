#!/usr/bin/env python3
"""Generate review-only direct and differential motor limit candidates."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from open_sprite_runtime.contracts import PolicyContract
from open_sprite_runtime.limit_derivation import derive_limit_candidates


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--hardware", required=True)
    parser.add_argument("--contract", required=True)
    parser.add_argument("--urdf", required=True)
    parser.add_argument("--soft-margin-rad", type=float, default=0.05)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    hardware = json.loads(Path(args.hardware).read_text(encoding="utf-8"))
    contract = PolicyContract.load(args.contract)
    report = derive_limit_candidates(
        hardware,
        contract,
        args.urdf,
        soft_margin_rad=args.soft_margin_rad,
    )
    Path(args.output).write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(
        f"DERIVED joints={len(report['joint_limits'])} motors={len(report['motor_limits'])} "
        f"confirmed={report['confirmed_physical_hard_limits']} output={args.output}"
    )


if __name__ == "__main__":
    main()
