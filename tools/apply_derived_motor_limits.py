#!/usr/bin/env python3
"""Apply audited URDF-derived motor limits to a hardware contract offline."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from open_sprite_runtime.limit_derivation import apply_limit_candidates


CONFIRMATION = "USE_URDF_LIMITS_AS_CONSERVATIVE_RUNTIME_GUARDS"


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--hardware", required=True)
    parser.add_argument("--limits", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--confirm", required=True)
    args = parser.parse_args()
    if args.confirm != CONFIRMATION:
        raise SystemExit(f"--confirm must equal {CONFIRMATION}")
    hardware = json.loads(Path(args.hardware).read_text(encoding="utf-8"))
    report = json.loads(Path(args.limits).read_text(encoding="utf-8"))
    updated = apply_limit_candidates(hardware, report)
    Path(args.output).write_text(json.dumps(hardware, indent=2) + "\n", encoding="utf-8")
    print(f"UPDATED limit_fields={updated} motors={updated // 2} output={args.output}")


if __name__ == "__main__":
    main()
