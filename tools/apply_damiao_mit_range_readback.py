#!/usr/bin/env python3
"""Apply a passed read-only Damiao PMAX/VMAX/TMAX report offline."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from open_sprite_runtime.damiao_registers import apply_mit_range_readback


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--hardware", required=True)
    parser.add_argument("--report", required=True)
    parser.add_argument("--evidence-report", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    hardware = json.loads(Path(args.hardware).read_text(encoding="utf-8"))
    report = json.loads(Path(args.report).read_text(encoding="utf-8"))
    updated = apply_mit_range_readback(hardware, report, args.evidence_report)
    Path(args.output).write_text(json.dumps(hardware, indent=2) + "\n", encoding="utf-8")
    print(f"UPDATED mit_ranges={updated} output={args.output}")


if __name__ == "__main__":
    main()
