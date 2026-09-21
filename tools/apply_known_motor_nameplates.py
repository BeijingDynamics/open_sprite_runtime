#!/usr/bin/env python3
"""Apply only user-confirmed Sprite motor nameplate values to a hardware contract."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


KNOWN_NAMEPLATES = {
    "DM-J4340P-2EC V1.1 (48V)": {
        "rated_torque_nm": 14.0,
        "peak_torque_nm": 40.0,
        "rated_speed_rad_s": 3.77,
        "max_speed_rad_s": 9.3,
    },
    "DM-J4310P-2EC (48V)": {
        "rated_torque_nm": 3.5,
        "peak_torque_nm": 12.5,
        "rated_speed_rad_s": 12.56,
        "max_speed_rad_s": 36.2,
    },
}


def apply_nameplates(hardware: dict) -> int:
    changed = 0
    for record in hardware.get("motor_map", {}).values():
        known = KNOWN_NAMEPLATES.get(record.get("model"))
        if known is None:
            continue
        for field, value in known.items():
            if record.get(field) != value:
                record[field] = value
                changed += 1
    return changed


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    hardware = json.loads(Path(args.input).read_text(encoding="utf-8"))
    changed = apply_nameplates(hardware)
    Path(args.output).write_text(json.dumps(hardware, indent=2) + "\n", encoding="utf-8")
    print(f"UPDATED fields={changed} output={args.output}")


if __name__ == "__main__":
    main()
