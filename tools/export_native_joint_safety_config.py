#!/usr/bin/env python3
"""Export ordered protected-target limits for the native IPC validator."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np


FIELDS = (
    "joint_name",
    "position_min_rad",
    "position_max_rad",
    "velocity_max_rad_s",
    "kp_max",
    "kd_max",
    "feedforward_torque_max_nm",
)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--contract", type=Path, required=True)
    parser.add_argument("--joint-limit-candidates", type=Path, required=True)
    parser.add_argument("--gain-scale", type=float, required=True)
    parser.add_argument("--maximum-embedded-kd", type=float, default=3.0)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if not 0.0 < args.gain_scale <= 1.0:
        raise SystemExit("gain scale must be in (0, 1]")
    if not math.isfinite(args.maximum_embedded_kd) or not (
        0.0 < args.maximum_embedded_kd <= 3.0
    ):
        raise SystemExit("maximum embedded Kd must be finite and in (0, 3]")

    contract = json.loads(args.contract.read_text(encoding="utf-8"))
    limits = json.loads(args.joint_limit_candidates.read_text(encoding="utf-8"))[
        "joint_limits"
    ]
    names = tuple(contract["joint_names"])
    arrays = {
        name: np.asarray(contract[name], dtype=np.float64)
        for name in ("velocity_limit", "stiffness", "damping", "effort_limit")
    }
    if any(value.shape != (31,) or not np.isfinite(value).all() for value in arrays.values()):
        raise SystemExit("contract safety arrays must contain 31 finite values")
    if (
        np.any(arrays["velocity_limit"] <= 0.0)
        or np.any(arrays["stiffness"] < 0.0)
        or np.any(arrays["damping"] < 0.0)
        or np.any(arrays["effort_limit"] < 0.0)
    ):
        raise SystemExit("contract safety arrays contain invalid limits or gains")
    if len(names) != 31 or len(set(names)) != 31:
        raise SystemExit("contract joint_names must contain 31 unique joints")
    if set(limits) != set(names):
        raise SystemExit("joint limit report does not exactly cover contract joints")

    rows = []
    for index, name in enumerate(names):
        low, high = map(float, limits[name]["soft_limit_rad_candidate"])
        if not math.isfinite(low) or not math.isfinite(high) or low >= high:
            raise SystemExit(f"invalid soft-limit candidate for {name}")
        rows.append(
            {
                "joint_name": name,
                "position_min_rad": low,
                "position_max_rad": high,
                "velocity_max_rad_s": abs(float(arrays["velocity_limit"][index])),
                "kp_max": args.gain_scale * float(arrays["stiffness"][index]),
                "kd_max": args.gain_scale
                * min(float(arrays["damping"][index]), args.maximum_embedded_kd),
                "feedforward_torque_max_nm": args.gain_scale
                * float(arrays["effort_limit"][index]),
            }
        )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(
            stream, fieldnames=FIELDS, dialect="excel-tab", lineterminator="\n"
        )
        writer.writeheader()
        writer.writerows(rows)
    print(
        f"EXPORTED joints={len(rows)} gain_scale={args.gain_scale} "
        f"output={args.output}"
    )


if __name__ == "__main__":
    main()
