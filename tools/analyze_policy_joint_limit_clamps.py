#!/usr/bin/env python3
"""Summarize raw policy targets that exceed reviewed joint soft limits."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np


def longest_true_run(mask: np.ndarray) -> int:
    longest = 0
    current = 0
    for value in mask:
        current = current + 1 if bool(value) else 0
        longest = max(longest, current)
    return longest


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--trace", type=Path, required=True)
    parser.add_argument("--joint-limit-candidates", type=Path, required=True)
    parser.add_argument("--contract", type=Path)
    parser.add_argument("--fail-on-violation-joint", action="append", default=[])
    parser.add_argument("--maximum-horizontal-gravity-norm", type=float)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    limits_data = json.loads(args.joint_limit_candidates.read_text(encoding="utf-8"))
    limits = limits_data["joint_limits"]
    with np.load(args.trace, allow_pickle=False) as trace:
        names = tuple(str(value) for value in trace["joint_names"])
        raw = np.asarray(trace["target_position_rad"], dtype=np.float64)
        projected = np.asarray(
            trace["projected_target_position_rad"], dtype=np.float64
        )
        measured = np.asarray(trace["joint_position_rad"], dtype=np.float64)
        raw_action = np.asarray(trace["raw_action"], dtype=np.float64)
        startup_alpha = np.asarray(trace["startup_alpha"], dtype=np.float64)
        projected_gravity = np.asarray(trace["projected_gravity"], dtype=np.float64)

    expected_shape = (raw.shape[0], len(names))
    if raw.shape != expected_shape or projected.shape != expected_shape:
        raise ValueError("trace target arrays do not match joint_names")
    if measured.shape != expected_shape:
        raise ValueError("trace measured joint array does not match joint_names")
    if raw_action.shape != expected_shape:
        raise ValueError("trace raw action array does not match joint_names")
    if startup_alpha.shape != (raw.shape[0],):
        raise ValueError("trace startup alpha does not match target ticks")
    if projected_gravity.shape != (raw.shape[0], 3):
        raise ValueError("trace projected gravity must be an Nx3 array")
    if set(names) != set(limits):
        raise ValueError("trace and limit report do not cover the same joints")

    contract = None
    if args.contract:
        contract = json.loads(args.contract.read_text(encoding="utf-8"))
        if tuple(contract["joint_names"]) != names:
            raise ValueError("trace and contract joint ordering differ")

    summaries: dict[str, dict[str, float | int | list[float]]] = {}
    phase_masks = {
        "hold": startup_alpha == 0.0,
        "ramp": (startup_alpha > 0.0) & (startup_alpha < 1.0),
        "full": startup_alpha == 1.0,
    }
    for index, name in enumerate(names):
        low, high = (float(value) for value in limits[name]["soft_limit_rad_candidate"])
        values = raw[:, index]
        low_mask = values < low
        high_mask = values > high
        mask = low_mask | high_mask
        projected_values = projected[:, index]
        summaries[name] = {
            "soft_limit_rad": [low, high],
            "sample_count": int(values.size),
            "violation_count": int(np.count_nonzero(mask)),
            "violation_fraction": float(np.mean(mask)),
            "lower_violation_count": int(np.count_nonzero(low_mask)),
            "upper_violation_count": int(np.count_nonzero(high_mask)),
            "longest_consecutive_violation_ticks": longest_true_run(mask),
            "raw_target_min_rad": float(np.min(values)),
            "raw_target_max_rad": float(np.max(values)),
            "maximum_lower_overshoot_rad": float(
                np.max(np.maximum(low - values, 0.0))
            ),
            "maximum_upper_overshoot_rad": float(
                np.max(np.maximum(values - high, 0.0))
            ),
            "projected_target_min_rad": float(np.min(projected_values)),
            "projected_target_max_rad": float(np.max(projected_values)),
            "measured_min_rad": float(np.min(measured[:, index])),
            "measured_max_rad": float(np.max(measured[:, index])),
            "raw_action_min": float(np.min(raw_action[:, index])),
            "raw_action_max": float(np.max(raw_action[:, index])),
        }
        if contract is not None:
            default = float(contract["default_joint_pos"][index])
            action_scale = float(contract["action_scale"][index])
            summaries[name].update(
                {
                    "contract_default_joint_pos_rad": default,
                    "contract_action_scale_rad": action_scale,
                    "contract_action_minus_one_target_rad": default - action_scale,
                    "contract_action_plus_one_target_rad": default + action_scale,
                }
            )
        summaries[name]["raw_action_by_startup_phase"] = {
            phase: {
                "sample_count": int(np.count_nonzero(mask)),
                "minimum": float(np.min(raw_action[mask, index])) if np.any(mask) else None,
                "maximum": float(np.max(raw_action[mask, index])) if np.any(mask) else None,
                "mean": float(np.mean(raw_action[mask, index])) if np.any(mask) else None,
            }
            for phase, mask in phase_masks.items()
        }

    ranked = sorted(
        summaries,
        key=lambda name: (
            summaries[name]["violation_count"],
            max(
                summaries[name]["maximum_lower_overshoot_rad"],
                summaries[name]["maximum_upper_overshoot_rad"],
            ),
        ),
        reverse=True,
    )
    errors = []
    unknown_gate_joints = set(args.fail_on_violation_joint) - set(names)
    if unknown_gate_joints:
        raise ValueError(f"unknown gated joints: {sorted(unknown_gate_joints)}")
    for name in args.fail_on_violation_joint:
        count = int(summaries[name]["violation_count"])
        if count:
            errors.append(f"{name} has {count} raw target soft-limit violations")
    horizontal_gravity_norm = np.linalg.norm(projected_gravity[:, :2], axis=1)
    maximum_horizontal_gravity_norm = float(np.max(horizontal_gravity_norm))
    if args.maximum_horizontal_gravity_norm is not None:
        limit = float(args.maximum_horizontal_gravity_norm)
        if not np.isfinite(limit) or not 0.0 < limit < 1.0:
            raise ValueError("maximum horizontal gravity norm must be finite and in (0, 1)")
        if maximum_horizontal_gravity_norm > limit:
            errors.append(
                "maximum horizontal projected-gravity norm "
                f"{maximum_horizontal_gravity_norm:.6f} exceeds {limit:.6f}"
            )

    report = {
        "mode": "offline_policy_joint_soft_limit_clamp_analysis",
        "trace": str(args.trace.resolve()),
        "joint_limit_candidates": str(args.joint_limit_candidates.resolve()),
        "contract": str(args.contract.resolve()) if args.contract else None,
        "tick_count": int(raw.shape[0]),
        "violating_joint_count": sum(
            summaries[name]["violation_count"] > 0 for name in names
        ),
        "maximum_horizontal_projected_gravity_norm": maximum_horizontal_gravity_norm,
        "maximum_horizontal_projected_gravity_norm_limit": (
            args.maximum_horizontal_gravity_norm
        ),
        "fail_on_violation_joints": args.fail_on_violation_joint,
        "ranked_violating_joints": [
            name for name in ranked if summaries[name]["violation_count"] > 0
        ],
        "joints": summaries,
        "errors": errors,
        "passed": not errors,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    if errors:
        raise SystemExit("policy startup readiness gates failed")


if __name__ == "__main__":
    main()
