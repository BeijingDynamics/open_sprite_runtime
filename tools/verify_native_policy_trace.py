#!/usr/bin/env python3
"""Deterministically replay actor observations from a native live-shadow trace."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from open_sprite_runtime.contracts import PolicyContract
from open_sprite_runtime.shadow import load_actor


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--contract", required=True)
    parser.add_argument("--trace", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--maximum-abs-error", type=float, default=1.0e-6)
    args = parser.parse_args()

    contract = PolicyContract.load(args.contract)
    actor = load_actor(contract)
    data = np.load(args.trace, allow_pickle=False)
    observation = data["observation"]
    expected_action = data["raw_action"]
    if observation.ndim != 2 or observation.shape[1] != contract.data["actor_observation_dim"]:
        raise SystemExit("trace observation shape does not match the policy contract")
    if expected_action.shape != (observation.shape[0], 31):
        raise SystemExit("trace action shape is invalid")
    maximum = 0.0
    for obs, expected in zip(observation, expected_action, strict=True):
        actual = actor.run(obs)
        maximum = max(maximum, float(np.max(np.abs(actual - expected))))
    report = {
        "mode": "offline_native_policy_trace_deterministic_replay",
        "trace": str(Path(args.trace).resolve()),
        "tick_count": int(observation.shape[0]),
        "maximum_abs_action_error": maximum,
        "maximum_allowed_abs_error": args.maximum_abs_error,
        "passed": maximum <= args.maximum_abs_error,
    }
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    if not report["passed"]:
        raise SystemExit("native policy trace replay failed")


if __name__ == "__main__":
    main()
