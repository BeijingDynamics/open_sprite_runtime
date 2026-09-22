#!/usr/bin/env python3
"""Attribute first-tick actor differences to observation term groups."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from open_sprite_runtime.contracts import PolicyContract
from open_sprite_runtime.policy_shadow import PolicyObservationHistory
from open_sprite_runtime.shadow import load_actor


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--trace", type=Path, required=True)
    parser.add_argument("--contract", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    contract = PolicyContract.load(args.contract)
    names = tuple(contract.data["joint_names"])
    actor = load_actor(contract)
    history = PolicyObservationHistory(contract)
    history.append(
        np.asarray(contract.data["default_joint_pos"], dtype=np.float32),
        np.zeros(31, dtype=np.float32),
        np.zeros(31, dtype=np.float32),
        np.zeros(3, dtype=np.float32),
        np.asarray([0.0, 0.0, -1.0], dtype=np.float32),
    )
    nominal = history.observation(np.zeros(3, dtype=np.float32))

    with np.load(args.trace, allow_pickle=False) as trace:
        trace_names = tuple(str(value) for value in trace["joint_names"])
        if trace_names != names:
            raise ValueError("trace and contract joint ordering differ")
        live = np.asarray(trace["observation"][0], dtype=np.float32)
        recorded_action = np.asarray(trace["raw_action"][0], dtype=np.float32)
        measured_joint_position = np.asarray(
            trace["joint_position_rad"][0], dtype=np.float64
        )
        measured_projected_gravity = np.asarray(
            trace["projected_gravity"][0], dtype=np.float64
        )

    offset = np.asarray(contract.data["action_offset"], dtype=np.float64)
    scale = np.asarray(contract.data["action_scale"], dtype=np.float64)

    def evaluate(observation: np.ndarray) -> dict:
        action = np.asarray(actor.run(observation), dtype=np.float64)
        target = offset + scale * action
        return {
            "action": {name: float(action[index]) for index, name in enumerate(names)},
            "target_position_rad": {
                name: float(target[index]) for index, name in enumerate(names)
            },
        }

    scenarios = {
        "nominal_default_pose": evaluate(nominal),
        "live_first_tick": evaluate(live),
    }
    for term in contract.data["observation_terms"]:
        name = term["name"]
        start = int(term["start"])
        end = int(term["end"])
        nominal_plus_live = nominal.copy()
        nominal_plus_live[start:end] = live[start:end]
        live_minus_term = live.copy()
        live_minus_term[start:end] = nominal[start:end]
        scenarios[f"nominal_plus_live_{name}"] = evaluate(nominal_plus_live)
        scenarios[f"live_minus_{name}"] = evaluate(live_minus_term)

    selected_outputs = (
        "left_ankle_pitch_joint",
        "right_ankle_pitch_joint",
        "left_ankle_roll_joint",
        "right_ankle_roll_joint",
        "left_elbow_joint",
    )
    joint_pos_term = next(
        term for term in contract.data["observation_terms"] if term["name"] == "joint_pos"
    )
    gravity_term = next(
        term
        for term in contract.data["observation_terms"]
        if term["name"] == "projected_gravity"
    )
    history_length = int(joint_pos_term["history_length"])
    joint_start = int(joint_pos_term["start"])
    gravity_start = int(gravity_term["start"])

    live_action = scenarios["live_first_tick"]["action"]
    nominal_action = scenarios["nominal_default_pose"]["action"]
    joint_position_attribution = {}
    for input_index, input_name in enumerate(names):
        nominal_plus = nominal.copy()
        live_minus = live.copy()
        for frame in range(history_length):
            observation_index = joint_start + frame * len(names) + input_index
            nominal_plus[observation_index] = live[observation_index]
            live_minus[observation_index] = nominal[observation_index]
        plus_action = evaluate(nominal_plus)["action"]
        minus_action = evaluate(live_minus)["action"]
        joint_position_attribution[input_name] = {
            "nominal_plus_live_input_delta": {
                output: float(plus_action[output] - nominal_action[output])
                for output in selected_outputs
            },
            "live_minus_input_delta": {
                output: float(minus_action[output] - live_action[output])
                for output in selected_outputs
            },
        }

    gravity_attribution = {}
    for axis, axis_name in enumerate(("x", "y", "z")):
        nominal_plus = nominal.copy()
        live_minus = live.copy()
        for frame in range(int(gravity_term["history_length"])):
            observation_index = gravity_start + frame * 3 + axis
            nominal_plus[observation_index] = live[observation_index]
            live_minus[observation_index] = nominal[observation_index]
        plus_action = evaluate(nominal_plus)["action"]
        minus_action = evaluate(live_minus)["action"]
        gravity_attribution[axis_name] = {
            "nominal_plus_live_axis_delta": {
                output: float(plus_action[output] - nominal_action[output])
                for output in selected_outputs
            },
            "live_minus_axis_delta": {
                output: float(minus_action[output] - live_action[output])
                for output in selected_outputs
            },
        }

    reproduced = np.asarray(
        [scenarios["live_first_tick"]["action"][name] for name in names]
    )
    report = {
        "mode": "offline_policy_observation_counterfactual_analysis",
        "trace": str(args.trace.resolve()),
        "contract": str(args.contract.resolve()),
        "inference_backend": actor.backend,
        "first_tick_reproduction_max_abs_action_error": float(
            np.max(np.abs(reproduced - recorded_action))
        ),
        "live_first_tick_inputs": {
            "joint_position_rad": {
                name: float(measured_joint_position[index])
                for index, name in enumerate(names)
            },
            "joint_position_delta_from_contract_default_rad": {
                name: float(
                    measured_joint_position[index]
                    - float(contract.data["default_joint_pos"][index])
                )
                for index, name in enumerate(names)
            },
            "projected_gravity": measured_projected_gravity.tolist(),
        },
        "joint_position_single_input_attribution": joint_position_attribution,
        "projected_gravity_single_axis_attribution": gravity_attribution,
        "scenarios": scenarios,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
