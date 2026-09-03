"""No-transmit policy replay used before connecting a CAN backend."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from .contracts import PolicyContract


def replay_mujoco_trace(
    contract_path: str | Path,
    trace_path: str | Path,
    sample_stride: int = 10,
) -> dict[str, object]:
    try:
        import onnxruntime as ort
    except ImportError as exc:
        raise RuntimeError(
            "trace replay requires onnxruntime; install the declared project dependency"
        ) from exc
    if sample_stride <= 0:
        raise ValueError("sample_stride must be positive")
    contract = PolicyContract.load(contract_path)
    contract_dir = contract.path.parent
    policy_path = Path(contract.data["policy_onnx"])
    if not policy_path.is_absolute():
        policy_path = contract_dir / policy_path
    trace = json.loads(Path(trace_path).read_text(encoding="utf-8"))
    session = ort.InferenceSession(str(policy_path), providers=["CPUExecutionProvider"])
    input_name = session.get_inputs()[0].name
    max_abs = 0.0
    rms_sum = 0.0
    value_count = 0
    sampled = 0
    skipped_handoff = 0
    handoff_seconds = float(contract.data.get("deployment_handoff_seconds", 0.0))
    command_slice = next(
        term for term in contract.data["observation_terms"] if term["name"] == "velocity_commands"
    )
    for row in trace[::sample_stride]:
        if float(row.get("time_s", 0.0)) < handoff_seconds:
            skipped_handoff += 1
            continue
        observation = np.asarray(row["observation"], dtype=np.float32)
        expected_action = np.asarray(row["action"], dtype=np.float32)
        if observation.shape != (contract.data["actor_observation_dim"],):
            raise ValueError(f"unexpected observation shape {observation.shape}")
        if expected_action.shape != (contract.data["action_dim"],):
            raise ValueError(f"unexpected action shape {expected_action.shape}")
        if not np.all(np.isfinite(observation)) or not np.all(np.isfinite(expected_action)):
            raise ValueError("trace contains non-finite policy data")
        command = observation[command_slice["start"] : command_slice["end"]]
        np.testing.assert_allclose(command, row["command"], atol=1.0e-6, rtol=0.0)
        actual_action = session.run(None, {input_name: observation[None, :]})[0][0]
        error = actual_action - expected_action
        max_abs = max(max_abs, float(np.max(np.abs(error))))
        rms_sum += float(error @ error)
        value_count += error.size
        sampled += 1
    return {
        "mode": "shadow_replay_no_hardware_tx",
        "trace_rows": len(trace),
        "sample_stride": sample_stride,
        "sampled_policy_ticks": sampled,
        "handoff_seconds": handoff_seconds,
        "skipped_handoff_samples": skipped_handoff,
        "actor_observation_dim": int(contract.data["actor_observation_dim"]),
        "action_dim": int(contract.data["action_dim"]),
        "horizontal_base_velocity_present": bool(
            contract.data["observation_has_horizontal_base_velocity"]
        ),
        "onnx_action_max_abs_error": max_abs,
        "onnx_action_rms_error": float(np.sqrt(rms_sum / max(value_count, 1))),
        "passed": max_abs <= 1.0e-4,
    }
