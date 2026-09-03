"""No-transmit policy replay used before connecting a CAN backend."""

from __future__ import annotations

import json
from pathlib import Path
import time

import numpy as np

from .contracts import PolicyContract
from .safety import RuntimeMode, SafetyInputs, SafetyLimits, SafetySupervisor


def expand_zero_order_hold(actions: np.ndarray, updates_per_policy: int) -> np.ndarray:
    """Expand policy-rate rows into state-rate held targets."""
    values = np.asarray(actions)
    if values.ndim != 2:
        raise ValueError("actions must be a 2-D array")
    if updates_per_policy <= 0:
        raise ValueError("updates_per_policy must be positive")
    return np.repeat(values, updates_per_policy, axis=0)


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


def replay_multirate_mujoco_trace(
    contract_path: str | Path,
    trace_path: str | Path,
    *,
    policy_hz: int,
    state_hz: int,
    maximum_state_age_ms: float,
    maximum_command_age_ms: float,
    maximum_policy_overrun_ms: float,
) -> dict[str, object]:
    """Exercise every recorded policy tick and every synthetic 500 Hz safety tick."""
    try:
        import onnxruntime as ort
    except ImportError as exc:
        raise RuntimeError(
            "trace replay requires onnxruntime; install the declared project dependency"
        ) from exc
    if policy_hz <= 0 or state_hz <= 0 or state_hz % policy_hz:
        raise ValueError("state_hz must be an integer multiple of policy_hz")
    contract = PolicyContract.load(contract_path)
    if int(round(1.0 / float(contract.data["policy_dt"]))) != policy_hz:
        raise ValueError("runtime policy_hz does not match the frozen policy contract")
    updates_per_policy = state_hz // policy_hz
    state_period_ns = round(1.0e9 / state_hz)
    limits = SafetyLimits(
        maximum_state_age_ms=maximum_state_age_ms,
        maximum_command_age_ms=maximum_command_age_ms,
        maximum_policy_overrun_ms=maximum_policy_overrun_ms,
    )
    supervisor = SafetySupervisor(RuntimeMode.SHADOW, limits)

    contract_dir = contract.path.parent
    policy_path = Path(contract.data["policy_onnx"])
    if not policy_path.is_absolute():
        policy_path = contract_dir / policy_path
    trace = json.loads(Path(trace_path).read_text(encoding="utf-8"))
    session = ort.InferenceSession(str(policy_path), providers=["CPUExecutionProvider"])
    input_name = session.get_inputs()[0].name
    handoff_seconds = float(contract.data.get("deployment_handoff_seconds", 0.0))
    rows = [row for row in trace if float(row.get("time_s", 0.0)) >= handoff_seconds]
    if not rows:
        raise ValueError("trace has no policy rows after deployment handoff")

    # Warm the execution provider before measuring policy latency.
    first_observation = np.asarray(rows[0]["observation"], dtype=np.float32)
    session.run(None, {input_name: first_observation[None, :]})
    actions: list[np.ndarray] = []
    target_rows: list[np.ndarray] = []
    inference_ms: list[float] = []
    max_abs_action_error = 0.0
    action_scale = np.asarray(contract.data["action_scale"], dtype=np.float32)
    action_offset = np.asarray(contract.data["action_offset"], dtype=np.float32)
    for row in rows:
        observation = np.asarray(row["observation"], dtype=np.float32)
        expected_action = np.asarray(row["action"], dtype=np.float32)
        if observation.shape != (contract.data["actor_observation_dim"],):
            raise ValueError(f"unexpected observation shape {observation.shape}")
        if not np.all(np.isfinite(observation)):
            raise ValueError("trace contains non-finite observations")
        started_ns = time.perf_counter_ns()
        action = session.run(None, {input_name: observation[None, :]})[0][0]
        finished_ns = time.perf_counter_ns()
        duration_ms = (finished_ns - started_ns) / 1.0e6
        inference_ms.append(duration_ms)
        max_abs_action_error = max(
            max_abs_action_error, float(np.max(np.abs(action - expected_action)))
        )
        target = action_offset + action_scale * action
        if not np.all(np.isfinite(target)):
            raise ValueError("actor produced non-finite joint targets")
        actions.append(action)
        target_rows.append(target)

    targets = np.asarray(target_rows, dtype=np.float32)
    held_targets = expand_zero_order_hold(targets, updates_per_policy)
    within_policy_delta = held_targets.reshape(
        len(targets), updates_per_policy, contract.data["action_dim"]
    )[:, 1:, :] - targets[:, None, :]
    hold_max_abs_delta = float(np.max(np.abs(within_policy_delta)))

    tx_permitted_count = 0
    unexpected_blockers: set[str] = set()
    now_ns = 1_000_000_000
    for policy_index, duration_ms in enumerate(inference_ms):
        command_timestamp_ns = now_ns
        for substep in range(updates_per_policy):
            tick_ns = now_ns + substep * state_period_ns
            decision = supervisor.evaluate(
                SafetyInputs(
                    now_ns=tick_ns,
                    state_timestamp_ns=tick_ns,
                    command_timestamp_ns=command_timestamp_ns,
                    policy_overrun_ms=duration_ms,
                    allow_hardware_tx=True,
                    hardware_configured=True,
                    left_ankle_calibrated=True,
                    right_ankle_calibrated=True,
                    imu_valid=True,
                    estop_healthy=True,
                )
            )
            tx_permitted_count += int(decision.hardware_tx_permitted)
            unexpected_blockers.update(
                blocker
                for blocker in decision.blockers
                if blocker not in ("mode_shadow_no_tx", "policy_overrun")
            )
        now_ns += updates_per_policy * state_period_ns

    latency = np.asarray(inference_ms, dtype=np.float64)
    policy_overruns = int((latency > maximum_policy_overrun_ms).sum())
    passed = (
        max_abs_action_error <= 1.0e-4
        and hold_max_abs_delta == 0.0
        and tx_permitted_count == 0
        and not unexpected_blockers
    )
    return {
        "mode": "deterministic_multirate_shadow_no_hardware_tx",
        "policy_hz": policy_hz,
        "state_hz": state_hz,
        "state_updates_per_policy": updates_per_policy,
        "policy_ticks": len(rows),
        "state_ticks": len(rows) * updates_per_policy,
        "target_semantics": "zero_order_hold",
        "target_hold_max_abs_delta": hold_max_abs_delta,
        "hardware_tx_permitted_count": tx_permitted_count,
        "unexpected_safety_blockers": sorted(unexpected_blockers),
        "onnx_action_max_abs_error": max_abs_action_error,
        "policy_inference_ms": {
            "p50": float(np.quantile(latency, 0.50)),
            "p95": float(np.quantile(latency, 0.95)),
            "p99": float(np.quantile(latency, 0.99)),
            "max": float(latency.max()),
            "limit": maximum_policy_overrun_ms,
            "overrun_count": policy_overruns,
            "host_diagnostic_only_not_sbc_certification": True,
        },
        "horizontal_base_velocity_present": bool(
            contract.data["observation_has_horizontal_base_velocity"]
        ),
        "passed": passed,
    }
