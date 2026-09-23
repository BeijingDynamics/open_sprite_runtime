"""Centered, amplitude-limited replay of a qualified MuJoCo policy trace."""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np
from numpy.typing import NDArray


Vector = NDArray[np.float64]


class CenteredActionTraceReplay:
    """Convert a policy action window into position deltas about a measured pose."""

    def __init__(
        self,
        actions: NDArray[np.float64],
        action_scale: Vector,
        amplitude_scale: float,
    ) -> None:
        if actions.ndim != 2 or actions.shape[1] != 31 or actions.shape[0] < 2:
            raise ValueError("replay actions must have shape [N, 31] with N >= 2")
        if action_scale.shape != (31,):
            raise ValueError("action_scale must have shape [31]")
        if not np.isfinite(actions).all() or not np.isfinite(action_scale).all():
            raise ValueError("replay actions and action_scale must be finite")
        if not math.isfinite(amplitude_scale) or not 0.0 < amplitude_scale <= 1.0:
            raise ValueError("amplitude_scale must be finite and in (0, 1]")
        self.actions = actions.copy()
        self.action_scale = action_scale.copy()
        self.amplitude_scale = float(amplitude_scale)
        self.mean_action = np.mean(self.actions, axis=0)

    @classmethod
    def load(
        cls,
        path: str | Path,
        action_scale: Vector,
        *,
        source_hz: float,
        start_seconds: float,
        duration_seconds: float,
        amplitude_scale: float,
    ) -> "CenteredActionTraceReplay":
        for name, value in (
            ("source_hz", source_hz),
            ("start_seconds", start_seconds),
            ("duration_seconds", duration_seconds),
        ):
            if not math.isfinite(value) or value <= 0.0 and name != "start_seconds":
                raise ValueError(f"{name} must be finite and positive")
        if start_seconds < 0.0:
            raise ValueError("start_seconds must be non-negative")
        records = json.loads(Path(path).read_text(encoding="utf-8"))
        if not isinstance(records, list):
            raise ValueError("replay trace must be a JSON list")
        start = int(round(start_seconds * source_hz))
        count = int(round(duration_seconds * source_hz))
        window = records[start : start + count]
        if len(window) != count:
            raise ValueError("requested replay window exceeds the trace")
        try:
            actions = np.asarray([record["action"] for record in window], dtype=np.float64)
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError("every replay record must contain a finite action vector") from exc
        return cls(actions, np.asarray(action_scale, dtype=np.float64), amplitude_scale)

    @property
    def sample_count(self) -> int:
        return int(self.actions.shape[0])

    def position_delta(self, tick: int) -> Vector:
        if tick < 0:
            raise ValueError("tick must be non-negative")
        index = min(tick, self.sample_count - 1)
        return (
            self.amplitude_scale
            * self.action_scale
            * (self.actions[index] - self.mean_action)
        )

    def report(self) -> dict:
        deltas = self.amplitude_scale * self.action_scale * (
            self.actions - self.mean_action
        )
        return {
            "enabled": True,
            "sample_count": self.sample_count,
            "amplitude_scale": self.amplitude_scale,
            "maximum_abs_position_delta_rad": float(np.max(np.abs(deltas))),
            "peak_to_peak_position_delta_rad_by_joint": np.ptp(deltas, axis=0).tolist(),
        }
