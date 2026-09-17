"""Timing measurement for the 500 Hz state loop and 50 Hz policy tick."""

from __future__ import annotations

from dataclasses import dataclass, field
import time

import numpy as np


@dataclass
class TimingMonitor:
    period_ns: int
    maximum_start_jitter_ns: int
    maximum_work_duration_ns: int
    start_jitter_ns: list[int] = field(default_factory=list)
    work_duration_ns: list[int] = field(default_factory=list)

    def record(self, scheduled_ns: int, started_ns: int, finished_ns: int) -> None:
        if finished_ns < started_ns:
            raise ValueError("finished_ns precedes started_ns")
        self.start_jitter_ns.append(started_ns - scheduled_ns)
        self.work_duration_ns.append(finished_ns - started_ns)

    def report(self) -> dict[str, object]:
        if not self.start_jitter_ns:
            raise ValueError("no timing samples recorded")
        jitter_ms = np.abs(np.asarray(self.start_jitter_ns, dtype=np.float64)) / 1.0e6
        work_ms = np.asarray(self.work_duration_ns, dtype=np.float64) / 1.0e6
        jitter_limit_ms = self.maximum_start_jitter_ns / 1.0e6
        work_limit_ms = self.maximum_work_duration_ns / 1.0e6
        return {
            "samples": len(jitter_ms),
            "period_ms": self.period_ns / 1.0e6,
            "start_jitter_ms": {
                "p50": float(np.quantile(jitter_ms, 0.50)),
                "p95": float(np.quantile(jitter_ms, 0.95)),
                "p99": float(np.quantile(jitter_ms, 0.99)),
                "max": float(jitter_ms.max()),
            },
            "work_duration_ms": {
                "p50": float(np.quantile(work_ms, 0.50)),
                "p95": float(np.quantile(work_ms, 0.95)),
                "p99": float(np.quantile(work_ms, 0.99)),
                "max": float(work_ms.max()),
            },
            "start_jitter_limit_ms": jitter_limit_ms,
            "work_duration_limit_ms": work_limit_ms,
            "start_jitter_overrun_count": int((jitter_ms > jitter_limit_ms).sum()),
            "work_duration_overrun_count": int((work_ms > work_limit_ms).sum()),
        }


def run_host_timing_probe(duration_s: float, state_hz: int = 500) -> dict[str, object]:
    """Measure host scheduling only; this does not certify a future SBC."""
    if duration_s <= 0.0 or state_hz <= 0:
        raise ValueError("duration_s and state_hz must be positive")
    period_ns = round(1.0e9 / state_hz)
    monitor = TimingMonitor(
        period_ns=period_ns,
        maximum_start_jitter_ns=1_000_000,
        maximum_work_duration_ns=1_000_000,
    )
    deadline = time.perf_counter_ns() + period_ns
    sample_count = max(1, round(duration_s * state_hz))
    accumulator = 0
    for tick in range(sample_count):
        remaining_ns = deadline - time.perf_counter_ns()
        if remaining_ns > 100_000:
            time.sleep((remaining_ns - 50_000) / 1.0e9)
        while time.perf_counter_ns() < deadline:
            pass
        started = time.perf_counter_ns()
        accumulator ^= tick
        finished = time.perf_counter_ns()
        monitor.record(deadline, started, finished)
        deadline += period_ns
    report = monitor.report()
    report["host_probe_only_not_sbc_certification"] = True
    report["accumulator"] = accumulator
    return report
