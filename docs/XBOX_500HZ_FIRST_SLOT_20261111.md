# 500 Hz candidate: first-slot wake-up correction

Scope: independent `xbox_500hz` checkout only. The stable runtime, model,
gains, limits, schedule phases and 500 microsecond slot budget are unchanged.
No motor enable or hardware transmission was performed while developing this fix.

## Evidence

Jetson preflight `20261111_170131` exited before final posture evaluation.
Its transport report `native_policy_ipc_transport_upperpd_20261111_170132.json`
recorded exactly one overrun, at slot 0:

- Wake-up lateness: 0.300614 ms.
- CAN transmit work: 0.208355 ms.
- CAN receive work: 0.008897 ms.
- IPC/preview work: 0.007200 ms.
- Total from the original deadline: 0.525066 ms (limit: 0.500000 ms).
- All 31 motors: 4000 TX and 4000 RX; no skipped slots or enables.

The first deadline was scheduled 100 ms ahead. Initialization did not consume
that whole interval; the measured wake-up lateness and first transmit cost
together exhausted the budget. CPU5 uses `schedutil`; its available `c7`
idle state advertises 5000 us latency. This makes cold wake-up a plausible
contributor, not proof of the particular idle state or kernel cause in that run.
We have not identified the source of the first 208 us transmit cost.

## Change

Only slot 0 in the experimental shadow and powered loops wakes 1 ms before
its already chosen deadline, then waits actively until that same deadline.
This adds at most approximately 1 ms of CPU work per run (excluding scheduler
preemption), not per slot. Normal subsequent absolute sleeps are unchanged.
The startup delay is still 100 ms for shadow / 20 ms for powered operation.

No CAN command is sent early. An oversleep is never rebased, a first-slot
overrun still fails, and no warm-up CAN samples are removed from the report.
The powered expired-slot guard, shutdown path and 0.5 ms budget remain intact.
The wait checks shutdown requests in the powered path. Reports now retain
the first slot separately even if a later slot is worse.

## Verification

- WSL Debug CTest: all eight tests passed, including unchanged-deadline,
  late-wakeup rejection, cancellation, schedule, payload and shutdown tests.
- Jetson CPU5 no-hardware benchmark: 100 interleaved trials per method,
  with 100 ms before every deadline. Scheduler was SCHED_OTHER, not the
  production SCHED_FIFO; no CAN/SPI/IMU/actor work was performed.

| Wake-up only | Median ms | P99 ms | Maximum ms |
| --- | ---: | ---: | ---: |
| Original absolute sleep | 0.099075 | 0.485402 | 0.486842 |
| First-slot 1 ms early wake | 0.000211 | 0.000285 | 0.000287 |

Benchmark source: `native/tests/benchmark_first_slot_wait.cpp`.
Onsite evidence: `reports/first_slot_review_20261111/wakeup_comparison.json`.
This is evidence for the waiting strategy only, not a new full-chain or
powered qualification. Previous successful 120 s shadow evidence remains
historical; the rebuilt candidate must pass the zero-gain preflight again.

After rebuilding, reinstall CAP_SYS_NICE on the experimental binary only,
then use `run_xbox_500hz_powered_120s_on_253.sh --preflight-only`. Do not
proceed to powered testing based on the CPU-only benchmark.
