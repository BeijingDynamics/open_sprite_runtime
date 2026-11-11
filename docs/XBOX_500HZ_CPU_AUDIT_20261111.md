# CPU wakeup investigation: zero gain only

The 120 s shared-feedback test `xbox_500hz_full_shadow_120s_1iu95nzg`
completed all 6000 actor frames and all 60000 TX/RX per motor, but FAILED the
unchanged 0.5 ms transport budget once. Slot 6271 woke 0.408909 ms late and
finished 0.523667 ms after its deadline. Its receive work was only 0.030753 ms.
The maximum receive work over the entire run was 0.126439 ms; 1,859,992 frames
executed the RMS/thermal monitors (the last eight replies were drained at exit).

This evidence points to wakeup delay in that run, not a new monitor stall.
It does not establish whether the delay was idle exit, interrupt handling,
late timer delivery, or waiting for CPU time.

## Read-only inventory

Jetson: Linux 5.15.199-tegra, PREEMPT (not a claim of PREEMPT_RT).
CAN: CPU5, SCHED_FIFO priority50. CPU5 allows WFI and C7; C7 advertises
5000 us exit latency. `schedutil` ranges from729600 to1344000 kHz and shares
policy4 with CPU4 (the actor). No CPUs are isolated. IRQ118 (SPI) and124
(USB xHCI) currently have effective affinity CPU0. Counts are cumulative,
not proof of the interrupt activity at the failed instant.

## Controlled test

Run from the independent experimental checkout, robot supported and all
motors powered but disabled. Keep stationary throughout, approximately6min:

```bash
cd /home/tony/open_sprite_runtime_xbox_500hz
sudo /home/tony/open_sprite_runtime/.venv/bin/python tools/run_xbox500_cpu_audit.py ZERO_GAIN_CPU_AUDIT_31_DISABLED
```

The tool runs the existing, sealed 120 s feedback-workload shadow twice:

- A: current power-management state.
- B: a temporary0 us CPU latency QoS request through `/dev/cpu_dma_latency`.

The request is system-wide while its descriptor is held. Closing the descriptor
removes only this request; no sysctl, governor, CPU-idle disable file, IRQ
affinity, boot setting, or stable-runtime file is changed. This is a diagnostic
condition, not yet a recommended production configuration.

Before either phase, a five-second, ordinary-priority CPU5 timer test checks
actual kernel trace formats and volume, without CAN or SPI access. At least
100 complete timer/wakeup/switch cycles and loss-free capture are required.

Both runs use a dedicated ftrace instance limited to CPU5. Recording starts
only after the CAN PID is detected, and stops after a requested20s (up to one
0.5s supervisor polling interval later). The kernel buffer is32MiB on CPU5;
other CPU buffers are minimal. Files have a64MiB compressed-file cap. No trace reader runs
while recording: the stopped buffer is read only after the120s phase finishes.
Scheduler switches/wakeups and timer starts are filtered to the target PID;
timer expiry, idle, IRQ and softirq context remains visible on CPU5.
The global tracer is untouched and the instance is removed on exit.
Trace overhead is real and included in the unchanged runtime gates. Check
trace_status for losses/errors before interpreting attribution. Missing or
nonzero loss counters invalidate the trace and stop the next phase. An A run that
finishes with a timing rejection is retained as FAILED and can proceed to B;
early hardware/status or incomplete-run faults stop the sequence.

Only the initial window is traced, not the complete120s. A late failure outside
this window cannot be attributed using this trace. Wakeup events emitted on
another CPU are also outside this capture; missing wakeups must not be treated
as zero scheduler delay. The native timing report still covers the full run.

Export is streaming gzip (`cpu5.trace.gz`), after the test and tracing have
stopped. Compression never runs alongside CAN in this wrapper. It retains
every recorded event, uses64KiB input chunks, and records compressed bytes,
expanded bytes and the SHA256 of the original text. The analyzer accepts both
plain and gzip traces. Compression does not waive kernel buffer loss checks.
Completed native/actor reports are saved independently before trace validation;
an invalid trace still prevents phase B and cannot qualify the runtime.

## Failed collector and correction

`xbox500_cpu_ab_d3ectxis/A_current` hit the old256MiB cap in about13s,
before CAN started (still in IMU calibration). In its first100000 trace lines,
78365 were emitted in the collector's `python-12744` context. Repeated IRQ-work
wakeups and scheduler events show the live reader tracing its own activity.
The failure is a collector defect, not evidence that CAN or CPU-idle latency
exceeded a gate. Phase B never started. The original evidence is retained.
The revised collector removes the live trace_pipe reader entirely, filters
the target and records a bounded window before reading anything out. No CAN
code, PD, safety threshold, or stable-runtime file is changed.

The next run, `xbox500_cpu_ab_o3g4eaa5`, completed its120s zero-gain phase A:
6000 actor frames,60000 TX/RX per motor, no CAN errors, no enable or nonzero
gain/torque attempts. It still FAILED: slot0 used0.505419ms, including
0.467689ms TX and only0.000162ms wake delay. Do not label this a wakeup fault.
The stopped kernel trace had1041078 events,30689776 binary bytes, and zero
overruns/drops. Exporting expanded text hit64MiB after roughly12.5s of the
20s capture. Softirq entries/exits dominated the exported portion, consistent
with real network/CAN work rather than the previous self-tracing defect.
The tail was not exported and cannot be recovered from the removed instance.
Phase B did not start. Streaming compression addresses export size without
dropping those events or changing the control budget or kernel buffer size.

`analyze_xbox500_cpu_trace.py` matches the CAN PID's hrtimer deadlines,
expiry timestamps, scheduler wakeup and switch-in. It reports callback lateness
separately from wake-to-run delay. Kernel event timestamps have finite precision;
no single event or A/B pair proves a permanent timing guarantee. Inspect idle
and IRQ events around the worst timestamps. Unsupported formats produce no
attribution rather than an invented result.

No CAN binary rebuild or new capability grant is required for this diagnostic.
No motor enable, nonzero gains, policy actuation, or automatic powered test.

## Completed A/B: 684mln8q

Source: `reports/xbox500_cpu_ab_684mln8q`, collected2026-11-11.
Both120s phases completed, with6000 actor frames,60000 TX and RX per motor,
byte-exact actor replay, and zero CAN errors, enable attempts or nonzero
gain/torque attempts. Both traces have zero buffer overruns/drops and export
errors. The full evidence directory was copied to the Windows project under
`work/xbox_500hz_review_20261111/`.

| Metric | A current | B temporary QoS0 |
| --- | ---: | ---: |
| Qualification | FAILED | PASSED |
| Overrun count | 1 (slot0) | 0 |
| Worst complete slot, ms | 0.524556 | 0.472623 |
| First-slot TX, ms | 0.484008 | 0.436049 |
| First-slot wake, ms | 0.000100 | 0.000128 |
| Maximum wake across120s, ms | 0.050251 | 0.186956 |
| Maximum RX across120s, ms | 0.149634 | 0.108925 |
| Matched timer cycles in trace | 40267 | 40215 |

The A failure is first-slot TX dominated, not delayed wakeup. B passing is
not proof that deep idle caused the failure: B's maximum wake delay was larger.
There was no C7 entry in either captured20s window; both show only WFI.
The wider before/after snapshots include IMU calibration and report processing:
C7 usage rose601 in A and0 in B. This verifies a change in idle behavior over
the broader phase, not causation at the failing slot. Fixed A-then-B ordering
also does not control kernel/device/cache warm-state effects.

The worst captured B wake had a timer callback158.628us late, followed by
only4us from scheduler wakeup to switch-in. The callback occurred in
`kworker/u12:15` context. These events locate delay before the wakeup, but do
not prove why the callback was late (no IRQ-off duration trace was taken).
The historical0.408909ms wake outlier did not recur in this pair.

Important coverage limit: CAN PID detection happens after process launch, so
the tracing window cannot be assumed to contain slot0. This trace does not
explain the internals of first-slot TX. The native aggregate timing does.

Next diagnostic priority is first-slot preparation versus individual send
system-call wall/CPU time, plus preemption/page-fault evidence. Instrument only
the independent candidate, preserve the complete first-slot budget, and use
offline then zero-gain validation. A reversed-order/repeated QoS comparison
would be needed before recommending QoS. No production QoS change, powered
qualification, threshold relaxation, or stable-runtime change follows from
this single pair. The temporary request was released by the wrapper.

## Reference Links

- [Linux CPUIdle and PM QoS](https://www.kernel.org/doc/html/v6.8/admin-guide/pm/cpuidle.html):
  the descriptor controls the lifetime of the latency request.
- [NVIDIA Jetson Orin power management](https://docs.nvidia.com/jetson/archives/r36.4.3/DeveloperGuide/SD/PlatformPowerAndPerformance/JetsonOrinNanoSeriesJetsonOrinNxSeriesAndJetsonAgxOrinSeries.html):
  supported idle states, cpufreq and idle configuration interfaces.
