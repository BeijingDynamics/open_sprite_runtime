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
other CPU buffers are minimal. Files have a64MiB text cap. No trace reader runs
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

`analyze_xbox500_cpu_trace.py` matches the CAN PID's hrtimer deadlines,
expiry timestamps, scheduler wakeup and switch-in. It reports callback lateness
separately from wake-to-run delay. Kernel event timestamps have finite precision;
no single event or A/B pair proves a permanent timing guarantee. Inspect idle
and IRQ events around the worst timestamps. Unsupported formats produce no
attribution rather than an invented result.

No CAN binary rebuild or new capability grant is required for this diagnostic.
No motor enable, nonzero gains, policy actuation, or automatic powered test.

## References

- [Linux CPUIdle and PM QoS](https://www.kernel.org/doc/html/v6.8/admin-guide/pm/cpuidle.html):
  the descriptor controls the lifetime of the latency request.
- [NVIDIA Jetson Orin power management](https://docs.nvidia.com/jetson/archives/r36.4.3/DeveloperGuide/SD/PlatformPowerAndPerformance/JetsonOrinNanoSeriesJetsonOrinNxSeriesAndJetsonAgxOrinSeries.html):
  supported idle states, cpufreq and idle configuration interfaces.
