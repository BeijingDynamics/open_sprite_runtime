# Experimental 500 Hz receive workload review

Stable `/home/tony/open_sprite_runtime` is untouched. This change is confined
to `xbox_500hz`; gains, model, targets, 500 Hz motor scheduling, 50 Hz actor,
800 Hz IMU and the 0.5 ms transport gate are unchanged. No powered retest is
authorized by the software checks below.

## Observed failure

`native_protected_policy_admission_icm_20261111_172018.json` reports:

- First slot: 0.373830 ms, passing.
- Slot 9979 (about 4.99 s): 0.522805 ms total; receive processing 0.414860 ms.
- Wake lateness at the failed slot: 0.010886 ms.
- All 31 motors confirmed disabled at exit.

The old log cannot separate kernel read time, batched feedback, descheduling,
and monitor calculations. It does not establish the root cause. In particular,
removing allocations is not proof that allocations caused that particular stall.

## Corrections and diagnostic coverage

1. Experimental motors allocate fixed 500/5000-element RMS rings before the
   realtime loop. Updates allocate nothing and retain the old sum update order,
   one/ten-second windows and startup/reset semantics. Legacy modes still use
   their original deque implementation.
2. Experimental zero-gain shadow now uses the powered receive/temperature/RMS
   processing, but **requires disabled status** and still transmits only zero
   gains/torque. It also checks hard position, speed and torque feedback bounds.
   The old zero-gain loop did not exercise this workload.
3. Both paths record fixed-size receive diagnostics: frame counts by bus,
   monitor update counts, read syscall wall time, RMS/temperature wall time,
   total wall time and thread CPU time. Timing instrumentation is included in
   the original full-slot budget. The 0.2 ms slow-sample threshold only selects
   diagnostic examples; it is not a replacement pass/fail gate.
4. Per-bus 64-frame work budget and all existing powered protections remain.
   Slow samples are bounded to 16 plus a separately retained maximum.

Wall time minus thread CPU time helps identify time not running on the thread,
but is not an exact attribution of kernel scheduling versus interrupt time.

## Offline checks

`test_xbox_500hz_feedback` uses Unix socketpairs only, not CAN devices:

- Exact RMS maximum parity for 60,000 varying samples including reset and wraps.
- Original deque: 1881 allocation calls in this fixture; fixed updates: zero.
- 372,000 fake frames alternate disabled-shadow and enabled processing; every
  frame updates the monitors; feedback processing makes zero allocations.
- Enabled replies are rejected in shadow; temperature and receive budget
  faults remain rejected.
- Existing schedule, payload/ankle mapping, acknowledgement and shutdown tests
  remain required.

## Zero-gain qualification

The new optional selection uses the same rebuilt experimental binary as the
powered candidate, without enabling its actuation mode:

```bash
cd /home/tony/open_sprite_runtime_xbox_500hz
sudo setcap cap_sys_nice+ep candidates/xbox_500hz_powered_20261111/sprite_can_xbox500
sudo bash run_xbox_500hz_full_shadow_120s_on_253.sh ZERO_GAIN_XBOX_500HZ_FULL_31_DISABLED --feedback-candidate
```

Robot supported, all motors powered but disabled; stationary first 15 seconds.
Total time includes IMU calibration and shutdown, approximately 150 seconds.
No policy targets or nonzero gains/torques are sent to motors. The native actor
is evaluated at 50 Hz; its captured inputs are replayed offline for byte parity.

The runner additionally requires the shared feedback workload marker and at
least 99% of the expected 1,860,000 RMS updates, on top of unchanged timing,
CAN coverage and error checks. A successful old binary run cannot satisfy this
new workload gate. The old transport-only binary remains preserved.

At preparation time, hardware qualification of this revision is still pending.
Do not proceed to powered tests until the new evidence has been reviewed.
