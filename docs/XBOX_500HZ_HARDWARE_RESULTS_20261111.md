# All-motor 500 Hz experiment: hardware results

Date: 2026-11-11. Branch: `xbox_500hz`. Baseline: `4515206`.

## Conclusion

**The 31-motor 500 Hz runtime is operational and can walk on the robot.
The operator's comparison did not show a clear improvement over the stable
mixed-rate runtime. The baseline's walking onset felt more decisive.**

Retain the original mixed-rate walking runtime as the baseline. Preserve this
branch for investigation and reproducibility; do not promote it to the default
or describe higher bus frequency as a demonstrated gait-quality improvement.

The operator initially perceived a large improvement with all-motor500Hz.
After testing the baseline again, the operator revised that assessment: the
baseline also walked well and started more decisively. This final comparison,
not the initial impression, is the conclusion recorded here. It is qualitative
onsite feedback, not a blinded, randomized or statistically powered study.
It does not establish that500Hz is universally worse or better.

## What was compared

| Item | Stable mixed-rate runtime | Experimental runtime |
| --- | --- | --- |
| Four ankle motor command/feedback polls | 500 Hz each | 500 Hz each |
| Other27 motor command/feedback polls | 50 Hz each | 500 Hz each |
| Host ankle torque controller | 500 Hz | 500 Hz |
| Native ONNX policy | 50 Hz | 50 Hz |
| ICM45686 raw-six-axis estimator | 800 Hz nominal, SPI1MHz | Same |
| Native transport scheduler | 2 kHz | 2 kHz |
| Nominal total command frames/s | 3350 | 15500 |

Motor replies add approximately the same traffic again. The experiment does
not create new policy actions at500Hz. Targets remain held at the50Hz policy
rate; non-ankle PD remains inside the drives. Actor, deployed Kp/Kd, torque
caps, mechanical limits, motor mappings and differential calibration inputs
were not deliberately changed for this comparison. The independent candidate
checks its copied model/parameter files against the stable deployment.

This is a runtime experiment, not retraining. Startup handling and receive-loop
implementation also evolved during commissioning, so it is not a perfectly
controlled one-variable experiment. No permanent QoS/governor modification was
adopted as a result of the CPU diagnostic.

## Inspectable evidence

The raw reports remain in onsite/local backups; only this concise summary and
the diagnostic implementation are published here. Times below measure host
work, not CAN wire latency or intrinsic sensor delay.

### Zero-gain full-chain test

Report directory: `xbox_500hz_full_shadow_120s_e8f_16ug`.

- Separate disabled startup completed in200.108ms with100 TX/RX per motor.
- Strict120-second phase completed6000 actor frames and60000 TX/RX per motor.
- Zero CAN error-counter increments, skipped slots or transport overruns.
- Strict first slot:0.153159ms; worst slot:0.386835ms, below0.5ms.
- Offline actor replay was byte-exact. No enable or nonzero-gain/torque attempts.

### Powered walking test

Report: `native_protected_policy_admission_icm_20261111_185755.json`.
SHA256: `2c4f6b0befac4bf54004bfef77e0cd5952ff747d3ba3b60b4c939f1e03b7c18c`.

- Native report `passed=true`, with no reported errors.
- Duration120s; all31 motors requested at500Hz,60000 commands per motor.
- 6000 policy targets received; zero transport overruns or expired slots skipped.
- First active slot:0.336434ms; maximum active slot:0.404813ms.
- All31 motors verified disabled at exit. No automatic zero-position reset.
- Operator reported that the robot could walk, then compared it with the baseline.

Earlier powered attempts were not all successful. The18:51:46 report rejected
a0.553490ms active slot, and the18:55:06 report stopped on the right-elbow dynamic
guard. Those failures are not reclassified as passes. The later successful
run does not establish a guarantee against rare timing or hardware faults.

## Relevant implementation work

- Kept the complete0.5ms strict transport budget, including the first active slot.
- Preallocated experimental torque-RMS windows and exercised the shared feedback
  monitors under zero gain; window durations stayed unchanged at the new rate.
- Added bounded disabled startup before strict operation: separate2ms startup
  slot budget,2s settling deadline, and a consecutive clean0.5ms timing window.
  Startup samples and TX/RX counts remain in a separate report, not discarded.
- Preserved bounded receive/IPC work, target/feedback watchdogs, temperature and
  torque protection, and whole-body disable/verification on shutdown.
- Kept CPU tracing and temporary QoS comparisons as diagnostics. An A/B pass
  difference did not prove that deep sleep caused the earlier failures.

Details: [startup](XBOX_500HZ_DISABLED_STARTUP_20261111.md),
[receive processing](XBOX_500HZ_RECEIVE_REVIEW_20261111.md),
[CPU investigation](XBOX_500HZ_CPU_AUDIT_20261111.md),
[experiment and deployment](XBOX_500HZ_EXPERIMENT.md).

## Checkout and deployment scope

```bash
git clone --branch xbox_500hz https://github.com/BeijingDynamics/open_sprite_runtime.git open_sprite_runtime_xbox_500hz
```

Keep the stable checkout separate; the pinned baseline is commit `4515206`.
The experimental powered entry is `run_xbox_500hz_powered_120s_on_253.sh`.
The unchanged baseline entry is `run_sprite0825_xbox_native_turn120_on_253.sh`.
Neither should run concurrently with another motor controller.

These are site-specific commissioning entries, not a portable, ready-to-run
robot package. Generated binaries, deployment seals and local reports are not
provided by this branch. External G74 model3000 assets, matched contracts,
onsite motor/differential calibrations, IMU mounting/bias preparation, four CAN
links and approved real-time permissions must be provisioned and verified.
The builder also expects the recorded local baseline and prerequisite evidence.
Do not fabricate a seal or bypass a missing-evidence check on a fresh checkout.

Hardware tests require a battery supply, fall arrest, clear arm/walking space
and an independent power safety operator. LT permits motion; LB+RB held3s
requests disable but is not a replacement for independent power cutoff.
Zero commanded velocity is not proof of stable standing; fall arrest must
support the robot after disable. A new installation needs its own staged
zero-gain and powered validation.
