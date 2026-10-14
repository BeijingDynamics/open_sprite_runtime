# Software IMU walking candidate, 2026-10-14

User reports three walks markedly improved, essentially straight instead of
the previous left drift. Preserve this candidate before further changes.
This is visual acceptance, not external heading/position ground truth.

## Evidence backed up on Windows

Reports and NPZ traces are in workspace `work/walk40`.

| Run timestamp | Actor/controller passed | Frames | IPC receive max age ms | Inference max ms | Left/right ankle pitch target clamps |
| --- | --- | --- | --- | --- | --- |
| 20261014_130711 | yes/yes | 2001 | 9.900 | 5.929 | 7/0 |
| 20261014_130901 | yes/yes | 2001 | 10.521 | 6.788 | 22/221 |
| 20261014_131201 | yes/yes | 2001 | 9.383 | 8.021 | 4/160 |

All three CAN reports have zero deadline misses and no errors. All actor
reports also correspond to31/31 motors finally disabled; maximum MOS37C,
rotor31C across all three runs. All actor
reports confirm software-v2 input, no rejected IMU frames and completed
stand/ramp/walk/ramp-stop/final-stand timeline at vx0.10m/s.
Receive age above is NOT physical sensor latency or maximum consumed age.
Right ankle target clamps require subsequent phase-resolved analysis; they
do not by themselves demonstrate physical joint-limit impact.

Producer folders respectively LFrn7Igk, mH4VP4XM, VNgSE7Va: host gap maxima
11.8883,10.4711,15.0497ms. Producer `passed:false/capture interrupted` is the
wrapper's intentional stop after controller exit, not a walking failure.
Two other attempts exist: rR3Ydezl rejected nonstationary bias initialization;
35swohW8 ended during preflight without a powered admission report. Do not
count these as completed walks or silently discard their diagnostics.

Session-heading summaries end at0.4263,0.9334,0.5589rad; these integrate the
whole test (including startup/support handling), and are not independently
measured walking-only trajectory headings. Do not assert drift eliminated
from these summaries. `gyro_bias_subtracted:false` in session-heading means
that diagnostic integrator does not subtract again: source software-v2 gyro
already has frozen native startup bias removed.

## Preserve comparison boundary

- BNO055 ACCGYRO/P0,200Hz burst18 native producer,15s stationary initialization.
- Same installed mounting transform and actual-dt complementary estimator.
- Policy gyro is raw minus frozen bias, mount-rotated, no fusion correction.
- Existing Python ONNX powered client50Hz and native CAN/ankle loop retained.
- Existing policy, PD gains, torque caps, startup and shutdown unchanged.
- Entry: run_sprite0825_software_imu_walk40_on_253.sh.
- Native actor has passed zero-gain replay parity; NOT powered-qualified yet.

## Next

1. Keep this walking candidate as rollback; no gain/filter tuning now.
2. Replay these real walking recordings through native policy, compare
   observations, history, commands, actions and protected targets.
3. Audit native powered lifecycle against existing client (startup, stop,
   watchdogs, disable verification, logging), then zero-gain validation.
4. Only after that request a separately authorized powered native comparison.
5. Analyze remaining ankle clamp phases and walking-only heading separately;
   do not confuse improved feedback with proof of a unique past root cause.

Completed initial step2 on latest run131201:2001 frames passed offline native
sensor-to-protected-target replay, raw actor outputs error0, observations max
error1.0838e-17, final protected targets max error2.7756e-17. Recorded commands
used; live profile/IPC/enable/disable lifecycle NOT qualified by offline replay.
Remote evidence: reports/software_imu_walk_native_parity_131201.
