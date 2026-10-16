# Wrist PD revision, 2026-10-14

The user observed high-frequency wrist pitch/roll oscillation in the completed
upper-body trial `upperpd_20261014_180808`. Completion and all-motor disable
passed, but this is not evidence that the wrist gains were suitable.

Requested revision, both sides:

| Joint | Previous Kp/Kd | Revised Kp/Kd |
| --- | --- | --- |
| Wrist yaw | 7 / 0.4 | 7 / 1 |
| Wrist pitch | 12 / 1 | 7 / 0.4 |
| Wrist roll | 12 / 1 | 7 / 0.4 |

Candidate: `candidates/upper_body_pd_wrist7_20261014`.
Other gains and the entire motor-limit table are unchanged. In particular,
wrist yaw command torque remains 0.35 Nm, wrist pitch/roll 0.5 Nm. These are
command limits, not guarantees on measured dynamic feedback torque.

The native input fixture was regenerated and checked against the Python
implementation over 500 recorded sensor frames. Gains and ramp fields in the
reference were recomputed offline; this is NOT new hardware-test evidence.
All runtime gain multipliers remain one, with a 1-second hold and 5-second ramp.

The generated 40-second entry must pass `qualification/entry_checks.json` and
its hash seal before hardware use. Fake-peer tests exercise the 40-second
command schedule, stale IMU rejection, and state disconnect rejection without
opening CAN or I2C. No powered trial is authorized by this document.

Entry: `run_sprite0825_upper_body_pd_native_walk40_on_253.sh`.
It uses BNO raw-six-axis 200 Hz and native policy 50 Hz. The 15-second stationary
bias calibration and zero-gain preflight are outside the powered 40 seconds.
The final 10 seconds command standing before disable verification.
