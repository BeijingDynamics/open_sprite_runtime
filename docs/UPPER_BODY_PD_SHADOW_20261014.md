# Upper-body PD candidate: zero-gain check

User authorized ONLY zero-gain testing on2026-10-14. No powered test performed.
Baseline walking entry remains unchanged.

Approved changes: waist yaw and shoulder pitch/roll130/5, motor caps28Nm;
wrist yaw7/0.4; wrist pitch/roll12/1. Waist roll60/3 and cap30Nm remain.
Head, legs, other motor caps and all feedback/temperature protections unchanged.
Effective baseline gains were taken from the143721 native bundle so head/wrist
commissioning multipliers are not accidentally applied twice.

Candidate directory on253: /home/tony/open_sprite_runtime/candidates/upper_body_pd_20261014
Zero-gain wrapper: probe_sprite0825_upper_body_pd_bno_shadow_on_253.sh
Acknowledgement: ZERO_GAIN_UPPER_BODY_CHECK
Uses15s stationary BNO raw six-axis calibration at200Hz, then10s zero-command
standing preview. Python ONNX50Hz supplies reference targets; native CAN transmits
only zero gains/torque. This is not a powered or dynamic walking qualification.

First attempt175216 failed because all fourkcan wereDOWN/MTU16. No policy targets
were generated. User configuredCAN; second attempt175424 completed successfully.
The wrapper now checks all fourinterfaces forUP/MTU72 and1M/5M before IMU startup.

## Evidence on253

- reports/native_policy_ipc_transport_upperpd_20261014_175424.json
- reports/native_policy_ipc_actor_upperpd_20261014_175424.json
- reports/native_policy_ipc_trace_upperpd_20261014_175424.npz
- reports/upper_body_imu_shadow_yMjNqHjh/imu.json
- reports/upper_body_pd_native_parity_175424/report.json

Transport:10.001571s, all31 motor RX/TX counts equal; ankle5000 samples per
motor, other motors500 each. No deadline misses; latenessmax0.031168ms.
Zero automatic enables; zero nonzero-gain/torque TX attempts.
Policy:500 states,499 targets, coverage99.8%, rejected IMU frames0.
Native-source maximum receive age7.399183ms (host read-start age, not physical
sensor latency). IMU gapP99=5.1118ms,max15.6128ms; readmax13.6612ms.
Producer's finite-duration report has passed=false after wrapper termination;
do not label that report as a completed long-duration stability test.

Bounded static torque PREVIEW, not torque applied to the robot:
left/right shoulder pitch11.9904/19.2468Nm; waist yaw1.33348Nm;
waist roll2.74639Nm. Left/right wrist pitch and right wrist roll reach0.49Nm,
near the retained3507 cap0.5Nm. Higher wrist gains do not remove that cap.

Offline native/Python chain replay:500 frames, passed; raw action error0,
protected target maximum difference1.39e-17. Model hash unchanged.
This replay exercises the recorded stationary commands, not a live native actor
or40s walking schedule. The existing adapter freezes a2001-frame baseline; new
powered entry must regenerate/validate its fixture and launch settings rather
than bypassing old hash checks or swapping TSVs into the old bundle.

## Subsequent offline entry qualification

Completed later the same day, without hardware access:
- Recomputed startup for the original1s hold +5s ramp from recorded projected
  targets; original raw recording retained. Native/Python parity passed again.
- Rebuilt a2001-frame40s pacing fixture (recorded stationary inputs extended
  by holding the last state; NOT evidence of real walking dynamics).
- Actual native powered-candidate binary with fake Unix-socket peers:2001 frames,
  byte-identical to offline replay, IPC targets equal computed targets, no20ms
  overruns, maximum response1.530823ms.
- Simulated IMU freeze and state disconnect: expected exits in68ms and31ms.
- All31 effective Kp/Kd values checked after ramp. Verified0 command before10s,
  vx0.10 in the middle segment, and0 command for the final10s.
- Generated actual shell settings and a fresh native bundle in an offline test;
  matched fixture header, motor caps and joint table. Old waist multiplier0.125
  explicitly rejected. Qualification seal checks model/config/binaries/entry.

New isolated entry on253:
`run_sprite0825_upper_body_pd_native_walk40_on_253.sh`
Static-only command: `bash run_sprite0825_upper_body_pd_native_walk40_on_253.sh --check`
Powered acknowledgement (NOT run): `ENABLE_UPPER_BODY_PD_WALK40`.
Continues using BNO raw200Hz, native ONNX50Hz, existing CAN protections. Fresh
onsite approval required before powered invocation. Baseline entry unchanged.
Evidence: candidates/upper_body_pd_20261014/qualification/{entry_checks.json,
seal.json,preparation.json,parity/report.json,fake_none/report.json,
fake_imu_freeze/report.json,fake_state_disconnect/report.json}.
