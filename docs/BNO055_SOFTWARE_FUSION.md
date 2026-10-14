# BNO055 raw six-axis candidate

2026-10-14. Existing policy weights and observation dimensions are unchanged.
This is an isolated candidate, NOT the default powered walking source.

## Evidence and design

Reviewed SpacemiT peripherals-imu commit
`6bf786b4c80ff1b532cba8923f0f3b60dbf14d9e`:
- `src/imu_core.c`: bias subtraction followed by mounting rotation, receive
  timestamp distinguished from driver sample timestamp.
- `src/drivers/drv_spi_bmi270.c`: six-axis complementary attitude estimator,
  gyro quaternion propagation, measured-up cross predicted-up correction,
  acceleration-norm confidence reduction, tilt-based initialization.
- https://github.com/spacemit-robotics/peripherals-imu/tree/6bf786b4c80ff1b532cba8923f0f3b60dbf14d9e

Our implementation independently expresses the standard nonlinear
complementary-filter equations (Mahony-style proportional correction, no
online integral bias adaptation). No vendor driver or firmware blob copied.
Correction gain starts at 1/s, with full trust through 5% acceleration-norm
deviation and zero trust from 25%. These are initial experimental parameters,
not evidence of suitability for walking. Norm gating cannot detect every
translational acceleration, notably sustained horizontal acceleration.

PM01 is used as the observation-interface reference: estimated gravity and
body angular velocity, not a claim that its internal estimator is this one.
https://github.com/engineai-robotics/engineai_robotics_native_sdk/blob/main/src/runner/rl_walking_example/src/rl_walking_example_runner.cc

Bosch documents non-fusion ACCGYRO=0x05, P0=0x21/0x04, and raw units.
In fusion mode gyro registers are already internally offset compensated;
this candidate instead uses non-fusion readings and its own startup bias.
https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bno055-ds000.pdf

## Exact pipeline

1. Standalone no-CAN producer takes the native BNO bus lock, checks chip/page,
   enters CONFIG and explicitly sets ACCGYRO, P0, m/s^2 and rad/s.
   Page1: accel4g/62.5Hz bandwidth (0x08=0x0D), gyro2000dps/47Hz
   bandwidth (0x0A=0x18), normal gyro power (0x0B=0). Read back writes.
2. Poll at100Hz, one18-byte burst0x08..0x19. Magnetometer bytes between accel
   and gyro are ignored. Do not read hardware quaternion/gravity for estimates.
3. Require15 seconds and >=1400 samples of stationary supported robot for
   bias. Reject >0.1rad/s rate norm, >10% acceleration-norm deviation,
   per-axis gyro standard deviation>0.005rad/s or accel>0.15m/s^2.
   Constant slow rotation cannot be distinguished from bias by this gate:
   operator must actually hold still. Bias is frozen after initialization.
4. Initialize true tilt from mean acceleration; do not force upright pose.
   Initial yaw is arbitrary/zero, no magnetic heading is used.
5. Propagate using actual host start-to-start dt and bias-corrected gyro.
   Accelerometer correction affects ONLY estimated attitude, never gyro obs.
6. Policy gyro = fixed mount matrix * (raw gyro - startup bias), with no host
   low-pass filter or accelerometer correction in that channel. Sensor's
   hardware bandwidth remains active; 'raw' does not mean unfiltered ADC.
7. Software quaternion is an internal transport representation, not a policy
   observation. Existing mount adapter computes the same 3D projected gravity.
   P0 output axes remain Xdown,Yright,Zrear; body vector=(-sz,-sy,-sx).

Read/gap/source-age30ms guards remain. No stale-state extrapolation on a gap,
no silent estimator reset during operation, no motor gain/limit changes.
100Hz host polling is NOT proof of100Hz unique samples or low physical latency.
Raw mode latency must still be measured; no claim that it fixes I2C tails.

## Isolation and protocol

Producer build define: SPRITE_BNO_SOFTWARE_FUSION. Normal build still uses
the original hardware-fusion path. Software packets use SPIMU002, not SPIMU001.
Same192-byte size, but32..49 are raw18,56..79 are frozen sensor-frame gyro
bias doubles,80..183 are accel3/correctedgyro3/softwarequat4/up*9.80665(3).
Unused bytes are zero. Candidate consumer checks raw-minus-bias and quaternion/
gravity consistency. Old consumers reject the new version and vice versa.
Consumer opt-in build define: SPRITE_SOFTWARE_IMU_INPUT, shadow-only build.
No policy-weight modification. Current powered scripts remain unchanged.

## Build and no-CAN review on253

Deployed candidate directory:
`/home/tony/open_sprite_runtime/candidates/bno_software_20261014`

```bash
cd /home/tony/open_sprite_runtime/candidates/bno_software_20261014
bash tools/build_bno_software_fusion.sh
bash tools/build_software_imu_policy_shadow.sh /home/tony/open_sprite_runtime
# Operator keeps robot stationary for first15 seconds, then gently tilts it.
bash probe_sprite0825_bno_software_no_can_on_253.sh 60
# Use the actual report path printed above:
python3 tools/analyze_software_imu.py reports/ACTUAL_RUN/report.json.packets
```

The probe changes only IMU configuration, opens no CAN device and runs no
policy. Do not run alongside any legacy Python/native IMU reader or control
session. Native lock protects cooperative native readers; it does not stop
unrelated vendor/Python processes. Script also checks for CAN/policy processes.
After the probe, BNO stays in raw mode. Existing IMUPLUS startup must configure
it back before legacy review/control. Do not bypass mode-mismatch errors.

## Qualification still required

- [x] Compile both candidate and unchanged legacy producer paths with Werror.
- [x] Compile isolated software-packet native policy shadow consumer on253.
- [x] Offline tests: stationary bias, motion rejection, tilted startup,
      all-axis gyro propagation, correction sign, unmodified gyro observation,
      timestamp rejection, packet discrimination and policy mounting.
- [ ] Raw-mode configuration/readback on hardware and P0 direction review.
- [ ] Static bias and guided tilted/yaw acquisition, inspect recorded packets.
- [ ] Latency and long stability capture under full native zero-gain load.
- [ ] Policy consumed-input/replay checks and explicit protected walking approval.

## First hardware capture, 2026-10-14

With operator confirmation of stationary supported robot and stopped readers,
ran isolated 60s no-CAN probe. Report: `bno_software_20261014_114732_RaNHSF`.
ACCGYRO/P0 configuration and stationary initialization passed. 6000 reads,
4499 published samples after initialization, zero skipped slots and zero CAN TX.
Read p99/max: 3.39324/4.16324ms; start gap p99/max: 10.0281/10.1022ms.
Frozen sensor-frame bias: [0.000764906, -0.000642107, 0.000832963] rad/s.
Offline packet validation passed, including raw-minus-bias consistency.
Body gravity first: [0.0721829,-0.0124915,-0.9973132];
last: [0.1357450,-0.0533928,-0.9893040]. Actual tilt is preserved.
These host timing measurements are NOT physical latency measurements or
direction/accuracy qualification against known motion. No full-load shadow,
policy control, or motor enable was performed. Existing walking path unchanged.
Candidate leaves the sensor in raw mode; legacy use requires IMUPLUS startup.
Report, raw packets, and CSV backed up to Windows `work/walk40`.

## 200Hz read-path comparison, 2026-10-14

Candidate CLI accepts optional `100|200 burst18|split6`; wrapper takes these
as second and third arguments. Default remains100/burst18. Sensor bandwidth,
fusion gain, 15s bias interval, and30ms guards unchanged. Legacy build unchanged.
Six useful axes occupy12 bytes but accel0x08..0x0D and gyro0x14..0x19 are
separated by6 magnetometer bytes. Split6 performs two independent six-byte
transactions, with possible extra inter-sensor read skew; not an atomic sample.

Each run30s, no CAN/policy, stationary calibration passed:

| Report suffix | Method | Samples | Skips | Read p99/max ms | Gap max ms |
| --- | --- | --- | --- | --- | --- |
| 115441_T9u7AH | burst18, grouped health | 5971 | 29 | 3.38737/3.71628 | 10.0118 |
| 115525_qcBk6O | split6, grouped health | 5971 | 29 | 3.38408/4.68564 | 10.0108 |
| 115655_VE0c7N | burst18, distributed health | 6000 | 0 | 3.42299/3.89667 | 5.61504 |

Grouped nine-register health reads took roughly6ms and caused periodic skipped
slots. Candidate now checks one register per100ms after startup, rotating all
nine, instead of nine consecutive reads each second. Full startup validation
is retained. No safety checks removed. Keep burst18: split6 showed no tail benefit.
All reports/packets/timing backed up to Windows work/walk40. This is only a
short unloaded timing test, not proof of unique200Hz sensor samples, physical
latency, full-load stability or gait suitability. Repeat longer under load.

## Full-chain admission attempt

Isolated launcher adds explicit candidate/runtime-root options, library and
CAN UP/MTU72/1M/5M/ERROR-ACTIVE preflight; legacy runtime on253 not replaced.
Candidate runtime library search path repaired to existing verified ORT stage.
After operator configured CAN, run `native_actor_hardware_shadow_47cwy9so`
received real motor feedback but failed startup pose gate before completion.
Offline decoding of first consumed frame and fixture matrix/limits identified:
- left_ankle_pitch_joint: +0.508581rad (29.1395deg)
- right_ankle_pitch_joint: +0.442257rad (25.3395deg)
- admitted interval for both: [-0.386,+0.425]rad.
No limits relaxed. No enables/nonzero commands intended by this shadow-only
binary; CAN exited on actor disconnect. Full-load long test NOT qualified.
Evidence backed up locally under work/walk40/native_actor_hardware_shadow_47cwy9so.

## Zero-gain full-chain qualification after manual pose adjustment

2026-10-14, operator confirmed supported/disabled/powered robot and corrected
ankle positions. No limits or motor gains changed.

- `native_actor_hardware_shadow_r7998zeq`:10s/500 actor frames passed,
  byte-exact offline replay; maximum compute1.39366ms, IMU age11.2532ms.
  All31 motors TX/RX equal. IMU45s capture:1 skipped slot, max gap10.0196ms.
- `native_actor_hardware_shadow_ihbjmfq_`:120s/6000 actor frames passed,
  byte-exact offline replay; maximum compute3.85394ms, motor source age6.88041ms,
  consumed IMU age11.1254ms. All31 motors TX/RX equal, CAN deadline misses0,
  max CAN lateness0.904306ms. Enable attempts0, nonzero command attempts0.
  IMU155s capture:30981 reads,19 skipped slots, gap p99/max5.03745/10.0301ms,
  read p99/max3.68258/5.26696ms, IPC send drops0.

This passes existing30ms freshness protection but is NOT jitter-free200Hz.
The bounded native CAN preview runs with zero gains, not powered ankle dynamics.
No claim of200Hz unique sensor samples or measured physical estimator latency.
Both full report directories copied to Windows work/walk40. Production powered
walking source remains unchanged. Known-direction attitude review and explicit
powered-test approval remain required before use in walking.

## Local display

On253 run:
```bash
bash /home/tony/open_sprite_runtime/candidates/bno_software_20261014/review_sprite0825_software_imu_mujoco_on_253.sh 120
```
Uses system python3 (MuJoCo is not installed in the runtime venv). Native raw
producer runs independently at200Hz; viewer requests latest packets at roughly
50Hz and applies the same P0 sensor-to-body mounting matrix. Keep still for15s
before the window opens. Display duration begins after calibration. Root tilt
is measured/estimated; all joint poses are model defaults, not live motor data.
Yaw is relative/arbitrary, not magnetic north. No CAN or policy. Display uses
100ms stale timeout only; policy30ms guards remain unchanged. Closing the viewer
terminates the producer intentionally, so its report may say capture interrupted.
BNO remains ACCGYRO/P0; legacy fusion launchers must configure IMUPLUS first.

## Prepared powered comparison (NOT executed)

`/home/tony/open_sprite_runtime/run_sprite0825_software_imu_walk40_on_253.sh`
accepts `--check` (no hardware) or `ENABLE_SOFTWARE_IMU_WALK40`.
Explicit protocol software-v2 adapter added; hardware-v1 remains default.
Original adapters backed up in candidate/adapter_backup. Eight packet tests and
on253 static entrypoint check passed. This entry uses the existing Python
powered policy client/native CAN baseline, NOT the native actor shadow binary.
It only changes IMU source, leaving the baseline gains/limits/command timeline
and admission/disable path intact.15s stationary calibration precedes baseline
preflight. Readiness consumes a validated fresh packet before invoking baseline.
Actual powered test requires renewed operator safety confirmation; no powered
test run while preparing this script. User confirmed new viewer directions and
interaction correct. Dynamic walking estimator performance remains unqualified.
