# Open Sprite Runtime

Safety-gated Sim2Real runtime work for Sprite0825. The current deployment
candidate is the native-50 Hz G74 `model3000`, qualified after the proximal
shoulder pitch/roll motors were changed to J4340P. Its frozen actor, contract,
and Isaac/MuJoCo qualification evidence are distributed in the external G74
Sim2Real candidate package. G59 `model2999` remains the pre-upgrade rollback
baseline; the qualified 100 Hz `model1050` release remains a teacher and
comparison baseline only.

This repository includes an explicitly acknowledged, finite-duration protected
whole-robot policy loop. Full-weight standing and bounded disturbance recovery
have been admitted on hardware with an independent power-cut operator. A bounded
battery-powered 0.10 m/s stand-walk-stop trial completed on 2026-10-10 with fall
arrest and operator supervision; this is not unrestricted walking qualification.
On October 14 the operator reported repeated successful supervised walks with
the native C++ actor and 200 Hz raw-six-axis software IMU candidate. See the
[native walk development record](docs/NATIVE_WALK40_READINESS_20261014.md)
for qualification scope, remaining limitations and the earlier exit-gate fix.
On October 16, three consecutive supervised 40-second ICM45686 walks completed.
The operator reported disappearance of the previous left-leg straightening
pattern, much straighter travel, and stable starts/stops. Controller reports
confirmed successful completion and all 31 motors disabled after each run.
See the [ICM walking milestone](docs/ICM45686_WALKING_20261016.md) for evidence,
fixed gains, commands, and limitations. BNO055 remains the rollback path.
This remains supervised commissioning, not unrestricted walking approval.
Motor mapping, directions,
mechanical limits, IMU convention, four 500 Hz CAN-FD bus tests, differential
calibrations, watchdogs, and integrated standing gates are recorded as completed
commissioning milestones. Active scripts remain fail-closed and require their
exact acknowledgement token; receive-only inspection tools remain available.

## Timing contract

The current candidate uses native ONNX Runtime CPU inference at **50 Hz**,
an independent native ICM45686 raw-six-axis reader/software estimator at nominal
**800 Hz** over **1 MHz SPI** (about 792 Hz measured),
and a native CAN controller with a **2 kHz scheduling loop**. That scheduler
transmits to **each ankle motor at 500 Hz** and **each other motor at 50 Hz**;
it does **not** update every motor at 2 kHz. Python still prepares and supervises
the run.
The estimator uses actual sample intervals; policy consumes the latest valid
state. Startup requires 15 seconds of stillness for gyro bias estimation.
Gyro observations are raw gyro minus bias, rotated into body coordinates;
projected gravity comes from the software estimator, without magnetometer fusion.
See [software IMU design](docs/BNO055_SOFTWARE_FUSION.md).
The BNO055 rollback uses the same estimator architecture at 200 Hz over I2C.
ICM data has its own validated IPC format; it is not decoded as BNO register data.

The `*_on_253.sh` entries are site-specific commissioning scripts. Their model,
contract, calibration and parity-evidence paths are not a portable deployment
bundle. A fresh installation must provision and validate its own matched inputs;
do not bypass missing evidence checks to run hardware.

See the [October 10 development update](docs/DEVELOPMENT_UPDATE_20261010.md)
and [Xbox Bluetooth pairing and walking control](docs/XBOX_BLUETOOTH_CONTROL.md).
The gamepad interface is an agreed design, not yet connected to motor control.

| Layer | Configured rate | What runs at this rate |
|---|---|---|
| Native policy inference | **50 Hz**, 20 ms | Produces joint targets; receives a latest-state motor snapshot via IPC at 50 Hz. |
| Native software IMU | ICM **800 Hz**, nominal 1.25 ms; BNO rollback **200 Hz**, 5 ms | Independently reads raw six-axis data and estimates gravity; policy consumes the latest valid state. SPI bitrate and sensor ODR are different quantities. |
| Native CAN scheduler | **2 kHz**, 0.5 ms | Services due transmit slots across four buses, drains received feedback and checks the policy target watchdog. Not a per-motor update rate. |
| Host ankle PD / parallel linkage calculation | **500 Hz**, 2 ms | Recomputes ankle motor feed-forward torques from held policy targets and latest joint feedback. |
| Four ankle motors: command and feedback polling | **500 Hz per motor**, 2 ms | Sends MIT feed-forward torque commands with motor-side Kp/Kd zero. The two motors in each pair occupy staggered CAN slots. |
| Other 27 motors: command and feedback polling | **50 Hz per motor**, 20 ms | Sends MIT position/Kp/Kd commands; PD runs inside the motor. Includes hips, knees, waist, arms and head. |
| Damiao drive internal control | **1 kHz**, as specified for this hardware | Runs inside the drive, independently of the host CAN transmit rate. |

All rates above are configured nominal rates, not latency guarantees. Feedback
arrives in response to motor commands and is drained on scheduler iterations;
a 2 kHz receive loop does not make a 50 Hz motor's measurements 2 kHz fresh.
Runtime freshness checks and timing reports remain necessary.

With this 31-motor mapping, nominal command traffic is 1,300 frames/s on each
leg bus (2 x 500 + 6 x 50), 400 frames/s on the left-arm bus, and 350 frames/s
on the right-arm bus: **3,350 transmitted command frames/s in total**, plus
motor replies. CAN-FD uses **1 Mbit/s arbitration and 5 Mbit/s data with BRS**;
these bit rates are distinct from control/update rates. The earlier 500 Hz
per-motor full-bus stress tests are not the normal walking schedule.

The 50 Hz policy target uses a 20 ms zero-order hold, matching both the Isaac
Lab decimation loop and the qualified MuJoCo runtime. The 500 Hz ankle controller
reuses that target with new feedback; it does not invent intermediate policy
targets. Its parallel-linkage torque mapping is computed on the host, not by
raising the policy inference rate.

Implementation: scheduling and slot selection are in
[`sprite_can_shadow.cpp`](native/src/sprite_can_shadow.cpp); the per-motor
500/50 Hz assignment is in
[`export_native_motor_config.py`](tools/export_native_motor_config.py).

The qualified `model1050` actor remains a 100 Hz teacher/baseline and is not the
hardware deployment policy. G74 `model3000` is trained natively at 50 Hz with
a 160 ms observation window. Contract validation rejects a 100 Hz actor when
the runtime is configured for 50 Hz.

## Development setup

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -e '.[dev]'
python -m unittest discover -s tests -v
```

Inspect a native-50 Hz candidate contract without touching hardware:

```bash
sprite-runtime inspect \
  --contract <candidate>/deploy/contract.json \
  --runtime-config config/runtime.example.json \
  --hardware-config config/hardware.example.json
```

For a native 50 Hz candidate, run the fail-closed safety scenarios and a host
scheduling probe before any CAN hardware is connected:

```bash
sprite-runtime safety-self-test --runtime-config config/runtime.example.json
sprite-runtime heading-self-test --runtime-config config/runtime.example.json
sprite-runtime timing-probe --duration 10 --state-hz 500
```

Damiao drives do not provide the required periodic disabled-state telemetry
without a request frame. The qualified commissioning gate therefore uses the
native active zero-gain position-echo probe while every motor remains disabled;
velocity, Kp, Kd, and feed-forward torque are all encoded as zero. The hardware
contract requires a 120-second report with complete feedback, zero timing/CAN
errors, and zero nonzero-command, enable, or mode-switch attempts. The older
listen-only receiver remains useful only when another approved controller is
already producing bus traffic.

The separate heading-controller implementation follows Isaac Lab's PM01 command interface: while
walking straight it computes `wz = clip(0.5 * wrap(target_yaw - imu_yaw),
-0.2, 0.2)`. Standing resets the target to the current integrated IMU yaw;
manual yaw-rate commands are passed through and the new heading is held when
the operator releases the turn command. Global yaw remains outside the actor.
This controller is **not wired into the current native powered-policy path**.
That path resets and logs relative gyro heading at session startup without
altering actor observations or applying automatic yaw correction. Resetting
the origin is not gyro bias calibration and does not guarantee drift bounds.

Replay a MuJoCo observation trace through the exported ONNX actor. This mode
never opens a CAN interface and checks that runtime inference reproduces the
actions recorded by the qualified simulator:

```bash
sprite-runtime replay-trace \
  --contract <candidate>/deploy/contract.json \
  --trace <candidate>/evaluation/mujoco_matrix/straight_60s_trace.json \
  --sample-stride 10
```

See [Sim2Real plan](docs/SIM2REAL_PLAN.md), [timing](docs/TIMING.md), and
[hardware contract](docs/HARDWARE_CONTRACT.md). The candidate evidence and
remaining blockers are summarized in [G60 model3450](docs/G60_MODEL3450.md),
including the [robust heading audit](docs/G60_ROBUST_HEADING_AUDIT.md).
The four-motor proximal-shoulder upgrade is documented in
[Sprite0825 4340P shoulder revision](docs/SPRITE0825_4340_SHOULDER_REVISION.md);
the receive-only Damiao frame contract and pinned manufacturer sources are in
[Damiao protocol source audit](docs/SPRITE0825_DAMIAO_PROTOCOL_SOURCE_AUDIT.md).
G74 model3000 is the frozen software candidate after independent-seed Isaac
qualification and the unchanged seven-case MuJoCo gate matrix. G59 model2999
remains the pre-upgrade rollback baseline.

The provisional Yahboom 9-axis pelvis IMU mount, frame transform, read-only
serial capture, and hardware-arrival checklist are documented in
[Sprite0825 Yahboom IMU integration](docs/SPRITE0825_YAHBOOM_IMU_INTEGRATION.md).
The Jetson's existing high-performance CH340 driver is a prerequisite and is
not modified by this project.

Synchronized raw CAN, policy, motor, temperature, and IMU logging for physical
parameter identification is documented in
[Sprite0825 real-data system identification](docs/SPRITE0825_REAL_DATA_SYSTEM_IDENTIFICATION.md).

## License

GNU Affero General Public License v3.0 only (`AGPL-3.0-only`).
