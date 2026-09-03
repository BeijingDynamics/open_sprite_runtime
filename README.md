# Open Sprite Runtime

Safety-gated Sim2Real runtime work for Sprite0825. The current deployment
candidate is the native-50 Hz G60 `model3450`; its ONNX actor, contract, and
compact Isaac/MuJoCo qualification evidence are under
`artifacts/g60_model3450`. The qualified 100 Hz `model1050` release remains a
teacher and comparison baseline only.

This repository is intentionally **not hardware-ready yet**. The initial code
can inspect a policy contract and validate differential-ankle math, but it has
no CAN transmit backend. Hardware transmission remains disabled until motor
mapping, zero offsets, directions, limits, IMU convention, watchdog, emergency
stop, and both ankle calibrations are complete.

## Timing contract

- Deployment policy inference: **50 Hz** (`dt=0.02 s`)
- SBC feedback, safety, ankle coupling, and held-target refresh: **500 Hz**
- Damiao internal MIT/current loop: **1 kHz**

The 50 Hz policy target uses a 20 ms zero-order hold, matching both the Isaac
Lab decimation loop and the qualified MuJoCo runtime. The 500 Hz layer must not
invent intermediate policy targets. It may refresh the held target and add the
measured differential-ankle cross-coupling feed-forward term.

The qualified `model1050` actor remains a 100 Hz teacher/baseline and is not the
hardware deployment policy. G60 `model3450` is trained natively at 50 Hz with
a 160 ms observation window. Contract validation rejects a 100 Hz actor when
the runtime is configured for 50 Hz.

## Development setup

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -e '.[dev]'
python -m unittest discover -s tests -v
```

Inspect the native-50 Hz candidate contract without touching hardware:

```bash
sprite-runtime inspect \
  --contract artifacts/g60_model3450/deploy/contract.json \
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

The heading controller follows Isaac Lab's PM01 command interface: while
walking straight it computes `wz = clip(0.5 * wrap(target_yaw - imu_yaw),
-0.2, 0.2)`. Standing resets the target to the current integrated IMU yaw;
manual yaw-rate commands are passed through and the new heading is held when
the operator releases the turn command. Global yaw remains outside the actor.

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
remaining blockers are summarized in [G60 model3450](docs/G60_MODEL3450.md).

## License

GNU Affero General Public License v3.0 only (`AGPL-3.0-only`).
