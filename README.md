# Open Sprite Runtime

Safety-gated Sim2Real runtime work for Sprite0825. The initial reference policy
is the qualified `model1050` release from
[`BeijingDynamics/open_sprite_rl`](https://github.com/BeijingDynamics/open_sprite_rl),
but that 100 Hz actor is a teacher and comparison baseline only.

This repository is intentionally **not hardware-ready yet**. The initial code
can inspect a policy contract and validate differential-ankle math, but it has
no CAN transmit backend. Hardware transmission remains disabled until motor
mapping, zero offsets, directions, limits, IMU convention, watchdog, emergency
stop, and both ankle calibrations are complete.

## Timing contract

- Deployment policy inference: **50 Hz** (`dt=0.02 s`)
- SBC state aggregation and target interpolation: **500 Hz**
- Damiao internal MIT/current loop: **1 kHz**

The qualified `model1050` actor remains a 100 Hz teacher/baseline and is not the
hardware deployment policy. The deployment actor will be trained natively at
50 Hz with an approximately 160 ms observation window. Contract validation
must reject a 100 Hz actor when the runtime is configured for 50 Hz.

## Development setup

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -e '.[dev]'
python -m unittest discover -s tests -v
```

Inspect the qualified 100 Hz teacher contract without touching hardware. The
report is expected to show a 50/100 Hz deployment mismatch until the new policy
is qualified:

```bash
sprite-runtime inspect \
  --contract ../open_sprite_rl/baselines/sprite0825_stage2_g58f_model1050_stage2_qualified/deploy/contract.json \
  --runtime-config config/runtime.example.json \
  --hardware-config config/hardware.example.json
```

See [Sim2Real plan](docs/SIM2REAL_PLAN.md), [timing](docs/TIMING.md), and
[hardware contract](docs/HARDWARE_CONTRACT.md).

## License

GNU Affero General Public License v3.0 only (`AGPL-3.0-only`).
