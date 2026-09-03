# Packaged runtime candidate

This directory contains the native-50 Hz G60 `model3450` ONNX actor, localized
deployment contract, and compact qualification reports. It intentionally does
not contain a CAN backend or calibrated hardware configuration.

Run from the repository root:

```bash
sprite-runtime inspect \
  --contract artifacts/g60_model3450/deploy/contract.json \
  --runtime-config config/runtime.example.json \
  --hardware-config config/hardware.example.json
```

The full checkpoint, source snapshots, logs, and MuJoCo trace are retained in
the separately hashed G60 candidate packages on the 4090 persistent disk, the
234 machine, and the Windows project backup.

`reports/isaac_robust_summary.json` covers repeated pushes, randomization,
20-cycle start/stop, direct yaw, contact, tracking, and motor envelopes.
`reports/isaac_robust_heading_hold_summary.json` compares the same three seeds
with and without the PM01-style outer heading controller.
