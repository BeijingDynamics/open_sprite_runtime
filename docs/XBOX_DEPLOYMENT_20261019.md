# Xbox deployment snapshot, October 19

This update publishes development history, not a self-contained installation or
general walking safety certification. No binary, raw hardware trace, credentials,
or machine-specific binary qualification seal is included.

## Current supervised entry

On the provisioned Jetson253, with fall arrest, clear walking/arm space, battery
power and an independent power safety operator:

```bash
cd /home/tony/open_sprite_runtime
bash run_sprite0825_xbox_native_turn120_on_253.sh --check
sudo bash run_sprite0825_xbox_native_turn120_on_253.sh ENABLE_XBOX_TURN120
```

Keep still for15 seconds for IMU bias calibration, before powered operation.
Powered schedule:0-10s stand/ramp;10-110s operator input; then ramp commands to
zero and request standing until120s disable. Faults exit earlier.
LT requires deliberate rearming with centered sticks. Forward0..0.10m/s,
turn+/-0.10rad/s; no reverse. Release LT to ramp commands to zero.
LB+RB held3s requests disable immediately through the controller fault path,
without a final standing delay. The fall arrest must support the disabled robot.
This Bluetooth command is not a substitute for the independent emergency cutoff.

## Evidence and retained versions

- `xbox_native_20261019`: initial40-second forward-only candidate.
- `xbox_native_120_20261019`:120-second forward-only candidate and button disable.
- `xbox_native_turn120_20261019`: low-speed steering; gains/limits unchanged.

The operator reported successful powered forward, stop and disable, and later
normal turning. This update does not claim an independently measured turn tracking
error or a qualified moving Bluetooth radio-loss stop latency.
Published `qualification/*/report.json` files are **fake-peer offline tests**,
not powered-run reports. Turning replay passed6001 frames with byte-identical
offline outputs and IPC targets, no20ms overruns, and maximum round trip3.236ms.
IMU freeze, state disconnect and operator-disable injection exited as expected.
Mapping/button files record commissioning on this controller only; verify your own.

## Provisioning boundary

Root shell entries refer to the site-specific `/home/tony/open_sprite_runtime`
deployment, its matched model/contract, calibration, motor tables, ICM binaries,
ONNX Runtime library and prior qualification evidence. A fresh clone is not ready
to control hardware. The candidate directories retain source/script snapshots;
executables, fixture binaries and binary seals must be regenerated and qualified
on the target host. Do not remove checks to bypass missing dependencies.

Generators in `tools/prepare_xbox*_candidate.py` are commissioning recipes, not
portable installers. The turn generator expects changed source files staged in
`/tmp/sprite_turn120_sources`; it refuses an already-existing candidate directory.
Published source of each deployed candidate is retained in its `src` directory.
The turn CAN executable is a symlink to the qualified120-second CAN executable.
Its capability check resolves the link before calling `getcap`.

Two startup defects were corrected during this work: the120s generator initially
selected stale CAN source with aKd3 guard instead of the establishedKd5 source;
and the turn entry initially queried capability on the symlink rather than its
target. Actual31-joint parameter loading and capability checks now pass. Neither
fix increased gains or relaxed torque limits.

## Offline input tests

```bash
cmake -S native -B build/gamepad-tests
cmake --build build/gamepad-tests --target test_gamepad
ctest --test-dir build/gamepad-tests -R '^gamepad$' --output-on-failure
```

These tests do not open joystick, IMU or CAN devices. Full replay additionally
requires the matching model and recorded-state fixture. See
`tools/test_native_policy_realtime_loop.py` and the published offline reports.
