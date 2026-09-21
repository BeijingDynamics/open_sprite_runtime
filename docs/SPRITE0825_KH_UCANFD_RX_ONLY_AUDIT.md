# Sprite0825 KH UCANFD RX-only audit

Date: 2026-09-13

## Scope

This audit covers `/home/tony/KH-UCANFD_Linux_SDK-1.3.1` on the 234 host.
No CAN device was opened, configured, or used to transmit frames.

## Findings

- The SDK exposes its four channels as standard Linux SocketCAN interfaces.
- `driver/src/kcan_netdev.c` advertises and maps `CAN_CTRLMODE_LISTENONLY` to
  the vendor driver's `KCANFD_INIT_LISTEN_ONLY` flag.
- The UCAN FD implementation emits a dedicated listen-only initialization
  command (`UCAN_CMD_LISTEN_ONLY_MODE`). This is stronger evidence than merely
  omitting `send()` calls in the application.
- The driver supports device/hardware receive timestamps. Its public frame
  contract marks timestamps with `KCANFD_TIMESTAMP` and
  `KCANFD_HWTIMESTAMP`; the SocketCAN path populates `skb_hwtstamps` with the
  adapter's raw device-clock value. This raw value is not host wall-clock time.
- Driver source and tests are GPL/LGPL licensed. The runtime should use the
  kernel SocketCAN ABI and must not copy vendor driver code into the AGPL
  runtime repository.
- No KH USB device or SocketCAN interface was present during this audit, so
  channel names, firmware version, timestamp quality, receive loss, and latency
  are not yet measured.

## Historical passive diagnostic

This document predates direct polling of the installed disabled Damiao drives.
Those drives do not supply the complete required state stream without request
frames, so listen-only capture is not the current arming gate. It remains valid
for inspecting traffic when another independently approved controller is
already polling the motors. The authoritative gate is now
`can_adapter.commissioning_shadow.method=active_zero_gain_position_echo`, as
documented in `HARDWARE_CONTRACT.md` and `SPRITE0825_LIVE_POLICY_SHADOW.md`.

## Passive diagnostic procedure

1. Attach the exact four-channel adapter and record USB identity, firmware, and
   the stable mapping from physical channel 0..3 to SocketCAN interface names.
2. Configure all four interfaces with kernel listen-only mode and capture
   `ip -details link` evidence before the runtime opens receive sockets.
3. Receive motor telemetry only. Verify zero application TX calls and zero CAN
   TX frame count while exercising every joint manually with motor torque off.
4. Verify raw hardware timestamps are present and monotonic. Separately verify
   kernel software RX timestamps and measure kernel-to-userspace queue age
   under all 31 motors; its P99 must be no more than 6 ms. Do not report raw
   hardware time minus host time as latency. Measure USB bus-to-kernel latency
   later with synchronized clocks or a physical loopback test.
5. Save the machine-readable report as passive diagnostic evidence. Do not use
   it to mark the active zero-gain commissioning gate complete.

Listen-only is a shadow-mode commissioning gate, not the final armed state.
Changing an interface to active mode later must be a separate deliberate
operation guarded by the independent e-stop, complete hardware inventory,
ankle calibration, fresh telemetry, and runtime arming checks.

The runtime command below audits a saved structured snapshot offline. It does
not open a CAN socket:

```bash
ip -j -d link show type can > kh_socketcan_snapshot.json
sprite-runtime socketcan-rx-preflight \
  --snapshot kh_socketcan_snapshot.json \
  --interfaces can0 can1 can2 can3
```

After all 31 motor mappings and register-readback ranges are filled, collect a
finite live shadow report while a separately approved controller supplies the
normal command traffic:

```bash
ip -j -d link show type can > kh_socketcan_snapshot.json
sprite-runtime damiao-rx-audit \
  --hardware-config config/hardware.sprite0825.local.json \
  --snapshot kh_socketcan_snapshot.json \
  --duration 10 \
  --output evidence/damiao_rx_only_10s.json
```

The command opens four receive-only sockets only after the saved snapshot
proves all interfaces are UP and in kernel `LISTEN-ONLY`. It accepts observed
command IDs as traffic from the separate controller, decodes only configured
Master IDs, and rejects every unknown endpoint. It never transmits.

After that report passes, `SocketCanReceiver.open()` can create an API that
only exposes `receive()` and `close()`. It enables CAN-FD plus Linux hardware
and software RX timestamping before binding. Every frame must carry both a
non-zero raw hardware timestamp and a kernel software receive timestamp; the
implementation deliberately fails closed if either is missing. Opening is
rejected unless the same interface appears in a passing listen-only preflight
report.

## Direct Jetson MuJoCo display

The same receive-only collector can display the decoded physical posture on the
Jetson display. MuJoCo is kinematic in this mode: the runtime writes decoded
joint positions and calls `mj_forward`; it does not step physics and has no CAN
send/enable API. Camera movement remains under normal MuJoCo viewer control.

```bash
DISPLAY=:1 sprite-runtime damiao-rx-audit \
  --hardware-config config/hardware.sprite0825.measurement.json \
  --snapshot reports/kh_socketcan_snapshot.json \
  --duration 600 \
  --output reports/damiao_rx_view_600s.json \
  --viewer \
  --contract /home/tony/sprite_runtime/sprite0825_stage2_g74_model3000_sim2real_candidate/deploy/contract.json \
  --mjcf /home/tony/sprite_runtime/sprite0825_stage2_g74_model3000_sim2real_candidate/assets/mujoco/sprite0825_v5_4340_shoulders/sprite0825_v5_external_pd.xml
```

Left and right ankle pitch/roll use their independently measured differential
matrices. Any uncalibrated differential is visibly reported and frozen at zero;
currently this applies to head pitch/roll. Operator-confirmed PMAX/VMAX/TMAX
may be used for this non-armable visual diagnostic, but the resulting audit has
`qualification_passed=false` until every drive range is register-readback
verified. Receive-only display also requires existing feedback traffic from a
separate controller; it never polls a silent motor bus.
