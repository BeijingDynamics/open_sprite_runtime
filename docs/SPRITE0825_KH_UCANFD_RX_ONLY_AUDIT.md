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
  `KCANFD_HWTIMESTAMP`; the SocketCAN path populates `skb_hwtstamps`.
- Driver source and tests are GPL/LGPL licensed. The runtime should use the
  kernel SocketCAN ABI and must not copy vendor driver code into the AGPL
  runtime repository.
- No KH USB device or SocketCAN interface was present during this audit, so
  channel names, firmware version, timestamp quality, receive loss, and latency
  are not yet measured.

## Required gate before torque enable

1. Attach the exact four-channel adapter and record USB identity, firmware, and
   the stable mapping from physical channel 0..3 to SocketCAN interface names.
2. Configure all four interfaces with kernel listen-only mode and capture
   `ip -details link` evidence before the runtime opens receive sockets.
3. Receive motor telemetry only. Verify zero application TX calls and zero CAN
   TX frame count while exercising every joint manually with motor torque off.
4. Verify hardware/device timestamps are present and monotonic. Measure P99
   receive age under all 31 motors at the intended CAN-FD rates; the current
   runtime contract requires no more than 6 ms.
5. Save the machine-readable report and set `rx_only_shadow.completed=true`
   only after all four channels pass. Leaving any field unset keeps the runtime
   fail-closed.

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

After that report passes, `SocketCanReceiver.open()` can create an API that
only exposes `receive()` and `close()`. It enables CAN-FD and Linux hardware RX
timestamping before binding. Every received frame must carry a non-zero raw
hardware timestamp; the implementation deliberately raises an error instead
of falling back to a software timestamp. Opening is rejected unless the same
interface appears in a passing listen-only preflight report.
