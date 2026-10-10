# Xbox Bluetooth pairing and walking control

This guide covers pairing the Bluetooth-capable Xbox controller with Jetson
253, testing input without robot actuation, and the agreed walking interface.
Pairing does not enable motors or authorize a physical walking test.

## Verified status on 2026-10-10

- Device: Xbox Wireless Controller, Bluetooth address `AC:8E:BD:66:55:8C`.
- Linux identity: vendor `045e`, product `0b13`, version `0523`.
- Paired, trusted and connected on 253; input handlers `event5` and `js0`.
- A 30-second read-only test received 23 initialization events without a
  disconnect, but no live button/stick events. Full manual input validation is
  still required. Input node numbers can change after reconnect or reboot.
- The control logic below is an agreed design, NOT an enabled hardware control
  path. Reverse and turning on the physical robot are not qualified by pairing.

## Pairing on 253

Turn on the controller and hold its top pairing button until the Xbox light
flashes rapidly. Prevent the previously paired computer from reconnecting.
Use the interactive session below over SSH or in the Jetson terminal:

```bash
bluetoothctl
```

At the bluetoothctl prompt:

```text
power on
agent on
default-agent
pairable on
scan on
```

Wait for `Xbox Wireless Controller`. Use its actual address; the address below
is the controller verified on this robot:

```text
pair AC:8E:BD:66:55:8C
```

Wait for `Pairing successful` before continuing:

```text
trust AC:8E:BD:66:55:8C
connect AC:8E:BD:66:55:8C
info AC:8E:BD:66:55:8C
scan off
quit
```

Confirm `Paired: yes`, `Trusted: yes`, and `Connected: yes`. If already
connected, the additional connect step is unnecessary. On this Jetson an
initial one-shot pairing failed authentication; pairing with the interactive
agent succeeded. If authentication fails, re-enter controller pairing mode
and retry with the agent active. Do not remove unrelated Bluetooth devices.

Check Linux input registration:

```bash
cat /proc/bus/input/devices
ls -l /dev/input/js*
```

No additional controller driver was installed for this successful connection.
These steps apply to this Bluetooth-capable controller, not an Xbox 360
wireless controller that requires its own receiver.

## Read-only input test

From the repository root:

```bash
python3 tools/test_xbox_input.py --device /dev/input/js0 --seconds 120
```

The commissioning copy on 253 is also available:

```bash
python3 /home/tony/test_xbox_input.py --seconds 120
```

Move both sticks through their ranges and release them; press and release
ABXY, D-pad, shoulder buttons and both triggers. The script prints `AXIS` and
`BUTTON` changes and a final range/state summary. Initialization events alone
do not prove that physical inputs work. It opens only the joystick read-only;
it does not open CAN, load a policy, enable motors, or send vibration commands.

Verify axis identity, endpoints, direction, and released trigger values before
mapping commands. The test uses Linux joystick zero-based axis numbers, not
evdev ABS codes. Do not assume other controller models have the same mapping.

## Agreed walking interface

| Input | Command meaning |
| --- | --- |
| LT, AXIS 5 near +1 | Hold-to-walk permission |
| Left stick forward/backward, AXIS 1 | Forward/reverse velocity `vx` |
| AXIS 2 | Left/right yaw rate `wz` |
| No lateral control | Always `vy = 0` |
| Release LT | Ramp commanded velocity to zero and keep standing; no immediate motor disable |
| Center sticks while holding LT | Corresponding velocity target is zero |

The operator identified AXIS 5 as LT and AXIS 1/2 as the desired controls.
Their polarity remains to be measured: stick forward must produce positive
`vx`, backward negative `vx`, left turn positive `wz`, right turn negative
`wz`. Do not guess polarity from axis numbers.

Required behavior before integrating this with hardware:

- Start disarmed. Require observed LT release and centered sticks, then a
  fresh LT press. Holding LT during startup must not start walking.
- Use trigger thresholds with hysteresis, not exact floating-point equality
  to +1. Choose thresholds after measuring the actual released/pressed range.
- Apply stick deadzones, bounded speeds and acceleration/deceleration ramps.
  Deadzones and limits must be explicit configuration values validated first
  in input-only tests and simulation. No speed limits are authorized here.
- Releasing LT overrides stick input and requests a controlled stop.
- Disconnect, input-read errors or failed connection-health checks revoke
  permission and request the protected stop path. Reconnection must not resume
  walking until the release/center/new-press sequence is repeated.
- Joystick events are change-driven: absence of events while a stick is held
  is not by itself a disconnect. Connection health needs separate validation.
- Keep the native control and sensor watchdogs active. A protected stop needs
  a healthy policy/control loop; it is not guaranteed after loss of power or
  critical sensors. The independent power-cut operator remains necessary.
- LT is walking permission, not a certified emergency stop. Pairing or input
  testing never enables motors. Reverse and turn tests require their own
  staged physical validation and explicit authorization.

## Heading boundary

The native client resets session-relative gyro heading before its first policy
inference. That value is currently telemetry only: no automatic heading-hold
correction is added to `wz`. Resetting heading does not remove gyro bias or
guarantee a fixed drift bound over 1-2 minutes.
