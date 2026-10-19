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

## Native candidate, 2026-10-19

The independent candidate is staged at `candidates/xbox_native_20261019` on the Jetson.
The existing ICM800 walking entry and powered binary are unchanged. `SPRITE_GAMEPAD_INPUT`
currently compiles only with `SPRITE_SHADOW_LOOP`; powered builds are explicitly rejected.
The native input worker owns joystick polling and ioctl health checks. Inference reads an in-memory
snapshot; it does not perform joystick IO. Kernel disconnect/error or worker heartbeat older than
100 ms requests ramp-to-zero. Bluetooth failure detection latency itself has not yet been qualified;
ioctl success is not proof that the radio link is delivering fresh physical input.

Candidate input-only envelope: forward 0.10 m/s, reverse 0.05 m/s, yaw 0.20 rad/s;
translation acceleration 0.05 m/s^2 and deceleration 0.10 m/s^2, yaw slew 0.20 rad/s^2.
These are NOT physical reverse/turn test approvals. Stick deadzone 0.12; LT press above 0.90,
release below 0.60. Center/release/new-press is required after startup and reconnection.
The 40-second shadow profile accepts commands only from seconds 10 through 30 and requires
a fresh release/press after that window opens. Final commands ramp to zero; sensor/control guards
remain independent and unchanged. Changing LT never directly enables or disables motors.

Offline C++ tests passed for startup, trigger hysteresis, rearm, disconnect, ramps, polarity,
speed bounds, stale input heartbeat and clock regression. Native ONNX shadow actor compiled on253.
The first guided input trace confirmed AXIS5 LT (-1 released, +1 pressed). Stick movements arrived
after automatic stage transitions, so that trace does not qualify steering polarity. Use the
interactive prompt version; press terminal Enter before each requested movement:

```bash
python3 /home/tony/open_sprite_runtime/candidates/xbox_native_20261019/verify_xbox_mapping.py --interactive
```

Interactive report `xbox_mapping_20261019_131850.json` subsequently confirmed:
AXIS1 forward -32767, backward +32767; AXIS2 left -32767, right +32767;
AXIS5 released -32767, fully pressed +32767. Both command-axis signs are -1.
The measured mapping is recorded in `config/xbox_one_253_mapping.json`.
Cross-axis events on AXIS0/3 do not affect commands, because neither is mapped.
This qualifies axis direction only, not Bluetooth failure detection or powered control.

Full-chain shadow and basic Bluetooth disconnect/reconnect checks passed (details below).
Staged powered validation remains pending.
Do not replace the working automatic walking script with this candidate yet.

Run the guided stop/reconnection check on the Jetson (no sudo, CAN or motor power needed):

```bash
python3 /home/tony/open_sprite_runtime/candidates/xbox_native_20261019/qualify_xbox_input.py
```

Each stage waits for keyboard Enter. Follow the native input log through startup, forward command,
LT release, rearm, controller power-off, reconnection without rearm, and deliberate rearm.
The report measures command ramp-to-zero after kernel disconnect, not radio-failure detection latency.
It stops at the first failed stage. A successful test still does not qualify powered operation.
The input worker heartbeat guard also latches disarm: freshness returning alone cannot resume walking.
Offline positive/negative report-evaluator tests and native state-machine tests pass; a 3-second
read-only smoke test observed connected=1, permission=0 and vx=wz=0 with both sticks centered.

Operator traces on 2026-10-19: `xbox_stop_20261019_132723` stopped at rearm because AXIS1 stayed
centered. Its release stage began already stopped, so the original 0-second result is not a measured
deceleration; the evaluator now rejects that missing-transition case. In `xbox_stop_20261019_132846`,
forward command and rearm succeeded and LT release reduced vx from 0.10 to zero in 0.984200411 s.
Disconnect was not observed; the operator confirmed the controller had not actually been fully
powered off. This is not evidence of a disconnect-detection defect or a passed disconnect test.
Chinese stage prompts now explicitly require controller power-off and indicator extinction.

Trace `xbox_stop_20261019_133330` completed all seven input stages. The operator removed the
controller battery: kernel disconnect was observed, reconnection remained disarmed with zero
commands, and deliberate rearm restored a forward command. LT release took 0.984423071 s to
ramp from 0.10 m/s to zero. At disconnect vx and wz were already zero throughout that stage;
the original reported zero-second stop is not a measured moving-disconnect response. The evaluator
now reports this duration as null with `moving_disconnect_observed=false`. Radio-loss detection
latency remains unmeasured. This is input-only evidence, not powered-control qualification.

`candidates/xbox_native_20261019/qualify_xbox_shadow_on_253.sh` completed successfully.
It requires sudo plus `ZERO_GAIN_ALL_DISABLED_SUPPORTED`, configured CAN, mechanical support,
all motors disabled but powered, and initial 15-second stationary IMU calibration. It uses the
unchanged 800Hz ICM source, shadow-only native actor, and zero-gain CAN transport. Recorded consumed
commands can be replayed offline for byte-exact inference comparison. The reference fixture uses
the existing parity-certified configuration; it is not a new powered gain qualification.

Completed shadow report `native_actor_hardware_shadow_nula5nj6` contains 2000 frames,
reverse down to -0.05 m/s and turns to both +/-0.20 rad/s. Raw axes, permission,
pre-smoothing targets and consumed commands are in `live.bin.gamepad.csv`.
Offline replay is byte-exact; all motor feedback coverage is 100%, with zero enable
or nonzero impedance/torque attempts. Two interrupted preceding runs are not passes.

## Forward-only powered candidate (first physical run passed)

The separate `run_sprite0825_xbox_native_forward40_on_253.sh` entry keeps the accepted
ICM800, native policy50Hz, mixed-rate CAN controller, upper-body wrist7 parameter
bundle, startup preflight, torque/thermal guards and disable verification. It does
not replace the automatic walking entry. Qualification seals reject changed files.

```bash
cd /home/tony/open_sprite_runtime
bash run_sprite0825_xbox_native_forward40_on_253.sh --check
sudo bash run_sprite0825_xbox_native_forward40_on_253.sh --preflight-only
# Only after fresh onsite safety approval, battery power and fall arrest:
sudo bash run_sprite0825_xbox_native_forward40_on_253.sh ENABLE_XBOX_FORWARD40
```

Initial stationary IMU calibration (15 seconds) and readiness checks precede the
40-second powered period. First 10 seconds: stand/ramp. From 10 to 30 seconds:
center sticks, squeeze/release LT and press it again to authorize forward motion.
Full forward is 0.10 m/s; reverse and turning are clamped to zero in this build.
Releasing LT ramps velocity to zero; it does NOT disable the motors. At 30 seconds
the command window closes and ramps to zero, then stands until disable at 40 seconds.
The final period is a requested stand, not proof of balance. Fall arrest must support
the robot after disable. Faults can terminate earlier; do not rely on the countdown
as an emergency stop. Bluetooth radio-loss detection latency remains unqualified.

`prepare_xbox_powered_candidate.py` only generates isolated entries and compiles.
Fake-peer integration tests must pass before a powered seal is created. The fake
input build rejects real joint/motor names; it is never the hardware actor.

2026-10-19 offline qualification passed: 2001-frame fake-peer run was byte-exact
against offline replay and IPC targets matched computed protected targets.
Maximum fake-state round trip was 2.837 ms, with no 20 ms overruns. IMU freeze and
state disconnect caused expected exits (about 70 ms and 35 ms after injected faults).
Forward-only limits, reconnect latch, final stop and missing-ack refusal were checked.
`qualification/bundle_compare_gyhr6487` confirms input fixture, ordered names,
motor caps, kinematics and joint safety tables are byte-identical to the stable
automatic entry. These are offline checks, not a passed powered Xbox walk.

### First powered forward result, 2026-10-19

Operator reported normal forward remote control. Controller report
`native_protected_policy_admission_icm_20261019_140930.json` passed the full40s,
with no errors or deadline misses and all31 motors confirmed finally disabled.
Actor trace has2001 frames, vx0..0.10m/s, wz0, and zero commands throughout the
last8s. LT permission releases occurred at10.302s and35.782s, both with vx already
zero. Therefore this run validates forward input and scheduled stopping, but NOT
release-to-stop while moving or radio-loss stopping under powered operation.
Keep the forward-only limits; next physical check should release LT during forward
motion before the30s automatic stop. Do not infer reverse/turn qualification.

### Second powered forward result, 2026-10-19

Report `native_protected_policy_admission_icm_20261019_141229.json` passed the full
40s with no errors or deadline misses and all31 motors finally disabled. The actor
recorded2001 frames. At19.8819s LT permission was released while the preceding
forward command was0.10m/s; the consumed command reached zero1.0s later. Commands
remained zero throughout the last8s. The operator reported normal behavior.
This supplies the missing powered release-to-stop evidence for forward operation;
the1.0s measures command ramp time, not physical stopping distance or velocity.
Reverse, turning and powered radio-loss response remain unqualified.

## Isolated120-second candidate and LB/RB disable

`candidates/xbox_native_120_20261019` does not replace the passed40s entry.
Its normal powered schedule is0-10s stand/ramp,10-110s forward-only remote control,
then ramp to zero and stand until120s disable. IMU calibration and preflight precede
the powered period. Gains, torque limits and speed limits are unchanged.

LB+RB held continuously for3s latches an operator disable request. Both buttons must
first have been released; releasing either during the hold resets the timer. A reader
gap over100ms also resets continuous-hold timing. The request closes actor IPC, using
the controller's existing fail-closed whole-body disable and feedback verification.
This deliberately takes the fault/abort path, so a nonzero exit / failed-admission
message is expected, not a completed120s walking pass. Inspect the controller report
for all31 `final_disabled=true`; do not interpret the button request itself as proof.
It bypasses the normal final standing period. Fall arrest is essential and independent
power cutoff remains the emergency stop. Radio-loss detection latency is unqualified.

Physical button mapping must be verified without CAN before the new entry is usable:

```bash
cd /home/tony/open_sprite_runtime
python3 candidates/xbox_native_120_20261019/qualify_xbox_disable_buttons.py
sudo setcap cap_sys_nice+ep /home/tony/open_sprite_runtime/candidates/xbox_native_120_20261019/sprite_can_xbox120
```

The button script prompts for LB, RB and a3-second combination, then seals the candidate.
It never opens CAN or runs policy. Before that check the powered entry fails closed.
After qualification, use `run_sprite0825_xbox_native_forward120_on_253.sh --check`
for static validation; powered testing requires fresh onsite safety approval and
`sudo bash run_sprite0825_xbox_native_forward120_on_253.sh ENABLE_XBOX_FORWARD120`.
Do not substitute this for the verified40s entry before its commissioning is complete.

Offline evidence:6001-frame120s fake-peer run passed byte-exact replay and IPC-target
comparison, maximum round trip3.151ms, no20ms overruns. IMU freeze, state disconnect
and injected operator-disable requests produced the expected fail-closed actor exit.
Duration parsing rejects121s and rejects120s with the old40/60-second acknowledgements.
Subsequent button-only qualification verified LB=6 and RB=7 and the3-second hold.
The operator subsequently reported normal powered forward, stopping and motor
disable. This report does not qualify powered turning or radio-loss timing.

## Isolated low-speed turning candidate, 2026-10-19

`candidates/xbox_native_turn120_20261019` preserves the forward-only entries.
Only the command envelope changes: vx0..0.10m/s, wz-0.10..+0.10rad/s, vy0.
Reverse remains blocked. AXIS1 is forward/back, AXIS2 is turning; verified signs
are both-1. Turning requires LT permission even at zero forward speed.
Yaw command slew is0.20rad/s/s; release LT ramps both commands to zero.
The initial deliberate LT rearm, disconnect latch and LB+RB3s disable remain.
The0-10s stand,10-110s command window and final stand until120s are unchanged.
Calibration/preflight time is outside the120-second powered period.

The turn candidate reuses the exact qualified120s CAN executable through a symlink,
including its realtime capability. Policy, motor gains, torque caps and ICM800Hz
configuration remain unchanged. Only the native actor and command mapping are rebuilt.
Fake-peer tests and a separate seal are required before the entry can run.
No powered turning test is implied by offline qualification.

Offline qualification on253 passed6001 frames at50Hz over120 seconds: byte-exact
offline replay, identical protected IPC targets, maximum fake-state round trip
3.236ms and zero20ms overruns. The trace reached both-0.10 and+0.10rad/s, never
commanded reverse, and returned both commands to zero after LT release,
disconnect and the final window. IMU-freeze/state-disconnect/operator-disable
injections exited as expected. Both the new turn entry and old forward entry
pass static seal checks. These measurements do not measure Bluetooth radio latency.

```bash
cd /home/tony/open_sprite_runtime
bash run_sprite0825_xbox_native_turn120_on_253.sh --check
# Only with onsite fall arrest, clear arm/walking space and power safety operator:
sudo bash run_sprite0825_xbox_native_turn120_on_253.sh ENABLE_XBOX_TURN120
```

Start with gentle left/right inputs during slow forward motion. Do not interpret
the maximum command or the120-second timeout as a required test duration.
Release LT to request standing; LB+RB3s bypasses standing and requests disable,
so the fall arrest must catch/support the robot. Independent cutoff is the emergency stop.

## Heading boundary (existing behavior)

The native client resets session-relative gyro heading before its first policy
inference. That value is currently telemetry only: no automatic heading-hold
correction is added to `wz`. Resetting heading does not remove gyro bias or
guarantee a fixed drift bound over 1-2 minutes.
