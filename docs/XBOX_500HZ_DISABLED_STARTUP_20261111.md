# Disabled startup before strict500Hz control

This change belongs only to the independent `xbox_500hz` candidate. The stable
runtime, model, gains, limits, IMU and actor rates are unchanged. No hardware
test is implied by an offline build or seal.

## Sequence

1. Open four CAN sockets, select CPU/RT priority and lock memory as before.
2. Before either experimental shadow or powered preparation, poll all31 motors
   at500Hz with position echo, velocity0, Kp0, Kd0 and feedforward torque0.
   Require disabled feedback and the existing position, velocity, torque and
   thermal guards. There is no enable, disable, mode or zero-reset command
   in this startup function.
3. Allow at most2s to settle. Each startup slot has a hard2ms budget, while
   every crossing of the ordinary0.5ms budget remains recorded. Expired slots
   send nothing; schedule a future slot, never a catch-up burst.
4. Require at least400 transport slots (about200ms) and200 consecutive clean
   slots (about100ms), each below0.5ms with fresh disabled feedback from all31
   motors. At the end, drain without transmitting for at most20ms; every motor
   must have matching positive TX/RX counts and feedback younger than20ms.
5. Write `<can-report>.startup.json` with separate counters, first/worst timing,
   all startup budget crossings and pass/fail. Failure exits before enable.
   Reset only the per-stage TX/RX counters and seen flags, not thermal/RMS state.
6. Run the existing shadow or protected powered preparation and strict loop.
   The strict loop still checks slot0, uses0.5ms, and keeps every later guard.
   The120s strict shadow still requires60000 TX per motor, excluding the
   separately recorded startup polls. The full-chain checker requires the
   startup evidence as well as the existing gates.

This is a cold-start preparation stage, not a fix for a later wake/receive
overrun, and not an exemption for the first enabled command. No permanent
QoS/governor change is made. A real zero-gain test is required before another
powered test. Existing powered preparation still verifies the measured-pose
zero-gain actor target before enabling.

## Tests and operator step

Offline tests cover retained525us startup evidence, unchanged active500us
rejection, expired-slot skipping, the consecutive-clean-window reset, missing
feedback/timeout, cancellation and exceptional feedback handling. Existing
socketpair monitor and shutdown tests remain in the suite; they are not CAN
hardware tests.

After rebuilding the independent candidate, reinstall its capability only:

```bash
cd /home/tony/open_sprite_runtime_xbox_500hz
sudo setcap cap_sys_nice+ep candidates/xbox_500hz_powered_20261111/sprite_can_xbox500
sudo bash run_xbox_500hz_full_shadow_120s_on_253.sh ZERO_GAIN_XBOX_500HZ_FULL_31_DISABLED --feedback-candidate
```

Robot supported, all31 motors powered but disabled, stationary for the initial
15s IMU calibration. This command does not enable motors or transmit nonzero
gain/torque. Keep the powered entry unused until this candidate is evaluated.
