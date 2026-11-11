# Xbox 500 Hz CAN experiment

Branch `xbox_500hz` starts from the G74 model3000 / ICM45686 / Xbox snapshot
`4515206`. The published walking entry and its qualification seals are
unchanged. The first stage exercises the new schedule at zero gain only;
it cannot run powered policy or measured-pose hold.

## What changes

| Layer | Walking baseline | Experiment |
| --- | --- | --- |
| Host CAN scheduler | 2 kHz | 2 kHz |
| Four ankle motor requests | 500 Hz each | 500 Hz each |
| Other 27 motor requests | 50 Hz each | 500 Hz each |
| Policy and Xbox target generation | 50 Hz | 50 Hz (unchanged) |
| ICM45686 | 800 Hz | 800 Hz (unchanged) |

The eventual powered comparison would resend the held policy target every
2 ms instead of every 20 ms; it would not generate new policy actions at
500 Hz or change embedded PD. The ankle PD and existing ankle slot phases
stay unchanged. Gains, limits, CAN IDs, controller mapping, and motor
firmware are not changed.

The four slots in a 2 ms cycle carry at most two requests per bus per slot.
The nominal request load is 4000/s on each eight-motor bus and 3500/s on the
seven-motor bus, 15500/s total, versus 3350/s for the baseline. Feedback adds
approximately the same number of frames. The per-bus stress tests do not
qualify the simultaneous traffic on the shared USB adapter.

## Offline build and test

On Linux, from this branch's separate checkout:

```bash
cmake -S native -B build/xbox_500hz -DCMAKE_BUILD_TYPE=Release
cmake --build build/xbox_500hz --target sprite_can_shadow test_xbox_500hz_schedule
ctest --test-dir build/xbox_500hz -R '^xbox_500hz_schedule$' --output-on-failure
```

The test checks the 31-motor schedule over one second, the frame budget per
bus/slot, endpoint ordering, preserved ankle phases, and refusal of powered
options. It does not open a device.

## First onsite test: transport only

Do not install this binary over the qualified one. With explicit onsite
approval, mechanical support, 31 powered but disabled motors, and all
other motor scripts stopped, use the existing verified motor TSV as input.
The binary does not write this TSV. Verify the four SocketCAN links are up
at CAN FD 1M/5M, and collect `ip -details -statistics link show` before and
after. First run for 10 seconds; only proceed to longer tests if clean.

```bash
build/xbox_500hz/sprite_can_shadow \
  --xbox-500hz-shadow \
  --config /path/to/qualified/motors.tsv \
  --output xbox_500hz_shadow_10s.json \
  --duration 10 --cpu 5 --realtime-priority 50 \
  --acknowledge-hardware-tx ZERO_GAIN_NATIVE_SHADOW \
  --all-motors-disabled-confirmed
```

Real-time scheduling requires the approved permissions on the separate
binary. This test transmits zero-gain echo requests, not listen-only frames;
it does not enable drives or apply policy targets. Check for 5000 requests
per motor in 10 s, at least 99% feedback coverage for each motor, no
transmit errors/drops, and zero `transport_slot_overruns`. An expired slot
is skipped instead of sending catch-up bursts. The slot timer measures
scheduled deadline through completion of host work, not wire latency.

For the isolated Jetson checkout, put a copy of the verified TSV at
`config/xbox_500hz.motors.local.tsv` (git-ignored). The helper validates the
four links and captures both counter snapshots and the native report:

```bash
python3 tools/run_xbox_500hz_shadow_10s.py --check
sudo setcap cap_sys_nice+ep build/xbox_500hz/sprite_can_shadow
python3 tools/run_xbox_500hz_shadow_10s.py --confirm ZERO_GAIN_XBOX_500HZ_31_DISABLED
```

`--check` does not transmit. The confirmation is only for the supported,
disabled-motor setup above. The helper rejects another active controller,
missing real-time capability, lost requests, and new CAN errors or drops.

## Second onsite test: full-chain shadow

After transport passes, with the same mechanical support and all 31 motors
still disabled, stop the transport test and all other IMU/control scripts:

```bash
bash run_xbox_500hz_full_shadow_120s_on_253.sh --check
sudo bash run_xbox_500hz_full_shadow_120s_on_253.sh ZERO_GAIN_XBOX_500HZ_FULL_31_DISABLED
```

This reuses the provisioned stable ICM reader, G74 model3000 and native
zero-gain actor read-only. It uses the experimental CAN binary and writes
reports only under the experimental checkout. The first 15 seconds are
stationary gyro-bias calibration, without CAN transmission. Then 120 s
of ICM 800 Hz, actor 50 Hz and 31-motor 500 Hz echo run together. The complete
command takes about 150 s including IMU capture and offline replay. No
gamepad input or powered operation is exercised.

Qualification checks 6000 policy frames, byte-exact offline replay, exact
60000 CAN requests per motor, coverage, deadline/slot overruns, and CAN
error counter deltas. Inspect the reported maximum and P99 compute time,
policy-state age and IMU age, rather than inferring latency from frame counts.
Do not copy
the original Xbox powered seal to this experiment. A powered A/B should
keep the model, gains, caps, command schedule and floor fixed, and compare
tracking and stop behavior as well as timing.
