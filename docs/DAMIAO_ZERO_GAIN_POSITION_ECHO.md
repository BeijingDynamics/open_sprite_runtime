# Damiao zero-gain position-echo commissioning

This is an active, single-motor CAN-FD link test. It is not the robot control
runtime and it does not enable, disable, or change a motor mode.

## Preconditions

- Mechanically support or unload the selected joint.
- Keep the emergency stop in reach.
- Confirm the selected drive is already configured for MIT mode.
- Connect only the bus being tested during the first test.
- Use arbitration bitrate 1 Mbps and data bitrate 5 Mbps with CAN FD enabled.

For the first test, use `head_yaw_motor` on `kcan3`, command ID `0x08`, feedback
Master ID `0x18`:

```bash
sudo ip link set dev kcan3 down
sudo ip link set dev kcan3 type can bitrate 1000000 dbitrate 5000000 fd on
sudo ip link set dev kcan3 up
ip -details link show dev kcan3

cd /home/tony
./probe_sprite0825_damiao_zero_gain_on_253.sh head_yaw_motor 2.0 50
```

Each transmitted frame contains the most recently reported position and
`velocity=0`, `Kp=0`, `Kd=0`, `feedforward torque=0`. Before the first valid
reply, the position field is zero; with zero gains it does not contribute to the
nominal MIT torque equation. The transport accepts only an eight-byte frame
whose raw gain fields are zero and whose encoded velocity and torque fields are
zero. It always transmits as CAN FD with bit-rate switching.

An isolated RX frame without complete kernel software plus raw-hardware
timestamp evidence is discarded and counted as `discarded_timestamp_frames`.
No fallback timestamp is invented. Persistent timestamp loss still fails via
the feedback-timeout or minimum sample-coverage gates.

During the probe, the terminal prints live `RX` lines at 10 Hz with position,
velocity, estimated torque, MOS/rotor temperatures, and drive status. The CAN
request/reply loop remains at the separately configured rate, normally 50 Hz.

The command stops transmitting immediately if the selected motor does not reply
within 0.2 seconds, reports a fault status, replies using a non-FD/non-BRS frame,
or the MuJoCo viewer is closed. Duration is capped at 10 seconds and rate at 100
Hz in code.

The JSON report is written under `/home/tony/open_sprite_runtime/reports/`. Check
`passed`, `tx_count`, `rx_count`, `status_codes`, and the first/last/min/max
positions. A passing result validates the selected command-ID/Master-ID pair and
bidirectional transport; it does not yet qualify motor enable, nonzero gains,
torque commands, multi-motor scheduling, or whole-robot operation.

After the test, take the interface down until the next planned step:

```bash
sudo ip link set dev kcan3 down
```
