# Sprite0825 Damiao protocol source audit

This audit covers receive-only state decoding. It does not authorize CAN
transmission, motor enable, parameter writes, zeroing, or torque commands.

## Pinned primary sources

- `dmBots/motor-sdk`, commit
  `0b2ede457bdbf0882e29ab9958ab8fda047b7f4a`
- `dmBots/damiao-document`, commit
  `5e5a6932b8f013636415174c7433b9685ba24aa2`
- The vendor-style U2CANFD C++ implementation mirrored at
  `gitee.com/kit-miao/motor-sdk`, specifically
  [`damiao.h`](https://gitee.com/kit-miao/motor-sdk/blob/master/C%2B%2B%E4%BE%8B%E7%A8%8B/u2canfd/include/protocol/damiao.h)
  and
  [`damiao.cpp`](https://gitee.com/kit-miao/motor-sdk/blob/master/C%2B%2B%E4%BE%8B%E7%A8%8B/u2canfd/src/protocol/damiao.cpp).
- `damiao-document/调试助手使用说明书（达妙驱动控制协议）V1.4.pdf`,
  section 4.1 for the common feedback frame and section 4.2 for MIT mapping.
- `damiao-document/3. 电机上手流程(三)-MIT模式说明.pdf` for the
  embedded MIT impedance equation.

The repositories are retained read-only under
`/home/tony/sprite/reference_repos/`. No vendor example was executed.

## Verified feedback contract

All control modes use one common eight-byte standard-CAN feedback frame:

```text
CAN StdID = configured Master ID
D0 = status[7:4] | controller CAN ID[3:0]
D1:D2 = position, unsigned 16-bit
D3:D4[7:4] = velocity, unsigned 12-bit
D4[3:0]:D5 = estimated output torque, unsigned 12-bit
D6 = MOS temperature in degrees C
D7 = rotor/coil temperature in degrees C
```

Status values verified from V1.4 are disabled `0`, enabled `1`, over-voltage
`8`, under-voltage `9`, over-current `A`, MOS over-temperature `B`, motor-coil
over-temperature `C`, communication lost `D`, and overload `E`.

Position, velocity, and torque are linearly mapped using the PMAX, VMAX, and
TMAX values stored in that individual drive. The runtime therefore rejects
SDK defaults or nameplate substitutions. Each motor's ranges must be captured
from register readback and stored with `source=motor_register_readback` before
its feedback can be decoded.

The decoded torque is named `estimated_output_torque_nm`. It is the drive's
reported estimate based on its identified motor parameters, gear ratio, and
torque coefficient. It is not an independent torque-sensor measurement.

## Parameter-read protocol and types

The inspected C++ source constructs a read request on standard CAN ID `0x7FF`
with payload `[motor_id_lo, motor_id_hi, 0x33, RID, 0, 0, 0, 0]`. Its response
decoder treats RIDs 7-10, 13-16, and 35-36 as little-endian `uint32`; all other
listed registers are little-endian `float32`. The runtime follows that exact
split. In particular, `hw_ver` (13), `sw_ver` (14), and `sub_ver` (36) are
integers, while `OT_Value` (2), `OC_Value` (3), `MAX_SPD` (6), PMAX (21), VMAX
(22), and TMAX (23) are floats.

The SDK source names these registers but does not define their engineering
semantics. The DM-J4340P-2EC V1.1 User Manual V1.1 (2026-04-09), pages 6, 19,
20, and 29, supplies the missing definitions:

- `OT_Value` is the motor-winding over-temperature threshold in degrees C. The
  manual recommends at most 100 C; crossing it disables motor output.
- `OC_Value` is the maximum phase-current fraction of rated phase current. A
  value of 0.8 means 80%, not 0.8 A. For the 48 V J4340P nameplate rated phase
  current of 4.11 A, this corresponds to a configured phase-current threshold
  of approximately 3.288 A.
- `MAX_SPD` is a rad/s limit on rotor speed before gear reduction and applies
  only in velocity mode. It is not the output-shaft or policy-joint speed limit.
- `hw_ver` is reserved, `sw_ver` is the firmware version, and `sub_ver` is the
  firmware sub-version/minor version.

The same register map is used by the inspected common Damiao SDK, but the
J4340P manual is not silently treated as a nameplate manual for J4310P, J3507,
or J6248P. Integer version contents remain raw when a drive reports zero or an
undocumented packed representation. Drive shutdown values also remain distinct
from lower project operating limits.

## Safety-critical distinctions

- DM-J4340P rated/peak capability remains 14/40 Nm in the Sprite hardware
  safety envelope. The official SDK's default DM4340 `TMAX=28` is only a CAN
  quantization setting and must not replace the 40 Nm hardware peak limit.
- DM-J4310P ankle rated/peak capability remains 3.5/12.5 Nm. Each physical
  ankle motor is decoded first; joint pitch/roll and motor loads are then
  related through the measured differential Jacobian.
- Four proximal shoulder motors are DM-J4340P: left/right shoulder pitch and
  roll. Distal shoulder yaw, elbow, and wrist motors remain DM-J4310P.
- Feedback identity is `(SocketCAN interface, Master ID)`, checked again
  against the controller CAN ID in D0. Enumeration order is never accepted.
- A motor control ID must currently fit D0's four-bit controller-ID field.
  Sprite's four-bus plan supports this by reusing low IDs on separate buses.

## Passive shadow limitation

Damiao's SDK obtains normal feedback after a control command and offers an
explicit status-refresh request. A kernel listen-only interface cannot issue
either. The RX-only shadow phase therefore requires an already operating,
separately approved controller or a verified drive configuration that produces
periodic feedback. Receiving zero frames is a failed test, not permission to
enable polling.

## Explicitly prohibited legacy code

- `/home/tony/can_test/can_tx_test.py` and `.c` transmit arbitrary ID `0x123`.
- `/home/tony/sprite/damiao_keyboard.py` configures/broadcasts and enables a
  motor before sending velocity commands.
- `/home/tony/superman_arm_2025_5_21/.../CanMotor.*` implements CANopen/CiA402
  CSP, not the Damiao MIT contract.

None of these files may be imported, copied into, or executed by the Sprite
commissioning runtime.

## Implemented RX-only evidence path

`DamiaoFeedbackDecoder` maps every frame by `(SocketCAN interface, Master ID)`
and checks the controller-ID nibble. `DamiaoRxAudit` then fails closed on a
missing motor, fault status, raw timestamp regression, insufficient samples,
feedback below 475 Hz, any raw timestamp gap above 6 ms, or kernel-to-userspace
queue-age P99 above 6 ms. Its report always records
`hardware_tx_attempts=0`.

Raw KH adapter time and host time remain separate. The hardware timestamp is
used only for per-motor rate/gap statistics; queue age uses the kernel software
RX timestamp and userspace receipt time.

## Remaining hardware evidence

Before any torque-enabled run, record for all 31 physical motors: interface,
CAN ID, Master ID, firmware, PMAX/VMAX/TMAX readback, encoder zero/direction,
current and temperature bounds, and measured hard/soft limits. Then capture a
listen-only hardware-timestamped trace and prove complete IDs, monotonic sample
ages, status health, and p99 receive age at most 6 ms.
## MIT command encoder audit

The official Python and C++ U2CANFD implementations at dmBots/motor-sdk commit
`0b2ede457bdbf0882e29ab9958ab8fda047b7f4a` both define `MIT_MODE = 0x000`.
Consequently, an MIT command uses the configured motor CAN ID unchanged. The
`+ mode` expression in the SDK is a common dispatch formula; offsets `0x100`,
`0x200`, and `0x300` apply only to the other control modes.

The runtime's pure encoder reproduces the official 16/12/12/12/12-bit payload
layout and truncating in-range quantization. It intentionally does not reproduce
unsafe SDK boundary behavior: Python silently clips and C++ can wrap values.
The runtime rejects non-finite values, protocol-range violations, and violations
of the narrower project command envelope. It requires fresh measured position
and velocity to gate the complete official MIT request
`kp*(q_des-q)+kd*(dq_des-dq)+tau_ff`, not only the feedforward term. The encoder
has no socket, transmit, enable, zero-setting, or register-writing capability.

PMAX/VMAX/TMAX remain motor register readbacks, not SDK model defaults. In
particular, motor mechanical ratings and the active MIT quantization TMAX are
different facts and must never be substituted for each other.
