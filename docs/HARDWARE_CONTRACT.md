# Hardware contract

Before shadow mode, fill every policy joint entry in `motor_map` with:

- Damiao model and firmware version
- USB-CAN FD channel and CAN ID
- mechanical zero and encoder zero
- positive policy direction versus positive motor direction
- reduction ratio and any linkage ratio
- soft limit and independently measured hard limit
- rated/peak torque, speed, current, and temperature limits
- MIT protocol ranges and quantization for position, velocity, Kp, Kd, torque

The runtime must reorder feedback into the exact 31-joint policy order from the
qualified contract. It must never use CAN enumeration order as policy order.

## Differential ankles

The ideal linear map is:

```text
[q_motor_a]   [1  1] [q_pitch]
[q_motor_b] = [1 -1] [q_roll ] + motor_zero
```

The real left and right matrices must be measured separately. The torque map is
the inverse transpose, not the position map:

```text
tau_motor = A^-T tau_joint
```

This preserves instantaneous mechanical power. For the ideal matrix and equal
pitch/roll gains, embedded motor gains are half the joint gains. With unequal
joint gains or a non-ideal measured Jacobian, the exact motor impedance contains
cross terms. Independent motor PD alone then cannot reproduce the trained joint
impedance; the 500 Hz layer must supply the cross-coupled correction through
MIT feed-forward torque, or the policy must be requalified with the realizable
impedance.

