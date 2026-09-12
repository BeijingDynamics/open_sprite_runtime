# Sprite0825 4340P Shoulder Revision

## Hardware decision

The following four proximal arm joints use DM-J4340P-2EC motors:

- `left_shoulder_pitch_joint`
- `left_shoulder_roll_joint`
- `right_shoulder_pitch_joint`
- `right_shoulder_roll_joint`

The shoulder yaw, elbow, and wrist-yaw joints remain DM-J4310P-2EC. This
revision does not change joint axes, origins, mechanical limits, policy joint
order, encoder directions, CAN IDs, or zero offsets.

The confirmed J4340P values used by simulation and acceptance tooling are
14 Nm rated torque, 40 Nm peak torque, 3.77 rad/s rated speed, and a
conservative 9.3 rad/s no-load envelope at the approximately 38 V robot bus.

## Policy compatibility

The old Sprite0825 v4/G59-G70 policies remain frozen rollback artifacts. They
were trained with J4310P shoulder pitch/roll dynamics and must not be relabeled
as hardware-faithful policies for this revision.

The replacement training line started from scratch on the isolated v5/G71
asset. It uses PM01-style reflected inertia, impedance gains, and action
scaling for J4340P shoulder pitch/roll. The resulting G74 model3000 passed
independent seed303/404 Isaac qualification and all gates in the unchanged
seven-case MuJoCo matrix, so it is the frozen software candidate. The
Isaac-only rank winner model5999 remains recorded but was not promoted because
its MuJoCo cadence was 115.38 steps/min, below the fixed 120 steps/min floor.

## Required real-robot measurements

The hardware manifest intentionally leaves CAN IDs, motor zeros, encoder
directions, linkage signs, soft/hard limits, MIT ranges, currents, and thermal
limits unset until they are measured on the assembled robot. Do not infer these
values from the URDF.

The v5 rigid-body mass, center-of-mass, and inertia values still originate from
the CAD model made before the physical motor replacement. Re-import the final
shoulder assemblies after the J4340P motors, brackets, and wiring are fixed;
then rerun both simulator and motor-envelope qualification before transmitting
torque on hardware.
