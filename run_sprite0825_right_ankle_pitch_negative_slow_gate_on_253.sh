#!/usr/bin/env bash
set -euo pipefail

ACK=ENABLE_RIGHT_ANKLE_PITCH_NEGATIVE_SLOW_GATE
if [[ "${1:-}" != "$ACK" ]]; then
  echo "UNLOADED SLOW NEGATIVE-PITCH GATE: right ankle only" >&2
  echo "Robot must be supported, BOTH FEET OFF THE FLOOR, right ankle clear, safety operator ready." >&2
  echo "Sequence: zero -> -0.03rad over 1.5s -> hold -> zero over 1.5s; 5s at 500Hz." >&2
  echo "Host PD Kp=16.0 Kd=0.10; |joint torque|<=0.50Nm; |motor torque|<=0.30Nm." >&2
  echo "Motor speed guard is 0.8rad/s, below 7% of the DM-J4310P rated speed." >&2
  echo "Any direction, displacement, torque, feedback, or timing failure disables both motors." >&2
  echo "This command never resets motor zero positions and never switches motor mode." >&2
  echo "Run only after explicit physical confirmation: $0 $ACK" >&2
  exit 2
fi

ROOT=/home/tony/open_sprite_runtime
STAMP="$(date +%Y%m%d_%H%M%S)"
SNAPSHOT="$ROOT/reports/kcan2_right_ankle_pitch_negative_slow_${STAMP}.json"
OUTPUT="$ROOT/reports/right_ankle_pitch_negative_slow_500hz_${STAMP}.json"
mkdir -p "$ROOT/reports"
ip -j -d link show type can > "$SNAPSHOT"
echo "ACTIVE SLOW NEGATIVE-PITCH GATE: right ankle motors kcan2 IDs 0x05/0x06 only"
echo "BOTH FEET MUST REMAIN OFF THE FLOOR; RIGHT ANKLE MUST BE CLEAR"
echo "5.0s; 500Hz; pitch -0.03rad over 1.5s then smooth return"
echo "|tau_joint|<=0.50Nm; |tau_motor|<=0.30Nm; |motor speed|<=0.8rad/s"
echo "NO set-zero and NO mode switch; automatic disable"
cd "$ROOT"
exec env PYTHONPATH="$ROOT/src" "$ROOT/.venv/bin/python" \
  "$ROOT/tools/run_ankle_pitch_direction_gate.py" \
  --side right --excitation-tier negative-slow \
  --hardware "$ROOT/config/hardware.sprite0825.measurement.json" \
  --contract "$ROOT/artifacts/g60_model3450/deploy/contract.json" \
  --socketcan-snapshot "$SNAPSHOT" --output "$OUTPUT" \
  --robot-supported --tested-foot-unloaded --safety-operator-ready --ankle-clear \
  --confirm-hardware-tx "$ACK"
