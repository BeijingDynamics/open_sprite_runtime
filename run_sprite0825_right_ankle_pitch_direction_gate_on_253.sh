#!/usr/bin/env bash
set -euo pipefail

ACK=ENABLE_RIGHT_ANKLE_PITCH_DIRECTION_GATE
if [[ "${1:-}" != "$ACK" ]]; then
  echo "UNLOADED BIDIRECTIONAL PITCH GATE: right ankle only" >&2
  echo "Robot must be supported, RIGHT FOOT FULLY OFF THE FLOOR, linkages clear, safety operator ready." >&2
  echo "Sequence: +0.03rad -> zero -> -0.03rad -> zero; 4.5s at 500Hz." >&2
  echo "Host PD Kp=4.0 Kd=0.05; |joint torque|<=0.15Nm; |motor torque|<=0.25Nm." >&2
  echo "Any direction, velocity, displacement, torque, feedback, or timing failure disables both motors." >&2
  echo "This command never resets motor zero positions and never switches motor mode." >&2
  echo "Run only after explicit physical confirmation: $0 $ACK" >&2
  exit 2
fi

ROOT=/home/tony/open_sprite_runtime
STAMP="$(date +%Y%m%d_%H%M%S)"
SNAPSHOT="$ROOT/reports/kcan2_right_ankle_pitch_direction_${STAMP}.json"
OUTPUT="$ROOT/reports/right_ankle_pitch_direction_500hz_${STAMP}.json"
mkdir -p "$ROOT/reports"
ip -j -d link show type can > "$SNAPSHOT"
echo "ACTIVE DIRECTION GATE: right ankle motors kcan2 IDs 0x05/0x06 only"
echo "RIGHT FOOT MUST BE FULLY UNLOADED AND CLEAR"
echo "4.5s; 500Hz; pitch +/-0.03rad; joint Kp=4.0 Kd=0.05"
echo "|tau_joint|<=0.15Nm; |tau_motor|<=0.25Nm; automatic disable"
echo "NO set-zero and NO mode switch"
cd "$ROOT"
exec env PYTHONPATH="$ROOT/src" "$ROOT/.venv/bin/python" \
  "$ROOT/tools/run_ankle_pitch_direction_gate.py" \
  --side right --hardware "$ROOT/config/hardware.sprite0825.measurement.json" \
  --contract "$ROOT/artifacts/g60_model3450/deploy/contract.json" \
  --socketcan-snapshot "$SNAPSHOT" --output "$OUTPUT" \
  --robot-supported --tested-foot-unloaded --safety-operator-ready --ankle-clear \
  --confirm-hardware-tx "$ACK"
