#!/usr/bin/env bash
set -euo pipefail

ACK=ENABLE_RIGHT_PROXIMAL_LEG_LOW_GAIN_HOLD
if [[ "${1:-}" != "$ACK" ]]; then
  echo "HIGHER-RISK FIXED HOLD: right hip/knee only; ankle and head excluded" >&2
  echo "Robot must be securely suspended, right leg/cables clear, and safety operator ready." >&2
  echo "This command never resets motor zero positions." >&2
  echo "Run only after explicit physical confirmation: $0 $ACK" >&2
  exit 2
fi

ROOT=/home/tony/open_sprite_runtime
STAMP="$(date +%Y%m%d_%H%M%S)"
SNAPSHOT="$ROOT/reports/kcan2_right_proximal_leg_hold_${STAMP}.json"
OUTPUT="$ROOT/reports/right_proximal_leg_low_gain_hold_${STAMP}.json"
mkdir -p "$ROOT/reports"
ip -j -d link show type can > "$SNAPSHOT"

echo "ACTIVE HOLD: right hip/knee IDs 0x01..0x04 only; ankle and head excluded"
echo "2.0s at 50Hz; Kp=0.2 Kd=0.05 tau=0; torque guard 0.5Nm"
echo "NO motion command; NO mode switch; NO set-zero"
cd "$ROOT"
exec env PYTHONPATH="$ROOT/src" "$ROOT/.venv/bin/python" \
  "$ROOT/tools/hold_proximal_leg_group_low_gain.py" \
  --side right --hardware "$ROOT/config/hardware.sprite0825.measurement.json" \
  --socketcan-snapshot "$SNAPSHOT" --output "$OUTPUT" \
  --robot-supported --safety-operator-ready --leg-clear \
  --confirm-hardware-tx "$ACK"
