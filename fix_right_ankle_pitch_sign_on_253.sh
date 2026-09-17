#!/usr/bin/env bash
set -euo pipefail

ROOT=/home/tony/open_sprite_runtime
CSV="$ROOT/calibration/right_ankle.csv"
FIT="$ROOT/calibration/right_ankle_fit.json"
CONFIG="$ROOT/config/hardware.sprite0825.measurement.json"
STAMP="$(date +%Y%m%d_%H%M%S)"

cp -a "$CSV" "$ROOT/calibration/right_ankle_pre_pitch_sign_fix_${STAMP}.csv"
cp -a "$FIT" "$ROOT/calibration/right_ankle_fit_pre_pitch_sign_fix_${STAMP}.json"
cp -a "$CONFIG" "$ROOT/config/hardware.sprite0825.measurement_pre_right_ankle_pitch_sign_fix_${STAMP}.json"

python3 - "$CSV" <<'PY'
import csv
import os
import sys

path = sys.argv[1]
tmp = path + ".tmp"
with open(path, newline="", encoding="utf-8") as src:
    rows = list(csv.reader(src))

if not rows or rows[0] != ["pitch_rad", "roll_rad", "motor_a_rad", "motor_b_rad"]:
    raise SystemExit(f"unexpected calibration CSV header: {rows[0] if rows else None}")

for row in rows[1:]:
    row[0] = f"{-float(row[0]):.6f}"

with open(tmp, "w", newline="", encoding="utf-8") as dst:
    csv.writer(dst, lineterminator="\n").writerows(rows)
os.replace(tmp, path)
PY

cd "$ROOT"
PYTHONPATH="$ROOT/src" python3 -m open_sprite_runtime.cli differential-calibrate \
  --pair right_ankle \
  --samples "$CSV" \
  --output "$FIT" \
  --maximum-rms-residual-rad 0.012

python3 - "$FIT" "$CONFIG" <<'PY'
import json
import math
import os
import sys

fit_path, config_path = sys.argv[1:]
with open(fit_path, encoding="utf-8") as f:
    fit = json.load(f)
if not fit.get("passed"):
    raise SystemExit("refitted right-ankle calibration did not pass")

expected = [
    [-1.3055003739519502, -1.011683046199467],
    [1.3051579126920603, -1.012140973324761],
]
matrix = fit["joint_to_motor_matrix"]
for actual_row, expected_row in zip(matrix, expected):
    for actual, wanted in zip(actual_row, expected_row):
        if not math.isclose(actual, wanted, rel_tol=0.0, abs_tol=1e-10):
            raise SystemExit(f"unexpected refitted matrix: {matrix}")

with open(config_path, encoding="utf-8") as f:
    config = json.load(f)
pair = config["differentials"]["right_ankle"]
pair["joint_to_motor_matrix"] = matrix
pair["motor_zero_rad"] = fit["motor_zero_rad"]
pair["source"] = (
    "calibration/right_ankle_fit.json; 25 unloaded samples; RMS gate 0.012 rad; "
    "pitch convention corrected by physical MuJoCo audit 2026-09-16"
)

tmp = config_path + ".tmp"
with open(tmp, "w", encoding="utf-8") as f:
    json.dump(config, f, indent=2, ensure_ascii=True)
    f.write("\n")
os.replace(tmp, config_path)

print("RIGHT_ANKLE_MATRIX", matrix)
print("RIGHT_ANKLE_ZERO", fit["motor_zero_rad"])
print("RIGHT_ANKLE_RMS", fit["rms_residual_rad"])
print("CONFIG_UPDATED", config_path)
PY
