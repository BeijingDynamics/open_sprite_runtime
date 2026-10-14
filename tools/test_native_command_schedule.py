"""Compare every millisecond of both native schedules with the Python baseline; no hardware."""
import subprocess
from pathlib import Path
from open_sprite_runtime.command_schedule import sample_stand_walk_stop

root = Path(__file__).resolve().parents[1]
binary = root / "build/policy_native_stage2/test_policy_command_schedule"
subprocess.run(["g++", "-std=c++17", "-O2", "-ffp-contract=off", "-Wall", "-Wextra", "-Werror",
                str(root / "native/tests/test_policy_command_schedule.cpp"), "-o", str(binary)], check=True)
result = subprocess.run([str(binary)], check=True, capture_output=True, text=True)
maximum = 0.0
lines = result.stdout.splitlines()
assert len(lines) == 80002
for line in lines:
    speed, ms, vx, stage = line.split()
    profile = "stand-walk010-stop-40s" if float(speed) == .1 else "stand-walk-stop-40s"
    expected, expected_stage = sample_stand_walk_stop(int(ms)/1000, profile)
    maximum = max(maximum, abs(float(vx)-expected))
    assert stage == expected_stage
assert maximum < 1e-14, maximum
print(f"PASS schedule_samples={len(lines)} maximum_vx_error={maximum} no_hardware=True")
