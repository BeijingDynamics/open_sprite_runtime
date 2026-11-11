"""Run a fixed-duration, zero-gain CAN transport test with link counters."""

import argparse
from collections import Counter
import csv
import fcntl
import json
from pathlib import Path
import subprocess
import tempfile


ROOT = Path(__file__).resolve().parents[1]
BINARY = ROOT / "build/xbox_500hz/sprite_can_shadow"
CONFIG = ROOT / "config/xbox_500hz.motors.local.tsv"
MOTOR_COUNTS = {"kcan1": 8, "kcan2": 8, "kcan3": 8, "kcan4": 7}
CONFIRM = "ZERO_GAIN_XBOX_500HZ_31_DISABLED"


def can_links():
    links = json.loads(
        subprocess.check_output(["ip", "-details", "-statistics", "-json", "link", "show"])
    )
    return {link["ifname"]: link for link in links if link["ifname"] in MOTOR_COUNTS}


def unready_links(selected):
    unready = []
    for name in MOTOR_COUNTS:
        link = selected.get(name, {})
        data = link.get("linkinfo", {}).get("info_data", {})
        if (
            "UP" not in link.get("flags", [])
            or link.get("mtu") != 72
            or "FD" not in data.get("ctrlmode", [])
            or data.get("state") != "ERROR-ACTIVE"
            or data.get("bittiming", {}).get("bitrate") != 1_000_000
            or data.get("data_bittiming", {}).get("bitrate") != 5_000_000
        ):
            unready.append(name)
    return unready


def require_ready(links):
    unready = unready_links(links)
    if unready:
        raise RuntimeError(f"{', '.join(unready)}: require UP, ERROR-ACTIVE, CAN FD 1M/5M")


def motor_names():
    with CONFIG.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream, delimiter="\t"))
    names = {row["motor_name"] for row in rows}
    endpoints = {(row["interface"], int(row["can_id"])) for row in rows}
    if (
        len(rows) != 31
        or len(names) != 31
        or len(endpoints) != 31
        or Counter(row["interface"] for row in rows) != MOTOR_COUNTS
    ):
        raise RuntimeError("expected 31 distinct motors on kcan1-4 (8/8/8/7)")
    return names


def record(path, data):
    path.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")


def link_errors(links):
    return {
        name: {
            **{
                f"{direction}_{key}": link["stats64"][direction][key]
                for direction in ("rx", "tx")
                for key in ("errors", "dropped")
            },
            **{
                key: link["linkinfo"]["info_xstats"][key]
                for key in ("bus_error", "error_warning", "error_passive", "bus_off", "restarts")
            },
        }
        for name, link in links.items()
    }


def run():
    p = argparse.ArgumentParser(description=__doc__)
    mode = p.add_mutually_exclusive_group(required=True)
    mode.add_argument("--check", action="store_true")
    mode.add_argument("--confirm", choices=[CONFIRM])
    args = p.parse_args()
    if not BINARY.is_file():
        raise RuntimeError(f"missing experimental binary: {BINARY}")
    names = motor_names()
    capabilities = subprocess.check_output(["getcap", str(BINARY)], text=True).strip()
    has_rt = capabilities.endswith(" cap_sys_nice=ep")
    if args.check:
        links = can_links()
        require_ready(links)
        link_errors(links)
        print(f"CHECK_OK motors={len(names)} buses=4 CAN_FD=1M/5M rt_cap={has_rt}; no CAN TX")
        return
    if not has_rt:
        raise RuntimeError(f"set CAP_SYS_NICE on the experimental binary only: {BINARY}")
    reports = ROOT / "reports"
    reports.mkdir(exist_ok=True)
    with (reports / ".xbox_500hz.lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        running = subprocess.run(
            ["pgrep", "-f", r"[s]prite_can_|[s]prite_policy_|[o]pen_sprite_runtime\.native_policy_client"],
            check=False,
            stdout=subprocess.DEVNULL,
        )
        if running.returncode != 1:
            raise RuntimeError("another control process is active or could not be checked")
        before = can_links()
        require_ready(before)
        before_errors = link_errors(before)
        folder = Path(tempfile.mkdtemp(prefix="xbox_500hz_shadow_10s_", dir=reports))
        record(folder / "can_before.json", before)
        native = folder / "native.json"
        print(f"REPORT_DIRECTORY {folder}", flush=True)
        print("ZERO GAIN ONLY: 31 disabled motors, 500 Hz each, 10 seconds", flush=True)
        try:
            result = subprocess.run(
                [
                    str(BINARY), "--xbox-500hz-shadow",
                    "--config", str(CONFIG), "--output", str(native),
                    "--duration", "10", "--cpu", "5", "--realtime-priority", "50",
                    "--acknowledge-hardware-tx", "ZERO_GAIN_NATIVE_SHADOW",
                    "--all-motors-disabled-confirmed",
                ],
                check=False,
            )
        finally:
            after = can_links()
            record(folder / "can_after.json", after)
        after_errors = link_errors(after)
        differences = {
            bus: {
                key: value - before_errors[bus][key] for key, value in counters.items()
            }
            for bus, counters in after_errors.items()
        }
        counters_ok = all(value == 0 for bus in differences.values() for value in bus.values())
        report = json.loads(native.read_text()) if native.is_file() else {}
        counts = report.get("tx_count_by_motor", {})
        exact_tx = counts.keys() == names and all(value == 5000 for value in counts.values())
        unready = unready_links(after)
        passed = (
            result.returncode == 0 and report.get("passed") is True
            and exact_tx and counters_ok and not unready
        )
        summary = {
            "passed": passed,
            "native_exit": result.returncode,
            "exactly_5000_tx_per_motor": exact_tx,
            "unready_links_after": unready,
            "can_error_counter_delta": differences,
        }
        record(folder / "summary.json", summary)
        print(f"XBOX_500HZ_SHADOW_{'PASSED' if passed else 'FAILED'} {folder / 'summary.json'}")
        if not passed:
            raise SystemExit(2)


if __name__ == "__main__":
    try:
        run()
    except (OSError, RuntimeError, KeyError, ValueError, subprocess.SubprocessError) as exc:
        raise SystemExit(f"xbox_500hz: {exc}") from exc
