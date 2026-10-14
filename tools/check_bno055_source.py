"""Finite live source check at actor cadence. No CAN, IPC, policy, or motors."""
import json
from pathlib import Path
import time
from open_sprite_runtime.bno055_source import Bno055Source


def main():
    ages = []
    report = {'errors': [], 'can_tx_count': 0}
    try:
        with Bno055Source(initialize=True) as source:
            start = time.monotonic()
            while time.monotonic()-start < 10:
                source.drain(lambda packet, stamp: ages.append((time.monotonic_ns()-stamp)/1e6))
                time.sleep(.02)
            report.update(source.report())
    except Exception as exc:
        report['errors'].append(str(exc))
    report.update(delivered_packets=len(ages), maximum_host_age_ms=max(ages, default=None))
    path = Path('reports') / time.strftime('bno055_source_check_%Y%m%d_%H%M%S.json')
    path.parent.mkdir(exist_ok=True)
    path.write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2), path)
    raise SystemExit(bool(report['errors']) or not ages)


if __name__ == '__main__':
    main()
