"""Attribute host sampling gaps using native diagnostic timestamps."""
import argparse
import csv
import json
from pathlib import Path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('trace')
    args = parser.parse_args()
    metadata_path=Path(args.trace.removesuffix('.timing.csv'))
    metadata=json.loads(metadata_path.read_text()) if metadata_path.exists() else {}
    queue_valid=metadata.get('schedstats_enabled')==1
    with open(args.trace, newline='') as source:
        rows = [{key: int(value) for key, value in row.items()}
                for row in csv.DictReader(source)]
    events = []
    for previous, current in zip(rows, rows[1:]):
        gap = current['start_ns'] - previous['start_ns']
        if gap > 15_000_000:
            events.append(dict(sequence=current['sequence'], gap_ms=gap/1e6,
                previous_read_ms=previous['read_ns']/1e6,
                previous_health_ms=previous['health_ns']/1e6,
                previous_lateness_ms=previous['lateness_ns']/1e6,
                previous_skipped=previous['skipped_slots'],
                current_lateness_ms=current['lateness_ns']/1e6))
    slow=[]
    for row in sorted(rows,key=lambda r:r['read_ns'],reverse=True)[:10]:
        item=dict(sequence=row['sequence'],read_ms=row['read_ns']/1e6,
                  cpu_ms=row.get('read_cpu_ns',0)/1e6)
        item['runqueue_ms']=row.get('runqueue_wait_ns',0)/1e6 if queue_valid else None
        slow.append(item)
    print(json.dumps(dict(samples=len(rows),schedstats_valid=queue_valid,
        skipped_total=sum(row['skipped_slots'] for row in rows),
        health_associated_skips=sum(row['skipped_slots'] for row in rows if row['health_ns']),
        maximum_ms={key:max((row[key] for row in rows), default=0)/1e6
                    for key in ('lateness_ns','read_ns','health_ns')},
        gaps_over_15ms=events,slowest_reads=slow), indent=2))


if __name__ == '__main__':
    main()
