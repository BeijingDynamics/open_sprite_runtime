"""Summarize a read-only native I2C length comparison."""
import csv
import json
import sys
from pathlib import Path


def summary(rows):
    values=sorted(int(r['read_ns'])/1e6 for r in rows)
    return dict(samples=len(values),median_ms=values[len(values)//2],
                p99_ms=values[int(.99*(len(values)-1))],maximum_ms=values[-1],
                over_10ms=sum(v>10 for v in values),
                skipped_slots=sum(int(r['skipped_slots']) for r in rows))


path=Path(sys.argv[1])
with path.open(newline='') as f:
    rows=list(csv.DictReader(f))
result=dict(by_mode={mode:summary([r for r in rows if r['mode']==mode])
                     for mode in ('46','32','26','23')},
            by_block={str(b):dict(mode=next(r['mode'] for r in rows if int(r['block'])==b),
                      **summary([r for r in rows if int(r['block'])==b]))
                      for b in sorted({int(r['block']) for r in rows})})
path.with_suffix('.summary.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result,indent=2))
