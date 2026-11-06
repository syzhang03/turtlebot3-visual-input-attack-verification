#!/usr/bin/env python3
"""Offline audit of distinct pairing and confirmation interval (not attack latency)."""
import argparse
import csv
import json
from pathlib import Path
import statistics

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('folder')
args=parser.parse_args()
root=Path(args.folder)
results=[]
for path in sorted(root.glob('*/*/*/pairs.csv')):
    with path.open() as stream: rows=list(csv.DictReader(stream))
    images=[r['image_stamp'] for r in rows]
    scans=[r['scan_stamp'] for r in rows]
    first_alert=next((i for i,r in enumerate(rows) if r['alert']=='True'),None)
    span=None
    if first_alert is not None:
        before=rows[:first_alert+1]
        start=next(r for r in reversed(before) if int(r['count'])==1)
        end=rows[first_alert]
        span=max(float(end['image_stamp']),float(end['scan_stamp']))-max(float(start['image_stamp']),float(start['scan_stamp']))
    checks=dict(unique_image_stamps=len(images)==len(set(images)),
                unique_scan_stamps=len(scans)==len(set(scans)),
                tolerance_respected=all(abs(float(r['image_stamp'])-float(r['scan_stamp']))<=.120000001 for r in rows),
                ambiguity_does_not_accumulate=all(int(r['count'])==0 for r in rows if r['direction']=='AMBIGUOUS'),
                first_alert_has_three_pairs=(first_alert is None or int(rows[first_alert]['count'])==3))
    results.append(dict(trial=path.relative_to(root).parts[0],pairs=len(rows),checks=checks,
                        confirmation_interval_sim_s=span))
spans=[r['confirmation_interval_sim_s'] for r in results if r['confirmation_interval_sim_s'] is not None]
report=dict(trials=len(results),all_checks_passed=all(all(r['checks'].values()) for r in results),
            confirmed_trials=len(spans),confirmation_interval_mean_sim_s=statistics.mean(spans) if spans else None,
            confirmation_interval_range_sim_s=[min(spans),max(spans)] if spans else None,
            note='Interval from the first pair in the confirming sequence to the confirming pair, not attack-onset detection latency.',
            per_trial=results)
(root/'pair-audit.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({k:v for k,v in report.items() if k!='per_trial'},indent=2))
