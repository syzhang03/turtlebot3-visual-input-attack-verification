#!/usr/bin/env python3
"""Summarize retained reset trials without inferring unmeasured collisions."""
import argparse
import json
from pathlib import Path
import statistics


def describe(values):
    values = [v for v in values if v is not None]
    return dict(n=len(values), mean=statistics.mean(values) if values else None,
                sample_sd=statistics.stdev(values) if len(values) > 1 else None,
                minimum=min(values) if values else None, maximum=max(values) if values else None)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('folder')
    args = parser.parse_args()
    folder = Path(args.folder)
    runs = json.loads((folder/'runs.json').read_text())
    groups = {}
    for name in dict.fromkeys(r['condition'] for r in runs):
        rows = [r for r in runs if r['condition'] == name]
        groups[name] = dict(
            total=len(rows), expected_outcome_count=sum(r['passed'] for r in rows),
            final_states={s: sum(r.get('final_state') == s for r in rows)
                          for s in dict.fromkeys(r.get('final_state') for r in rows)},
            lidar_gap_m=describe([r.get('min_lidar_corridor_gap_m') for r in rows]),
            time_to_stop_sim_s=describe([r.get('time_to_stop_sim_s') for r in rows]),
            time_to_stop_wall_s=describe([r.get('time_to_stop_wall_s') for r in rows]),
            endpoint_overshoot_m=(describe([r['final_distance_m']-4.0 for r in rows
                                           if r.get('final_distance_m') is not None])
                                   if name == 'empty-clean' else None))
    result = dict(groups=groups, collision_rate=None,
                  notes=['All retained runs included, including failures.',
                         'Gap statistics are LiDAR-derived, not ground truth.',
                         'Simulation and wall durations are reported separately.',
                         'Expected outcome counts are not collision-free success counts.'])
    (folder/'statistics.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
