"""Read one run's CSV; does not declare collision avoidance or success."""
import argparse
import csv
import json
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument('run_dir', type=Path)
args = parser.parse_args()
with (args.run_dir / 'events.csv').open(newline='', encoding='utf-8') as stream:
    rows = list(csv.DictReader(stream))
if not rows:
    raise SystemExit('No samples recorded')
states = list(dict.fromkeys(row['state'] for row in rows))
result = {
    'samples': len(rows), 'states_seen': states, 'last_state': rows[-1]['state'],
    'final_forward_displacement_m': float(rows[-1]['distance_m']),
    'maximum_visual_area_ratio': max(float(row['red_area_ratio']) for row in rows),
    'last_measured_forward_velocity_m_s': float(rows[-1]['odom_v']),
    'note': 'A stop command is not proof of physical stopping or safe clearance.',
}
print(json.dumps(result, indent=2))
