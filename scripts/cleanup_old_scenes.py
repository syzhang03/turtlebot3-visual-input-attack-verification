#!/usr/bin/env python3
"""Remove obsolete local scene assets, preserving evidence and dependencies.

Run in Ubuntu: python3 /mnt/hgfs/VM_share/delay-replay-2026-10-30/scripts/cleanup_old_scenes.py
Only known project folders are scanned; no recursive directory deletion is used.
"""
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET


EXPECTED_SCENE_SHA256 = '3a17682115ef1a864b6417b2a02fb39240c4ec65ddfbd845d8ce7c43043950df'


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    if sys.platform != 'linux':
        raise SystemExit('This helper is for the Ubuntu VM only.')
    home = Path.home().resolve()
    project = home / 'delay-replay-2026-10-30'
    current = project / 'worlds' / 'intersection-white-line.sdf'
    # Abort before deleting anything if the current baseline is not intact.
    required = [current, project / 'scripts/start_scene.sh',
                project / 'scripts/start_scene.launch.py',
                project / 'configs/baseline_white_line.yaml',
                project / 'configs/delay_replay.yaml']
    for path in required:
        if not path.is_file() or path.resolve() != path.absolute():
            raise SystemExit(f'Missing file or symlink in current project: {path}')
    if digest(current) != EXPECTED_SCENE_SHA256:
        raise SystemExit('Current scene differs from the agreed baseline; no files deleted.')
    launch_text = (project / 'scripts/start_scene.launch.py').read_text()
    if 'intersection-white-line.sdf' not in launch_text:
        raise SystemExit('Current launcher does not reference the retained scene; stopping.')

    # These are the known development locations from this project, not ROS installs.
    roots = [project / 'worlds', home / 'tb3-visual-baseline/attack_ws/worlds',
             home / 'intersection-prototype', home / 'delay-replay-2026-10-30/intersection-prototype']
    candidates, skipped = [], []
    for root in roots:
        if not root.is_dir():
            continue
        if root.resolve() != root.absolute() or not root.resolve().is_relative_to(home):
            skipped.append(str(root))
            continue
        for path in sorted(root.iterdir()):
            if path == current or not path.is_file() or path.is_symlink():
                continue
            if not (path.name == 'red_obstacle.sdf' or
                    (path.name.startswith('intersection') and path.suffix in ('.sdf', '.world'))):
                continue
            try:
                sdf = ET.parse(path).getroot()
            except (OSError, ET.ParseError):
                skipped.append(str(path))
                continue
            models = sdf.findall('model')
            if len(models) != 1 or models[0].get('name') not in (
                    'boulder_intersection', 'baseline_red_obstacle'):
                skipped.append(str(path))
                continue
            candidates.append(path)

    report = {'retained_scene': str(current), 'scene_sha256': digest(current),
              'planned': [{'path': str(p), 'sha256': digest(p)} for p in candidates],
              'removed': [], 'skipped': skipped,
              'preserved': ['baseline-results', 'replay-results', 'turtlebot3_ws',
                            'all code, configs, Git history and experiment evidence']}
    run_id = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
    report_path = project / f'cleanup-report-{run_id}.json'

    def save_report():
        report_path.write_text(json.dumps(report, indent=2), encoding='utf-8')

    save_report()
    for entry in report['planned']:
        path = Path(entry['path'])
        if path.resolve() != path.absolute() or not path.resolve().is_relative_to(home):
            raise RuntimeError(f'Path changed during cleanup: {path}')
        if digest(path) != entry['sha256']:
            raise RuntimeError(f'File changed during cleanup: {path}')
        path.unlink()
        report['removed'].append(str(path))
        save_report()
        print(f'Removed: {path}')
    assert digest(current) == EXPECTED_SCENE_SHA256
    print(f'Complete: removed {len(report["removed"])} old scene files.')
    print(f'Kept: {current}')
    print('Code, experiment logs/screenshots, ROS and TurtleBot3 dependencies are untouched.')
    print(f'Report: {report_path}')


if __name__ == '__main__':
    main()
