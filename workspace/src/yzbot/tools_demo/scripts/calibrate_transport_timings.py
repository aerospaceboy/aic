#!/usr/bin/env python3
"""Export successful navigation timings from controlled, same-layout rounds."""

import argparse
import hashlib
import json
from pathlib import Path
import re


def navigation_samples(text, source):
    """Retain measured starts, route shapes, entities and navigation times."""
    cube, side, destination, current = None, None, None, None
    samples = []
    for line in text.splitlines():
        selected = re.search(
            r'cube_selection_override name=(\w+) side=(\w+)', line)
        if selected:
            cube, side = selected.groups()
        state = re.search(r'\{"color":.*\}', line)
        if state:
            try:
                snapshot = json.loads(state[0])
                destination = snapshot.get('target_zone') or destination
            except ValueError:
                pass
        start = re.search(
            r'nav_start location=(\w+) side=(\w+) '
            r'robot=\(([-\d.]+),([-\d.]+),yaw=([-\d.]+)\)',
            line)
        if start:
            location = start[1]
            if cube and ('_CUBE_' in location or location in 'ABC'):
                current = {
                    'phase': 'pickup' if '_CUBE_' in location else 'delivery',
                    'cube': cube,
                    'side': side,
                    'zone': destination if '_CUBE_' in location else location,
                    'start_pose': [
                        float(v) for v in start.groups()[
                            2:]],
                    'source': source,
                    '_start_stamp': float(
                        re.search(
                            r'\[([0-9]{9,}\.[0-9]+)\]',
                            line)[1])}
            else:
                current = None
        if current is not None and 'nav_plan zone=' in line:
            points = re.findall(r'\(([-\d.]+),([-\d.]+),yaw=([-\d.]+)\)', line)
            current['waypoints_xy'] = [
                [float(x), float(y)] for x, y, _ in points[:-1]]
        if current is not None and 'nav_goal zone=' in line:
            points = current.get('waypoints_xy', ())
            origin = next(
                (i for i, p in enumerate(points) if p == [
                    0., 0.]), None)
            stage = re.search(r'stage=(\d+)/', line)
            if (current['phase'] == 'pickup'
                    and current['start_pose'][0] >= 13.
                    and origin is not None and stage
                    and int(stage[1]) == origin+2
                    and not current.get('_prefix_saved')):
                stamp = float(re.search(r'\[([0-9]{9,}\.[0-9]+)\]', line)[1])
                samples.append({k: v for k, v in {
                    **current, 'phase': 'spawn',
                    'waypoints_xy': points[:origin+1],
                    'elapsed_sec': stamp-current['_start_stamp']}.items()
                    if not k.startswith('_')})
                current['_prefix_saved'] = True
        end = re.search(r'nav_result state=(\w+) elapsed=([\d.]+)s', line)
        if end and current is not None:
            if (end[1] == 'succeeded' and current.get('waypoints_xy') and not (
                    current['phase'] == 'pickup'
                    and current['start_pose'][0] >= 13.)):
                measured = {**current, 'elapsed_sec': float(end[2])}
                samples.append({k: v for k, v in measured.items()
                                if not k.startswith('_')})
            current = None
    return samples


def main():
    """Write a versioned profile without changing runtime or entities."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('logs', nargs='+', type=Path)
    args = parser.parse_args()
    package = Path(__file__).resolve().parents[1]
    files = {
        'navigation_math.py': package / 'tools_demo/navigation_math.py',
        'transport_routes.py': package / 'tools_demo/transport_routes.py',
        'task_navigator.py': package / 'tools_demo/task_navigator.py',
        'competition_zones.yaml': package.parent / 'bot_navigation' / 'config'
        / 'competition_zones.yaml',
        'originbot_nav2.yaml': package.parent / 'bot_navigation' / 'param'
        / 'originbot_nav2.yaml',
        'navigation_risk_zones.yaml': (
            package.parent / 'bot_navigation' / 'config')
        / 'navigation_risk_zones.yaml',
        'mission.yaml': package / 'config/mission.yaml'}
    samples, sources = [], {}
    for path in args.logs:
        text = path.read_text()
        if not re.search(
            r'"completed":5,.*"skipped":0,"state":"completed"',
                text):
            raise ValueError(
                f'Not a complete zero-skip calibration round: {path}')
        sources[str(path)] = hashlib.sha256(path.read_bytes()).hexdigest()
        samples.extend(navigation_samples(text, path.parent.name))
    payload = {
        'schema_version': 1,
        'scope': ('Controlled same-layout simulation navigation timings; '
                  'sparse samples blend with geometric priors.'),
        'geometry_sha256': {key: hashlib.sha256(p.read_bytes()).hexdigest()
                            for key, p in files.items()},
        'source_logs_sha256': sources, 'samples': samples}
    args.output.write_text(
        json.dumps(
            payload,
            ensure_ascii=False,
            indent=2) + '\n')
    print(f'Wrote {len(samples)} navigation measurements to {args.output}')


if __name__ == '__main__':
    main()
