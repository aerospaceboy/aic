"""Synchronize consumers after adopting the confirmed complex world."""

from pathlib import Path
import json
import math
import re
import subprocess
import sys

import yaml


REPO = Path(__file__).resolve().parents[2]


def replace_vector(text, key, values):
    """Replace one flat ROS parameter without losing its surrounding notes."""
    replacement = '[' + ', '.join(str(float(v)) for v in values) + ']'
    result, count = re.subn(
        r'^(\s*' + re.escape(key) + r':\s*)\[[^\n]*?\]',
        lambda match: match.group(1) + replacement, text, flags=re.M)
    if count != 1:
        raise ValueError(f'{key}: expected one parameter, found {count}')
    return result


def replace_dictionary(text, name, values, tuples=False):
    """Synchronize fallback constants with the same world coordinates."""
    left, right = ('(', ')') if tuples else ('[', ']')
    lines = [f'{name} = {{']
    for key, item in values.items():
        body = ', '.join(f'{v:.4f}' for v in item)
        lines.append(f'    "{key}": {left}{body}{right},')
    lines.append('}')
    result, count = re.subn(name + r' = \{.*?\n\}',
                            '\n'.join(lines), text, flags=re.S)
    if count != 1:
        raise ValueError(name)
    return result


def synchronize():
    """Update defaults/configs and regenerate derived artifacts."""
    metadata = json.loads((REPO / 'config/complex_layout.json').read_text())
    positions = metadata['positions']
    nav_file = REPO / 'bot_navigation/config/competition_zones.yaml'
    mission_file = REPO / 'tools_demo/config/mission.yaml'
    nav = nav_file.read_text()
    mission = mission_file.read_text()
    docks, entries, aisles = {}, {}, {}
    for name, (x, y) in positions.items():
        if not name.startswith(('red_cube_', 'blue_cube_')):
            continue
        dock = [round(x - .36, 4), y, 0.0]
        entry = [round(x - .90, 4), y, 0.0]
        docks[name.upper()] = dock
        entries[name.upper()] = entry
        nav = replace_vector(nav, name, dock)
        nav = replace_vector(nav, name + '_entry', entry)
        mission = replace_vector(mission, name + '_dock', dock)
        if name.startswith('red'):
            # Stage below the north wall for high-y resources.
            aisle = [dock[0], round(y + (-.65 if y > 3.5 else .65), 4), 0.0]
            aisles[name.upper()] = aisle
            nav = replace_vector(nav, name + '_entry_aisle', aisle)
        else:
            inner = [round(x + .36, 4), y, math.pi]
            nav = replace_vector(nav, name + '_dock_inner', inner)
            nav = replace_vector(nav, name + '_entry_inner',
                                 [round(x + .90, 4), y, math.pi])
            mission = replace_vector(mission, name + '_dock_inner', inner)
    for letter in 'abc':
        x, y = positions['zone_' + letter]
        pose = ([round(x + .4, 4), y, math.pi] if letter == 'a'
                else [round(x - .4, 4), y, 0.0])
        nav = replace_vector(nav, 'zone_' + letter, pose)
        mission = replace_vector(mission, 'zone_' + letter + '_center', [x, y])
        mission = replace_vector(mission, 'zone_' + letter + '_nav', pose)
    # Keep the old policy table available as a backup, but never use its
    # measured costs or old-layout flight evidence in the new world.
    if 'use_legacy_layout_routes:' not in nav:
        nav = nav.replace('    use_sim_time: true',
                          '    use_sim_time: true\n'
                          '    use_legacy_layout_routes: false', 1)
    if 'use_legacy_selection_costs:' not in mission:
        mission = mission.replace('    use_sim_time: true',
                                  '    use_sim_time: true\n'
                                  '    use_legacy_selection_costs: false\n'
                                  '    cube_order_policy_file: ""', 1)
    nav_file.write_text(nav)
    mission_file.write_text(mission)
    # Add only the new mover instances; the original two patrol blocks remain
    # byte-for-byte intact, including phase and amplitude.
    additions = {}
    topics = []
    boxes = []
    for wall in metadata['new_dynamic_partitions']:
        name = wall['id']
        x, y, z = wall['pose'][:3]
        topic = '/competition/obstacles/' + name
        topics.append(topic)
        boxes.append(':'.join([name] + [str(v) for v in wall['size'][:2]]))
        additions[name] = {'ros__parameters': {
            'use_sim_time': True, 'model_name': name,
            'reference_frame': 'world',
            'center_x': x, 'center_y': y, 'center_z': z,
            'amplitude_x': wall['amplitude_x'], 'amplitude_y': 0.0,
            'period_sec': wall['period_sec'],
            'phase_offset_sec': wall['phase_offset_sec'],
            'motion_mode': 'continuous', 'state_topic': topic,
        }}
    # Modify the monitor region separately, not the original actor configs.
    text = nav_file.read_text()
    prefix, monitor = text.split('competition_avoidance_monitor:', 1)
    if 'box_obstacles:' not in monitor:
        monitor = monitor.replace('    expected_obstacle_count: 2',
                                  '    expected_obstacle_count: 4\n'
                                  '    box_obstacles: ' + json.dumps(boxes))
        for topic in topics:
            monitor = monitor.replace('    state_topic:',
                                      f'      - {topic}\n    state_topic:', 1)
    for name, config in additions.items():
        if '\n' + name + ':' not in prefix + monitor:
            monitor += '\n' + yaml.safe_dump({name: config}, sort_keys=False)
    nav_file.write_text(prefix + 'competition_avoidance_monitor:' + monitor)

    navigator = REPO / 'tools_demo/tools_demo/task_navigator.py'
    text = navigator.read_text()
    for name, values in [('DEFAULT_CUBE_DOCKS', docks),
                         ('DEFAULT_CUBE_ENTRIES', entries),
                         ('DEFAULT_RED_AISLE_ENTRIES', aisles)]:
        text = replace_dictionary(text, name, values)
    for letter in 'abc':
        x, y = positions['zone_' + letter]
        default = ([x + .4, y, math.pi] if letter == 'a'
                   else [x - .4, y, 0.0])
        pattern = (r'(self.declare_parameter\("zone_' + letter +
                   r'", )\[[^\]]+\]')
        text = re.sub(pattern, lambda m: m.group(1) + repr(default), text)
    navigator.write_text(text)
    coordinator = REPO / 'tools_demo/tools_demo/mission_coordinator.py'
    text = replace_dictionary(coordinator.read_text(), 'CUBE_WORLD_SLOTS',
                              {k: v for k, v in positions.items()
                               if 'cube' in k}, True)
    for letter in 'abc':
        x, y = positions['zone_' + letter]
        default = ([x + .4, y, math.pi] if letter == 'a'
                   else [x - .4, y, 0.0])
        for suffix, values in [('center', [x, y]), ('nav', default)]:
            pattern = (r'(self.declare_parameter\("zone_' + letter +
                       '_' + suffix + r'", )\[[^\]]+\]')
            text = re.sub(pattern, lambda m: m.group(1) + repr(values), text)
    coordinator.write_text(text)
    for script in ['generate_final_layout.py', 'generate_static_map.py',
                   'generate_navigation_risk_zones.py']:
        command = [sys.executable, str(REPO / 'tools_demo/scripts' / script)]
        subprocess.run(command,
                       cwd=REPO, check=True)


if __name__ == '__main__':
    synchronize()
