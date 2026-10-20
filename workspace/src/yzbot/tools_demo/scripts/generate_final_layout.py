"""
Generate config/final_mission_layout.yaml from the authoritative world.

The world file is the single source of truth for entity positions. Deriving
the layout config from it means the config can never silently disagree with
what Gazebo actually spawns.
"""
import math
import pathlib
import re
import xml.etree.ElementTree as ElementTree

import yaml

REPO = pathlib.Path('.')
WORLD = REPO / 'mybot_description' / 'worlds' / 'offic_room.world'
zones_text = (REPO / 'bot_navigation/config/competition_zones.yaml').read_text(
    encoding='utf-8'
)
params = yaml.safe_load(zones_text)['competition_task_navigator']['ros__parameters']

# Geometry constants, each justified by a measurement.
CUBE_DOCK_STANDOFF = 0.36   # arm reach window [0.285, 0.430] midpoint
CUBE_ENTRY_OFFSET = 0.54    # validated Nav2-to-dock staging distance
CHASSIS_HALF = 0.16         # 0.320 m square base, from large_base.xacro
INFLATION_RADIUS = 0.45     # global/local costmap, originbot_nav2.yaml
ZONE_STANDOFF = 0.40        # base stops this far from a zone centre
ZONE_HALF_EXTENT = (0.485, 0.235)  # keep cube centre inside 1.0 x 0.5 zone

root = ElementTree.parse(WORLD).getroot()
models = {}
for model in root.findall('./world/model'):
    name = model.attrib.get('name')
    if name:
        models.setdefault(name, model)


def pose_of(model):
    text = model.findtext('pose')
    values = [float(v) for v in text.split()] if text else []
    return (values + [0.0] * 6)[:6]


def box_size(model):
    text = model.findtext('.//collision/geometry/box/size')
    return [float(v) for v in text.split()] if text else None


lines = []
add = lines.append
add('# Authoritative layout for the 2026 distributed scene.')
add('#')
add('# GENERATED from mybot_description/worlds/offic_room.world by')
add('# tools_demo/scripts/generate_final_layout.py. Do not hand-edit: rerun the')
add('# generator after changing the world, then run the consistency test')
add('# (tools_demo/test/test_layout_config.py) to prove every consumer agrees.')
add('#')
add('# Synchronized consumer values are checked against this generated file:')
add('#   - tools_demo/config/mission.yaml          (cube docks, zone nav)')
add('#   - bot_navigation/config/competition_zones.yaml (docks, entries, zones)')
add('#   - tools_demo/tools_demo/mission_coordinator.py (CUBE_WORLD_SLOTS)')
add('#   - tools_demo/tools_demo/task_navigator.py      (dock/entry defaults)')
add('#   - tools_demo/tools_demo/competition_dashboard.py (panel layout)')
add('')
add('frame_config:')
add('  mission_frame: map')
add('  robot_frame: base_link')
add('  world_frame: world')
add('')

# Robot start comes from the Gazebo launch file, not the world.
launch = (REPO / 'mybot' / 'launch' / 'gazebo_world.launch.py').read_text(
    encoding='utf-8'
)


def launch_default(key):
    pattern = (
        r"'" + re.escape(key) + r"',\s*default_value='([-0-9.eE]+)'"
    )
    match = re.search(pattern, launch)
    if match is None:
        raise SystemExit('missing launch arg ' + key)
    return float(match.group(1))


add('robot_start:')
add('  x: %.4f' % launch_default('robot_x'))
add('  y: %.4f' % launch_default('robot_y'))
add('  yaw: %.4f' % launch_default('robot_yaw'))
add('')

add('geometry:')
add('  # Distance from base_link to the cube centre at a per-cube dock.')
add('  cube_dock_standoff: %.2f' % CUBE_DOCK_STANDOFF)
add('  cube_entry_offset: %.2f' % CUBE_ENTRY_OFFSET)
add('  chassis_half_length: %.2f' % CHASSIS_HALF)
add('  costmap_inflation_radius: %.2f' % INFLATION_RADIUS)
add('  zone_standoff: %.2f' % ZONE_STANDOFF)
add('')

# Cubes and their trays.
tray_by_cube = {}
for color in ('red', 'blue'):
    for number in range(1, 6):
        suffix = '' if number == 1 else '_%d' % number
        tray_by_cube['%s_cube_%d' % (color, number)] = (
            'competition_%s_pickup_tray%s' % (color, suffix)
        )

add('pickup_objects:')
cube_records = []
for color in ('red', 'blue'):
    for number in range(1, 6):
        cube_name = '%s_cube_%d' % (color, number)
        cube = models[cube_name]
        cpose = pose_of(cube)
        tray = models[tray_by_cube[cube_name]]
        tpose = pose_of(tray)
        tsize = box_size(tray)
        dock_x, dock_y, dock_yaw = params[cube_name]
        dx, dy = cpose[0] - dock_x, cpose[1] - dock_y
        forward = dx * math.cos(dock_yaw) + dy * math.sin(dock_yaw)
        lateral = -dx * math.sin(dock_yaw) + dy * math.cos(dock_yaw)
        if not (math.isclose(forward, CUBE_DOCK_STANDOFF, abs_tol=1e-4)
                and abs(lateral) <= 1e-4):
            raise SystemExit('deployed dock does not face the cube: ' + cube_name)
        cube_records.append(
            {
                'id': cube_name,
                'color': color,
                'x': cpose[0],
                'y': cpose[1],
                'z': cpose[2],
                'dock_x': dock_x,
                'dock_y': dock_y,
                'dock_yaw': dock_yaw,
                'tray': tray_by_cube[cube_name],
                'tray_pose': tpose,
                'tray_size': tsize,
            }
        )

# Entries: pick the nearest gap along the dock's X line that clears every
# static box by more than the inflation radius. Search outward from the cube
# so the final direct-dock leg stays as short as possible.
tray_boxes = []
for rec in cube_records:
    tpose = rec['tray_pose']
    tsize = rec['tray_size']
    tray_boxes.append(
        (tpose[0], tpose[1], tsize[0] / 2.0, tsize[1] / 2.0, tpose[5])
    )


def clearance(px, py):
    best = float('inf')
    for cx, cy, hx, hy, yaw in tray_boxes:
        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)
        dx = px - cx
        dy = py - cy
        lx = cos_y * dx + sin_y * dy
        ly = -sin_y * dx + cos_y * dy
        ox = max(abs(lx) - hx, 0.0)
        oy = max(abs(ly) - hy, 0.0)
        best = min(best, math.hypot(ox, oy))
    return best


# Entries are read from the deployed navigation config so this file always
# describes what actually runs. The generator independently recomputes the
# tightest geometrically valid entry and reports it, so a shorter final
# direct-dock leg stays visible as an optimization without silently changing
# validated behaviour.
def deployed_entry(cube_id, suffix='entry'):
    key = '%s_%s:' % (cube_id, suffix)
    for line in zones_text.splitlines():
        stripped = line.strip()
        if stripped.startswith(key):
            body = stripped.split('[', 1)[1].rsplit(']', 1)[0]
            numbers = [float(v) for v in body.split(',')]
            return numbers[0], numbers[1]
    raise SystemExit(
        'missing deployed %s for %s' % (suffix, cube_id)
    )


def tightest_entry(dock_x, dock_y):
    for step in range(1, 40):
        offset = step * 0.05
        for candidate in (dock_y + offset, dock_y - offset):
            if clearance(dock_x, candidate) > INFLATION_RADIUS + 0.10:
                return round(candidate, 2)
    return None


for rec in cube_records:
    entry_x, entry_y = deployed_entry(rec['id'])
    # An entry may sit on the dock's X line (aisle-gap approach), or behind
    # it along the configured heading for a straight approach from either
    # side. Clearance is evaluated at the actual entry pose.
    same_x_entry = abs(entry_x - rec['dock_x']) <= 1e-6
    dx, dy = rec['dock_x'] - entry_x, rec['dock_y'] - entry_y
    forward = dx * math.cos(rec['dock_yaw']) + dy * math.sin(rec['dock_yaw'])
    lateral = -dx * math.sin(rec['dock_yaw']) + dy * math.cos(rec['dock_yaw'])
    straight_entry = (
        0.30 <= forward <= 0.60
        and abs(lateral) <= 1e-6
    )
    if not (same_x_entry or straight_entry):
        raise SystemExit(
            'deployed entry X disagrees with dock X for ' + rec['id']
        )
    got = clearance(entry_x, entry_y)
    if got <= INFLATION_RADIUS:
        raise SystemExit(
            'deployed entry for %s clears only %.3f m, needs > %.2f'
            % (rec['id'], got, INFLATION_RADIUS)
        )
    alternate = None
    if rec['color'] == 'red':
        alternate_x, alternate_y = deployed_entry(
            rec['id'], 'entry_aisle'
        )
        if abs(alternate_x - rec['dock_x']) > 1e-6:
            raise SystemExit(
                'aisle entry X disagrees with dock X for ' + rec['id']
            )
        alternate_clearance = clearance(alternate_x, alternate_y)
        if alternate_clearance <= INFLATION_RADIUS:
            raise SystemExit(
                'aisle entry for %s clears only %.3f m, needs > %.2f'
                % (rec['id'], alternate_clearance, INFLATION_RADIUS)
            )
        alternate = (alternate_x, alternate_y, alternate_clearance)
        rec['aisle_entry_x'] = alternate_x
        rec['aisle_entry_y'] = alternate_y
    rec['entry_y'] = entry_y
    rec['entry_x'] = entry_x
    rec['tightest_y'] = tightest_entry(rec['dock_x'], rec['dock_y'])
    add('  - id: %s' % rec['id'])
    add('    color: %s' % rec['color'])
    allow = params.get('cube_approach_allowlist', [])
    if allow:
        sides = [v.split(':', 1)[1] for v in allow if v.startswith(rec['id'] + ':')]
        add('    allowed_sides: %s' % sides)
    add('    x: %.4f' % rec['x'])
    add('    y: %.4f' % rec['y'])
    add('    z: %.4f' % rec['z'])
    add('    tray: %s' % rec['tray'])
    yaw_text = '0.00' if abs(rec['dock_yaw']) < 1e-9 else '%.8f' % rec['dock_yaw']
    if abs(rec['dock_yaw']) < 1e-9:
        add('    # West approach, yaw 0: the cube sits cube_dock_standoff ahead.')
    else:
        add('    # Configured approach: the cube sits cube_dock_standoff ahead.')
    add('    pickup_approach: [%.2f, %.2f, %s]' % (rec['dock_x'], rec['dock_y'], yaw_text))
    add('    # Nav2 goal outside every tray inflation footprint.')
    add('    retreat_pose: [%.2f, %.2f, %s]' % (entry_x, entry_y, yaw_text))
    add('    entry_clearance: %.3f' % got)
    add(
        '    dock_to_entry: %.2f'
        % math.hypot(
            entry_x - rec['dock_x'],
            entry_y - rec['dock_y'],
        )
    )
    if alternate is not None:
        add(
            '    alternate_retreat_pose: [%.2f, %.2f, %s]'
            % (alternate[0], alternate[1], yaw_text)
        )
        add('    alternate_entry_clearance: %.3f' % alternate[2])
        add(
            '    alternate_dock_to_entry: %.2f'
            % math.hypot(
                alternate[0] - rec['dock_x'],
                alternate[1] - rec['dock_y'],
            )
        )
add('')

# Zones. Use the first (non-state) definition of each zone model.
ZONE_COLORS = {'A': 'green', 'B': 'yellow', 'C': 'blue'}
add('zones:')
for letter in ('A', 'B', 'C'):
    zone = models['zone_%s' % letter.lower()]
    zpose = pose_of(zone)
    add('  %s:' % letter)
    add('    zone_id: %s' % letter)
    add('    zone_color: %s' % ZONE_COLORS[letter])
    add('    center: [%.2f, %.2f]' % (zpose[0], zpose[1]))
    add(
        '    half_extent: [%.3f, %.3f]'
        % ZONE_HALF_EXTENT
    )
add('')

add('dynamic_obstacles:')
for key, model_name in (
    ('obstacle_1', 'competition_moving_obstacle_1'),
    ('obstacle_2', 'competition_moving_obstacle_2'),
    ('preview_moving_partition_1', 'preview_moving_partition_1'),
    ('preview_moving_partition_2', 'preview_moving_partition_2'),
):
    if model_name not in models:
        continue  # the world carries only the scored obstacles
    model = models[model_name]
    mpose = pose_of(model)
    radius = model.findtext('.//collision/geometry/cylinder/radius')
    add('  %s:' % key)
    add('    model: %s' % model_name)
    add('    initial: [%.2f, %.2f]' % (mpose[0], mpose[1]))
    if radius:
        add('    radius: %s' % radius)
    else:
        add('    size: %s' % box_size(model))
add('')

(REPO / 'config').mkdir(exist_ok=True)
out = REPO / 'config' / 'final_mission_layout.yaml'
out.write_text('\n'.join(lines) + '\n', encoding='utf-8')
print('wrote %s (%d lines)' % (out, len(lines)))
total_deployed = 0.0
total_tightest = 0.0
for rec in cube_records:
    leg = math.hypot(
        rec['entry_x'] - rec['dock_x'],
        rec['entry_y'] - rec['dock_y'],
    )
    total_deployed += leg
    tight = rec['tightest_y']
    tight_leg = abs(tight - rec['dock_y']) if tight is not None else leg
    total_tightest += tight_leg
    flag = '' if abs(tight_leg - leg) < 1e-6 else '  <- could be %.2f' % tight_leg
    print(
        '  %-12s dock=(%.2f,%.2f) entry=(%.2f,%.2f) leg=%.2f clr=%.3f%s'
        % (
            rec['id'],
            rec['dock_x'],
            rec['dock_y'],
            rec['entry_x'],
            rec['entry_y'],
            leg,
            clearance(rec['entry_x'], rec['entry_y']),
            flag,
        )
    )
print(
    'primary direct-dock legs total %.2f m; tightest aisle total %.2f m'
    % (total_deployed, total_tightest)
)
