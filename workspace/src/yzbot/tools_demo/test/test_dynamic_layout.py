"""Physical sweeps and protected precision docks for the central carts."""
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import yaml
from types import SimpleNamespace

from tools_demo.navigation_math import triangle_wave
from tools_demo.dynamic_motion import PolylinePatrol
from tools_demo.task_navigator import TaskNavigator
from tools_demo.task_navigator import (
    CENTRAL_MOVER_GATE_WEST, CENTRAL_MOVER_GATE_EAST,
)
from tools_demo.navigation_math import ZonePose
from test_navigation_contract import (
    _compose_planar, _planar_pose, _point_to_oriented_box,
)

REPO = Path(__file__).resolve().parents[2]
ROOT = ET.parse(REPO/'mybot_description/worlds/offic_room.world').getroot()
CONFIG = yaml.safe_load((REPO/'bot_navigation/config/competition_zones.yaml').read_text())
NAMES = ('crossing_cart_1', 'crossing_cart_2')


def position(name, elapsed):
    p = CONFIG[name]['ros__parameters']
    if 'patrol_points' in p:
        return PolylinePatrol(p['patrol_points'],p['period_sec']).position(
            elapsed+p['phase_offset_sec'])
    wave = triangle_wave(elapsed+p['phase_offset_sec'], p['period_sec'])
    return p['center_x']+p['amplitude_x']*wave, p['center_y']+p['amplitude_y']*wave


def boxes():
    result = []
    for model in ROOT.findall('./world/model'):
        for link in model.findall('link'):
            for collision in link.findall('collision'):
                text = collision.findtext('geometry/box/size')
                if text is None:
                    continue
                sx, sy, sz = map(float, text.split())
                pose = _compose_planar(_compose_planar(_planar_pose(model),
                                                       _planar_pose(link)),
                                       _planar_pose(collision))
                if pose[2]-sz/2 > .5 or pose[2]+sz/2 <= 0:
                    continue
                result.append((model.attrib['name'],
                               (pose[0], pose[1], sx/2, sy/2, pose[5])))
    return result


@pytest.mark.parametrize('name', NAMES)
def test_entire_physical_sweep_clears_walls_trays_goods_and_zones(name):
    model = ROOT.find(f'./world/model[@name="{name}"]')
    radius = float(model.findtext('link/collision/geometry/cylinder/radius'))
    assert .12 <= radius <= .18
    p = CONFIG[name]['ros__parameters']
    physical = boxes()
    for i in range(1001):
        x, y = position(name, p['period_sec']*i/1000)
        assert min(_point_to_oriented_box(x,y,box)-radius for _, box in physical) >= .20
        # No new geometry in the user's protected spawn-to-origin segment.
        assert y+radius < -1.8


def test_carts_cannot_collide_with_each_other_at_any_independent_phase():
    def segment(name):
        p = CONFIG[name]['ros__parameters']
        points = list(zip(p['patrol_points'][::2],p['patrol_points'][1::2]))
        return list(zip(points,points[1:]))
    def vector(a,b): return b[0]-a[0], b[1]-a[1]
    def cross(a,b): return a[0]*b[1]-a[1]*b[0]
    def distance(point, segment):
        a,b = segment
        v, offset = vector(a,b), vector(a,point)
        t = max(0,min(1,(offset[0]*v[0]+offset[1]*v[1])/(v[0]**2+v[1]**2)))
        return math.hypot(point[0]-a[0]-t*v[0],point[1]-a[1]-t*v[1])
    for a in segment(NAMES[0]):
        for b in segment(NAMES[1]):
            v,w,offset = vector(*a),vector(*b),vector(a[0],b[0])
            denominator = cross(v,w)
            if abs(denominator) > 1e-8:
                assert not (0 <= cross(offset,w)/denominator <= 1 and
                            0 <= cross(offset,v)/denominator <= 1)
            radii = sum(float(ROOT.findtext(
                f'./world/model[@name="{name}"]/link/collision/geometry/cylinder/radius'
            )) for name in NAMES)
            assert min([distance(p,b) for p in a]+[distance(p,a) for p in b])-radii >= .05


@pytest.mark.parametrize('name', NAMES)
def test_precision_entry_to_dock_stays_safe_for_entire_sweep(name):
    p = CONFIG['competition_task_navigator']['ros__parameters']
    radius = float(ROOT.findtext(
        f'./world/model[@name="{name}"]/link/collision/geometry/cylinder/radius'))
    for entry in p['cube_approach_allowlist']:
        cube, side = entry.split(':')
        if side == 'east' and cube.startswith('blue'):
            start, end = p[cube+'_entry_inner'], p[cube+'_dock_inner']
        elif side == 'east':
            start = [p[cube+'_entry'][0]+1.8, p[cube+'_entry'][1]]
            end = [p[cube][0]+.72, p[cube][1]]
        else:
            start, end = p[cube+'_entry'], p[cube]
        for i in range(201):
            x, y = position(name, CONFIG[name]['ros__parameters']['period_sec']*i/200)
            for j in range(21):
                bx = start[0]+j/20*(end[0]-start[0])
                by = start[1]+j/20*(end[1]-start[1])
                assert math.hypot(bx-x, by-y)-.28-radius >= .10, (name,cube,side,x,y)


def test_new_motion_is_bounded_and_initial_world_pose_matches_phase():
    for name in NAMES:
        p = CONFIG[name]['ros__parameters']
        maximum_speed = PolylinePatrol(p['patrol_points'],p['period_sec']).maximum_speed
        assert maximum_speed <= .5
        assert maximum_speed/p['update_rate_hz'] <= .05
        initial = _planar_pose(ROOT.find(f'./world/model[@name="{name}"]'))
        assert initial[:2] == pytest.approx(position(name, 0), abs=1e-9)


def test_retreat_patrols_match_physical_motion_not_an_obsolete_lane():
    from tools_demo.task_navigator import TaskNavigator
    import inspect
    source = inspect.getsource(TaskNavigator.__init__)
    for index, name in enumerate(NAMES, 1):
        points = CONFIG[name]['ros__parameters']['patrol_points']
        assert f'"central_patrol_{index}", {points}' in source


@pytest.mark.parametrize('cube,expected', [
    ('RED_CUBE_2', ['CENTRAL_MOVER_GATE_WEST', 'CENTRAL_MOVER_GATE_EAST']),
    ('BLUE_CUBE_3', ['central_patrol_north_flank', 'central_patrol_south_flank']),
])
def test_central_pickups_retain_their_physical_clearance_guides(cube, expected):
    node = SimpleNamespace(use_legacy_layout_routes=False,
                           get_logger=lambda: SimpleNamespace(info=lambda _: None),
                           dock_pose=ZonePose(cube, 2., -5., 0.))
    waypoints = TaskNavigator._cube_approach_waypoints(node, cube, 0., 0.)
    assert [point.zone for point in waypoints[-2:]] == expected


def test_central_gate_itself_clears_all_static_geometry():
    physical = boxes()
    west, east = CENTRAL_MOVER_GATE_WEST, CENTRAL_MOVER_GATE_EAST
    for index in range(201):
        x = west.x + (east.x-west.x)*index/200
        assert min(_point_to_oriented_box(x, west.y, box)-.28
                   for _, box in physical) >= .10


def test_central_waiting_anchors_clear_patrol_even_with_arrival_error():
    """A 20 cm Nav2 arrival error cannot park a waiting base in the sweep."""
    from tools_demo.dynamic_retreat import distance_to_patrol

    for gate in (CENTRAL_MOVER_GATE_WEST, CENTRAL_MOVER_GATE_EAST):
        for name in NAMES:
            points = CONFIG[name]['ros__parameters']['patrol_points']
            assert distance_to_patrol((gate.x, gate.y), points) - .20 \
                - .28 - .14 >= .10


def test_central_gate_is_inside_encounter_but_outside_collision_envelope():
    config = CONFIG['competition_avoidance_monitor']['ros__parameters']
    encounter = config['encounter_distance']
    safety_center_distance = .28 + .14 + config['safety_margin']
    # The old y=-4.55 line was inside the unchanged final guard/planning
    # envelope at the closest cart phase. The new line is outside it;
    # encounters still require actual measured approach/yield, not a claim
    # that every far-end phase necessarily lies inside the scoring radius.
    for name in NAMES:
        patrol = CONFIG[name]['ros__parameters']['patrol_points']
        distances = [abs(CENTRAL_MOVER_GATE_WEST.y-y) for y in patrol[1::2]]
        assert safety_center_distance < min(distances)
        assert min(distances) > .80
        assert min(distances) < encounter


def test_relocated_carts_clear_the_unchanged_hard_prediction_with_arrival_error():
    # A path physically clear at the old line still met the local hard body:
    # 0.14 actor + 0.35 planning air + 0.07 max accepted fit uncertainty +
    # 0.28 robot inflation. Move the actor, NEVER shrink that envelope.
    from tools_demo.dynamic_retreat import distance_to_patrol
    import ast
    import inspect
    import textwrap
    source = inspect.getsource(TaskNavigator.__init__)
    declarations = {node.args[0].value: ast.literal_eval(node.args[1])
                    for node in ast.walk(ast.parse(textwrap.dedent(source)))
                    if isinstance(node, ast.Call)
                    and isinstance(node.func, ast.Attribute)
                    and node.func.attr == 'declare_parameter'
                    and len(node.args) > 1
                    and isinstance(node.args[0], ast.Constant)
                    and str(node.args[0].value).startswith('central_patrol_')}
    for i, name in enumerate(NAMES, 1):
        patrol = CONFIG[name]['ros__parameters']['patrol_points']
        assert declarations[f'central_patrol_{i}'] == patrol
        for gate in (CENTRAL_MOVER_GATE_WEST, CENTRAL_MOVER_GATE_EAST):
            assert distance_to_patrol((gate.x, gate.y), patrol)-.20 \
                >= .14+.35+.07+.28+.05


def test_a_cargo_west_staging_clears_static_and_entire_patrol():
    """The stage-to-A leg must clear all cart phases and map geometry."""
    from tools_demo.dynamic_retreat import distance_to_patrol

    physical = boxes()
    for i in range(201):
        ratio = i/200
        point = (-1.2-.9*ratio, -4.55+4.25*ratio)
        # Include the ordinary 0.20 m goal/tracking allowance as well as
        # the robot radius and scored margin, rather than checking endpoints.
        assert min(_point_to_oriented_box(*point, box)-.28-.20
                   for _, box in physical) >= .10
        for name in NAMES:
            points = CONFIG[name]['ros__parameters']['patrol_points']
            assert distance_to_patrol(point, points)-.28-.14-.20 >= .10


def test_corridor_geometry_and_forecast_exclusion_are_preserved():
    from tools_demo.navigation_math import CORRIDOR_PATROL_PERIOD
    exclusion = CONFIG['competition_dynamic_obstacles']['ros__parameters'][
        'prediction_exclusion_bounds']
    for name, expected_x, phase in [('competition_moving_obstacle_1',8.5,0),
                                     ('competition_moving_obstacle_2',5.,CORRIDOR_PATROL_PERIOD/2)]:
        p = CONFIG[name]['ros__parameters']
        assert (p['center_x'],p['center_y'],p['amplitude_x'],p['amplitude_y'],
                p['period_sec'],p['phase_offset_sec']) == (
                    expected_x,-.10,0,.70,CORRIDOR_PATROL_PERIOD,phase)
        assert exclusion[0] <= expected_x <= exclusion[2]
        assert exclusion[1] <= -.8 <= .6 <= exclusion[3]


def test_only_new_central_carts_enable_physical_proximity_guard():
    for name in NAMES:
        parameters = CONFIG[name]['ros__parameters']
        # Backstop only: stop 0.03 m before physical overlap (0.28 + 0.14 m
        # radii). The old 0.75/0.95 froze the cart while the base was still
        # 0.75 m away, so the crossing window the navigator waits for never
        # opened and a round timed out on its first red leg (20261014).
        assert parameters['robot_clearance_stop_distance'] == .45
        assert parameters['robot_clearance_release_distance'] == .55
    for name in ('competition_moving_obstacle_1',
                 'competition_moving_obstacle_2'):
        parameters = CONFIG[name]['ros__parameters']
        assert 'robot_clearance_stop_distance' not in parameters
        assert 'robot_clearance_release_distance' not in parameters


def test_every_cube_tray_clears_the_docked_chassis():
    """Every cube sits on its own 0.24 x 0.24 x 0.10 plinth centred under it.

    At the 0.36 m standoff the docked chassis front face is exactly 80 mm from
    that plinth edge for every cube, so any forward creep or overshoot at the
    dock touches the tray: 20261013T231641 hit
    competition_blue_pickup_tray_5 that way. The retreat poses stay 0.78 m
    clear. A layout or standoff change that tightens this must fail here.
    """
    import xml.etree.ElementTree as ElementTree

    layout = yaml.safe_load(
        (REPO / 'config' / 'final_mission_layout.yaml').read_text())
    half = layout['geometry']['chassis_half_length']
    world = ElementTree.parse(
        REPO / 'mybot_description' / 'worlds' / 'offic_room.world')
    trays = []
    for model in world.getroot().iter('model'):
        name = model.get('name') or ''
        if 'pickup_tray' not in name:
            continue
        pose = [float(v) for v in (model.findtext('pose') or '').split()]
        box = model.find('.//box/size')
        size = [float(v) for v in box.text.split()]
        trays.append((pose[0], pose[1], size[0] / 2.0, size[1] / 2.0, pose[5]))
    assert len(trays) == len(layout['pickup_objects']) == 10

    def clearance(px, py):
        best = float('inf')
        for cx, cy, hx, hy, yaw in trays:
            cos_y, sin_y = math.cos(yaw), math.sin(yaw)
            dx, dy = px - cx, py - cy
            lx = cos_y * dx + sin_y * dy
            ly = -sin_y * dx + cos_y * dy
            ox = max(abs(lx) - hx, 0.0)
            oy = max(abs(ly) - hy, 0.0)
            best = min(best, math.hypot(ox, oy))
        return best

    worst = None
    for obj in layout['pickup_objects']:
        ax, ay, _ = obj['pickup_approach']
        yaw = math.atan2(obj['y'] - ay, obj['x'] - ax)
        front = (ax + half * math.cos(yaw), ay + half * math.sin(yaw))
        gap = clearance(*front)
        assert gap >= 0.06, (obj['id'], gap)
        assert clearance(obj['retreat_pose'][0],
                         obj['retreat_pose'][1]) >= 0.5, obj['id']
        if worst is None or gap < worst[0]:
            worst = (gap, obj['id'])
    # The uniform 0.36 m standoff leaves 80 mm; keep that as the design value.
    assert abs(worst[0] - 0.08) < 1e-6, worst


def test_a_retry_beside_the_dock_skips_the_mover_gates():
    """The corridor crossing is earned once; a retry must not re-run it.

    Evidence 20261014T123209: the second red_cube_2 attempt started 0.42 m from
    the dock, still drove back to x=-0.20 through both movers, and aborted with
    "Bounded recovery rejected: no clear self-clearance escape within 1.50m".
    """
    node = SimpleNamespace(
        use_legacy_layout_routes=False,
        dock_pose=ZonePose('RED_CUBE_2', 2.0, -5.0, 0.0),
    )
    far = TaskNavigator._cube_approach_waypoints(node, 'RED_CUBE_2', 0.0, 0.0)
    assert [point.zone for point in far[-2:]] == [
        'CENTRAL_MOVER_GATE_WEST', 'CENTRAL_MOVER_GATE_EAST']

    near = TaskNavigator._cube_approach_waypoints(
        node, 'RED_CUBE_2', 2.3, -5.1)
    assert 'CENTRAL_MOVER_GATE_WEST' not in [point.zone for point in near]


def test_runtime_config_keeps_the_dock_gates_that_code_defaults_use():
    """The deployed YAML overrides the code default, so both must agree.

    A 0.18 rad value here silently defeated a 0.30 rad code default and
    red_cube_2 kept failing at 0.23 rad (20261014T051247).
    """
    params = CONFIG['competition_task_navigator']['ros__parameters']
    assert params['dock_pickup_yaw_realign_limit'] == .30
    # The fail line lives in code only; keep the pair documented together.
    source = (
        REPO / 'tools_demo' / 'tools_demo' / 'task_navigator.py'
    ).read_text(encoding='utf-8')
    assert 'declare_parameter("dock_pickup_lateral_fail", 0.15)' in source
    for name in NAMES:
        guard = CONFIG[name]['ros__parameters']
        assert guard['robot_clearance_stop_distance'] == .45
        assert guard['robot_clearance_release_distance'] == .55
