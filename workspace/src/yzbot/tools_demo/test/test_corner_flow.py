"""Verify broad corner geometry, original anchors and ordered recovery."""

import math
from unittest.mock import Mock

import pytest
from nav_msgs.msg import Odometry

from tools_demo.corner_flow import (
    flow_prefix_size, has_corner_flow, round_navigation_corners,
)
from tools_demo.navigation_math import ZonePose, lower_east_corridor_waypoints
from tools_demo.task_navigator import TaskNavigator
from tools_demo.transport_routes import current_delivery_route, current_pickup_guides
from test_navigation_batch_recovery import RouteHarness
from test_navigation_contract import _point_to_oriented_box
from test_north_pickup_clearance import loaded_boxes


def east_route(outgoing=True, lower_bay=False):
    low = (11., -10.) if lower_bay else (11.3, -5.)
    start, end = ((0., 3.8), low) if outgoing else (low, (-5., 3.8))
    raw = list(lower_east_corridor_waypoints(start, end)) + [ZonePose('end', *end, 0.)]
    return start, raw, round_navigation_corners(start, raw, 'B' if outgoing else 'A')


@pytest.mark.parametrize('outgoing,lower_bay', [
    (True, False), (False, False), (True, True), (False, True)])
def test_east_arc_preserves_wall_gate_and_full_loaded_tracking_reserve(outgoing, lower_bay):
    _, raw, result = east_route(outgoing, lower_bay)
    flow = [p for p in result if p.zone.startswith('FLOW_EAST_')]
    gate = next(p for p in raw if p.zone == 'lower_east_bay_gate')
    assert any(p.x == gate.x and p.y == gate.y for p in flow)
    boxes = loaded_boxes()
    for a, b in zip(flow, flow[1:]):
        for i in range(21):
            x, y = a.x+(b.x-a.x)*i/20, a.y+(b.y-a.y)*i/20
            gap = min(_point_to_oriented_box(x, y, box)-.28-.20 for _, box in boxes)
            assert gap >= .10, (x, y, gap)
    assert max(p.x for p in flow) == pytest.approx(14.4)
    assert round_navigation_corners((11.3, -5.), result, 'A') == result


@pytest.mark.parametrize('start', [(-6.6, -2.8), (-.4, .8), (-3.8, -12.4)])
def test_a_return_preserves_north_anchor_and_finishes_before_original_dock(start):
    place = ZonePose('A', -5., 3.747, math.pi)
    raw, _ = current_delivery_route('A', *start, place)
    result = round_navigation_corners(start, raw, 'A')
    assert result[-1] == place
    assert has_corner_flow(result)
    assert any(p.x == -2.1 and p.y == 1.85 for p in result)
    if start == (-3.8, -12.4):
        assert not any(p.zone in ('A_WALL_WEST_CLEAR', 'A_WALL_SOUTH_TIP_CLEAR')
                       for p in result)
    boxes = loaded_boxes()
    index = next(i for i, p in enumerate(result) if p.x == -2.1 and p.y == 1.85)
    for a, b in zip(result[index:-1], result[index+1:-1]):
        for i in range(21):
            x, y = a.x+(b.x-a.x)*i/20, a.y+(b.y-a.y)*i/20
            assert min(_point_to_oriented_box(x, y, box)-.28-.20
                       for _, box in boxes) >= .10


def test_outbound_north_curve_reaches_both_original_poles_before_patrol_flanks():
    start = (-4.55, 3.8)
    entry = ZonePose('entry', -1.1, -6., math.pi)
    raw = current_pickup_guides('BLUE_CUBE_3', *start, entry) + [entry]
    result = round_navigation_corners(start, raw, 'BLUE_CUBE_3')
    count = flow_prefix_size(result)
    assert count > 2
    assert any(p.x == -2.1 and p.y == 1.8 for p in result[:count])
    assert (result[count-1].x, result[count-1].y) == (-2.1, 0.)
    assert [p.zone for p in result[count:]] == [p.zone for p in raw[2:]]


def test_arc_batch_proof_and_fallback_never_replay_visited_arc_samples():
    _, _, result = east_route()
    node = RouteHarness()
    node.pending_poses = result[1:]  # The straight lane was already visited.
    node._send_next_goal()
    batch = list(node.nav_batch_poses)
    assert len(batch) > 10
    node.nav_batch_proven_count = 0
    node._request_nav_handoff_if_reached = Mock()
    message = Odometry()
    message.pose.pose.orientation.w = 1.
    for p in batch[:5]:
        message.pose.pose.position.x, message.pose.pose.position.y = p.x, p.y
        TaskNavigator.odom_callback(node, message)
    assert node.nav_batch_proven_count == 5
    assert not TaskNavigator._central_anchor_arrived(node)
    node._fallback_batch_to_serial('injected_curve_failure')
    assert node.corner_flow_failed
    assert node.sent_serial + node.pending_poses == batch[5:] + result[-1:]


def test_later_corner_can_batch_after_an_earlier_success_but_not_after_failure():
    _, _, result = east_route()
    node = RouteHarness()
    node.nav_batch_attempted = True  # A different corner completed earlier.
    node.pending_poses = result[1:]
    node._send_next_goal()
    assert len(node.sent_batches) == 1
    node._fallback_batch_to_serial('injected_failure')
    before = len(node.sent_batches)
    while node.pending_poses:
        node._send_next_goal()
    assert len(node.sent_batches) == before


def test_narrow_and_unrelated_paths_are_not_rounded():
    raw = [ZonePose('WALL57_POCKET_1', -4.1, -2.8, 0.),
           ZonePose('WALL57_POCKET_2', -4.1, 0., 0.)]
    assert round_navigation_corners((-6.6, -2.8), raw, 'RED_CUBE_1') == raw
    assert flow_prefix_size(raw) == 0


@pytest.mark.parametrize('start,end,zone', [
    ((6.5, 3.6), (11.3, -5.), 'B'), ((10.7, -5.), (6.5, 3.6), 'BLUE_CUBE_1'),
])
def test_upper_right_tray_uses_the_same_east_arc_without_old_lane_guides(start, end, zone):
    dock = ZonePose(zone, *end, 0.)
    result = round_navigation_corners(start, [dock], zone)
    assert result[-1] == dock
    arc = result[1:] if zone == 'B' else result
    assert flow_prefix_size(arc) > 10
    assert max(p.x for p in result[:-1]) == pytest.approx(14.4)
    assert any(p.x == 13.4 and abs(p.y+2.5) < 1e-6 for p in result[:-1])
    assert round_navigation_corners(start, result, zone) == result


def test_upper_right_loaded_approach_clears_the_roof_before_the_east_arc():
    dock = ZonePose('B', 11.3, -5., 0.)
    result = round_navigation_corners((8.3, 3.6), [dock], 'B')
    approach = result[0]
    assert (approach.x, approach.y) == (4.4, -2.5)
    assert flow_prefix_size(result) == 1
    assert flow_prefix_size(result[1:]) > 10
    boxes = loaded_boxes()
    for i in range(101):
        x = approach.x + (result[1].x - approach.x) * i / 100
        assert min(_point_to_oriented_box(x, -2.5, box) - .28 - .20
                   for _, box in boxes) >= .10
    node = RouteHarness()
    node.use_legacy_layout_routes = False
    node.active_zone = 'B'
    node.latest_x, node.latest_y = 8.3, 3.6
    node.pending_poses = list(result)
    node._send_next_goal()
    assert node.sent_serial == [approach]
    assert not node.sent_batches
    node.latest_x, node.latest_y = approach.x, approach.y
    assert TaskNavigator._central_anchor_arrived(node)
    node._send_next_goal()
    assert node.sent_batches == [result[1:-1]]


def test_corner_behavior_trees_reference_registered_collision_safe_plugins():
    # Catch a missing planner before fallback/recovery is first needed live.
    from pathlib import Path
    import xml.etree.ElementTree as ET
    import yaml

    root = Path(__file__).resolve().parents[2]
    params = yaml.safe_load((root/'bot_navigation/param/originbot_nav2.yaml').read_text())
    planners = params['planner_server']['ros__parameters']
    controllers = params['controller_server']['ros__parameters']
    for name in ('navigate_through_corner.xml', 'navigate_corner_to_pose.xml'):
        tree = ET.parse(root/'tools_demo/config'/name)
        for element in tree.iter():
            for key, registry in [('planner_id', planners['planner_plugins']),
                                  ('controller_id', controllers['controller_plugins']),
                                  ('goal_checker_id', controllers['goal_checker_plugins'])]:
                if key in element.attrib:
                    assert element.attrib[key] in registry, (name, element.attrib)
    planner = planners['CornerExact']
    assert planner['tolerance'] == 0.0
    assert planner['cost_travel_multiplier'] == planners['GridBased']['cost_travel_multiplier']
    follower = controllers['CornerFlow']
    assert follower['use_collision_detection']
    assert follower['max_allowed_time_to_collision_up_to_carrot'] >= .65
    assert follower['use_cost_regulated_linear_velocity_scaling']
