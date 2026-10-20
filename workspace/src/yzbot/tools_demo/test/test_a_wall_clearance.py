"""Keep A deliveries clear of the recorded five-blue wall-tip impact."""

import math
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from tools_demo.navigation_math import a_wall_tip_waypoints, ZonePose
from tools_demo.navigation_math import NavigationValueError
from tools_demo.task_navigator import TaskNavigator


@pytest.mark.parametrize('start', [(-10.1, -5.6), (-.9, -8.8), (11.4, -13)])
def test_southern_deliveries_clear_the_actual_wall_tip(start):
    """West and east pickups must approach from the safe east column."""
    route = a_wall_tip_waypoints(start, (-5, 3.8))
    south, north = route[-2:]
    for index in range(101):
        y = south.y + (north.y - south.y) * index / 100
        distance = math.hypot(south.x - (-2.85729),
                              max(0, abs(y - .89644) - .075))
        assert distance - .28 > .10


def test_existing_north_and_other_zone_routes_are_unchanged():
    """Do not add detours to northern pickups or different layouts."""
    assert not a_wall_tip_waypoints((7, 3.6), (-5, 3.8))
    assert not a_wall_tip_waypoints((-1, -6), (11.3, -5))
    assert not a_wall_tip_waypoints((-1, -6), (2.9, -12.1))
    assert not a_wall_tip_waypoints((-1, -6), (-12, 3.8))
    assert len(a_wall_tip_waypoints((-2.1, .5), (-5, 3.8))) == 1
    with pytest.raises(NavigationValueError):
        a_wall_tip_waypoints((float('nan'), 0), (-5, 3.8))


def test_navigator_inserts_guides_before_configured_final_dock():
    """The active nonlegacy route must actually use the geometry helper."""
    place = ZonePose('A', -5, 3.8, math.pi)
    node = SimpleNamespace(
        _route_pose=lambda zone, key: place,
        _delivery_dock_pose=lambda zone, p: p,
        _route_waypoints=lambda zone, key: [], zones={'A': place},
        dock_arrival_zone=.04, dock_speed_zone=.25,
        use_legacy_layout_routes=False, latest_x=-10.1, latest_y=-5.6,
        get_logger=lambda: Mock())
    TaskNavigator._configure_delivery_route(node, 'A')
    assert [p.zone for p in node.pending_poses] == [
        'A_WALL_WEST_CLEAR', 'A_WALL_SOUTH_TIP_CLEAR',
        'A_WALL_SOUTH_CLEAR', 'A_WALL_NORTH_CLEAR', 'A']
    assert node.dock_pose == place


@pytest.mark.parametrize('start,uses_central', [
    ((.4, -8.8), True), ((.9, -5.4), True), ((1.9, -5.4), True), ((-1.1, -6.0), True),
    ((-10.1, -5.6), False), ((11.4, -13), False), ((-.6, -3.7), False),
])
def test_only_south_central_a_deliveries_stage_outside_the_patrol(
        start, uses_central):
    """Prevent cargo from stopping beside a cart without changing other legs."""
    place = ZonePose('A', -5, 3.8, math.pi)
    node = SimpleNamespace(
        _route_pose=lambda zone, key: place,
        _delivery_dock_pose=lambda zone, p: p,
        _route_waypoints=lambda zone, key: [], zones={'A': place},
        dock_arrival_zone=.04, dock_speed_zone=.25,
        use_legacy_layout_routes=False, latest_x=start[0], latest_y=start[1],
        get_logger=lambda: Mock())
    TaskNavigator._configure_delivery_route(node, 'A')
    names = [p.zone for p in node.pending_poses]
    assert ('A_WALL_CENTRAL_CLEAR' in names) is uses_central
    if uses_central:
        offset = 1 if start == (1.9, -5.4) else 0
        if offset:
            assert names[0] == 'A_WALL_RED2_NORTH_CLEAR'
            assert (node.pending_poses[0].x, node.pending_poses[0].y) == (1.9, -4.7)
        assert names[offset:offset+2] == ['A_WALL_CENTRAL_CLEAR', 'A_WALL_SOUTH_CLEAR']
        assert (node.pending_poses[offset].x, node.pending_poses[offset].y) == (-1.2, -4.55)
    assert names[-2:] == ['A_WALL_NORTH_CLEAR', 'A']


def test_corner_speed_is_released_and_handoff_cannot_cut_the_tip():
    """A route stage changes speed without leaking into later goals."""
    published = []
    node = SimpleNamespace(
        stage_speed_limit_publisher=SimpleNamespace(publish=published.append),
        get_logger=lambda: Mock(), active_route_zone='A',
        active_return_zone=None, pending_poses=[ZonePose('A', -5, 3.8, 0)],
        nav_goal_pose=ZonePose('A_WALL_NORTH_CLEAR', -2.1, 1.85, 0),
        nav_handoff_distance=.20, use_legacy_layout_routes=False,
        route_profiles={})
    assert TaskNavigator._route_handoff_distance(node) == .08
    for stage in ['A_WALL_SOUTH_CLEAR', 'A_WALL_NORTH_CLEAR', 'A', None]:
        TaskNavigator._publish_stage_speed_limit(
            node, ZonePose(stage, 0, 0, 0) if stage else None)
    # Corner reserve raised to 0.75 with the 20261014 matrix (0.6 left the
    # heaviest rounds within seconds of the 300 s limit).
    assert [m.speed_limit for m in published] == [1.2, .75, 0., 0.]


def test_a_guides_preserve_the_existing_lower_east_bay_exit():
    """Adding an A corner must not remove the prior safe bay corridor."""
    place = ZonePose('A', -5, 3.8, math.pi)
    node = SimpleNamespace(
        _route_pose=lambda zone, key: place,
        _delivery_dock_pose=lambda zone, p: p,
        _route_waypoints=lambda zone, key: [], zones={'A': place},
        dock_arrival_zone=.04, dock_speed_zone=.25,
        use_legacy_layout_routes=False, latest_x=11.4, latest_y=-13.,
        get_logger=lambda: Mock())
    TaskNavigator._configure_delivery_route(node, 'A')
    assert [p.zone for p in node.pending_poses] == [
        'lower_east_bay_gate', 'lower_east_lane',
        'A_WALL_SOUTH_CLEAR', 'A_WALL_NORTH_CLEAR', 'A']


def test_wall_stages_cannot_batch_away_their_speed_and_arrival_limits():
    """NavigateThroughPoses resets stage speed, so wall stages stay serial."""
    route = list(a_wall_tip_waypoints((-10, -5.6), (-5, 3.8)))
    route.append(ZonePose('A', -5, 3.8, math.pi))
    node = SimpleNamespace(
        _wait_for_action_server=lambda: True, nav_batch_attempted=False,
        pending_poses=route, stage_index=0, _send_pose=Mock(),
        _send_pose_batch=Mock())
    for name in [p.zone for p in route]:
        TaskNavigator._send_next_goal(node)
        assert node._send_pose.call_args.args[0].zone == name
    node._send_pose_batch.assert_not_called()


def test_west_carry_route_clears_the_wall_for_wrist_and_camera():
    """The recorded arm-first impact needs more than the base radius."""
    route = a_wall_tip_waypoints((-10.1, -5.6), (-5, 3.8))
    assert [p.zone for p in route[:2]] == [
        'A_WALL_WEST_CLEAR', 'A_WALL_SOUTH_TIP_CLEAR']
    for index in range(101):
        x = route[0].x + (route[1].x - route[0].x) * index / 100
        distance = math.hypot(max(0, abs(x + 3.26216) - .075),
                              max(0, abs(route[0].y + 4.85553) - 1.5))
        assert distance - .60 > .10
