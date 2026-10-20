"""Keep A's northern tray bypass out of the wall's tracking-error band."""

import math
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from tools_demo.navigation_math import (
    A_NORTH_WALL_INNER_Y,
    APPROACH_EAST,
    APPROACH_WEST,
    NavigationValueError,
    TRAY_BASE_RADIUS,
    TRAY_REQUIRED_CLEARANCE,
    TRAY_TRACKING_RESERVE,
    ZonePose,
    tray_crossing_waypoints,
)
from tools_demo.task_navigator import TaskNavigator
from test_dynamic_layout import boxes
from test_navigation_contract import _point_to_oriented_box


def wall_route(side=APPROACH_EAST, target=(-5., 3.8)):
    """Use the same physical wall boundary as current-layout A commands."""
    return tray_crossing_waypoints(
        (0., 3.8), side, target, north_wall_y=A_NORTH_WALL_INNER_Y)


@pytest.mark.parametrize('target_y', [3.8 - 1e-6, 3.8, 3.8 + 1e-6])
def test_wall_bypass_is_stable_across_target_y_roundoff(target_y):
    """A north-side target cannot select the wall-adjacent lane again."""
    route = wall_route(target=(-5., target_y))
    assert [v for p in route for v in (p.x, p.y)] == pytest.approx(
        [.85, 2.95, -.85, 2.95])
    assert [p.zone for p in route] == [
        'tray_wall_clear_near', 'tray_wall_clear_far']
    assert all(p.yaw == math.pi for p in route)


def test_reverse_crossing_faces_east_and_same_side_needs_no_bypass():
    """The geometry depends on the approach, not a hard-coded cube name."""
    route = wall_route(APPROACH_WEST, (5., 3.8))
    assert route[0].x < route[1].x
    assert all(p.yaw == 0. for p in route)
    assert not wall_route(APPROACH_EAST, (5., 3.8))


def test_other_trays_and_unspecified_wall_keep_the_existing_route():
    """No legacy/B/C route or well-clear tray changes implicitly."""
    normal = tray_crossing_waypoints((0., 3.8), APPROACH_EAST, (-5., 3.8))
    assert [(p.x, p.y) for p in normal] == [(.65, 4.45), (-.65, 4.45)]
    for cube in [(0., 3.6), (0., -5.4), (-7.7, -2.8)]:
        target = (cube[0] - 5., 3.8)
        assert tray_crossing_waypoints(
            cube, APPROACH_EAST, target,
            north_wall_y=A_NORTH_WALL_INNER_Y,
        ) == tray_crossing_waypoints(cube, APPROACH_EAST, target)
    with pytest.raises(NavigationValueError):
        tray_crossing_waypoints(
            (0., 3.8), APPROACH_EAST, (-5., 3.8), north_wall_y=math.nan)


def test_complete_bypass_clears_actual_world_with_existing_tracking_reserve():
    """Check segments, not only endpoints, against physical walls and trays."""
    physical = boxes()
    wall_dist = min(_point_to_oriented_box(0., 4.45, box)
                    for name, box in physical if name == 'officeroom')
    assert wall_dist == pytest.approx(A_NORTH_WALL_INNER_Y - 4.45)
    assert wall_dist - TRAY_BASE_RADIUS - TRAY_TRACKING_RESERVE \
        < TRAY_REQUIRED_CLEARANCE
    route = [(.908, 3.799)] + [(p.x, p.y) for p in wall_route()] + [(-5., 3.8)]
    for start, end in zip(route, route[1:]):
        for index in range(201):
            fraction = index / 200
            point = tuple(a + (b - a) * fraction for a, b in zip(start, end))
            clearance = min(_point_to_oriented_box(*point, box)
                            for _, box in physical)
            assert clearance - TRAY_BASE_RADIUS - TRAY_TRACKING_RESERVE \
                > TRAY_REQUIRED_CLEARANCE


def test_wall_tray_anchors_cannot_be_batched_or_handed_off_early():
    """Keep the safe lane's tracking and speed bounds active at both bends."""
    route = list(wall_route()) + [ZonePose('A', -5., 3.8, math.pi)]
    published = []
    node = SimpleNamespace(
        _wait_for_action_server=lambda: True, nav_batch_attempted=False,
        pending_poses=route, stage_index=0, _send_pose=Mock(),
        _send_pose_batch=Mock(), active_route_zone='A',
        active_return_zone=None, nav_handoff_distance=.20,
        use_legacy_layout_routes=False, route_profiles={},
        stage_speed_limit_publisher=SimpleNamespace(publish=published.append),
        get_logger=lambda: Mock())
    for name in ['tray_wall_clear_near', 'tray_wall_clear_far', 'A']:
        TaskNavigator._send_next_goal(node)
        pose = node._send_pose.call_args.args[0]
        assert pose.zone == name
        if name != 'A':
            assert TaskNavigator._route_handoff_distance(node) == .08
        TaskNavigator._publish_stage_speed_limit(node, pose)
    node._send_pose_batch.assert_not_called()
    assert [message.speed_limit for message in published] == [.75, .75, 0.]
