"""Only the verified blue-5 to current-C region bypasses north bay staging."""

import math
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from tools_demo.navigation_math import c_southeast_transport_waypoints, ZonePose
from tools_demo.task_navigator import TaskNavigator
from test_dynamic_layout import boxes
from test_navigation_contract import _point_to_oriented_box


def test_c_south_route_retains_static_clearance_with_full_tracking_reserve():
    physical = boxes()
    for sx in (10.6, 11.):
        for sy in (-13.2, -12.8):
            for tx in (2.5, 3.3):
                for ty in (-12.4, -11.8):
                    guides = c_southeast_transport_waypoints((sx, sy), (tx, ty))
                    assert len(guides) == 1
                    chain = [(sx, sy), (guides[0].x, guides[0].y), (tx, ty)]
                    for a, b in zip(chain, chain[1:]):
                        for i in range(201):
                            p = (a[0]+(b[0]-a[0])*i/200,
                                 a[1]+(b[1]-a[1])*i/200)
                            assert min(_point_to_oriented_box(*p, box)-.28-.20
                                       for _, box in physical) >= .10


@pytest.mark.parametrize('start,target', [
    ((12., -13.), (2.9, -12.1)), ((10.9, -11.), (2.9, -12.1)),
    ((10.9, -13.), (-5., 3.8)), ((10.9, -13.), (4., -5.)),
    ((float('nan'), -13.), (2.9, -12.1))])
def test_other_bays_zones_and_invalid_poses_keep_existing_routes(start, target):
    assert not c_southeast_transport_waypoints(start, target)


def test_c_route_hook_owns_only_current_layout_southeast_delivery():
    place = ZonePose('C', 2.9, -12.1, 0.)
    node = SimpleNamespace(
        _route_pose=Mock(return_value=place), zones={'C': place},
        _delivery_dock_pose=lambda zone, pose: pose,
        _route_waypoints=lambda *args: [],
        dock_arrival_zone=.025, dock_speed_zone=.1,
        use_legacy_layout_routes=False, latest_x=10.93, latest_y=-13.,
        get_logger=lambda: Mock())
    TaskNavigator._configure_delivery_route(node, 'C')
    assert [p.zone for p in node.pending_poses] == ['C_SOUTH_WALL_CLEAR', 'C']
    assert node.pending_poses[-1] == place


def test_c_wall_tip_anchor_cannot_be_batched_or_handed_off_early():
    anchor = ZonePose('C_SOUTH_WALL_CLEAR', 6., -13.2, math.pi)
    node = SimpleNamespace(
        _wait_for_action_server=lambda: True, nav_batch_attempted=False,
        pending_poses=[anchor, ZonePose('C', 2.9, -12.1, 0.)],
        stage_index=0, latest_x=10.9,
        _send_pose=Mock(), _send_pose_batch=Mock())
    TaskNavigator._send_next_goal(node)
    node._send_pose.assert_called_once_with(anchor)
    node._send_pose_batch.assert_not_called()
    node.nav_handoff_distance = .35
    node.active_route_zone = 'C'
    node.route_profiles = {}
    assert TaskNavigator._route_handoff_distance(node) == .08
