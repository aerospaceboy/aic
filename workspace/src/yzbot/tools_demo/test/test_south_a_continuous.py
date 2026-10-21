"""South cargo follows the open north corridor without replaying west anchors."""

import math
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from nav_msgs.msg import Odometry

from tools_demo.navigation_math import ZonePose
from tools_demo.task_navigator import TaskNavigator
from tools_demo.transport_routes import continuous_a_route, current_delivery_route
from test_navigation_batch_recovery import RouteHarness
from test_north_pickup_clearance import loaded_boxes
from test_navigation_contract import _point_to_oriented_box


def route(start=(-3.789, -12.4)):
    return current_delivery_route('A', *start, ZonePose('A', -5, 3.747, math.pi))[0]


@pytest.mark.parametrize('start', [(-3.789, -12.4), (-4.3, -13), (-3.3, -11.5)])
def test_south_departure_omits_both_old_anchors_and_preserves_wall_clearance(start):
    poses = route(start)
    assert continuous_a_route(poses)
    assert not {'A_WALL_WEST_CLEAR', 'A_WALL_SOUTH_TIP_CLEAR'} & {p.zone for p in poses}
    # The first leg is PLANNED around the diagonal, not a direct drive line.
    # Above it, the corridor retains body + original arrival reserve + 10cm.
    boxes = loaded_boxes()
    for a, b in zip(poses, poses[1:]):
        for i in range(201):
            x, y = a.x+(b.x-a.x)*i/200, a.y+(b.y-a.y)*i/200
            gap = min(_point_to_oriented_box(x, y, box)-.28-.20 for _, box in boxes)
            assert gap >= .10-1e-9, (x, y, gap)


def test_other_departures_keep_their_existing_routes():
    assert route((-10.1, -5.6))[0].zone == 'A_WALL_WEST_CLEAR'
    assert not continuous_a_route(route((11.4, -13)))
    assert continuous_a_route(route()[:-1])
    assert not continuous_a_route(route()[:-2])


def test_continuous_dispatch_and_failed_batch_retire_only_physical_visits():
    node = RouteHarness()
    node.active_zone = 'A'
    node.pending_poses = route()
    original = list(node.pending_poses)
    node._send_next_goal()
    assert node.sent_batches == [original[:-1]]
    assert not node.sent_serial
    node._request_nav_handoff_if_reached = Mock()
    node._request_spawn_wait_capture = Mock()
    node._request_blue3_shortcut_handoff = Mock()
    # Feedback claiming all poses passed must not retire unvisited anchors.
    node.nav_batch_proven_count = 0
    assert not TaskNavigator._central_anchor_arrived(node)
    message = Odometry()
    message.pose.pose.orientation.w = 1.0
    # Visiting the second first cannot retire the first.
    for pose, count in [(original[1], 0), (original[0], 1), (original[1], 2)]:
        message.pose.pose.position.x = pose.x
        message.pose.pose.position.y = pose.y
        TaskNavigator.odom_callback(node, message)
        assert node.nav_batch_proven_count == count
    node._fallback_batch_to_serial('injected_replan_failure')
    assert node.sent_serial + node.pending_poses == original[2:]


def test_initial_alignment_uses_planned_tangent_instead_of_wall_crossing_bearing():
    poses = route()
    node = SimpleNamespace(
        pending_poses=poses, dock_pose=poses[-1], latest_x=-3.789,
        latest_y=-12.4, latest_yaw=math.pi, get_logger=lambda: Mock(),
        pre_navigation_alignment_timeout_ns=10_000_000_000,
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=1)),
        publish_state=Mock(), _send_next_goal=Mock())
    TaskNavigator.start_pre_navigation_alignment(node)
    assert not node.pre_navigation_alignment
    node._send_next_goal.assert_called_once_with()


def test_dispatch_selects_the_scoped_continuous_tree():
    node = RouteHarness()
    poses = route()[:-1]
    node.nav_batch_poses = poses
    node._log_costmap_probe = Mock()
    node.goal_from_zones = Mock(return_value=object())
    node.through_action_client = SimpleNamespace(
        server_is_ready=lambda: True, send_goal_async=Mock(return_value=Mock()))
    node.feedback_callback = Mock()
    node.goal_response_callback = Mock()
    TaskNavigator._send_pose_batch(node, poses)
    assert node.goal_from_zones.call_args.args[2].endswith('navigate_through_south_a.xml')


@pytest.mark.parametrize('zone,label,skip', [
    ('A', 'A_place_pose_slot', True), ('B', 'B', True), ('C', 'C_place_pose', True),
    ('A', 'A_WALL_NORTH_CLEAR', False), ('RED_CUBE_5', 'entry', False),
])
def test_only_ordinary_delivery_entries_delegate_alignment_to_the_path(zone, label, skip):
    pose = ZonePose(label, 5., 0., 0.)
    node = SimpleNamespace(
        active_zone=zone, pending_poses=[pose], dock_pose=pose,
        latest_x=0., latest_y=0., latest_yaw=math.pi, get_logger=lambda: Mock(),
        pre_navigation_alignment_timeout_ns=10_000_000_000,
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=1)),
        publish_state=Mock(), _send_next_goal=Mock())
    TaskNavigator.start_pre_navigation_alignment(node)
    assert node.pre_navigation_alignment is not skip
    assert node._send_next_goal.called is skip


def test_display_orders_active_path_before_suffix_and_hides_proven_anchors():
    poses = route()
    node = SimpleNamespace(
        active_zone='A', active_route='primary', active_route_zone='A',
        active_return_zone=None, recovery_attempts=0, latest_x=-2.1, latest_y=.2,
        nav_batch_poses=poses[:-1], pending_poses=poses[-1:],
        nav_batch_proven_count=2, dock_pose=poses[-1],
        _publish_command_path=Mock(), state_publisher=Mock())
    TaskNavigator.publish_state(node, 'active')
    assert node._publish_command_path.call_args.args[0] == [
        [-2.1, .2], [-2.1, 1.85], [-4.15, 3.747], [-5., 3.747]]


def test_batch_feedback_reserves_the_pending_docking_stage():
    node = RouteHarness()
    poses = route()
    node.stage_total = len(poses)
    node.stage_index = 1
    node.nav_batch_poses = poses[:-1]
    node.pending_poses = poses[-1:]
    message = SimpleNamespace(feedback=SimpleNamespace(
        number_of_poses_remaining=4, distance_remaining=20., number_of_recoveries=0))
    TaskNavigator.feedback_callback(node, message)
    assert node.stage_index == 1
    node.last_feedback_ns = 0
    message.feedback.number_of_poses_remaining = 1
    TaskNavigator.feedback_callback(node, message)
    assert node.stage_index == 4
    node._send_next_goal()
    assert node.stage_index == 5
