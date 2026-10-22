"""A completed wall arc keeps its tangent into the next planned stage."""

from functools import partial
import math
from types import SimpleNamespace
from unittest.mock import Mock

from action_msgs.msg import GoalStatus
import pytest

from tools_demo.navigation_math import ZonePose
from tools_demo.task_navigator import TaskNavigator


def continuation_node(batch=True):
    exit_pose = ZonePose('FLOW_EAST_99::exit', 12.4, -2.5, math.pi)
    pickup = ZonePose('BLUE_CUBE_1', 8.3, 3.6, math.pi)
    node = SimpleNamespace(
        active_zone='BLUE_CUBE_1', latest_x=12.4, latest_y=-2.5,
        nav_batch_poses=[exit_pose] if batch else [], nav_goal_pose=exit_pose,
        nav_batch_proven_count=0, goal_handle=Mock(), goal_timed_out=False,
        nav_handoff_requested=False, obstacle_reverse_active=False,
        progress_recovery_pending=False, pending_poses=[pickup], stage_index=14,
        stage_total=15, get_logger=lambda: Mock(), publish_state=Mock(),
        start_pre_navigation_alignment=Mock(), _send_next_goal=Mock())
    node._continue_staged_navigation = partial(
        TaskNavigator._continue_staged_navigation, node)
    return node


@pytest.mark.parametrize('batch', [True, False])
def test_odom_handoff_remembers_corner_before_clearing_the_old_action(batch):
    node = continuation_node(batch)
    node.nav_handoff_requested = True
    TaskNavigator._complete_nav_handoff(node)
    assert not node.nav_batch_poses
    assert node.nav_goal_pose is None
    node.start_pre_navigation_alignment.assert_not_called()
    node._send_next_goal.assert_called_once_with()


@pytest.mark.parametrize('batch', [True, False])
def test_success_result_hands_the_pickup_path_to_nav2_without_a_bearing_turn(batch):
    node = continuation_node(batch)
    future = Mock()
    future.result.return_value = SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED)
    TaskNavigator.result_callback(node, future)
    node.start_pre_navigation_alignment.assert_not_called()
    node._send_next_goal.assert_called_once_with()


def test_ordinary_stages_retain_their_existing_alignment_owner():
    node = continuation_node()
    TaskNavigator._continue_staged_navigation(node)
    node.start_pre_navigation_alignment.assert_called_once_with()
    node._send_next_goal.assert_not_called()


def test_failed_corner_keeps_the_recovery_route_instead_of_advancing():
    node = continuation_node()
    node._retry_nav_goal = Mock(return_value=True)
    future = Mock()
    future.result.return_value = SimpleNamespace(status=GoalStatus.STATUS_ABORTED)
    TaskNavigator.result_callback(node, future)
    node._retry_nav_goal.assert_called_once_with()
    node._send_next_goal.assert_not_called()


def test_success_cannot_skip_an_unvisited_arc_anchor():
    node = continuation_node()
    node.nav_batch_poses.insert(0, ZonePose('FLOW_EAST_06::arc', 14.4, -3.5, 1.57))
    node._retry_nav_goal = Mock(return_value=True)
    future = Mock()
    future.result.return_value = SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED)
    TaskNavigator.result_callback(node, future)
    node._retry_nav_goal.assert_called_once_with()
    node._send_next_goal.assert_not_called()
