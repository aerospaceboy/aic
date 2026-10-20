"""Exercise continuous-route failures through the actual serial dispatcher."""

from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from tools_demo.task_navigator import TaskNavigator, ZonePose


class RouteHarness:
    """Run route dispatch and callbacks without a ROS executor."""

    _send_next_goal = TaskNavigator._send_next_goal
    _fallback_batch_to_serial = TaskNavigator._fallback_batch_to_serial

    def __init__(self):
        self.pending_poses = [
            ZonePose("wall_corner", 1.0, 2.0, 0.0),
            ZonePose("aisle", 3.0, 4.0, 0.0),
            ZonePose("endpoint", 5.0, 6.0, 0.0),
        ]
        self.nav_batch_poses = []
        self.nav_batch_attempted = False
        self.nav_goal_pose = None
        self.goal_handle = None
        self.nav_handoff_requested = False
        self.goal_timed_out = False
        self.obstacle_reverse_active = False
        self.progress_recovery_pending = False
        self.stage_index = 0
        self.stage_total = 3
        self.latest_x = 0.0
        self.latest_y = 0.0
        self.latest_yaw = 0.0
        self.last_feedback_ns = 0
        self.active_zone = "B"
        self.through_behavior_tree = "strict.xml"
        self.sent_batches = []
        self.sent_serial = []
        self.publish_state = Mock()
        self.publish_error = Mock()
        self._logger = Mock()
        self._publish_obstacle_velocity = Mock()

    def _wait_for_action_server(self, **kwargs):
        return True

    def get_logger(self):
        return self._logger

    def get_clock(self):
        now = SimpleNamespace(nanoseconds=2_000_000_000, to_msg=lambda: None)
        return SimpleNamespace(now=lambda: now)

    def _send_pose_batch(self, poses):
        self.sent_batches.append(list(poses))

    def _send_pose(self, pose):
        self.sent_serial.append(pose)

    def start_pre_navigation_alignment(self):
        self._send_next_goal()


def test_fallback_dispatches_every_anchor_serially_without_rebatching():
    harness = RouteHarness()
    original = list(harness.pending_poses)
    harness._send_next_goal()
    assert harness._fallback_batch_to_serial("injected_abort")
    while harness.pending_poses:
        harness._send_next_goal()
    assert harness.sent_serial == original
    assert harness.sent_batches == [original]
    assert not harness._fallback_batch_to_serial("second_failure")


@pytest.mark.parametrize("remaining", [0, 1, 2])
def test_feedback_cannot_discard_strict_path_recovery_anchors(remaining):
    harness = RouteHarness()
    original = list(harness.pending_poses)
    harness._send_next_goal()
    message = SimpleNamespace(feedback=SimpleNamespace(
        number_of_poses_remaining=remaining,
        distance_remaining=1.5,
        number_of_recoveries=0,
    ))
    TaskNavigator.feedback_callback(harness, message)
    assert harness.nav_batch_poses == original
    harness._fallback_batch_to_serial("injected_after_feedback")
    assert harness.sent_serial + harness.pending_poses == original


def test_obstacle_reverse_resumes_original_route_serially():
    harness = RouteHarness()
    original = list(harness.pending_poses)
    harness._send_next_goal()
    harness.obstacle_reverse_active = True
    TaskNavigator._finish_obstacle_reverse(harness)
    assert harness.sent_batches == [original]
    assert harness.sent_serial + harness.pending_poses == original
    assert harness.obstacle_reverse_active is False


def test_synchronous_batch_send_exception_restores_route():
    harness = RouteHarness()
    harness._send_next_goal()
    original = list(harness.nav_batch_poses)
    harness._log_costmap_probe = Mock()
    harness.goal_from_zones = Mock(return_value=object())
    harness.through_action_client = SimpleNamespace(
        server_is_ready=lambda: True,
        send_goal_async=Mock(side_effect=RuntimeError("transport unavailable")),
    )
    harness.feedback_callback = Mock()
    harness.goal_response_callback = Mock()
    TaskNavigator._send_pose_batch(harness, original)
    assert harness.sent_serial + harness.pending_poses == original


def test_result_transport_exception_falls_back_without_losing_mission():
    harness = RouteHarness()
    harness._send_next_goal()
    original = list(harness.nav_batch_poses)
    future = Mock()
    future.result.side_effect = RuntimeError("result transport unavailable")
    harness._clear_goal = Mock()
    TaskNavigator.result_callback(harness, future)
    assert harness.sent_serial + harness.pending_poses == original
    harness._clear_goal.assert_not_called()


def test_new_command_can_batch_after_previous_command_finishes():
    harness = RouteHarness()
    original = list(harness.pending_poses)
    harness._send_next_goal()
    harness._fallback_batch_to_serial("injected_abort")
    TaskNavigator._clear_goal(harness)
    harness.pending_poses = original
    harness._send_next_goal()
    assert harness.sent_batches == [original, original]


def test_unavailable_optional_server_falls_back_without_discovery_wait():
    """Serial navigation must not pay the optional server's 120-second wait."""
    harness = RouteHarness()
    harness._send_next_goal()
    original = list(harness.nav_batch_poses)
    harness.through_action_client = SimpleNamespace(server_is_ready=lambda: False)
    harness._wait_for_action_server = Mock(return_value=True)
    TaskNavigator._send_pose_batch(harness, original)
    assert harness.sent_serial + harness.pending_poses == original
    # Only the serial dispatch's ordinary readiness check is reached.
    harness._wait_for_action_server.assert_called_once_with()
