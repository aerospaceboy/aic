"""Reject the actual unsafe auxiliary reverse seen in the GUI48 batch."""

import math
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from tools_demo.task_navigator import TaskNavigator
from tools_demo.navigation_math import mover_reverse_segment_clear


@pytest.mark.parametrize('x,y', [
    (8.207, 0.058), (8.443, 0.269), (8.344, 0.218), (8.301, 0.048),
])
def test_recorded_reverse_does_not_cancel_nav2_inside_actor_sweep(x, y):
    """An instantaneous clear sample cannot admit an unsafe swept path."""
    handle = Mock()
    harness = SimpleNamespace(
        latest_x=x, latest_y=y, latest_yaw=math.pi,
        obstacle_reverse_count=0, obstacle_max_reverses=1,
        obstacle_reverse_disabled=False,
        _obstacle_reverse_distance_for=Mock(return_value=0.20),
        goal_handle=handle, nav_handoff_requested=False,
        obstacle_reverse_speed=0.25,
        _publish_obstacle_velocity=Mock(),
        get_logger=Mock(return_value=Mock()),
        _segment_is_free=Mock(return_value=True),
    )
    TaskNavigator._begin_obstacle_reverse(
        harness, '/competition/obstacles/obstacle_1',
    )
    handle.cancel_goal_async.assert_not_called()
    harness._publish_obstacle_velocity.assert_not_called()
    assert harness.goal_handle is handle
    assert harness.obstacle_reverse_count == 0


def test_reverse_endpoint_cannot_enter_sweep_from_safe_start():
    """A clear start alone cannot validate a reverse endpoint."""
    assert not mover_reverse_segment_clear((9.3, 0.0), (8.9, 0.0), 8.5)


def test_reverse_clear_of_full_sweep_remains_available():
    """The guard does not disable all obstacle recovery."""
    assert mover_reverse_segment_clear((9.3, 0.0), (9.5, 0.0), 8.5)
    assert mover_reverse_segment_clear((8.5, 0.8), (8.7, 0.8), 8.5)
    assert not mover_reverse_segment_clear((5.1, 0.2), (5.3, 0.2), 5.0)
