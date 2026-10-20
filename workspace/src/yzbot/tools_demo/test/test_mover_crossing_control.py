"""Exercise admitted crossing commands and bounded failure handling."""

from types import SimpleNamespace
from unittest.mock import Mock

from tools_demo.task_navigator import TaskNavigator
from tools_demo.navigation_math import CORRIDOR_TRANSIT_SPEED


def harness():
    """Build the minimal navigator crossing state without spinning ROS."""
    return SimpleNamespace(
        mover_crossing_yaw=0.0, mover_crossing_exit_x=5.7,
        latest_x=4.0, latest_y=0.0, latest_yaw=0.0,
        mover_crossing_admitted_ns=1_000_000_000,
        mover_crossing_anchor_ns=1_000_000_000,
        mover_crossing_anchor_x=4.0, mover_crossing_anchor_y=0.0,
        mover_crossing_min_move=0.02,
        mover_crossing_stall_ns=8_000_000_000,
        mover_crossing_total_ns=30_000_000_000,
        mover_hold_topic=None,
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(
            nanoseconds=1_000_000_000)),
        get_logger=lambda: Mock(), _publish_direct_velocity=Mock(),
        _publish_obstacle_velocity=Mock(), _finish_obstacle_reverse=Mock(),
        publish_error=Mock(), publish_state=Mock(), _clear_goal=Mock())


def test_admitted_aligned_crossing_keeps_a_straight_command():
    """The admitted leg does not hand back to a replanning controller."""
    node = harness()
    TaskNavigator._mover_crossing_step(node)
    command = node._publish_direct_velocity.call_args.args[0]
    assert command.linear.x == CORRIDOR_TRANSIT_SPEED and command.angular.z == 0.0
    node._finish_obstacle_reverse.assert_not_called()


def test_crossing_hands_back_only_after_clearing_the_patrol():
    """The next Nav2 goal starts beyond the actor's sweep lane."""
    node = harness()
    node.latest_x = 5.71
    TaskNavigator._mover_crossing_step(node)
    node._finish_obstacle_reverse.assert_called_once()
    assert node.mover_hold_topic is None


def test_crossing_timeout_fails_instead_of_driving_blindly():
    """A blocked crossing reports failure and publishes zero."""
    node = harness()
    # The episode bound runs from admission, not from a shared clock.
    node.mover_crossing_admitted_ns = 1_000_000_000 - 31_000_000_000
    TaskNavigator._mover_crossing_step(node)
    node._publish_obstacle_velocity.assert_called_once_with(0.0)
    node.publish_state.assert_called_once_with(
        "failed", reason="mover_crossing_timeout")
    node._clear_goal.assert_called_once()


def test_clear_window_still_enters_the_controlled_crossing_profile():
    """A predicted window must not hand an unconstrained path to Nav2."""
    node = SimpleNamespace(
        _corridor_mover_topics=lambda: ["mover"],
        _mover_base_frame=lambda topic: (1.0, 0.1, 1.0, 0.1, 1.0, 0.0, 0.8),
        mover_admitted=set(), mover_hold_lateral=0.6,
        mover_hold_decel=1.0, mover_hold_standoff=0.7,
        mover_hold_entry_lateral=1.2, _mover_hold_clear=lambda topic: True)
    assert TaskNavigator._mover_yield_topic(node) == "mover"
    assert not node.mover_admitted


def test_admission_rejects_a_window_unsafe_after_slowdown():
    """The nominal-speed window cannot override a slower unsafe profile."""
    import time
    from unittest.mock import patch
    node = SimpleNamespace(
        mover_tracks={"mover": (0.0, time.monotonic())},
        _mover_base_frame=lambda topic: (1.0, 0.1, 1.0, 0.1, 1.0, 0.0, 0.8),
        obstacle_positions={"mover": (5.0, -0.4)},
        latest_x=6.4, latest_y=0.1, latest_yaw=3.14159,
        _mover_world_vy=lambda topic: -0.225)
    with patch("tools_demo.task_navigator.corridor_crossing_clear",
               side_effect=lambda *args, **kwargs:
               kwargs["crossing_speed"] == 0.8):
        assert not TaskNavigator._mover_hold_clear(node, "mover")


def test_held_crossing_keeps_commanding_the_admitted_lane():
    """A held base keeps the crossing command so the gate can release it."""
    node = harness()
    node.mover_crossing_admitted_ns = 500_000_000
    node.mover_crossing_anchor_ns = 500_000_000
    TaskNavigator._mover_crossing_step(node)
    # Zeroing the command here deadlocked the base on 2026-10-06: the lane
    # gate only ever releases the command it can see, so it must keep flowing.
    command = node._publish_direct_velocity.call_args.args[0]
    assert command.linear.x == CORRIDOR_TRANSIT_SPEED
    node.publish_state.assert_not_called()
    node._clear_goal.assert_not_called()


def test_held_crossing_fails_only_after_its_progress_clock_expires():
    """A base that never moves inside the lane expires on the stall rule."""
    node = harness()
    node.mover_crossing_admitted_ns = 1_000_000_000 - 9_000_000_000
    node.mover_crossing_anchor_ns = 1_000_000_000 - 9_000_000_000
    TaskNavigator._mover_crossing_step(node)
    node._publish_obstacle_velocity.assert_called_once_with(0.0)
    node.publish_state.assert_called_once_with(
        "failed", reason="mover_crossing_stalled")
    node._clear_goal.assert_called_once()


def test_crossing_progress_restarts_the_stall_clock():
    """Real progress re-anchors the episode instead of expiring it."""
    node = harness()
    node.mover_crossing_admitted_ns = 1_000_000_000 - 9_000_000_000
    node.mover_crossing_anchor_ns = 1_000_000_000 - 9_000_000_000
    node.latest_x = 4.5
    TaskNavigator._mover_crossing_step(node)
    assert node.mover_crossing_anchor_x == 4.5
    assert node.mover_crossing_anchor_ns == 1_000_000_000
    node.publish_state.assert_not_called()
    command = node._publish_direct_velocity.call_args.args[0]
    assert command.linear.x == CORRIDOR_TRANSIT_SPEED
