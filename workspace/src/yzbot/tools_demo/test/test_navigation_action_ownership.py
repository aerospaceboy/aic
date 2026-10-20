"""Regress late Nav2 callbacks and overlapping obstacle recovery owners."""

from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from tools_demo.task_navigator import TaskNavigator


def test_canceled_old_result_cannot_abort_new_navigation():
    """Late cancellation must neither read status nor consume new retries."""
    state = SimpleNamespace(nav_request_id=8, get_logger=Mock())
    old_result = Mock()
    TaskNavigator.result_callback(state, old_result, request_id=7)
    old_result.result.assert_not_called()
    assert state.nav_request_id == 8


def test_late_accepted_goal_is_canceled_without_replacing_current_handle():
    """An old asynchronous acceptance cannot acquire navigation ownership."""
    current = Mock()
    old = Mock(accepted=True)
    state = SimpleNamespace(nav_request_id=4, goal_handle=current)
    reply = Mock()
    reply.result.return_value = old
    TaskNavigator.goal_response_callback(state, reply, request_id=3)
    old.cancel_goal_async.assert_called_once_with()
    assert state.goal_handle is current
    old.get_result_async.assert_not_called()


def test_old_feedback_cannot_advance_the_new_route():
    """A stale route cannot change safety-anchor telemetry."""
    state = SimpleNamespace(nav_request_id=5, stage_index=1)
    TaskNavigator.feedback_callback(state, Mock(), request_id=4)
    assert state.stage_index == 1


def test_bounded_escape_cannot_start_a_second_obstacle_retreat():
    """Only the checked escape may drive until its endpoint is reached."""
    state = SimpleNamespace(recovery_target=object())
    TaskNavigator._obstacle_reverse_tick(state)


def test_direct_alignment_cannot_trigger_old_nav2_progress_watchdog():
    """A live dock/recovery timer replaces the completed Nav2 watchdog."""
    state = SimpleNamespace(
        pending_start_command=None, obstacle_reverse_active=False,
        alignment_target_yaw=0.0,
    )
    TaskNavigator.timeout_callback(state)


def test_watchdog_cancel_uses_bounded_mover_wait_before_escape_or_failure():
    """A self-requested cancel must not bypass the mover retry branch."""
    state = SimpleNamespace(
        nav_handoff_requested=False, goal_timed_out=False,
        obstacle_reverse_active=False, progress_recovery_pending=True,
        _retry_nav_goal=Mock(return_value=False),
        _try_mover_yield_retry=Mock(return_value=True),
        _try_bounded_recovery=Mock(),
    )
    result = Mock()
    TaskNavigator.result_callback(state, result)
    state._try_mover_yield_retry.assert_called_once_with()
    state._try_bounded_recovery.assert_not_called()
    result.result.assert_not_called()
    assert state.progress_recovery_pending is False


@pytest.mark.parametrize('x_value,stamp,allowed', [
    (-.724, 1.0, True), (-.2, 1.0, False), (-.724, 0.0, False),
])
def test_central_wait_needs_fresh_actor_and_full_patrol_clearance(
        x_value, stamp, allowed):
    """Soft prediction retries may wait only outside the physical sweeps."""
    topic = '/competition/obstacles/crossing_cart_1'
    state = SimpleNamespace(
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=1_000_000_000)),
        nav_goal_pose=object(), mover_involved_ns=None,
        mover_involved_horizon_ns=30_000_000_000,
        latest_x=x_value, latest_y=-3.168,
        central_patrols=([0., -3.85, 0., -3.27], [.4, -3.85, .4, -3.27]),
        obstacle_positions={topic: (0., -3.4)},
        obstacle_source_stamps={topic: stamp}, _lethal_start_cost=lambda: None,
        mover_yield_retries_used=0, mover_yield_retries=2,
        mover_yield_wait_ns=15_000_000_000,
        _publish_obstacle_velocity=Mock(), get_logger=lambda: Mock(),
    )
    assert TaskNavigator._try_mover_yield_retry(state) is allowed
    assert state.mover_yield_retries_used == int(allowed)
