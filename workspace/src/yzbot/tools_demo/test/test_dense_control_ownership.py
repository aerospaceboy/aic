"""A moving feasible MPPI action must not be canceled by proactive retreat."""
from types import SimpleNamespace
from unittest.mock import Mock
import pytest
from tools_demo.task_navigator import TaskNavigator, ZonePose


def node():
    state=dict(stamp=.9,source='laser_stamped_map_tf',mode='sensor_mppi',
               open_space_gate=True,selected_command=[.4,.2,.1])
    return SimpleNamespace(dense_controller_state=state,latest_vx=.4,latest_vy=.1,
        get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=1_000_000_000)))


def test_fresh_feasible_mppi_with_actual_progress_retains_control():
    assert TaskNavigator._dense_mppi_progress_owns_transit(node())


@pytest.mark.parametrize('change',[
    dict(stamp=.4),dict(mode='nominal'),dict(source='simulator'),
    dict(open_space_gate=False),dict(dynamic_fallback='optimizer failed'),
    dict(selected_command=[0.,0.,1.]),dict(selected_command=[float('nan'),0.,0.]),
    dict(selected_command=[.4]),dict(stamp=None)])
def test_stale_invalid_or_fallback_commands_cannot_suppress_retreat(change):
    n=node();n.dense_controller_state.update(change)
    assert not TaskNavigator._dense_mppi_progress_owns_transit(n)


def test_a_stopped_base_keeps_original_recovery_even_if_mppi_requests_motion():
    n=node();n.latest_vx=0.;n.latest_vy=0.
    assert not TaskNavigator._dense_mppi_progress_owns_transit(n)


def test_central_retreat_is_only_suppressed_during_healthy_moving_mppi():
    n=node();topic='/competition/obstacles/crossing_cart_1'
    n.obstacle_reverse_disabled=False;n.goal_handle=object()
    n.nav_goal_pose=ZonePose('gate',0.,-5.,0.)
    n.nav_handoff_requested=False;n.goal_timed_out=False
    n._obstacle_ahead=Mock(return_value=(topic,1.1))
    n._central_cart_can_block_active_segment=Mock(return_value=True)
    n._central_retreat_clear=Mock(return_value=True)
    assert TaskNavigator._obstacle_reverse_triggered(n) is None
    n._central_retreat_clear.assert_not_called()
    n.dense_controller_state['stamp']=.4
    assert TaskNavigator._obstacle_reverse_triggered(n)==topic
    n._central_retreat_clear.assert_called_once_with(.50)
