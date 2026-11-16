"""Recovery follows fresh environmental change, never a lifetime retry count."""
import json
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from std_msgs.msg import Bool, String
from tools_demo.task_navigator import TaskNavigator, ZonePose
from tools_demo.moving_obstacle import MovingObstacle
from tools_demo.dynamic_motion import PolylinePatrol


def harness():
    n = SimpleNamespace(
        now_ns=10_000_000_000, latest_x=0., latest_y=0., latest_yaw=0.,
        nav_goal_pose=ZonePose('test', 4., 0., 0.), goal_timed_out=False,
        nav_batch_poses=[], pending_poses=[], obstacle_positions={},
        obstacle_source_stamps={}, dense_recovery_movers={'transport_mover_03': ((.7,0.),10.)},
        dynamic_recovery_wait=None, dynamic_recovery_waits=0,
        recovery_attempts=2, nav_retries=1, nav_request_id=1,
        _publish_direct_velocity=Mock(), _release_direct_control=Mock(),
        _clear_costmaps=Mock(), _lethal_start_cost=Mock(return_value=None),
        _send_pose=Mock(), _fallback_batch_to_serial=Mock(),
        publish_state=Mock(), publish_error=Mock(), _clear_goal=Mock(),
        get_logger=Mock(return_value=Mock()))
    n.get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=n.now_ns))
    for name in ('_recovery_mover_observations','_nearby_recovery_movers',
                 '_try_dynamic_recovery_wait','_dynamic_recovery_wait_tick'):
        setattr(n,name,getattr(TaskNavigator,name).__get__(n))
    return n


def test_new_command_gets_its_own_recovery_budget():
    n=harness()
    n.recovery_target=ZonePose('stale_escape',1.,0.,0.)
    n.recovery_pending_poses=[n.nav_goal_pose]
    TaskNavigator._reset_mover_control_for_command(n)
    assert n.recovery_attempts == 0
    assert n.dynamic_recovery_waits == 0
    assert n.dynamic_recovery_deadline_ns is None
    assert n.recovery_target is None
    assert n.recovery_pending_poses == []


def test_terminal_cleanup_cannot_leave_an_escape_latched_over_the_next_leg():
    n=harness()
    n.recovery_target=ZonePose('old',1.,0.,0.)
    n.recovery_pending_poses=[n.nav_goal_pose]
    TaskNavigator._clear_goal(n)
    assert n.recovery_target is None
    assert n.recovery_pending_poses == []
    assert n.alignment_target_yaw is None


def test_blocked_escape_timeout_restores_original_route_before_dynamic_wait():
    n=harness();original=n.nav_goal_pose
    n.recovery_target=ZonePose('escape',1.,0.,0.)
    n.recovery_pending_poses=[original,ZonePose('next',5.,0.,0.)]
    n.recovery_dock_pose=ZonePose('dock',6.,0.,0.)
    n.recovery_next_dock_pose=None;n.recovery_final_yaw=1.
    n.alignment_target_yaw=0.;n.alignment_deadline_ns=n.now_ns
    n.stop_alignment=Mock();n._try_bounded_recovery=Mock()
    n._restore_bounded_recovery_route=TaskNavigator._restore_bounded_recovery_route.__get__(n)
    TaskNavigator.alignment_callback(n)
    n.stop_alignment.assert_called_once()
    assert n.recovery_target is None
    assert n.nav_goal_pose == original
    assert n.pending_poses == [ZonePose('next',5.,0.,0.)]
    assert n.dock_pose == ZonePose('dock',6.,0.,0.)
    assert n.dynamic_recovery_wait is not None
    n._try_bounded_recovery.assert_not_called()
    n._clear_goal.assert_not_called()


def test_observed_actor_motion_replans_the_retained_goal():
    n=harness();goal=n.nav_goal_pose
    assert n._try_dynamic_recovery_wait()
    assert n.nav_goal_pose == goal
    n.now_ns+=1_000_000_000
    n.dense_recovery_movers={'transport_mover_03': ((1.1,0.),11.)}
    n._dynamic_recovery_wait_tick()
    n._send_pose.assert_not_called()  # Wait for costmap update too.
    n.now_ns+=500_000_000
    n._dynamic_recovery_wait_tick()
    n._send_pose.assert_called_once_with(goal)
    assert n.recovery_attempts == 0
    assert n.dynamic_recovery_waits == 1
    n._release_direct_control.assert_called_once()
    for call in n._publish_direct_velocity.call_args_list:
        twist=call.args[0]
        assert twist.linear.x == twist.linear.y == twist.angular.z == 0.


@pytest.mark.parametrize('kind', ['missing','stale','too_close','lethal','unchanged'])
def test_wait_never_releases_for_an_unproved_window(kind):
    n=harness();assert n._try_dynamic_recovery_wait()
    for seconds in (1.,2.):
        n.now_ns=int((10+seconds)*1e9)
        n.dense_recovery_movers={'transport_mover_03': ((1.1,0.),10+seconds)}
        if kind=='missing':n.dense_recovery_movers={}
        if kind=='stale':n.dense_recovery_movers['transport_mover_03']=((1.1,0.),9.)
        if kind=='too_close':n.dense_recovery_movers['transport_mover_03']=((.3,0.),10+seconds)
        if kind=='unchanged':n.dense_recovery_movers['transport_mover_03']=((.7,0.),10+seconds)
        if kind=='lethal':n._lethal_start_cost.return_value=100
        n._dynamic_recovery_wait_tick()
    n._send_pose.assert_not_called()
    assert n.recovery_attempts == 2
    n.now_ns=22_000_000_000
    n._dynamic_recovery_wait_tick()
    n._clear_goal.assert_called_once()
    n.publish_state.assert_called_with('failed',reason='dynamic_recovery_no_safe_window')


def test_actor_observed_leaving_neighbourhood_can_release_but_disappearance_cannot():
    n=harness();assert n._try_dynamic_recovery_wait()
    for seconds in (1.,1.6):
        n.now_ns=int((10+seconds)*1e9)
        n.dense_recovery_movers={'transport_mover_03': ((2.2,0.),10+seconds)}
        n._dynamic_recovery_wait_tick()
    n._send_pose.assert_called_once()


def test_wait_count_and_absolute_deadline_do_not_replenish_on_actor_motion():
    n=harness()
    n.dynamic_recovery_waits=3
    assert not n._try_dynamic_recovery_wait()
    n.dynamic_recovery_waits=0;n.dynamic_recovery_deadline_ns=n.now_ns
    assert not n._try_dynamic_recovery_wait()


def test_wait_can_be_canceled_without_a_nav2_goal_handle():
    n=harness();assert n._try_dynamic_recovery_wait()
    n.goal_handle=None
    TaskNavigator.cancel_callback(n,Bool(data=True))
    n.publish_state.assert_called_with('canceled')
    n._clear_goal.assert_called_once()


def test_invalid_dense_snapshot_cannot_leave_stale_recovery_actors():
    n=harness()
    TaskNavigator._dense_recovery_callback(n,String(data=json.dumps({
        'actors':[dict(model='test',x=float('nan'),y=0,stamp_sec=10,success=True)]})))
    assert not n.dense_recovery_movers


def test_central_cart_reverses_smoothly_with_robot_stopped_on_its_rail():
    n=SimpleNamespace(
        motion_elapsed_sec=1.2,motion_started=True,update_period_sec=.1,
        motion_phase_offset_sec=0.,patrol=PolylinePatrol([0,0,0,2],8.),
        last_motion_position=(0.,.6),robot_x=0.,robot_y=0.,
        robot_clearance_stop_distance=.58,robot_clearance_release_distance=.7,
        robot_clearance_freeze_timeout_sec=2.,proximity_guard_paused=False,
        proximity_guard_paused_since=None)
    previous=n.last_motion_position
    # Initially heading toward the robot by reversing the phase direction.
    n.motion_direction=-1
    positions=[]
    for _ in range(120):
        p=MovingObstacle._continuous_position(n)
        assert abs(p[1]-previous[1]) <= .05+1e-9
        assert p[1] >= .58-1e-9
        positions.append(p);previous=p
    assert len(set(positions)) > 20
