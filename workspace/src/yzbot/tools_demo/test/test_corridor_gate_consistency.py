"""Replay the GUI corridor deadlock without relaxing safety limits."""

import math
import time
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from tools_demo.navigation_math import corridor_crossing_clear
from tools_demo.navigation_math import corridor_lane_release_clear
from tools_demo.navigation_math import dynamic_gate_waypoints
from tools_demo.navigation_math import ZonePose
from tools_demo.task_navigator import TaskNavigator


def test_predictor_cannot_override_unreachable_east_gate():
    """The recorded wait admits a prediction but never the real gate."""
    assert corridor_crossing_clear(9.726, .194, math.pi, 8.5, -.3, -.225)
    node = SimpleNamespace(
        mover_tracks={'east': (-.3, time.monotonic())},
        _mover_base_frame=lambda topic: (1, 0, 1, 0, -1, 0, .8),
        obstacle_positions={'east': (8.5, -.3)},
        latest_x=9.726, latest_y=.194, latest_yaw=math.pi,
        _mover_world_vy=lambda topic: -.225)
    assert not TaskNavigator._mover_hold_clear(node, 'east')


def test_gate_keeps_one_metre_and_requires_retreat():
    """A near pass or an approaching actor cannot release the base."""
    assert not corridor_lane_release_clear(.194, -.8, True)
    assert not corridor_lane_release_clear(.45, -.7, False)
    assert corridor_lane_release_clear(.45, -.7, True)


def test_retry_from_recorded_wait_keeps_remaining_crossings():
    """A coordinator retry still has a staging point outside the gate."""
    route = dynamic_gate_waypoints((9.726, .194), (-1.64, -6.0))
    assert route[0].x == 10.1 and route[0].y == .55
    assert [p.zone for p in route[1:3]] == [
        'universal_mover_1_clear', 'universal_mover_2_clear']
    assert not dynamic_gate_waypoints((7.25, .45), (-1.64, -6.0))


def test_first_crossing_acquires_hold_before_nav2_dispatch():
    """Prevent the rounded Nav2 path from moving off the waiting line."""
    node = SimpleNamespace(
        _wait_for_action_server=lambda: True, nav_batch_attempted=False,
        pending_poses=[ZonePose(
            'universal_mover_1_clear', 7.25, .45, math.pi)],
        stage_index=3, latest_x=9.96, _send_pose=Mock(),
        _begin_mover_crossing_hold=Mock())
    TaskNavigator._send_next_goal(node)
    node._send_pose.assert_not_called()
    node._begin_mover_crossing_hold.assert_called_once_with(
        '/competition/obstacles/obstacle_1', 'staged_admission')


@pytest.mark.parametrize('cost,moving,skip', [
    (71, 0., True), (71, .8, False), (0, 0., False)])
def test_only_failed_stationary_inflated_start_skips_identical_retry(
        cost, moving, skip):
    """Healthy motion and a free start retain the normal retry path."""
    node = SimpleNamespace(
        nav_batch_poses=[], nav_retries=0, nav_retry_attempts=1,
        nav_goal_pose=ZonePose('A', -5, 3.8, math.pi),
        _lethal_start_cost=lambda: None,
        latest_x=-1.84, latest_y=-3.24, latest_vx=moving, latest_vy=0.,
        _global_cost_at=lambda x, y: cost, get_logger=lambda: Mock(),
        _clear_costmaps=Mock(), _send_pose=Mock(), publish_state=Mock(),
        active_zone='A', nav_retry_delay_sec=0.)
    assert TaskNavigator._retry_nav_goal(node) is not skip
    assert node._send_pose.called is not skip


def test_bad_wait_reanchors_only_through_checked_space():
    """Preserve the interrupted goal while Nav2 returns to a safe line."""
    old_goal = ZonePose('universal_mover_1_clear', 7.25, .45, math.pi)
    node = SimpleNamespace(
        _publish_obstacle_velocity=Mock(), mover_hold_blocked=False,
        mover_hold_topic='/competition/obstacles/obstacle_1',
        latest_x=9.726, latest_y=.194, mover_hold_started_ns=0,
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(
            nanoseconds=1_000_000_000)),
        _recovery_segment_clear_of_movers=lambda pose: True,
        _costmap_cell_free=lambda x, y: True,
        nav_goal_pose=old_goal, pending_poses=[], stage_total=7,
        get_logger=lambda: Mock(), _finish_obstacle_reverse=Mock())
    TaskNavigator._mover_hold_step(node)
    assert node.nav_goal_pose.zone == 'universal_east_reentry'
    assert node.pending_poses == [old_goal]
    node._finish_obstacle_reverse.assert_called_once()


def test_arrival_error_still_leaves_multiple_actor_updates_to_admit():
    """Replay the measured early stop and every 5 Hz patrol sample phase."""
    route = dynamic_gate_waypoints((13.2, -.95), (-1.64, -6.0))
    x, y = route[2].x + .12, route[2].y - .04
    # Patrol moves 0.225 m/s; at 5 Hz consecutive positions differ 0.045 m.
    # Require at least three consecutive admissible samples for every
    # sample alignment, without changing separation, slowdown or delay.
    for offset in [i * .001 for i in range(45)]:
        admitted = []
        for step in range(20):
            oy = -.8 + offset + step * .045
            if oy > .1:
                break
            admitted.append(corridor_lane_release_clear(y, oy, True) and all(
                corridor_crossing_clear(
                    x, y, math.pi, 8.5, oy, -.225,
                    crossing_speed=speed, dispatch_delay=delay, patrol_speed=.225)
                for speed in (.56, .8) for delay in (.4, .8)))
        assert any(all(admitted[i:i + 3])
                   for i in range(len(admitted) - 2)), offset


def test_corner_speed_limit_is_removed_before_regular_delivery_and_cancel():
    """A slow first turn must not leak into later routes or manual goals."""
    published = []
    node = SimpleNamespace(
        stage_speed_limit_publisher=SimpleNamespace(
            publish=lambda message: published.append(message)),
        get_logger=lambda: Mock())
    TaskNavigator._publish_stage_speed_limit(node, ZonePose(
        'universal_wall_tip_clear', 12.6, .45, math.pi))
    TaskNavigator._publish_stage_speed_limit(node, ZonePose(
        'universal_east_wait', 9.8, .55, math.pi))
    TaskNavigator._publish_stage_speed_limit(node, ZonePose(
        'zone_A', -5.6, 3.4, math.pi))
    TaskNavigator._publish_stage_speed_limit(node, None)
    # The corner reserve is 0.75 since the 20261014 matrix: 0.6 left the
    # heaviest rounds within a few seconds of the 300 s limit.
    assert [m.speed_limit for m in published] == [.75, 0., 0., 0.]
    assert all(not m.percentage for m in published)
