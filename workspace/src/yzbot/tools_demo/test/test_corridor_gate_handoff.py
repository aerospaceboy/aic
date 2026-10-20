"""Preserve the admitted lane across BOTH corridor gates, without Nav2 drift."""

import math
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from tools_demo.navigation_math import (
    CORRIDOR_PATROL_SPEED, CORRIDOR_TRANSIT_SPEED, UNIVERSAL_MOVER_1_CLEAR_X,
    ZonePose, corridor_crossing_clear, corridor_lane_release_clear,
    dynamic_gate_waypoints,
)
from tools_demo.task_navigator import TaskNavigator
from test_north_pickup_clearance import loaded_boxes
from test_navigation_contract import _point_to_oriented_box


def test_first_crossing_tail_is_static_clear_and_outside_second_whole_patrol():
    route = dynamic_gate_waypoints((13.2, -.95), (-1.2, -5.))
    assert route[3].x == UNIVERSAL_MOVER_1_CLEAR_X == 6.65
    assert UNIVERSAL_MOVER_1_CLEAR_X-.20-5.-.28-.18 >= .10
    assert UNIVERSAL_MOVER_1_CLEAR_X-.08 > 6.55  # final western gate starts here
    boxes = loaded_boxes()
    for i in range(201):
        x = 7.8+(6.65-7.8)*i/200
        for y in (.51, .55, .59):
            assert min(_point_to_oriented_box(x,y,b)-.28 for _,b in boxes) >= .10


def test_second_gate_acquires_direct_owner_before_any_nav2_dispatch():
    node = SimpleNamespace(
        _wait_for_action_server=lambda: True, nav_batch_attempted=False,
        pending_poses=[ZonePose('universal_mover_2_clear',3.8,.55,math.pi)],
        stage_index=4, latest_x=6.70, _send_pose=Mock(),
        _begin_mover_crossing_hold=Mock())
    TaskNavigator._send_next_goal(node)
    node._send_pose.assert_not_called()
    node._begin_mover_crossing_hold.assert_called_once_with(
        '/competition/obstacles/obstacle_2','staged_admission')


@pytest.mark.parametrize('x,y', [(6.70,.51),(6.60,.59)])
def test_second_wait_has_multiple_safe_samples_for_every_10hz_phase(x,y):
    step = CORRIDOR_PATROL_SPEED*.1
    for phase in range(50):
        admitted=[]
        for i in range(40):
            oy=-.8+phase*step/50+i*step
            if oy>.1:
                break
            admitted.append(corridor_lane_release_clear(y,oy,True,.60) and all(
                corridor_crossing_clear(x,y,math.pi,5.,oy,-CORRIDOR_PATROL_SPEED,
                    crossing_speed=speed,dispatch_delay=delay,
                    patrol_speed=CORRIDOR_PATROL_SPEED)
                for speed in (.56,.8,.85,CORRIDOR_TRANSIT_SPEED)
                for delay in (.4,.8)))
        assert any(all(admitted[i:i+3]) for i in range(len(admitted)-2))


def test_controlled_first_crossing_reaches_second_wait_without_a_nav2_leg():
    node=SimpleNamespace(
        _publish_obstacle_velocity=Mock(),mover_hold_blocked=False,
        mover_hold_topic='/competition/obstacles/obstacle_1',
        obstacle_positions={'/competition/obstacles/obstacle_1':(8.5,-.7)},
        latest_x=9.80,latest_y=.55,latest_yaw=math.pi,
        mover_hold_started_ns=0,mover_hold_min_ns=1_500_000_000,
        obstacle_reverse_wait_deadline_ns=12_000_000_000,
        mover_admitted=set(),_mover_hold_clear=lambda topic:True,
        get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=2_000_000_000)),
        get_logger=lambda:Mock(),nav_goal_pose=ZonePose(
            'universal_mover_1_clear',6.65,.55,math.pi))
    TaskNavigator._mover_hold_step(node)
    assert node.obstacle_reverse_state=='crossing'
    assert node.mover_crossing_exit_x==6.65
    assert node.mover_crossing_lane_y==.55
