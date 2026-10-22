"""Passing below mover 2 must not reverse a collision-checked Nav2 path."""

from functools import partial
import math
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

from tools_demo.corner_flow import flow_prefix_size, round_navigation_corners
from tools_demo.navigation_math import ZonePose
from tools_demo.task_navigator import TaskNavigator
from test_navigation_contract import _point_to_oriented_box
from test_north_pickup_clearance import loaded_boxes


def crossing_node(x, y, yaw, target_x, actor_y=-.5):
    topic = '/competition/obstacles/obstacle_2'
    node = SimpleNamespace(
        latest_x=x, latest_y=y, latest_yaw=yaw, latest_vx=1.1, latest_vy=0.,
        nav_goal_pose=ZonePose('BLUE_CUBE_1', target_x, 3.6, math.pi),
        obstacle_positions={topic: (5., actor_y)}, mover_admitted=set(),
        mover_response_seen={topic}, mover_hold_lateral=.56,
        mover_hold_decel=1.5, mover_hold_standoff=1.05,
        mover_hold_entry_lateral=1.45, _corridor_mover_topics=lambda: (topic,))
    node._mover_base_frame = partial(TaskNavigator._mover_base_frame, node)
    return node, topic


@pytest.mark.parametrize('x,yaw,target_x', [
    (4.867, 2.96155, 8.3), (5.133, .18004, 3.8)])
def test_lateral_projection_cannot_turn_a_departing_base_back_across_the_mover(x, yaw, target_x):
    node, topic = crossing_node(x, -1.57944, yaw, target_x)
    frame = node._mover_base_frame(topic)
    # Reproduce the old false trigger: the actor is slightly forward in
    # body coordinates, while the base is already heading away along X.
    assert 0 < frame[2] < .2
    assert abs(frame[3]) < node.mover_hold_entry_lateral
    assert frame[0] * frame[4] < 0
    assert TaskNavigator._mover_yield_topic(node) is None
    assert not node.mover_admitted


@pytest.mark.parametrize('x,yaw,target_x', [
    (6.3, math.pi, 3.8), (3.7, 0., 6.3)])
def test_a_real_crossing_still_requires_admission_in_both_directions(x, yaw, target_x):
    node, topic = crossing_node(x, .55, yaw, target_x)
    assert TaskNavigator._mover_yield_topic(node) == topic
    assert not node.mover_admitted  # A prior response cannot admit it.


@pytest.mark.parametrize('target_x', [6.5, 8.3])
def test_blue1_pickup_reaches_the_south_lane_before_turning_north(target_x):
    target = ZonePose('BLUE_CUBE_1', target_x, 3.6, math.pi)
    poses = round_navigation_corners((10.33, -5.01), [target], 'BLUE_CUBE_1')
    count = flow_prefix_size(poses)
    assert poses[count-1].x == 3.2
    assert poses[count-1].y == pytest.approx(-1.0970562748477144)
    assert poses[count:] == [target]
    exit_index = next(i for i, p in enumerate(poses) if p.x == 12.4 and p.y == -2.5)
    assert exit_index < count-1
    assert round_navigation_corners((10.33, -5.01), poses, 'BLUE_CUBE_1') == poses
    boxes = loaded_boxes()
    root = Path(__file__).resolve().parents[2]
    actor = yaml.safe_load((root/'bot_navigation/config/competition_zones.yaml').read_text())[
        'competition_moving_obstacle_2']['ros__parameters']
    minimum_y = actor['center_y'] - actor['amplitude_y']
    maximum_y = actor['center_y'] + actor['amplitude_y']
    for a, b in zip(poses[exit_index:count-1], poses[exit_index+1:count]):
        for i in range(81):
            x, y = a.x+(b.x-a.x)*i/80, a.y+(b.y-a.y)*i/80
            assert min(_point_to_oriented_box(x, y, box) - .28 - .20
                       for _, box in boxes) >= .10
            # Full patrol envelope, original chassis/tracking margins and
            # actor radius: safety cannot depend on its instantaneous phase.
            horizontal = max(0., abs(x-actor['center_x'])-actor['amplitude_x'])
            vertical = max(minimum_y-y, y-maximum_y, 0.)
            assert math.hypot(horizontal, vertical) - .18 - .28 - .20 >= .10
    entry = poses[exit_index+1]
    assert (entry.x, entry.y) == pytest.approx((5.1, -2.5))
    assert math.remainder(entry.yaw-math.pi, 2*math.pi) == pytest.approx(0.)
    last_arc, handoff = poses[count-2:count]
    bearing = math.atan2(handoff.y-last_arc.y, handoff.x-last_arc.x)
    assert math.remainder(bearing-last_arc.yaw, 2*math.pi) == pytest.approx(0.)
