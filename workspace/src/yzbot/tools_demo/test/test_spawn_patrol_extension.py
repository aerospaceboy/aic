"""The positive-only extension must reach the new bound and remain predictable."""

import json
import math
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml
from std_msgs.msg import String

from tools_demo.moving_obstacle import MovingObstacle
from tools_demo.navigation_math import (
    CORRIDOR_PATROL_MIN_Y, CORRIDOR_PATROL_MAX_Y, CORRIDOR_PATROL_PERIOD,
    CORRIDOR_PATROL_SPEED, corridor_crossing_clear, mover_reverse_segment_clear,
)
from test_dynamic_layout import boxes, ROOT
from test_navigation_contract import _point_to_oriented_box

REPO = Path(__file__).resolve().parents[2]
CONFIG = yaml.safe_load((REPO / 'bot_navigation/config/competition_zones.yaml').read_text())


@pytest.mark.parametrize('index,x', [(1, 8.5), (2, 5.)])
def test_real_motion_uses_positive_only_extension_and_early_cycle(index, x):
    p = CONFIG[f'competition_moving_obstacle_{index}']['ros__parameters']
    assert p['center_x'] == x
    assert p['center_y'] - p['amplitude_y'] == pytest.approx(-.8)
    assert p['center_y'] + p['amplitude_y'] == pytest.approx(.6)
    assert CORRIDOR_PATROL_MIN_Y == -.8
    assert CORRIDOR_PATROL_MAX_Y == .6
    assert p['period_sec'] == CORRIDOR_PATROL_PERIOD == 10.181818181818182
    assert p['reset_phase_on_plan'] is True
    assert p['mission_phase_offset_sec'] == (3.54 if index == 1 else .29)
    assert 4 * p['amplitude_y'] / p['period_sec'] == pytest.approx(
        CORRIDOR_PATROL_SPEED)
    assert p['phase_offset_sec'] == (0. if index == 1 else 5.090909090909091)
    node = SimpleNamespace(
        center_x=x, center_y=p['center_y'], amplitude_x=0., amplitude_y=p['amplitude_y'],
        phase_offset_sec=p['phase_offset_sec'],
        motion_phase_offset_sec=p['phase_offset_sec'], period_sec=p['period_sec'],
        motion_elapsed_sec=0., motion_started=False, update_period_sec=.1,
        robot_x=None, robot_y=None, proximity_guard_paused_since=None,
        robot_clearance_stop_distance=0., robot_clearance_release_distance=0.,
        robot_clearance_freeze_timeout_sec=5., proximity_guard_paused=False,
        last_motion_position=None)
    positions = [MovingObstacle._continuous_position(node) for _ in range(225)]
    assert min(y for _, y in positions) == pytest.approx(-.8, abs=.015)
    assert max(y for _, y in positions) == pytest.approx(.6, abs=.015)
    assert all(px == x for px, _ in positions)
    assert all(abs(b[1] - a[1]) / .1 <= CORRIDOR_PATROL_SPEED + 1e-9
               for a, b in zip(positions, positions[1:]))
    physical = boxes()
    radius = float(ROOT.findtext(
        f'./world/model[@name="competition_moving_obstacle_{index}"]'
        '/link/collision/geometry/cylinder/radius'))
    for px, py in positions:
        assert min(_point_to_oriented_box(px, py, box) - radius
                   for _, box in physical) >= .10


def test_admission_accepts_new_north_positions_as_observations_but_refuses_unsafe_entry():
    # A robot already west of the actor may leave. An east-side entry in
    # the same Y band must not be admitted through a returning actor.
    assert corridor_crossing_clear(7., .55, math.pi, 8.5, .55, -.25)
    assert not corridor_crossing_clear(9.3, .55, math.pi, 8.5, .55, -.25)
    assert not corridor_crossing_clear(7., .55, math.pi, 8.5, .65, -.25)


def test_reverse_protects_the_added_north_half_metre():
    assert not mover_reverse_segment_clear((8.5, .65), (8.7, .65), 8.5)
    assert mover_reverse_segment_clear((8.5, 1.3), (8.7, 1.3), 8.5)



def test_plan_reset_reanchors_patrol_phase_and_clears_old_response():
    """A new plan must start both actors from the configured phase."""
    from tools_demo.continuous_response import ContinuousResponse
    node = SimpleNamespace(
        reset_phase_on_plan=True, mission_task_id=None,
        motion_elapsed_sec=7.3, motion_started=True,
        mission_phase_offset_sec=3.54, motion_phase_offset_sec=0.,
        last_motion_position=(8.5, .2), proximity_guard_paused=True,
        proximity_guard_paused_since=1., response_observed=True,
        response_kind='yield', trigger_distance=2.6,
        response_distance=1.3, response_speed=.8,
        get_logger=lambda: SimpleNamespace(info=lambda *_: None))
    MovingObstacle.plan_callback(
        node, String(data=json.dumps({'task_id': 'task-1'})))
    assert node.mission_task_id == 'task-1'
    assert node.motion_elapsed_sec == 0.
    assert node.motion_started is False
    assert node.last_motion_position is None
    assert node.proximity_guard_paused is False
    assert node.response_observed is False
    assert node.response_kind is None
    assert isinstance(node.continuous_response, ContinuousResponse)
    MovingObstacle.plan_callback(
        node, String(data=json.dumps({'task_id': 'task-1'})))
    assert node.motion_elapsed_sec == 0.


def test_plan_reset_is_opt_in_for_non_competition_obstacles():
    node = SimpleNamespace(reset_phase_on_plan=False, mission_task_id=None,
                           motion_elapsed_sec=3.)
    MovingObstacle.plan_callback(
        node, String(data=json.dumps({'task_id': 'task-2'})))
    assert node.motion_elapsed_sec == 3.
    assert node.mission_task_id is None
