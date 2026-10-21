"""The positive-only extension must reach the new bound and remain predictable."""

import math
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

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
def test_real_motion_uses_positive_only_extension_without_a_speed_increase(index, x):
    p = CONFIG[f'competition_moving_obstacle_{index}']['ros__parameters']
    assert p['center_x'] == x
    assert p['center_y'] - p['amplitude_y'] == pytest.approx(-.8)
    assert p['center_y'] + p['amplitude_y'] == pytest.approx(.6)
    assert CORRIDOR_PATROL_MIN_Y == -.8
    assert CORRIDOR_PATROL_MAX_Y == .6
    assert p['period_sec'] == CORRIDOR_PATROL_PERIOD == 11.2
    assert 4 * p['amplitude_y'] / p['period_sec'] == CORRIDOR_PATROL_SPEED == .25
    assert p['phase_offset_sec'] == (0. if index == 1 else 5.6)
    node = SimpleNamespace(
        center_x=x, center_y=p['center_y'], amplitude_x=0., amplitude_y=p['amplitude_y'],
        phase_offset_sec=p['phase_offset_sec'], period_sec=p['period_sec'],
        motion_elapsed_sec=0., motion_started=False, update_period_sec=.1,
        robot_x=None, robot_y=None, proximity_guard_paused_since=None,
        robot_clearance_stop_distance=0., robot_clearance_release_distance=0.,
        robot_clearance_freeze_timeout_sec=5., proximity_guard_paused=False,
        last_motion_position=None)
    positions = [MovingObstacle._continuous_position(node) for _ in range(225)]
    assert min(y for _, y in positions) == pytest.approx(-.8)
    assert max(y for _, y in positions) == pytest.approx(.6)
    assert all(px == x for px, _ in positions)
    assert all(abs(b[1] - a[1]) / .1 <= .25 + 1e-9
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
