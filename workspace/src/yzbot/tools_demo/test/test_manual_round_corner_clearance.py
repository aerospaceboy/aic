"""Interior corners must be able to turn without an unsafe forward arc."""

from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import yaml

from tools_demo.navigation_math import ZonePose
from tools_demo.task_navigator import TaskNavigator

REPO = Path(__file__).resolve().parents[2]


def test_corner_preview_preserves_cruise_collision_projection_and_cost_rules():
    parameters=yaml.safe_load((REPO/'bot_navigation/param/originbot_nav2.yaml').read_text())
    follow=parameters['controller_server']['ros__parameters']['FollowPath']
    assert follow['desired_linear_vel'] == 1.78
    assert follow['min_lookahead_dist'] == .60
    assert follow['max_lookahead_dist'] == 1.20
    assert follow['lookahead_time'] == .80
    assert follow['use_rotate_to_heading'] is False
    assert follow['rotate_to_heading_min_angle'] == .45
    assert follow['use_collision_detection'] is True
    assert follow['max_allowed_time_to_collision_up_to_carrot'] == .65
    assert follow['use_cost_regulated_linear_velocity_scaling'] is True
    assert follow['use_regulated_linear_velocity_scaling'] is True
    assert follow['angular_dist_threshold'] == .45
    # At tracked steady speeds, including 0.05m/s allowance, preview must
    # span the unchanged collision horizon. This is NOT a claim about
    # instantaneous velocity tracking during acceleration.
    for i in range(1831):
        speed=i/1000
        carrot=max(follow['min_lookahead_dist'],min(follow['max_lookahead_dist'],
                   speed*follow['lookahead_time']))
        assert carrot >= speed*.65


def test_rejected_long_serial_red1_route_is_not_retained():
    node=SimpleNamespace(use_legacy_layout_routes=False,get_logger=lambda:Mock(),
                         dock_pose=ZonePose('entry_east',-6.6,-2.8,3.14))
    route=TaskNavigator._cube_approach_waypoints(node,'RED_CUBE_1',-4.55,3.77)
    assert not any(p.zone.startswith('static_clear_') for p in route)
    source=(REPO/'tools_demo/tools_demo/task_navigator.py').read_text()
    assert 'static_clear_B_' not in source
    assert 'blue2_outer_' not in source
    assert 'C_RED2_' not in source  # abandoned relocation, cube still at x=1
