"""Rollback only ordinary navigation, not dynamic avoidance or manipulation."""

import hashlib
import json
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import yaml

from tools_demo.navigation_math import ZonePose
from tools_demo.task_navigator import TaskNavigator

REPO = Path(__file__).resolve().parents[2]


def test_ordinary_planner_and_controller_exactly_match_first_frozen_release():
    params = yaml.safe_load((REPO/'bot_navigation/param/originbot_nav2.yaml').read_text())
    # Canonical parameter hashes of the read-only 20261012 frozen ZIP.
    for group, name, expected in (
        ('controller_server', 'FollowPath',
         '27b960e46455a460bce57417e2a74190354e46729436c0f4f0aae3f1d9f5664b'),
        ('planner_server', 'GridBased',
         '70d3081c1fd5263083891d9b26c89c8d9a14d948a52c94d82c34458bc897ee27'),
    ):
        canonical = json.dumps(params[group]['ros__parameters'][name], sort_keys=True)
        assert hashlib.sha256(canonical.encode()).hexdigest() == expected
    for group in ('local_costmap', 'global_costmap'):
        p = params[group][group]['ros__parameters']
        assert p['prediction_layer']['enabled'] is True
        assert p['prediction_layer']['robot_egress_radius'] == .33
    assert params['controller_server']['ros__parameters']['CentralTransit']['use_collision_detection']


def test_only_confirmed_s_passage_gets_local_waypoints_after_frozen_replay_failed():
    node = SimpleNamespace(use_legacy_layout_routes=False, get_logger=lambda: Mock(),
                           dock_pose=ZonePose('entry_east', -6.6, -2.8, 3.14))
    route = TaskNavigator._cube_approach_waypoints(node, 'RED_CUBE_1', -4.55, 3.77)
    assert len(route) == 4
    assert all(p.zone.startswith('WALL57_POCKET_') for p in route)
    assert not TaskNavigator._cube_approach_waypoints(node, 'RED_CUBE_1', -9., -5.)
    source = (REPO/'tools_demo/tools_demo/task_navigator.py').read_text()
    assert 'def _try_central_backoff_replan' in source
    assert 'pickup_dock_drive_distance' in source
