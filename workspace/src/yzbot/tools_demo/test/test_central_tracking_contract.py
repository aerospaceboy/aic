"""Central tracking must preserve hard guards and prove requested arrivals."""
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock
import xml.etree.ElementTree as ET

from action_msgs.msg import GoalStatus
import pytest
from rclpy.time import Time
import yaml

from tools_demo.navigation_math import ZonePose
from tools_demo.task_navigator import TaskNavigator

REPO = Path(__file__).resolve().parents[2]


def test_central_profile_changes_tracking_not_collision_or_cost_protection():
    params = yaml.safe_load((REPO/'bot_navigation/param/originbot_nav2.yaml').read_text())
    control = params['controller_server']['ros__parameters']
    base, central = control['FollowPath'], control['CentralTransit']
    changed = {key for key in base if base[key] != central[key]}
    assert changed == {'desired_linear_vel', 'lookahead_dist', 'min_lookahead_dist',
                       'max_lookahead_dist', 'lookahead_time'}
    assert base['use_rotate_to_heading'] is False
    assert central['use_rotate_to_heading'] is False
    assert central['use_collision_detection'] is True
    assert central['max_allowed_time_to_collision_up_to_carrot'] == .65
    assert central['desired_linear_vel'] == 1.0
    assert control['general_goal_checker']['xy_goal_tolerance'] == .20
    assert control['central_goal_checker']['xy_goal_tolerance'] == .15
    # A short carrot must not truncate the unchanged collision projection.
    # Include the stage cap's 0.05 m/s measured tracking allowance.
    for i in range(1051):
        speed = i/1000
        carrot = max(central['min_lookahead_dist'], min(
            central['max_lookahead_dist'], speed*central['lookahead_time']))
        assert carrot >= speed*central['max_allowed_time_to_collision_up_to_carrot']
    planning = params['planner_server']['ros__parameters']
    assert planning['GridBased']['tolerance'] == .5
    assert planning['CentralExact']['tolerance'] == 0.
    assert planning['GridBased']['cost_travel_multiplier'] == 8.
    assert planning['CentralExact']['cost_travel_multiplier'] == .1
    changed_planner = {'tolerance', 'cost_travel_multiplier'}
    assert {k: v for k, v in planning['CentralExact'].items() if k not in changed_planner} \
        == {k: v for k, v in planning['GridBased'].items() if k not in changed_planner}


def test_only_central_trees_use_exact_goal_and_local_tracking_profile():
    for name in ('navigate_central_to_pose.xml', 'navigate_through_central_adaptive.xml'):
        tree = ET.parse(REPO/'tools_demo/config'/name)
        assert tree.find('.//IsPathValid') is not None
        assert tree.find('.//FollowPath').attrib['controller_id'] == 'CentralTransit'
        assert tree.find('.//FollowPath').attrib['goal_checker_id'] == 'central_goal_checker'
        for tag in ('ComputePathToPose', 'ComputePathThroughPoses'):
            assert all(node.attrib['planner_id'] == 'CentralExact'
                       for node in tree.findall('.//'+tag))
    ordinary = ET.parse(REPO/'bot_navigation/config/navigate_to_pose_recovery.xml')
    assert ordinary.find('.//FollowPath').attrib['controller_id'] == 'FollowPath'
    assert ordinary.find('.//FollowPath').attrib['goal_checker_id'] == 'general_goal_checker'
    assert ordinary.find('.//ComputePathToPose').attrib['planner_id'] == 'GridBased'


def test_two_goal_checkers_cannot_make_normal_or_default_trees_ambiguous():
    normal = ET.parse(REPO/'tools_demo/config/navigate_through_poses_strict.xml')
    assert normal.find('.//FollowPath').attrib['goal_checker_id'] == 'general_goal_checker'
    launch = (REPO/'bot_navigation/launch/competition_navigation.launch.py').read_text()
    assert '"default_nav_through_poses_bt_xml"' in launch
    assert 'f"{tools_share}/config/navigate_through_poses_strict.xml"' in launch


def test_red2_batch_ends_before_shelf_entry_without_dropping_it():
    west = ZonePose('CENTRAL_MOVER_GATE_WEST', -.6, -4.7, 0.)
    east = ZonePose('CENTRAL_MOVER_GATE_EAST', 1., -4.7, 3.14)
    entry = ZonePose('entry_east', 1.9, -5.4, 3.14)
    node = SimpleNamespace(
        active_zone='RED_CUBE_2', pending_poses=[west, east, entry],
        _wait_for_action_server=lambda: True, nav_batch_attempted=False,
        stage_index=0, latest_x=-1.2, _send_pose=Mock(), _send_pose_batch=Mock())
    TaskNavigator._send_next_goal(node)
    node._send_pose_batch.assert_called_once_with([west, east])
    assert node.pending_poses == [entry]
    assert node.nav_goal_pose == east


@pytest.mark.parametrize('x,y,arrived', [
    (1.39, -5.07, False), (.82, -4.77, True), (1., -4.7, True),
    (None, -4.7, False), (float('nan'), -4.7, False)])
def test_nearby_planner_success_is_not_a_real_gate_arrival(x, y, arrived):
    node = SimpleNamespace(nav_goal_pose=ZonePose('CENTRAL_MOVER_GATE_EAST', 1., -4.7, 0.),
                           latest_x=x, latest_y=y)
    assert TaskNavigator._central_anchor_arrived(node) is arrived


@pytest.mark.parametrize('retry', [True, False])
def test_unproven_central_success_cannot_advance_to_the_tray(retry):
    node = SimpleNamespace(
        nav_handoff_requested=False, goal_timed_out=False, obstacle_reverse_active=False,
        progress_recovery_pending=False, goal_handle=Mock(),
        nav_goal_pose=ZonePose('CENTRAL_MOVER_GATE_EAST', 1., -4.7, 0.),
        latest_x=1.39, latest_y=-5.07, _retry_nav_goal=Mock(return_value=retry),
        get_logger=lambda: Mock(), publish_state=Mock(), publish_error=Mock(),
        _clear_goal=Mock(), _continue_staged_navigation=Mock())
    future = SimpleNamespace(result=lambda: SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED))
    TaskNavigator.result_callback(node, future)
    node._retry_nav_goal.assert_called_once()
    node._continue_staged_navigation.assert_not_called()
    assert node._clear_goal.called is not retry


@pytest.mark.parametrize('stage,uses_central', [
    ('CENTRAL_MOVER_GATE_EAST', True), ('central_red2_north_entry', True),
    ('A_WALL_CENTRAL_CLEAR', True), ('A_WALL_RED2_NORTH_CLEAR', True),
    ('BLUE_CUBE_3', False), ('entry_east', True), ('A_WALL_NORTH_CLEAR', False)])
@pytest.mark.parametrize('legacy', [False, True])
@pytest.mark.parametrize('current_dock', [False, True])
def test_single_pose_scopes_central_tracking_to_gates_and_current_red2_entry(stage, uses_central, legacy, current_dock):
    action = Mock()
    node = SimpleNamespace(
        _wait_for_action_server=lambda: True, _lethal_start_cost=lambda: None,
        _publish_stage_speed_limit=Mock(), _log_costmap_probe=Mock(),
        latest_x=-.6, latest_y=-4.7, latest_yaw=0.,
        get_clock=lambda: SimpleNamespace(now=lambda: Time()), get_logger=lambda: Mock(),
        active_zone='RED_CUBE_2', use_legacy_layout_routes=legacy,
        dock_pose=ZonePose('entry_east', 1.9, -5.4, 3.14) if current_dock else ZonePose('entry_east', 2., -5., 0.),
        next_dock_pose=ZonePose('dock_east', 1.36, -5.4, 3.14),
        stage_index=1, stage_total=3,
        goal_from_zone=TaskNavigator.goal_from_zone, action_client=action,
        through_behavior_tree='/tmp/config/through.xml')
    TaskNavigator._send_pose(node, ZonePose(stage, 1., -4.7, 0.))
    goal = action.send_goal_async.call_args.args[0]
    expected = uses_central and not (stage in ('entry_east', 'central_red2_north_entry') and (legacy or not current_dock))
    if stage in ('A_WALL_CENTRAL_CLEAR', 'A_WALL_RED2_NORTH_CLEAR') and legacy:
        expected = False
    assert goal.behavior_tree.endswith('navigate_central_to_pose.xml') is expected


def test_current_red2_entry_keeps_the_proven_northeast_tray_clearance():
    # Match command_callback: before navigation dock_pose is the selected
    # entry, not the later physical grasp pose. This distinction previously
    # made the new branch pass isolated tests but never activate live.
    node = SimpleNamespace(use_legacy_layout_routes=False, get_logger=lambda: Mock(),
                           dock_pose=ZonePose('entry_east', 1.9, -5.4, 3.14),
                           next_dock_pose=ZonePose('dock_east', 1.36, -5.4, 3.14))
    poses = TaskNavigator._cube_approach_waypoints(node, 'RED_CUBE_2', 0., 0.)
    assert [p.zone for p in poses[-3:]] == [
        'CENTRAL_MOVER_GATE_WEST', 'CENTRAL_MOVER_GATE_EAST',
        'central_red2_north_entry']
    model = ET.parse(REPO/'mybot_description/worlds/offic_room.world').find(
        ".//model[@name='competition_red_pickup_tray_2']")
    assert list(map(float, model.findtext('pose').split()))[:2] == [1., -5.4]
    assert list(map(float, model.findtext('link/collision/geometry/box/size').split())) == [.24, .24, .10]
    # Actual tray remains at (1,-5.4); do not keep the abandoned relocation
    # branch at x=.9 when the physical cube is back at its original slot.
    import math
    chain = [(1., -4.7), (1.9, -4.7), (1.9, -5.4)]
    for a, b in zip(chain, chain[1:]):
        for i in range(201):
            x, y = a[0]+(b[0]-a[0])*i/200, a[1]+(b[1]-a[1])*i/200
            clearance = math.hypot(max(abs(x-1.)-.12, 0.), max(abs(y+5.4)-.12, 0.))-.28-.20
            assert clearance >= .10-1e-9
    unproven = SimpleNamespace(nav_goal_pose=ZonePose('central_red2_north_entry', 1.9, -4.7, 0.), latest_x=1.9, latest_y=-5.)
    assert not TaskNavigator._central_anchor_arrived(unproven)
