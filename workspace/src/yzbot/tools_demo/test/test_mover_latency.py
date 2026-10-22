"""Shorter cycles must retain crossing proofs and ordered safety anchors."""

import inspect
import json
import math
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock
import xml.etree.ElementTree as ET

from nav_msgs.msg import Odometry
import pytest
import yaml

from tools_demo.navigation_math import (
    CORRIDOR_PATROL_PERIOD, CORRIDOR_PATROL_SPEED, CORRIDOR_TRANSIT_SPEED,
    ZonePose, corridor_crossing_clear, corridor_lane_release_clear,
)
from tools_demo.task_navigator import TaskNavigator
from tools_demo.safety_limiter import SafetyLimiter

REPO = Path(__file__).resolve().parents[2]


def test_corridor_actor_predictor_and_command_share_one_speed_contract():
    config = yaml.safe_load((REPO/'bot_navigation/config/competition_zones.yaml').read_text())
    for i in (1, 2):
        p = config[f'competition_moving_obstacle_{i}']['ros__parameters']
        assert p['period_sec'] == CORRIDOR_PATROL_PERIOD == 10.181818181818182
        assert 4*p['amplitude_y']/p['period_sec'] == pytest.approx(CORRIDOR_PATROL_SPEED)
        assert CORRIDOR_PATROL_SPEED/p['update_rate_hz'] < .06
        assert p['update_rate_hz'] == p['truth_read_rate_hz'] == 10.
    assert CORRIDOR_TRANSIT_SPEED == 1.10
    assert '0.1,' in inspect.getsource(TaskNavigator.__init__).split(
        'self.obstacle_reverse_timer = self.create_timer(', 1)[1][:40]
    assert 'CORRIDOR_TRANSIT_SPEED' in inspect.getsource(SafetyLimiter.__init__)
    assert 'patrol_speed=CORRIDOR_PATROL_SPEED' in inspect.getsource(
        TaskNavigator._mover_hold_clear)


@pytest.mark.parametrize('x,y', [(9.65, .495), (9.68, .51), (9.50, .55),
                                (9.80, .55), (9.84, .51), (9.84, .59)])
def test_faster_patrol_still_has_robust_admission_windows(x, y):
    # Every relative 10 Hz sample phase must see at least two consecutive
    # admissible observations, including the actual 0.04m wait proof bound.
    # Old 0.20m arrivals are now explicitly refused, not assumed safe.
    delta = CORRIDOR_PATROL_SPEED*.1
    for offset in [i*delta/50 for i in range(50)]:
        admitted = []
        for i in range(40):
            oy = -.8+offset+i*delta
            if oy > .1:
                break
            admitted.append(corridor_lane_release_clear(y, oy, True) and all(
                corridor_crossing_clear(
                    x, y, math.pi, 8.5, oy, -CORRIDOR_PATROL_SPEED,
                    crossing_speed=speed, dispatch_delay=delay,
                    patrol_speed=CORRIDOR_PATROL_SPEED)
                for speed in (.56, .8, .85, CORRIDOR_TRANSIT_SPEED)
                for delay in (.4, .8)))
        assert any(admitted[i] and admitted[i+1]
                   for i in range(len(admitted)-1)), (x, y, offset)


def test_predictor_does_not_silently_reuse_old_patrol_velocity():
    # Speed changes are an input to the full reflected sweep, not only a
    # publication rate or a sign change on the obsolete .225 m/s model.
    # Faster candidate's actual .243 m/s must have been modeled during the
    # rejected 7.4 s trial, rather than its sign being applied to .225 m/s.
    candidate_speed = 1.8/7.4
    assert not corridor_crossing_clear(9.3, .194, math.pi, 8.5, -.11, -.225,
                                        patrol_speed=.225)
    assert corridor_crossing_clear(
        9.3, .194, math.pi, 8.5, -.11, -candidate_speed,
        patrol_speed=candidate_speed)


def test_closer_wait_remains_outside_the_complete_patrol_with_arrival_error():
    from tools_demo.navigation_math import UNIVERSAL_EAST_WAIT_X
    assert UNIVERSAL_EAST_WAIT_X - 8.5 - .20 - .28 - .18 >= .10
    # Westbound arrival stops on the east side of this clear goal. It must
    # finish before entering the independent final limiter's waiting band.
    assert UNIVERSAL_EAST_WAIT_X > 9.75


def test_recorded_early_arrival_cannot_advance_into_patrol_admission():
    pose = ZonePose('universal_east_wait', 9.8, .55, math.pi)
    node = SimpleNamespace(nav_goal_pose=pose, latest_x=9.974, latest_y=.441)
    assert not TaskNavigator._central_anchor_arrived(node)
    node.latest_x, node.latest_y = 9.82, .525
    assert TaskNavigator._central_anchor_arrived(node)
    node.active_route_zone = node.active_return_zone = None
    node.nav_handoff_distance = .20
    node.pending_poses = [ZonePose('universal_mover_1_clear', 7.25, .55, math.pi)]
    assert TaskNavigator._route_handoff_distance(node) == .03
    config = yaml.safe_load((REPO/'bot_navigation/param/originbot_nav2.yaml').read_text())
    p = config['controller_server']['ros__parameters']
    assert p['general_goal_checker']['xy_goal_tolerance'] == .20
    tolerance = p['corridor_wait_goal_checker']['xy_goal_tolerance']
    assert tolerance == .003
    assert math.hypot(.025, .025) + tolerance < .04
    planner = config['planner_server']['ros__parameters']
    assert planner['CorridorWaitExact']['tolerance'] == 0.
    assert {k: v for k, v in planner['CorridorWaitExact'].items() if k != 'tolerance'} == {
        k: v for k, v in planner['GridBased'].items() if k != 'tolerance'}
    # All ordinary tracking, collision prediction and recovery nodes remain.
    normal = ET.parse(REPO/'bot_navigation/config/navigate_to_pose_recovery.xml').getroot()
    wait = ET.parse(REPO/'tools_demo/config/navigate_corridor_wait.xml').getroot()
    follow = wait.find('.//FollowPath')
    assert follow.attrib['goal_checker_id'] == 'corridor_wait_goal_checker'
    follow.attrib['goal_checker_id'] = 'general_goal_checker'
    assert follow.attrib['controller_id'] == 'CorridorApproach'
    follow.attrib['controller_id'] = 'FollowPath'
    for compute in wait.findall('.//ComputePathToPose'):
        assert compute.attrib['planner_id'] == 'CorridorWaitExact'
        compute.attrib['planner_id'] = 'GridBased'
    assert ET.tostring(wait) == ET.tostring(normal)


def test_recorded_quantized_wait_arrival_still_cannot_advance():
    node = SimpleNamespace(nav_goal_pose=ZonePose('universal_east_wait', 9.8, .55, math.pi),
                           latest_x=9.7659, latest_y=.5015)
    assert not TaskNavigator._central_anchor_arrived(node)
    # Reaching the actual endpoint cell center, rather than stopping 3cm
    # before it, satisfies the original proof without enlarging its boundary.
    node.latest_x, node.latest_y = 9.775, .525
    assert TaskNavigator._central_anchor_arrived(node)


def test_central_tree_replans_from_live_prediction_with_tight_anchor_proof():
    tree = ET.parse(REPO/'tools_demo/config/navigate_through_central_adaptive.xml')
    assert tree.find('.//RateController').attrib['hz'] == '2.0'
    assert tree.find('.//PredictionPathClear') is None
    assert tree.find('.//IsPathValid') is not None
    remove = tree.find('.//RemovePassedGoals')
    assert tree.find('.//PipelineSequence/RemovePassedGoals') is not None
    assert tree.find('.//RateController//RemovePassedGoals') is None
    assert float(remove.attrib['radius']) == .20
    assert remove.attrib['robot_base_frame'] == 'base_footprint'
    source = inspect.getsource(TaskNavigator._send_pose_batch)
    assert 'CENTRAL_MOVER_GATE_' in source
    assert 'else self.through_behavior_tree' in source


def test_only_ordered_physical_arrival_can_retire_a_central_anchor():
    poses = [ZonePose('wall', -2., 0., 0.),
             ZonePose('CENTRAL_MOVER_GATE_EAST', .6, -4.55, math.pi),
             ZonePose('dock_entry', -1.1, -6., math.pi)]
    node = SimpleNamespace(nav_batch_poses=poses, nav_batch_proven_count=0,
                           get_clock=lambda: SimpleNamespace(now=lambda:
                               SimpleNamespace(nanoseconds=1_000_000_000)),
                           _request_nav_handoff_if_reached=Mock())
    message = Odometry()
    message.pose.pose.orientation.w = 1.
    # Being near a later anchor cannot skip the unvisited wall anchor.
    message.pose.pose.position.x, message.pose.pose.position.y = .6, -4.55
    TaskNavigator.odom_callback(node, message)
    assert node.nav_batch_proven_count == 0
    message.pose.pose.position.x, message.pose.pose.position.y = -2., .19
    TaskNavigator.odom_callback(node, message)
    assert node.nav_batch_proven_count == 1
    message.pose.pose.position.x, message.pose.pose.position.y = .6, -4.55
    TaskNavigator.odom_callback(node, message)
    assert node.nav_batch_proven_count == 2


def test_central_fallback_retains_every_unproven_anchor():
    poses = [ZonePose('old', 1., 1., 0.),
             ZonePose('CENTRAL_MOVER_GATE_EAST', .6, -4.55, 0.),
             ZonePose('entry', -1.1, -6., 0.)]
    node = SimpleNamespace(
        nav_batch_poses=poses, nav_batch_proven_count=1, pending_poses=[],
        stage_index=3, get_logger=lambda: Mock(), publish_state=Mock(),
        start_pre_navigation_alignment=Mock())
    assert TaskNavigator._fallback_batch_to_serial(node, 'test')
    assert node.pending_poses == poses[1:]


def test_current_blue3_does_not_visit_obsolete_south_bay_before_carts():
    node = SimpleNamespace(
        use_legacy_layout_routes=False, get_logger=lambda: Mock(),
        dock_pose=ZonePose('BLUE_CUBE_3', -1.64, -6., math.pi))
    poses = TaskNavigator._cube_approach_waypoints(node, 'BLUE_CUBE_3', -4.5, 3.75)
    assert [p.zone for p in poses] == [
        'obstacle_2_north_gate', 'obstacle_2_south_gate',
        'central_patrol_north_flank', 'central_patrol_south_flank']


def test_central_arrival_anchors_stay_outside_final_stop_even_with_error():
    from test_dynamic_layout import CONFIG, NAMES
    from tools_demo.dynamic_retreat import distance_to_patrol
    from tools_demo.task_navigator import CENTRAL_MOVER_GATE_WEST, CENTRAL_MOVER_GATE_EAST
    patrols = [CONFIG[name]['ros__parameters']['patrol_points'] for name in NAMES]
    for gate in (CENTRAL_MOVER_GATE_WEST, CENTRAL_MOVER_GATE_EAST):
        assert min(distance_to_patrol((gate.x, gate.y), p) for p in patrols)-.20 > .75


def test_crossing_decelerates_before_anchor_instead_of_overshooting_it():
    from test_mover_crossing_control import harness
    node = harness()
    node.latest_x = node.mover_crossing_exit_x-.10
    TaskNavigator._mover_crossing_step(node)
    cmd = node._publish_direct_velocity.call_args.args[0]
    assert cmd.linear.x == pytest.approx(.20)
    node.latest_x = node.mover_crossing_exit_x-.05
    TaskNavigator._mover_crossing_step(node)
    node._finish_obstacle_reverse.assert_called_once()


@pytest.mark.parametrize('moving,age,clear,release', [
    (0., 0., True, True), (.1, 0., True, False),
    (0., .3, True, False), (0., 0., False, False)])
def test_early_admission_requires_measured_rest_freshness_and_safe_window(
        moving, age, clear, release):
    clock = SimpleNamespace(nanoseconds=800_000_000)
    node = SimpleNamespace(
        _publish_obstacle_velocity=Mock(), mover_hold_blocked=False,
        mover_hold_topic='west', obstacle_positions={'west': (5., -.6)},
        get_clock=lambda: SimpleNamespace(now=lambda: clock),
        latest_x=6.4, latest_y=.55, latest_yaw=math.pi,
        latest_vx=moving, latest_vy=0., latest_odom_received_ns=int((.8-age)*1e9),
        mover_hold_stationary_since_ns=100_000_000,
        mover_hold_started_ns=0, mover_hold_min_ns=1_500_000_000,
        obstacle_reverse_wait_deadline_ns=12_000_000_000,
        mover_admitted=set(), _mover_hold_clear=lambda topic: clear,
        nav_goal_pose=ZonePose('entry', 3.8, .55, math.pi),
        get_logger=lambda: Mock())
    TaskNavigator._mover_hold_step(node)
    assert ('west' in node.mover_admitted) is release


def test_complete_northern_approach_clears_static_and_whole_patrol_sweeps():
    from test_dynamic_layout import boxes, CONFIG, NAMES
    from test_navigation_contract import _point_to_oriented_box
    from tools_demo.dynamic_retreat import distance_to_patrol
    from tools_demo.task_navigator import (
        CENTRAL_PATROL_NORTH_FLANK, CENTRAL_PATROL_SOUTH_FLANK,
        CENTRAL_MOVER_GATE_WEST, CENTRAL_MOVER_GATE_EAST,
    )
    physical = boxes()
    north, south = CENTRAL_PATROL_NORTH_FLANK, CENTRAL_PATROL_SOUTH_FLANK
    west, east = CENTRAL_MOVER_GATE_WEST, CENTRAL_MOVER_GATE_EAST
    chains = [
        [(0., 0.), (north.x, north.y), (south.x, south.y),
         (west.x, west.y), (east.x, east.y)],
        [(-2.1, 0.), (north.x, north.y), (south.x, south.y),
         (west.x, west.y), (east.x, east.y), (west.x, west.y), (-1.1, -6.)],
        # An already verified encounter removes only the redundant loop, not
        # the full-patrol-safe approach or the real tray-clear entry.
        [(-2.1, 0.), (north.x, north.y), (south.x, south.y), (-1.1, -6.)],
    ]
    for chain in chains:
        for a, b in zip(chain, chain[1:]):
            for i in range(201):
                p = (a[0]+(b[0]-a[0])*i/200, a[1]+(b[1]-a[1])*i/200)
                assert min(_point_to_oriented_box(*p, box)-.28-.20
                           for _, box in physical) >= .10
                assert min(distance_to_patrol(p, CONFIG[name]['ros__parameters'][
                    'patrol_points'])-.28-.14-.20 for name in NAMES) >= .10


def test_blue3_cannot_batch_away_its_mandatory_out_and_back_anchors():
    from tools_demo.task_navigator import CENTRAL_MOVER_GATE_EAST, CENTRAL_MOVER_GATE_WEST
    node = SimpleNamespace(
        active_zone='BLUE_CUBE_3', _wait_for_action_server=lambda: True,
        nav_batch_attempted=False, stage_index=0, latest_x=-1.2,
        pending_poses=[CENTRAL_MOVER_GATE_EAST, CENTRAL_MOVER_GATE_WEST,
                       ZonePose('entry', -1.1, -6., math.pi)],
        _send_pose=Mock(), _send_pose_batch=Mock())
    TaskNavigator._send_next_goal(node)
    node._send_pose.assert_called_once_with(CENTRAL_MOVER_GATE_EAST)
    node._send_pose_batch.assert_not_called()


def mission_node():
    return SimpleNamespace(
        central_mission_id=None, central_mission_start_sec=None,
        central_mission_active=False, central_mission_responses=set(),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(
            nanoseconds=100_000_000_000)))


def response_payload(name='crossing_cart_1', stamp=100.):
    return dict(model=name, success=True, response_observed=True,
                response_kind='yield', stamp_sec=100., continuous_response={
                    'distance': 1.2, 'peak_speed': 1.,
                    'response_speed': .5, 'stamp_sec': stamp})


def mission_message(task='one', state='navigating_pickup'):
    return SimpleNamespace(data=json.dumps({'task_id': task, 'state': state}))


def test_repeat_suppression_requires_both_current_task_timestamped_responses():
    node = mission_node()
    TaskNavigator._central_mission_callback(node, mission_message())
    for name in ('crossing_cart_1', 'crossing_cart_2'):
        TaskNavigator._record_central_response(node, response_payload(name))
    assert node.central_mission_responses == {'crossing_cart_1', 'crossing_cart_2'}
    # Repeated status messages must not discard a real response.
    TaskNavigator._central_mission_callback(node, mission_message())
    assert len(node.central_mission_responses) == 2
    TaskNavigator._central_mission_callback(node, mission_message(task='two'))
    assert node.central_mission_responses == set()
    TaskNavigator._record_central_response(node, response_payload(stamp=99.))
    assert not node.central_mission_responses


def test_same_question_id_after_terminal_cannot_reuse_last_missions_response():
    node = mission_node()
    clock = SimpleNamespace(nanoseconds=100_000_000_000)
    node.get_clock = lambda: SimpleNamespace(now=lambda: clock)
    TaskNavigator._central_mission_callback(node, mission_message())
    TaskNavigator._record_central_response(node, response_payload())
    assert node.central_mission_responses == {'crossing_cart_1'}
    TaskNavigator._central_mission_callback(node, mission_message(state='completed'))
    assert not node.central_mission_active
    clock.nanoseconds += 10_000_000_000
    TaskNavigator._central_mission_callback(node, mission_message())
    assert node.central_mission_start_sec == 110.
    assert not node.central_mission_responses
    payload = response_payload()
    payload['stamp_sec'] = 110.  # fresh position, but OLD response evidence
    TaskNavigator._record_central_response(node, payload)
    assert not node.central_mission_responses


def test_all_nonterminal_machine_states_preserve_the_current_encounter_epoch():
    from tools_demo.mission_state_machine import MISSION_STATES
    node = mission_node()
    TaskNavigator._central_mission_callback(node, mission_message())
    TaskNavigator._record_central_response(node, response_payload())
    for state in (set(MISSION_STATES) | {'navigating_departure'}) - {
            'idle', 'completed', 'failed', 'canceled'}:
        TaskNavigator._central_mission_callback(node, mission_message(state=state))
        assert node.central_mission_active, state
        assert node.central_mission_responses == {'crossing_cart_1'}, state
        assert node.central_mission_start_sec == 100., state


def blue2_prefix_node(zone='BLUE_CUBE_2'):
    return SimpleNamespace(
        active_zone=zone, _wait_for_action_server=lambda: True,
        nav_batch_attempted=False, stage_index=0, latest_x=-4.5,
        pending_poses=[ZonePose('obstacle_2_north_gate', -2.1, 1.8, -1.57),
                       ZonePose('obstacle_2_south_gate', -2.1, 0., -1.48),
                       ZonePose('obstacle_2_west_upper_gate', -4.2, 0., -2.58),
                       ZonePose('BLUE_CUBE_2', -10.1, -5.6, math.pi)],
        _send_pose=Mock(), _send_pose_batch=Mock())


def test_blue2_wall_prefix_confirms_every_anchor_without_a_replayable_batch():
    node = blue2_prefix_node()
    expected = list(node.pending_poses)
    while node.pending_poses:
        TaskNavigator._send_next_goal(node)
    assert [call.args[0] for call in node._send_pose.call_args_list] == expected
    node._send_pose_batch.assert_not_called()
    assert node.stage_index == 4


def test_blue2_prefix_fix_does_not_change_other_pickup_batch_policy():
    node = blue2_prefix_node('BLUE_CUBE_4')
    expected = list(node.pending_poses)
    TaskNavigator._send_next_goal(node)
    node._send_pose_batch.assert_called_once_with(expected)
    node._send_pose.assert_not_called()


@pytest.mark.parametrize('key,value', [
    ('success', False), ('response_observed', False),
    ('response_kind', 'none'), ('model', 'unexpected'), ('stamp_sec', 98.)])
def test_missing_stale_or_unknown_response_keeps_encounter_route(key, value):
    node = mission_node()
    TaskNavigator._central_mission_callback(node, mission_message())
    payload = response_payload()
    payload[key] = value
    TaskNavigator._record_central_response(node, payload)
    assert not node.central_mission_responses


@pytest.mark.parametrize('key,value', [
    ('stamp_sec', 99.), ('stamp_sec', 101.), ('stamp_sec', float('nan')),
    ('distance', .4), ('distance', 1.4), ('peak_speed', .7),
    ('response_speed', .8), ('response_speed', -.1)])
def test_unproven_or_unsafe_slowdown_never_suppresses_repeat(key, value):
    node = mission_node()
    TaskNavigator._central_mission_callback(node, mission_message())
    payload = response_payload()
    payload['continuous_response'][key] = value
    TaskNavigator._record_central_response(node, payload)
    assert not node.central_mission_responses


@pytest.mark.parametrize('active,names', [
    (True, {'crossing_cart_1', 'crossing_cart_2'}),
    (True, {'crossing_cart_1'}),
    (False, {'crossing_cart_1', 'crossing_cart_2'})])
def test_task_route_keeps_blue3_flanks_without_response_detours(active, names):
    node = SimpleNamespace(
        use_legacy_layout_routes=False, get_logger=lambda: Mock(),
        dock_pose=ZonePose('BLUE_CUBE_3', -1.64, -6., math.pi),
        central_mission_active=active, central_mission_responses=names)
    poses = TaskNavigator._cube_approach_waypoints(node, 'BLUE_CUBE_3', -4.5, 3.75)
    assert [p.zone for p in poses[:4]] == [
        'obstacle_2_north_gate', 'obstacle_2_south_gate',
        'central_patrol_north_flank', 'central_patrol_south_flank']
    assert not any(p.zone.startswith('CENTRAL_MOVER_GATE_') for p in poses)


def test_flank_safety_anchor_is_serial_even_without_an_encounter_loop():
    poses = [ZonePose('central_patrol_south_flank', -1.2, -4.9, 0.),
             ZonePose('entry', -1.1, -6., math.pi)]
    node = SimpleNamespace(
        _wait_for_action_server=lambda: True, nav_batch_attempted=False,
        pending_poses=list(poses), stage_index=0, latest_x=-1.2,
        _send_pose=Mock(), _send_pose_batch=Mock())
    TaskNavigator._send_next_goal(node)
    node._send_pose.assert_called_once_with(poses[0])
    node._send_pose_batch.assert_not_called()
