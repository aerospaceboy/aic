"""Drop completed encounter stages without bypassing route safety anchors."""

import math
import copy
import json
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
import yaml
from action_msgs.msg import GoalStatus
from nav_msgs.msg import OccupancyGrid

from tools_demo.task_navigator import (
    TaskNavigator, ZonePose, static_footprint_line_clear,
)
from tools_demo.transport_routes import (
    CENTRAL_MOVER_GATE_EAST as EAST,
    CENTRAL_MOVER_GATE_WEST as WEST,
    CENTRAL_PATROL_SOUTH_FLANK as FLANK,
    current_pickup_guides,
)

PRUNE = TaskNavigator._prune_blue3_encounter_loop
ENTRY = ZonePose('entry_east', -1.1, -6., math.pi)
CONFIG = yaml.safe_load((Path(__file__).parents[2]
                         / 'bot_navigation/config/competition_zones.yaml')
                        .read_text())
PATROLS = [CONFIG[name]['ros__parameters']['patrol_points']
           for name in ('crossing_cart_1', 'crossing_cart_2')]
FREE_MAP = OccupancyGrid()
FREE_MAP.info.width = FREE_MAP.info.height = 200
FREE_MAP.info.resolution = .1
FREE_MAP.info.origin.position.x = FREE_MAP.info.origin.position.y = -10.
FREE_MAP.info.origin.orientation.w = 1.
FREE_MAP.data = [0] * 40000


def ready_node(start=FLANK, gates=(WEST, EAST, WEST)):
    return SimpleNamespace(
        use_legacy_layout_routes=False, active_zone='BLUE_CUBE_3',
        central_mission_active=True, central_mission_id='current-round',
        central_mission_responses={'crossing_cart_1', 'crossing_cart_2'},
        pending_poses=[*gates, ENTRY], stage_index=4, stage_total=8,
        latest_x=start.x, latest_y=start.y,
        latest_odom_received_ns=100_000_000_000,
        central_patrols=PATROLS, static_map=FREE_MAP,
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(
            nanoseconds=100_100_000_000)),
        get_logger=lambda: Mock(),
        _costmap_segment_clear=Mock(return_value=True),
        start_pre_navigation_alignment=Mock(), _send_pose=Mock(),
        _wait_for_action_server=Mock(return_value=True))


@pytest.mark.parametrize('start,gates', [
    (FLANK, (WEST, EAST, WEST)), (WEST, (EAST, WEST)), (EAST, (WEST,)),
])
def test_completed_responses_retire_the_remaining_loop_at_each_safe_boundary(
    start, gates,
):
    node = ready_node(start, gates)
    assert PRUNE(node)
    assert node.pending_poses == [ENTRY]
    assert node.stage_total == node.stage_index + 1
    node._costmap_segment_clear.assert_called_once_with(
        start.x, start.y, ENTRY, maximum_cost=77)
    assert not PRUNE(node)  # Rechecking cannot discard the real pickup entry.


@pytest.mark.parametrize('key,value', [
    ('central_mission_responses', {'crossing_cart_1'}),
    ('central_mission_responses', {'crossing_cart_2'}),
    ('central_mission_responses', set()),
    ('central_mission_active', False), ('central_mission_id', None),
    ('active_zone', 'RED_CUBE_2'), ('active_zone', 'BLUE_CUBE_2'),
    ('use_legacy_layout_routes', True), ('goal_handle', object()),
    ('nav_handoff_requested', True), ('nav_batch_poses', [WEST]),
    ('obstacle_reverse_active', True),
    ('latest_y', -4.0), ('latest_x', math.nan),
    ('latest_odom_received_ns', None),
    ('latest_odom_received_ns', 99_000_000_000),
    ('latest_odom_received_ns', 101_000_000_000),
    ('central_patrols', []),
    ('static_map', None),
])
def test_incomplete_evidence_or_unsafe_state_keeps_the_original_route(
    key, value,
):
    node = ready_node()
    setattr(node, key, value)
    before = list(node.pending_poses)
    assert not PRUNE(node)
    assert node.pending_poses == before


def test_wall_and_flank_anchors_are_confirmed_before_any_pruning():
    node = ready_node()
    safety = ZonePose('central_patrol_south_flank', -1.2, -4.9, 0.)
    node.pending_poses.insert(0, safety)
    assert not PRUNE(node)
    assert node.pending_poses[0] is safety
    node.pending_poses.pop(0)  # Only the normal completed stage retires it.
    assert PRUNE(node)


def test_unknown_or_blocked_map_cannot_authorize_a_shortcut():
    node = ready_node()
    node._costmap_segment_clear.return_value = False
    before = list(node.pending_poses)
    assert not PRUNE(node)
    assert node.pending_poses == before


def test_full_patrol_geometry_overrides_a_clear_instantaneous_costmap():
    node = ready_node()
    node.central_patrols = [[-1.2, -5.4, -.9, -5.4], PATROLS[1]]
    assert not PRUNE(node)
    node._costmap_segment_clear.assert_not_called()


@pytest.mark.parametrize('pending', [[WEST, EAST], [WEST, ZonePose(
    'unsafe_entry', -1., -3.7, 0.)]])
def test_missing_or_unsafe_pickup_entry_keeps_the_loop(pending):
    node = ready_node()
    node.pending_poses = list(pending)
    assert not PRUNE(node)
    assert node.pending_poses == pending


def test_evidence_during_alignment_recomputes_heading_before_dispatch():
    node = ready_node(WEST, (EAST, WEST))
    TaskNavigator._send_next_goal(node)
    assert node.pending_poses == [ENTRY]
    node.start_pre_navigation_alignment.assert_called_once()
    node._send_pose.assert_not_called()
    node._wait_for_action_server.assert_not_called()


@pytest.mark.parametrize('start', [(2., -5.), (0., -5.), (-4.5, 3.75)])
def test_already_verified_route_has_no_encounter_gates_for_any_start(start):
    poses = current_pickup_guides('BLUE_CUBE_3', *start, ENTRY,
                                  responses_complete=True)
    assert not any(p.zone.startswith('CENTRAL_MOVER_GATE_') for p in poses)
    if start[1] > 0:
        assert any(p.zone == FLANK.zone for p in poses)


def test_verified_responses_do_not_remove_an_unfinished_northeast_approach():
    poses = current_pickup_guides('BLUE_CUBE_3', 2., -1., ENTRY,
                                  responses_complete=True)
    assert FLANK in poses
    assert not any(p.zone.startswith('CENTRAL_MOVER_GATE_') for p in poses)


@pytest.mark.parametrize('start', [
    (-4.5, 3.75), (-1.2, -4.9), (-1.2, -3.5), (2., -1.), (0., -5.),
])
@pytest.mark.parametrize('responses', [False, True])
def test_route_generation_does_not_add_a_response_demonstration(start, responses):
    poses = current_pickup_guides('BLUE_CUBE_3', *start, ENTRY,
                                  responses_complete=responses)
    assert not any(p.zone.startswith('CENTRAL_MOVER_GATE_') for p in poses)
    if start == (-4.5, 3.75):
        assert FLANK in poses


def test_eastern_mid_patrol_start_only_clears_south_before_heading_to_pickup():
    poses = current_pickup_guides('BLUE_CUBE_3', 2., -3.5, ENTRY)
    gates = [p for p in poses if p.zone.startswith('CENTRAL_MOVER_GATE_')]
    assert gates == [EAST]  # Westbound clearance, no west->east->west pass.


def live_node():
    node = ready_node(ZonePose('live', .3, -4.7, 0.), (WEST,))
    node.nav_goal_pose = EAST
    node.goal_handle = Mock()
    node.latest_vx, node.latest_vy, node.latest_yaw = .35, 0., 0.
    node.nav_request_id = 7
    node.goal_timed_out = False
    node.nav_handoff_requested = False
    node._handoff_cancel_callback = Mock()
    node._continue_staged_navigation = Mock()
    node._finish_route_endpoint = Mock()
    node._complete_nav_handoff = lambda: TaskNavigator._complete_nav_handoff(
        node)
    node.publish_state = Mock()
    return node


def test_live_eastward_leg_is_canceled_without_waiting_for_its_endpoint():
    node = live_node()
    pending = list(node.pending_poses)
    assert TaskNavigator._request_blue3_shortcut_handoff(node)
    node.goal_handle.cancel_goal_async.assert_called_once()
    assert node.blue3_shortcut_goal is EAST
    assert node.pending_poses == pending  # Queue changes only after stopping.
    node._continue_staged_navigation.assert_not_called()
    assert not TaskNavigator._request_blue3_shortcut_handoff(node)
    node.goal_handle.cancel_goal_async.assert_called_once()


def test_cancel_ack_alone_does_not_send_a_new_route():
    node = live_node()
    assert TaskNavigator._request_blue3_shortcut_handoff(node)
    TaskNavigator._handoff_cancel_callback(node, Mock())
    assert node.nav_handoff_requested
    assert node.pending_poses == [WEST, ENTRY]
    node._continue_staged_navigation.assert_not_called()


def test_stopped_action_drops_both_live_east_goal_and_queued_west_return():
    node = live_node()
    assert TaskNavigator._request_blue3_shortcut_handoff(node)
    future = Mock()
    future.result.return_value.status = GoalStatus.STATUS_CANCELED
    TaskNavigator.result_callback(node, future, request_id=7)
    assert node.pending_poses == [ENTRY]
    assert node.nav_goal_pose is None and node.goal_handle is None
    assert node.blue3_shortcut_goal is None
    assert not node.nav_handoff_requested
    node._continue_staged_navigation.assert_called_once()


def test_map_changed_during_cancel_retains_the_existing_safe_gate_route():
    node = live_node()
    assert TaskNavigator._request_blue3_shortcut_handoff(node)
    node._costmap_segment_clear.return_value = False
    TaskNavigator._complete_nav_handoff(node)
    assert node.pending_poses == [EAST, WEST, ENTRY]
    node._continue_staged_navigation.assert_called_once()


@pytest.mark.parametrize('key,value', [
    ('latest_vx', .8), ('latest_vx', None), ('latest_yaw', math.nan),
    ('central_mission_responses', {'crossing_cart_1'}),
    ('latest_odom_received_ns', 99_000_000_000),
    ('nav_goal_pose', FLANK), ('goal_timed_out', True),
])
def test_live_cutover_keeps_existing_speed_freshness_and_response_guards(
    key, value,
):
    node = live_node()
    setattr(node, key, value)
    assert not TaskNavigator._request_blue3_shortcut_handoff(node)
    node.goal_handle.cancel_goal_async.assert_not_called()


def test_cancel_transport_failure_cannot_mutate_or_restart_the_route():
    node = live_node()
    node.goal_handle.cancel_goal_async.side_effect = RuntimeError(
        'unavailable')
    assert not TaskNavigator._request_blue3_shortcut_handoff(node)
    assert node.pending_poses == [WEST, ENTRY]
    assert not node.nav_handoff_requested
    node._continue_staged_navigation.assert_not_called()


def test_unknown_action_result_never_starts_shortcut_navigation():
    node = live_node()
    assert TaskNavigator._request_blue3_shortcut_handoff(node)
    future = Mock()
    future.result.side_effect = RuntimeError('result unavailable')
    TaskNavigator.result_callback(node, future, request_id=7)
    assert node.pending_poses == [WEST, ENTRY]
    assert node.nav_handoff_requested
    node._continue_staged_navigation.assert_not_called()


def test_explicit_user_cancel_clears_the_automatic_cutover():
    node = live_node()
    node.alignment_target_yaw = None
    assert TaskNavigator._request_blue3_shortcut_handoff(node)
    TaskNavigator.cancel_callback(node, SimpleNamespace(data=True))
    assert node.blue3_shortcut_goal is None
    assert not node.nav_handoff_requested
    node._continue_staged_navigation.assert_not_called()


@pytest.mark.parametrize('cost', [-1, 77, 100])
def test_static_body_tube_blocks_wall_cells_off_the_centerline(cost):
    grid = copy.deepcopy(FREE_MAP)
    # This cell is beside the chord, inside the body + tracking + margin.
    col, row = 93, 45
    grid.data[row * grid.info.width + col] = cost
    assert not static_footprint_line_clear(
        grid, (FLANK.x, FLANK.y), (ENTRY.x, ENTRY.y))
    node = ready_node()
    node.static_map = grid
    assert not PRUNE(node)
    node._costmap_segment_clear.assert_not_called()


def test_soft_dynamic_cost_is_considered_only_after_full_static_clearance():
    node = ready_node()
    assert PRUNE(node)
    assert node._costmap_segment_clear.call_args.kwargs['maximum_cost'] == 77
    assert static_footprint_line_clear(
        FREE_MAP, (FLANK.x, FLANK.y), (ENTRY.x, ENTRY.y))


def response(name, distance=1.1, stamp=100.):
    return {'model': name, 'success': True, 'stamp_sec': 100.,
            'response_kind': 'yield', 'response_observed': True,
            'continuous_response': {'stamp_sec': stamp, 'distance': distance,
                                    'peak_speed': 1.1, 'response_speed': .6}}


@pytest.mark.parametrize('models', [
    ('competition_moving_obstacle_1', 'competition_moving_obstacle_2'),
    ('competition_moving_obstacle_2', 'crossing_cart_1'),
    ('crossing_cart_1', 'crossing_cart_2'),
])
def test_two_distinct_current_responses_match_the_actual_acceptance_policy(
    models,
):
    node = ready_node()
    node.central_mission_responses = set()
    node.central_mission_start_sec = 100.
    for model in models:
        TaskNavigator._record_central_response(node, response(model))
    assert node.mission_effective_responses == set(models)
    assert TaskNavigator._blue3_responses_complete(node)
    assert PRUNE(node)


def test_original_response_replay_cannot_carry_over_to_another_task():
    node = ready_node()
    node.central_mission_responses = set()
    node.central_mission_start_sec = 100.
    payload = response('competition_moving_obstacle_2')
    TaskNavigator._record_central_response(node, payload)
    assert node.mission_effective_responses == {
        'competition_moving_obstacle_2'}
    message = SimpleNamespace(data=json.dumps(
        {'task_id': 'next-round', 'state': 'navigating_pickup'}))
    TaskNavigator._central_mission_callback(node, message)
    payload['stamp_sec'] = 100.1  # Fresh position, previous task's slowdown.
    TaskNavigator._record_central_response(node, payload)
    assert not node.mission_effective_responses
    assert not TaskNavigator._blue3_responses_complete(node)


def test_original_second_slowdown_outside_encounter_band_needs_real_approach():
    node = ready_node()
    node.central_mission_responses = set()
    node.central_mission_start_sec = 100.
    payload = response('competition_moving_obstacle_2', distance=1.39)
    payload.update(x=5., y=-.5)
    node.latest_x, node.latest_y = 7., .55
    TaskNavigator._record_central_response(node, payload)
    assert not getattr(node, 'mission_effective_responses', set())
    node.latest_x = 5.5
    TaskNavigator._record_central_response(node, payload)
    assert node.mission_effective_responses == {
        'competition_moving_obstacle_2'}
