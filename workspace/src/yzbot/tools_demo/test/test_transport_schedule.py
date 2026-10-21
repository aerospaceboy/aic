"""Exercise remaining-task optimization at real coordinator boundaries."""

from collections import Counter
import json
import math
from pathlib import Path
from types import MethodType, SimpleNamespace
from unittest.mock import Mock

import pytest
import yaml

from tools_demo.mission_coordinator import CUBE_WORLD_SLOTS, MissionCoordinator
from tools_demo.mission_state_machine import MissionJob, MissionStateMachine
from tools_demo.task_protocol import TaskItem, TaskPlan
from tools_demo.transport_schedule import (
    TransportTimeEstimator, route_estimate,
)
from tools_demo.transport_schedule import load_timing_profile
from tools_demo.unordered_schedule import optimize_schedule

ROOT = Path(__file__).resolve().parents[2]


def coordinator(monkeypatch):
    """Use real task state, candidates and the production solver."""
    monkeypatch.setattr(
        'tools_demo.mission_coordinator.get_package_share_directory',
        lambda name: str(
            ROOT / name))
    config = ROOT / 'tools_demo/config/mission.yaml'
    params = yaml.safe_load(config.read_text())[
        'competition_mission_coordinator']['ros__parameters']
    machine = MissionStateMachine()
    machine.start(TaskPlan('schedule-test', 'rules', (
        TaskItem('red', 3, 'B'), TaskItem('blue', 2, 'A'))))
    node = SimpleNamespace(
        machine=machine, _transport_automatic=True,
        _transport_responses_complete=False,
        _task_order_entries=[], _override_index=0, _override_pending_side=None,
        _cube_done=set(), _cube_excluded=set(), _cube_failures={},
        cube_approach_allowlist=set(params['cube_approach_allowlist']),
        zone_centers={z: params[f'zone_{z.lower()}_center'] for z in 'ABC'},
        zone_nav_poses={z: params[f'zone_{z.lower()}_nav'] for z in 'ABC'},
        place_slot_spacing=.08, place_maximum_abs_y=.16,
        get_parameter=lambda key: SimpleNamespace(value=params[key]),
        has_parameter=lambda key: key in params,
        _odom_pose=lambda: (13.2, -.95, math.pi), get_logger=lambda: Mock())
    for method in ('_cube_approach_candidates', '_schedule_transport_jobs'):
        setattr(
            node,
            method,
            MethodType(
                getattr(
                    MissionCoordinator,
                    method),
                node))
    return node, params


def test_remaining_replan_preserves_deliveries_slots_and_inventory(
        monkeypatch):
    node, params = coordinator(monkeypatch)
    assert node._schedule_transport_jobs()
    first = node.machine.jobs[0]
    prefix = (first,)
    node.machine.job_offset = 1
    node.machine.completed_count = 1
    node._cube_done.add(first.cube_name)
    node._cube_excluded.add('blue_cube_1')
    node._override_index = 1
    node._odom_pose = lambda: (10.33, -5., 0.)
    params['transport_replan_minimum_gain_sec'] = 0.0
    before = Counter((j.color, j.target_zone) for j in node.machine.jobs)
    assert node._schedule_transport_jobs(replan=True)
    assert node.machine.jobs[:1] == prefix
    assert node.machine.completed_count == 1 and node.machine.job_offset == 1
    assert Counter((j.color, j.target_zone)
                   for j in node.machine.jobs) == before
    assert [j.index for j in node.machine.jobs] == [1, 2, 3, 4, 5]
    names = [j.cube_name for j in node.machine.jobs]
    assert len(set(names)) == 5 and 'blue_cube_1' not in names[1:]
    assert 'blue_cube_3' in names


def test_small_gain_keeps_the_current_plan(monkeypatch):
    node, params = coordinator(monkeypatch)
    assert node._schedule_transport_jobs()
    jobs, entries = node.machine.jobs, list(node._task_order_entries)
    params['transport_replan_minimum_gain_sec'] = 1e6
    assert not node._schedule_transport_jobs(replan=True)
    assert node.machine.jobs == jobs and node._task_order_entries == entries


def test_initial_selection_also_preserves_a_legal_incumbent_for_small_gains(
        monkeypatch):
    node, params = coordinator(monkeypatch)
    params['transport_replan_minimum_gain_sec'] = 1e6
    node._cube_order_override_entries = lambda: []
    node._zone_nav_pose = lambda zone: node.zone_nav_poses[zone]
    node.use_legacy_selection_costs = False
    assert MissionCoordinator._schedule_unordered_jobs(node)
    assert node._task_order_entries == [
        'red_cube_2:east', 'blue_cube_1:east', 'blue_cube_3:east',
        'red_cube_3:east', 'red_cube_4:east']


def test_seeded_operator_policy_does_not_get_replaced(monkeypatch):
    node, _ = coordinator(monkeypatch)
    node._cube_order_override_entries = lambda: []
    node._task_order_entries = ['explicit-policy-entry']
    assert not MissionCoordinator._schedule_unordered_jobs(node)
    assert node._task_order_entries == ['explicit-policy-entry']


@pytest.mark.parametrize('state',
                         ['picking',
                          'placing',
                          'verifying',
                          'navigating_destination'])
def test_carrying_and_arm_actions_never_replan(monkeypatch, state):
    node, _ = coordinator(monkeypatch)
    node.machine.state = state
    jobs = node.machine.jobs
    assert not node._schedule_transport_jobs(replan=True)
    assert node.machine.jobs == jobs


def test_operator_order_is_preserved(monkeypatch):
    node, _ = coordinator(monkeypatch)
    node._transport_automatic = False
    assert not node._schedule_transport_jobs()


def test_encounter_cube_cannot_be_dropped_to_reduce_cost():
    jobs = [MissionJob(1, 1, 'blue', 'blue_cube_1', 'A')]
    options = {'blue': [('blue_cube_1', 'east', 1),
                        ('blue_cube_3', 'east', 100)]}
    _, entries, _ = optimize_schedule(
        jobs, options, (0., 0., 0.), {'A': (0., 0., 0.)},
        lambda pose, candidate, zone: candidate,
        required_cubes=('blue_cube_3',))
    assert entries == ['blue_cube_3:east']
    with pytest.raises(ValueError, match='unavailable'):
        optimize_schedule(jobs, options, (0., 0., 0.), {'A': (0., 0., 0.)},
                          lambda *args: 1, required_cubes=('blue_cube_5',))


def test_transition_accounts_for_occupied_slots_and_terminal_exit():
    jobs = [MissionJob(i, 2, 'red', f'red_cube_{i}', 'A') for i in (1, 2)]
    options = {'red': [('red_cube_1', 'east', 1), ('red_cube_2', 'east', 2)]}
    seen = []

    def transition(pose, candidate, zone, slot, final):
        seen.append((pose, slot, final))
        return candidate + (0 if final else 10), (slot, 2., 0.)

    _, entries, cost = optimize_schedule(
        jobs, options, (5., 5., 0.), {}, None, transition=transition,
        zone_counts={'A': 1})
    assert cost == 13 and len(entries) == 2
    assert ((5., 5., 0.), 1, False) in seen
    assert ((1., 2., 0.), 2, True) in seen


def test_estimator_charges_the_actual_long_guides_and_egress(monkeypatch):
    node, _ = coordinator(monkeypatch)
    config = ROOT / 'bot_navigation/config/navigation_risk_zones.yaml'
    strategies = yaml.safe_load(config.read_text())['zone_strategies']
    estimator = TransportTimeEstimator(
        node.zone_nav_poses, strategies, CUBE_WORLD_SLOTS, {
            'A': 2, 'B': 3, 'C': 0})
    candidate = node._cube_approach_candidates(
        'blue_cube_3', CUBE_WORLD_SLOTS['blue_cube_3'])[0]
    poses = estimator.pickup_poses((-4.53, 3.6, math.pi), candidate)
    assert not any(p.zone.startswith('CENTRAL_MOVER_GATE_') for p in poses)
    assert any(p.zone == 'central_patrol_south_flank' for p in poses)
    normal = estimator.components(
        (-4.53, 3.6, math.pi), candidate, 'A', 0, False)
    final = estimator.components(
        (-4.53, 3.6, math.pi), candidate, 'A', 0, True)
    assert normal['egress'] > 0 and final['egress'] == 0
    assert estimator.zone_dock('A', 0).y == pytest.approx(3.64)
    assert estimator.zone_dock('A', 1).y == pytest.approx(3.96)
    assert estimator.zone_exit('B', estimator.zone_dock('B', 0))[
        :2] == pytest.approx((10.3, -5.))
    assert math.isinf(route_estimate(node.machine.jobs, [],
                      {}, (0, 0, 0), estimator.transition))


@pytest.mark.parametrize('epoch,task,responses,expected', [
    (9., 'round', ['crossing_cart_1', 'crossing_cart_2'], False),
    (11., 'old-round', ['crossing_cart_1', 'crossing_cart_2'], False),
    (11., 'round', ['crossing_cart_1'], False),
    (11., 'round', ['crossing_cart_1', 'crossing_cart_2'], True),
])
def test_response_proof_is_scoped_to_current_task_epoch(
        epoch, task, responses, expected):
    node = SimpleNamespace(
        machine=SimpleNamespace(
            state='navigating_pickup'),
        navigation_location='BLUE_CUBE_3',
        _transport_automatic=True,
        active_task_id='round',
        _transport_epoch_sec=10.,
        _transport_responses_complete=False)
    payload = {
        'zone': 'BLUE_CUBE_3', 'state': 'active', 'encounter_task_id': task,
        'encounter_epoch_sec': epoch, 'verified_central_responses': responses}
    MissionCoordinator.navigation_state_callback(
        node, SimpleNamespace(data=json.dumps(payload)))
    assert node._transport_responses_complete is expected


def test_stale_geometry_cannot_load_a_timing_table(tmp_path):
    geometry = tmp_path / 'geometry.py'
    geometry.write_text('changed layout')
    profile = tmp_path / 'profile.json'
    profile.write_text(json.dumps(
        {'schema_version': 1,
         'geometry_sha256': {'geometry': 'old'}, 'samples': []}))
    with pytest.raises(ValueError, match='does not match'):
        load_timing_profile(profile, {'geometry': geometry})


def test_timing_measurement_cannot_transfer_across_an_unrelated_route(
        monkeypatch):
    node, _ = coordinator(monkeypatch)
    config = ROOT / 'bot_navigation/config/navigation_risk_zones.yaml'
    strategies = yaml.safe_load(config.read_text())['zone_strategies']
    estimator = TransportTimeEstimator(
        node.zone_nav_poses, strategies, CUBE_WORLD_SLOTS, {
            'A': 2, 'B': 3, 'C': 0})
    candidate = node._cube_approach_candidates(
        'blue_cube_3', CUBE_WORLD_SLOTS['blue_cube_3'])[0]
    start = (-4.53, 3.6, math.pi)
    poses = estimator.pickup_poses(start, candidate)
    estimator.timing_samples = [{
        'phase': 'pickup', 'start_pose': list(start),
        'waypoints_xy': [[p.x, p.y] for p in poses],
        'cube': candidate.cube, 'side': candidate.side,
        'zone': 'A', 'elapsed_sec': 30.}]
    matched, _ = estimator.calibrated_seconds(
        'pickup', start, poses, candidate, 'A', 20.)
    assert 20. < matched < 30.
    assert estimator.calibrated_seconds('pickup', (10.3, -5., 0.), poses,
                                        candidate, 'A', 20.)[0] == 20.
    assert estimator.calibrated_seconds('pickup', start, poses[:-1],
                                        candidate, 'A', 20.)[0] == 20.


def test_shared_spawn_delay_applies_to_other_initial_cube_choices(monkeypatch):
    node, _ = coordinator(monkeypatch)
    config = ROOT / 'bot_navigation/config/navigation_risk_zones.yaml'
    strategies = yaml.safe_load(config.read_text())['zone_strategies']
    estimator = TransportTimeEstimator(
        node.zone_nav_poses, strategies, CUBE_WORLD_SLOTS, {
            'A': 2, 'B': 3, 'C': 0})
    from tools_demo.navigation_math import dynamic_gate_waypoints
    start = (13.2, -.95, math.pi)
    prefix = dynamic_gate_waypoints(start, (0., 0.))
    estimator.timing_samples = [{
        'phase': 'spawn', 'start_pose': list(start),
        'waypoints_xy': [[p.x, p.y] for p in prefix],
        'cube': 'red_cube_2', 'side': 'east', 'zone': 'B', 'elapsed_sec': 40.}]
    differences = []
    for name in ('red_cube_1', 'red_cube_2'):
        candidate = node._cube_approach_candidates(
            name, CUBE_WORLD_SLOTS[name])[0]
        measured = estimator.components(
            start, candidate, 'B', 0, False)['pickup']
        samples, estimator.timing_samples = estimator.timing_samples, []
        prior = estimator.components(start, candidate, 'B', 0, False)['pickup']
        estimator.timing_samples = samples
        differences.append(measured-prior)
    assert differences[0] > 0 and differences[0] == pytest.approx(
        differences[1])
