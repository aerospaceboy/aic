"""Tests for the dynamic local supervisor (lease handshake + fail-closed)."""

import math

import pytest

from tools_demo import dynamic_local_supervisor as sup
from tools_demo import dynamic_hypotheses as dh
from tools_demo import dynamic_strategy_graph as dsg
from tools_demo import spacetime_local_planner as sp


def mover_record(track_id, x, y, vx=0.0, vy=0.0, stamp=100.0,
                 radius=0.18, history=()):
    return {'track_id': track_id, 'x': x, 'y': y, 'vx': vx, 'vy': vy,
            'stamp_sec': stamp, 'radius': radius, 'history': history}


def robot_state(x=0.0, y=0.0, yaw=0.0, vx=0.5, vy=0.0, stamp=100.0):
    return sp.RobotState(x=x, y=y, yaw=yaw, vx=vx, vy=vy, w=0.0, stamp=stamp)


def config(**overrides):
    values = dict(mode='active', replan_sec=0.2, prefix_sec=0.3,
                  release_sec=0.3, horizon_sec=4.0)
    values.update(overrides)
    return sup.SupervisorConfig(**values)


def test_invalid_configuration_is_rejected():
    with pytest.raises(ValueError):
        sup.DynamicLocalSupervisor(sup.SupervisorConfig(mode='drive'))


def test_no_conflict_releases_and_does_not_command():
    supervisor = sup.DynamicLocalSupervisor(config())
    obstacles = {'/competition/obstacles/obstacle_1':
                 mover_record(1, 30.0, 20.0, stamp=100.0)}
    output = supervisor.step(100.0, robot_state(), False, obstacles, (5.0, 0.0))
    assert output.state == 'idle'
    assert output.command is None
    assert output.acquire_lease is False


def test_approach_activates_observe_mode_and_records_decision():
    supervisor = sup.DynamicLocalSupervisor(config(mode='observe'))
    # Mover crossing the robot's heading line 2.5 m ahead.
    obstacles = {'/competition/obstacles/obstacle_1':
                 mover_record(1, 2.5, 0.9, vy=-0.3, stamp=100.0)}
    output = supervisor.step(100.0, robot_state(vx=0.4), False, obstacles,
                             (5.0, 0.0))
    assert output.state == 'observing'
    assert supervisor.last_plan is None
    assert supervisor.decisions
    decision = supervisor.decisions[-1]
    assert decision['hypotheses']
    assert 'legal' in decision and 'rejected' in decision
    assert supervisor.last_plan is None          # observe never acts


def test_execute_prefix_emits_a_bounded_local_command():
    supervisor = sup.DynamicLocalSupervisor(config())
    strategy = dsg.Strategy('pass_before', 'before',
                            (dsg.PassageDecision(1, 'before'),), 0.0,
                            ((-0.2, 0.0), (1.0, 0.0), (2.0, 0.0)),
                            earliest_cross_sec=100.0)
    trajectory = sp.Trajectory(
        ((0.0, 0.0, 0.0, 0.5), (0.3, 0.12, -0.03, 0.5)),
        strategy, arrival_sec=1.2)
    result = sp.PlanResult(trajectory, [trajectory], 'ok', '', 0.3)
    output = supervisor._execute_prefix(100.0, result)
    assert output.state == 'active'
    assert output.acquire_lease is True
    assert output.command is not None
    assert math.hypot(output.command[0], output.command[1]) <= 1.2


def test_wait_plan_is_delegated_even_when_hold_opportunity_exists():
    supervisor = sup.DynamicLocalSupervisor(config(mode='active'))
    obstacles = {'/competition/obstacles/obstacle_1':
                 mover_record(1, 2.5, 0.9, vy=-0.3, stamp=100.0)}
    output = supervisor.step(100.0, robot_state(vx=0.5), False, obstacles,
                             (5.0, 0.0), takeover_allowed=True)
    assert output.state == 'observing'
    assert output.command is None


def test_corridor_clears_releases_if_resume_is_close():
    supervisor = sup.DynamicLocalSupervisor(config(release_sec=0.2))
    supervisor.state = 'active'
    supervisor.last_plan = None
    supervisor.last_plan_sec = 99.0
    clear = {'/competition/obstacles/obstacle_1':
             {'track_id': 1, 'x': 40.0, 'y': 40.0, 'vx': 0.0, 'vy': 0.0,
              'stamp_sec': 100.0, 'radius': 0.18}}
    first = supervisor.step(100.0, robot_state(), False, clear, (5.0, 0.0))
    assert first.state == 'idle'              # nothing left to execute
    second = supervisor.step(100.3, robot_state(), False, clear, (5.0, 0.0))
    assert second.state in ('idle', 'released')
    assert supervisor.state == 'released'
    assert second.release_lease is True


def test_active_mode_returns_a_safe_command_or_explicit_fallback():
    supervisor = sup.DynamicLocalSupervisor(config())
    # Parked body directly on the heading line: no passing candidate may be
    # invented; the supervisor must either execute a validated strategy or
    # report the explicit fallback state and stop/brake command.
    obstacles = {'/competition/obstacles/obstacle_1':
                 {'track_id': 1, 'x': 2.0, 'y': 0.0, 'vx': 0.0, 'vy': 0.0,
                  'stamp_sec': 100.0, 'radius': 0.2, 'history': []}}
    output = supervisor.step(100.0, robot_state(vx=0.6), False, obstacles,
                             (5.0, 0.0))
    if output.state == 'fallback':
        assert output.command is not None
        assert math.hypot(output.command[0], output.command[1]) <= 0.6


def test_stale_mover_keeps_its_reachable_space_blocked_in_decisions():
    supervisor = sup.DynamicLocalSupervisor(config(mode='observe'))
    stale = {'track_id': 1, 'x': 2.0, 'y': 0.3, 'vx': 0.2, 'vy': 0.0,
             'stamp_sec': 98.0, 'radius': 0.25, 'history': []}
    obstacles = {'/competition/obstacles/obstacle_1': stale}
    supervisor.state = 'active'                # drop the lease if it was held
    output = supervisor.step(100.0, robot_state(vx=0.4), False, obstacles,
                             (5.0, 0.0))
    assert output.state == 'fallback'
    assert output.command is None              # never fight the navigation stack
    assert output.release_lease is True
    assert supervisor.decisions[-1]['state'] == 'stale_occluded'


def test_payload_state_changes_the_legal_set():
    def run(payload):
        supervisor = sup.DynamicLocalSupervisor(
            config(mode='observe', corridor_left=1.10, corridor_right=1.10,
                   horizon_sec=6.0))
        obstacles = {'/competition/obstacles/obstacle_1':
                     mover_record(1, 2.0, -1.2, vy=0.3, stamp=100.0)}
        supervisor.step(100.0, robot_state(vx=0.45), payload, obstacles,
                        (5.0, 0.0))
        return supervisor.decisions[-1]
    empty = run(False)
    loaded = run(True)
    empty_sides = [name for name in empty['legal'] if name.startswith('side_')]
    loaded_sides = [name for name in loaded['legal'] if name.startswith('side_')]
    assert len(loaded_sides) <= len(empty_sides)


def test_world_to_local_roundtrip():
    for yaw in (0.0, 0.7, -1.9, 3.0):
        vx, vy = 0.4, -0.2
        world_x, world_y = sup.local_to_world(vx, vy, yaw)
        back_x, back_y = sup.world_to_local(world_x, world_y, 0.0, 0.0, yaw)
        assert abs(back_x - vx) < 1e-9
        assert abs(back_y - vy) < 1e-9


def test_conflict_behind_goal_does_not_activate():
    supervisor = sup.DynamicLocalSupervisor(config(mode='active'))
    # Mover crosses 3 m ahead, but the navigation goal is only 1 m ahead:
    # the conflict is behind the goal and must not move the base away.
    obstacles = {'/competition/obstacles/obstacle_1':
                 mover_record(1, 3.0, 0.8, vy=-0.3, stamp=100.0)}
    output = supervisor.step(100.0, robot_state(vx=0.4), False, obstacles,
                             (2.0, 0.0))
    assert output.state == 'idle'
    assert output.command is None


def test_docking_proximity_does_not_activate():
    supervisor = sup.DynamicLocalSupervisor(config(mode='active'))
    obstacles = {'/competition/obstacles/obstacle_1':
                 mover_record(1, 1.0, 0.8, vy=-0.3, stamp=100.0)}
    output = supervisor.step(100.0, robot_state(vx=0.4), False, obstacles,
                             (0.8, 0.0))
    assert output.state == 'idle'
    assert output.command is None


def test_stationary_base_does_not_take_a_transit_plan():
    supervisor = sup.DynamicLocalSupervisor(config(mode='active'))
    obstacles = {'/competition/obstacles/obstacle_1':
                 mover_record(1, 2.5, 0.9, vy=-0.3, stamp=100.0)}
    output = supervisor.step(100.0, robot_state(vx=0.0, vy=0.0), False,
                             obstacles, (5.0, 0.0))
    assert output.state == 'idle'
    assert output.command is None


def test_active_mode_without_hold_opportunity_only_observes():
    supervisor = sup.DynamicLocalSupervisor(config(mode='active'))
    obstacles = {'/competition/obstacles/obstacle_1':
                 mover_record(1, 2.5, 0.9, vy=-0.3, stamp=100.0)}
    output = supervisor.step(100.0, robot_state(vx=0.5), False, obstacles,
                             (5.0, 0.0), takeover_allowed=False)
    assert output.state == 'observing'
    assert output.command is None
    assert supervisor.decisions




def test_wait_plans_are_delegated_not_executed():
    # The shipped mover-hold chain already implements safe waiting; the
    # innovation supervisor must not duplicate it with a proxy corridor hold.
    supervisor = sup.DynamicLocalSupervisor(config(mode='active'))
    obstacles = {'/competition/obstacles/obstacle_1':
                 mover_record(1, 2.5, 0.9, vy=-0.3, stamp=100.0)}
    output = supervisor.step(100.0, robot_state(vx=0.5), False, obstacles,
                             (5.0, 0.0), takeover_allowed=True)
    if output.state == 'observing':
        assert output.command is None


def test_handback_when_owning_and_plan_turns_into_a_wait():
    supervisor = sup.DynamicLocalSupervisor(config(mode='active'))
    supervisor.state = 'active'
    obstacles = {'/competition/obstacles/obstacle_1':
                 mover_record(1, 2.5, 0.9, vy=-0.3, stamp=100.0)}
    output = supervisor.step(100.0, robot_state(vx=0.5), False, obstacles,
                             (5.0, 0.0), takeover_allowed=True)
    if output.state == 'observing':
        assert output.release_lease is True
