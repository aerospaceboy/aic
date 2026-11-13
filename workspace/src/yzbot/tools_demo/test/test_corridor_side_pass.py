"""Restore normal transit while retaining physical admission and live stops."""

import math
from types import SimpleNamespace

import pytest

from tools_demo.continuous_response import ContinuousResponse
from tools_demo.corridor_admission import spawn_corridor_admission
from tools_demo.navigation_math import (
    CORRIDOR_PATROL_PERIOD, CORRIDOR_PATROL_SPEED,
    corridor_crossing_clear, triangle_wave,
)
from tools_demo.safety_limiter import SafetyLimiter
from tools_demo.safety_math import compute_scale, spawn_corridor_scale

FIRST = 'competition_moving_obstacle_1'
SECOND = 'competition_moving_obstacle_2'


@pytest.mark.parametrize('model,lane_x', [(FIRST, 8.5), (SECOND, 5.)])
def test_side_front_actor_uses_the_shared_return_sweep_envelope(model, lane_x):
    node = SimpleNamespace(
        robot=(lane_x + .3, .525, math.pi), danger=.75, caution=1.6,
        cone_half=math.radians(65), obstacle_timeout=.75,
        obstacles={model: (lane_x, -.6, 100_000_000_000)},
        obstacle_source_stamps={model: 99.95},
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(
            nanoseconds=100_000_000_000)))
    assert 0. < 1.1 * SafetyLimiter.scale_for_obstacles(node, 1.) < .8


@pytest.mark.parametrize('distance,expected', [
    (.5, 0.), (.75, 0.), (1., .25 / .85), (1.7, 1.),
])
def test_front_obstacle_keeps_the_original_stop_and_caution(distance, expected):
    node = SimpleNamespace(
        robot=(9.8, .55, math.pi), danger=.75, caution=1.6,
        cone_half=math.radians(65), obstacle_timeout=.75,
        obstacles={FIRST: (9.8 - distance, .55, 100_000_000_000)},
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(
            nanoseconds=100_000_000_000)))
    assert SafetyLimiter.scale_for_obstacles(node, 1.) == pytest.approx(expected)


def test_restored_guard_admits_safe_window_but_rejects_the_turnaround():
    assert all(corridor_crossing_clear(
        9.8, .55, math.pi, 8.5, -.60, -CORRIDOR_PATROL_SPEED,
        crossing_speed=speed, dispatch_delay=delay,
        patrol_speed=CORRIDOR_PATROL_SPEED)
        for speed in (.56, .8, .85, 1.1) for delay in (0., .4, .8))
    assert spawn_corridor_admission((9.8, .55, math.pi), (8.5, -.60), -CORRIDOR_PATROL_SPEED)
    assert spawn_corridor_admission((9.8, .55, math.pi), (8.5, -.5), -CORRIDOR_PATROL_SPEED)
    assert not spawn_corridor_admission((9.8, .55, math.pi), (8.5, -.8), -CORRIDOR_PATROL_SPEED)


def actor_y(phase):
    return -.1 + .7 * triangle_wave(phase, CORRIDOR_PATROL_PERIOD)


@pytest.mark.parametrize('initial_phase', [i * CORRIDOR_PATROL_PERIOD / 36 for i in range(36)])
@pytest.mark.parametrize('control_delay', [0., .10])
def test_restored_transit_stays_safe_and_records_only_real_responses(initial_phase, control_delay):
    x, y, yaw = 9.852, .525, math.pi
    admitted = False
    second_admitted = False
    monitors = [ContinuousResponse(2.6, 1.3, .8), ContinuousResponse(2.6, 1.4, .85)]
    dt, actual_speed = .02, 0.
    queued = []
    for index in range(2500):
        elapsed = index * dt
        phases = [initial_phase + elapsed, initial_phase + elapsed + CORRIDOR_PATROL_PERIOD / 2]
        actors = {name: (ox, actor_y(phase)) for name, ox, phase in
                  zip((FIRST, SECOND), (8.5, 5.), phases)}
        velocity = (CORRIDOR_PATROL_SPEED if phases[0] % CORRIDOR_PATROL_PERIOD
                    < CORRIDOR_PATROL_PERIOD / 2 else -CORRIDOR_PATROL_SPEED)
        if not admitted and spawn_corridor_admission(
                (x, y, yaw), actors[FIRST], velocity):
            admitted = True
        assert min(math.hypot(x - ox, y - oy) for ox, oy in actors.values()) >= .56
        if index % 5 == 0:
            for name, monitor in zip((FIRST, SECOND), monitors):
                ox, oy = actors[name]
                monitor.observe(math.hypot(x - ox, y - oy), actual_speed, elapsed)
        if x <= 3.8:
            break
        scale = min(compute_scale(
            (x, y, yaw), actors, .75, 1.6, math.radians(65)),
            spawn_corridor_scale((x, y, yaw), actors))
        # Mirror the real handoff: prove mover 2 while rolling, or stop
        # outside its sweep at the existing x=6.65 safe staging point.
        if admitted and not second_admitted and 5.8 <= x <= 7.2:
            second_velocity = (CORRIDOR_PATROL_SPEED
                               if phases[1] % CORRIDOR_PATROL_PERIOD
                               < CORRIDOR_PATROL_PERIOD / 2
                               else -CORRIDOR_PATROL_SPEED)
            second_admitted = spawn_corridor_admission(
                (x, y, yaw), actors[SECOND], second_velocity,
                rolling=actual_speed > .05)
        blocked_second = admitted and not second_admitted and x <= 6.65 + .02
        queued.append(1.1 * scale if admitted and not blocked_second else 0.)
        target = queued[max(0, len(queued) - 1 - round(control_delay / dt))]
        actual_speed += max(-5. * dt, min(5. * dt, target - actual_speed))
        x -= actual_speed * dt
    else:
        pytest.fail('Crossing did not clear')
    assert actual_speed > 1.09
    # The original detector remains authoritative; a straight full-speed
    # passage does not fabricate a response just to satisfy a count.
    for monitor in monitors:
        proof = monitor.evidence
        if proof is None:
            continue
        assert proof['peak_speed'] - proof['response_speed'] >= .15
        assert proof['distance'] >= .56


@pytest.mark.parametrize('pose,sign', [
    ((8.8, .525, 0.), 1.), ((8.8, -.2, math.pi), 1.),
    ((8.8, .525, math.pi), -1.), ((2., .525, math.pi), 1.),
])
def test_return_sweep_envelope_is_confined_to_the_proved_spawn_lane(pose, sign):
    assert spawn_corridor_scale(pose, {FIRST: (8.5, -.6)}, sign) == 1.0


def test_old_side_observation_cannot_create_a_slowdown():
    node = SimpleNamespace(
        robot=(8.8, .525, math.pi), danger=.70, caution=1.6,
        cone_half=math.radians(50), obstacle_timeout=.75,
        obstacles={FIRST: (8.5, -.6, 98_000_000_000)},
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(
            nanoseconds=100_000_000_000)))
    assert SafetyLimiter.scale_for_obstacles(node, 1.) == 1.0


def test_another_corridor_actor_does_not_change_the_other_obstacle_guard():
    assert spawn_corridor_scale(
        (8.8, .525, math.pi), {'crossing_cart_1': (8.5, -.6)}) == 1.0


@pytest.mark.parametrize('arrival_jitter', [-.6, -.3, 0., .3, .6])
@pytest.mark.parametrize('command_delay', [0., .10])
def test_coordinated_pair_has_two_measured_yields_with_arrival_and_command_jitter(
    arrival_jitter, command_delay,
):
    from pathlib import Path
    import yaml

    config = yaml.safe_load((Path(__file__).resolve().parents[2]
                             / 'bot_navigation/config/competition_zones.yaml').read_text())
    offsets = [config[name]['ros__parameters']['mission_phase_offset_sec']
               for name in (FIRST, SECOND)]
    x, y, speed, dt = 9.85, .525, 0., .02
    start = held_since = 15. + arrival_jitter
    mode, rolling = 'hold_first', False
    second_hold_sec = None
    commands = []
    monitors = [ContinuousResponse(2.6, 1.3, .8),
                ContinuousResponse(2.6, 1.4, .85)]
    for index in range(1500):
        stamp = start + index * dt
        phases = [stamp + offset for offset in offsets]
        actors = {name: (ox, actor_y(phase)) for name, ox, phase in
                  zip((FIRST, SECOND), (8.5, 5.), phases)}
        velocities = [CORRIDOR_PATROL_SPEED
                      if phase % CORRIDOR_PATROL_PERIOD < CORRIDOR_PATROL_PERIOD / 2
                      else -CORRIDOR_PATROL_SPEED for phase in phases]
        for (ox, oy), monitor in zip(actors.values(), monitors):
            distance = math.hypot(x - ox, y - oy)
            assert distance > .56
            if index % 5 == 0:
                monitor.observe(distance, speed, stamp)
        if x <= 3.86:
            break
        target = 0.
        if mode.startswith('hold'):
            name = FIRST if mode == 'hold_first' else SECOND
            vy = velocities[0 if name == FIRST else 1]
            if stamp - held_since >= .5 and spawn_corridor_admission(
                    (x, y, math.pi), actors[name], vy):
                mode = 'cross_first' if name == FIRST else 'cross_second'
                if name == SECOND:
                    second_hold_sec = stamp - held_since
        else:
            if (mode == 'cross_first' and 6.9 <= x <= 7.2 and speed >= .56
                    and spawn_corridor_admission(
                        (x, y, math.pi), actors[SECOND], velocities[1], True)):
                mode, rolling = 'cross_second', True
            if mode == 'cross_first' and x <= 6.71:
                mode, held_since = 'hold_second', stamp
            else:
                exit_x = 6.65 if mode == 'cross_first' else 3.8
                target = min(1.1, 2 * (x - exit_x)) * spawn_corridor_scale(
                    (x, y, math.pi), actors)
        commands.append(target)
        target = commands[max(0, len(commands) - 1 - round(command_delay / dt))]
        speed += max(-5 * dt, min(5 * dt, target - speed))
        x -= speed * dt
    else:
        pytest.fail('Coordinated pair did not clear the spawn corridor')
    assert stamp < 27.5
    assert not rolling
    assert second_hold_sec is not None and 2.0 <= second_hold_sec <= 3.0
    for monitor in monitors:
        assert monitor.evidence is not None
        assert monitor.evidence['peak_speed'] - monitor.evidence['response_speed'] >= .15
