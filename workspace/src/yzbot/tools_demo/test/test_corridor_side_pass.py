"""Restore normal transit while retaining physical admission and live stops."""

import math
from types import SimpleNamespace

import pytest

from tools_demo.continuous_response import ContinuousResponse
from tools_demo.corridor_admission import spawn_corridor_admission
from tools_demo.navigation_math import corridor_crossing_clear
from tools_demo.safety_limiter import SafetyLimiter
from tools_demo.safety_math import compute_scale

FIRST = 'competition_moving_obstacle_1'
SECOND = 'competition_moving_obstacle_2'


@pytest.mark.parametrize('model,lane_x', [(FIRST, 8.5), (SECOND, 5.)])
def test_side_actor_no_longer_adds_a_demonstration_slowdown(model, lane_x):
    node = SimpleNamespace(
        robot=(lane_x + .3, .525, math.pi), danger=.75, caution=1.6,
        cone_half=math.radians(65), obstacle_timeout=.75,
        obstacles={model: (lane_x, -.6, 100_000_000_000)},
        obstacle_source_stamps={model: 99.95},
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(
            nanoseconds=100_000_000_000)))
    assert 1.1 * SafetyLimiter.scale_for_obstacles(node, 1.) == 1.1


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
        9.8, .55, math.pi, 8.5, -.65, -.25,
        crossing_speed=speed, dispatch_delay=delay, patrol_speed=.25)
        for speed in (.56, .8, .85, 1.1) for delay in (0., .4, .8))
    assert spawn_corridor_admission((9.8, .55, math.pi), (8.5, -.65), -.25)
    assert spawn_corridor_admission((9.8, .55, math.pi), (8.5, -.5), -.25)
    assert not spawn_corridor_admission((9.8, .55, math.pi), (8.5, -.8), -.25)


def actor_y(phase):
    phase = phase % 11.2
    return -.8 + .25 * phase if phase <= 5.6 else 2. - .25 * phase


@pytest.mark.parametrize('initial_phase', [i * 11.2 / 36 for i in range(36)])
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
        phases = [initial_phase + elapsed, initial_phase + elapsed + 5.6]
        actors = {name: (ox, actor_y(phase)) for name, ox, phase in
                  zip((FIRST, SECOND), (8.5, 5.), phases)}
        velocity = .25 if phases[0] % 11.2 < 5.6 else -.25
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
        scale = compute_scale((x, y, yaw), actors, .75, 1.6, math.radians(65))
        # Mirror the real handoff: prove mover 2 while rolling, or stop
        # outside its sweep at the existing x=6.65 safe staging point.
        if admitted and not second_admitted and 5.8 <= x <= 7.2:
            second_velocity = .25 if phases[1] % 11.2 < 5.6 else -.25
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
