"""Physical side-pass yielding without a demonstration detour."""

import math
from types import SimpleNamespace

import pytest

from tools_demo.continuous_response import ContinuousResponse
from tools_demo.corridor_admission import spawn_corridor_admission
from tools_demo.navigation_math import corridor_crossing_clear
from tools_demo.safety_limiter import SafetyLimiter
from tools_demo.safety_math import compute_scale, corridor_side_pass_scale

FIRST = 'competition_moving_obstacle_1'
SECOND = 'competition_moving_obstacle_2'


def test_side_caution_depends_on_the_actual_nearby_mover():
    robot = (8.8, .525, math.pi)
    actors = {FIRST: (8.5, -.6)}
    assert compute_scale(robot, actors, .75, 1.6, math.radians(65)) == 1.
    assert 0. < corridor_side_pass_scale(robot, actors, .75, 1.6) < .8 / 1.1
    assert corridor_side_pass_scale(robot, {}, .75, 1.6) == 1.
    assert corridor_side_pass_scale((8.2, .525, math.pi), actors, .75, 1.6) == 1.


def test_late_window_is_rejected_when_added_slowdown_would_miss_its_margin():
    assert all(corridor_crossing_clear(
        9.8, .55, math.pi, 8.5, -.65, -.25,
        crossing_speed=speed, dispatch_delay=delay, patrol_speed=.25)
        for speed in (.56, .8, .85, 1.1) for delay in (0., .4, .8))
    assert not spawn_corridor_admission((9.8, .55, math.pi), (8.5, -.65), -.25)
    assert spawn_corridor_admission((9.8, .55, math.pi), (8.5, -.5), -.25)


@pytest.mark.parametrize('robot,actors', [
    ((8.8, .1, math.pi), {FIRST: (8.5, -.6)}),
    ((8.8, .525, 0.), {FIRST: (8.5, -.6)}),
    ((8.8, .525, math.pi), {'unrelated': (8.5, -.6)}),
    ((8.8, .525, math.pi), {FIRST: (8.0, -.6)}),
    ((8.8, .525, math.pi), {FIRST: (8.5, -1.)}),
])
def test_side_pass_keeps_other_lanes_and_actors_unchanged(robot, actors):
    assert corridor_side_pass_scale(robot, actors, .75, 1.6) == 1.


@pytest.mark.parametrize('age,forward,slows', [
    (.05, True, True), (1., True, False), (.05, False, False),
])
def test_limiter_requires_fresh_truth_for_side_caution(age, forward, slows):
    node = SimpleNamespace(
        robot=(8.8, .525, math.pi), danger=.75, caution=1.6,
        cone_half=math.radians(65), obstacle_timeout=.75,
        obstacles={FIRST: (8.5, -.6, 100_000_000_000)},
        obstacle_source_stamps={FIRST: 100. - age},
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(
            nanoseconds=100_000_000_000)))
    value = SafetyLimiter.scale_for_obstacles(node, 1. if forward else 0.)
    assert (value < 1.) is slows


def actor_y(phase):
    phase = phase % 7.2
    return -.8 + .25 * phase if phase <= 3.6 else 1. - .25 * phase


@pytest.mark.parametrize('initial_phase', [i * .2 for i in range(36)])
@pytest.mark.parametrize('control_delay', [0., .10])
def test_straight_route_produces_two_real_responses_safely(initial_phase, control_delay):
    x, y, yaw = 9.852, .525, math.pi
    admitted = False
    monitors = [ContinuousResponse(2.6, 1.3, .8), ContinuousResponse(2.6, 1.4, .85)]
    dt, actual_speed = .02, 0.
    queued = []
    for index in range(1000):
        elapsed = index * dt
        phases = [initial_phase + elapsed, initial_phase + elapsed + 3.6]
        actors = {name: (ox, actor_y(phase)) for name, ox, phase in
                  zip((FIRST, SECOND), (8.5, 5.), phases)}
        velocity = .25 if phases[0] % 7.2 < 3.6 else -.25
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
        scale = min(compute_scale((x, y, yaw), actors, .75, 1.6, math.radians(65)),
                    corridor_side_pass_scale((x, y, yaw), actors, .75, 1.6))
        queued.append(1.1 * scale if admitted else 0.)
        target = queued[max(0, len(queued) - 1 - round(control_delay / dt))]
        actual_speed += max(-5. * dt, min(5. * dt, target - actual_speed))
        x -= actual_speed * dt
    else:
        pytest.fail('Crossing did not clear')
    assert all(m.evidence is not None for m in monitors)
    for monitor in monitors:
        proof = monitor.evidence
        assert proof['peak_speed'] - proof['response_speed'] >= .15
        assert proof['distance'] >= .56
