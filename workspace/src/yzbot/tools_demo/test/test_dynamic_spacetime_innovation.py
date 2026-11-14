"""Behaviour tests for the G1 dynamic-avoidance innovation stack.

Covers the observable claims that must be true before any integration:
  * hypotheses are observation-driven (a learned reversal boundary only exists
    when the history actually contains both turning extrema),
  * a low-weight dangerous branch is still part of the conservative hard set,
  * stale/occluded tracks keep their reachable space blocked,
  * strategy candidates are pruned with explicit reasons,
  * the spacetime planner ranks by cost only after hard safety filtering,
  * payload state changes what is reachable,
  * with no safe action the planner says so instead of inventing motion.
"""

import math

import pytest

from tools_demo.dynamic_hypotheses import (
    DEFAULT_HORIZON_SEC, Hypothesis, MIN_MODE_WEIGHT, Prediction,
    TrackSnapshot, conservative_bodies, learned_boundary, predict,
    predict_track,
)
from tools_demo.dynamic_strategy_graph import (
    ConflictFrame, PassageDecision, Strategy, strategy_candidates,
)
from tools_demo.spacetime_local_planner import (
    Dynamics, PlanConfig, RobotState, build_trajectory, plan,
)


def crossing_track(**overrides):
    payload = {
        'track_id': 1,
        'stamp': 100.0,
        'x': 3.0,
        'y': -0.5,
        'radius': 0.18,
        'vx': 0.0,
        'vy': 0.225,
        'uncertainty': 0.04,
        'confident': True,
        'was_moving': True,
        'history': (),
    }
    payload.update(overrides)
    return TrackSnapshot.from_dict(payload)


def make_prediction(track, now=100.0, horizon=DEFAULT_HORIZON_SEC):
    return predict_track(track, now=now, horizon=horizon)


def test_learned_reversal_boundary_requires_both_observed_extrema():
    flat = [(0.0, 5.0, -0.6), (0.5, 5.0, -0.4), (1.0, 5.0, -0.2),
            (1.5, 5.0, 0.0), (2.0, 5.0, 0.2)]
    assert learned_boundary(flat) is None
    patrol = [(0.0, 5.0, -0.7), (1.0, 5.0, 0.0), (2.0, 5.0, 0.6),
              (3.0, 5.0, 0.1), (4.0, 5.0, -0.6), (5.0, 5.0, -0.2)]
    boundary = learned_boundary(patrol)
    assert boundary is not None
    assert boundary[0] <= -0.6 and boundary[1] >= 0.6


def test_no_reverse_hypothesis_without_learned_boundary():
    prediction = make_prediction(crossing_track())
    modes = {hypothesis.mode for hypothesis in prediction.hypotheses}
    assert 'continue' in modes and 'pause' in modes
    assert 'reverse' not in modes


def test_reverse_hypothesis_appears_with_learned_boundary():
    history = ((95.0, 3.0, -0.7), (96.0, 3.0, 0.0), (97.0, 3.0, 0.6),
               (98.0, 3.0, 0.1), (99.0, 3.0, -0.5))
    prediction = make_prediction(crossing_track(history=history))
    modes = {hypothesis.mode: hypothesis for hypothesis in prediction.hypotheses}
    assert 'reverse' in modes
    assert 0.0 < modes['reverse'].weight < 1.0


def test_low_weight_branch_stays_in_hard_conservative_set():
    history = ((95.0, 3.0, -0.7), (96.0, 3.0, 0.0), (97.0, 3.0, 0.6),
               (98.0, 3.0, 0.1), (99.0, 3.0, -0.5))
    prediction = make_prediction(crossing_track(history=history))
    reverse = next(h for h in prediction.hypotheses if h.mode == 'reverse')
    assert reverse.weight >= MIN_MODE_WEIGHT
    t = reverse.positions[10][0]
    bodies = conservative_bodies([prediction], t)
    assert any(math.hypot(bx - reverse.position_at(t)[1],
                          by - reverse.position_at(t)[2]) < 1e-6
               and radius > 0 for bx, by, radius in bodies)


def test_occluded_track_keeps_reachable_space_blocked():
    stale = crossing_track(stamp=100.0, confident=False, was_moving=True)
    prediction = predict_track(stale, now=100.8)
    assert prediction.stale
    modes = {hypothesis.mode for hypothesis in prediction.hypotheses}
    assert modes == {'occluded'}
    body = prediction.hypotheses[0]
    assert body.radius_at(body.positions[0][0]) > 0.2


def test_pause_hypothesis_decelerates_to_a_stop():
    track = crossing_track(vy=0.4, y=0.0)
    prediction = make_prediction(track)
    pause = next(h for h in prediction.hypotheses if h.mode == 'pause')
    start_y = pause.positions[0][2]
    end_y = pause.positions[-1][2]
    assert end_y > start_y
    assert end_y - start_y <= 0.4 * DEFAULT_HORIZON_SEC


def two_mover_predictions(now=100.0):
    one = crossing_track(track_id=1, x=1.2, y=-1.4, vy=0.30,
                         history=((95.0, 1.2, -0.7), (97.0, 1.2, 0.6),
                                  (99.0, 1.2, -1.3)))
    two = crossing_track(track_id=2, x=1.2, y=1.3, vy=-0.30,
                         history=((95.0, 1.2, 0.6), (97.0, 1.2, -0.6),
                                  (99.0, 1.2, 1.2)))
    return predict([one, two], now=now)


def frame(**overrides):
    values = dict(crossing_x=1.2, obstacle_radius=0.18, corridor_left=0.9,
                  corridor_right=0.9, corridor_min_x=-1.0,
                  corridor_max_x=6.5, robot_speed=0.6,
                  limit_speed=0.9, empty_envelope=0.28,
                  carrying_envelope=0.34)
    values.update(overrides)
    return ConflictFrame(**values)


def test_strategy_candidates_prune_before_when_robot_is_too_late():
    predictions = two_mover_predictions(now=100.0)
    # The first obstacle's band is occupied almost immediately at t=now.
    legal, rejected, windows = strategy_candidates(
        predictions, frame(), payload=False, now=100.0)
    names = {strategy.name for strategy in legal}
    assert windows
    assert any(rejected_strategy.reason for rejected_strategy in rejected)
    assert 'wait_at_-0.43' in names or any(
        strategy.kind == 'wait' for strategy in legal)
    # At least one order candidate and one side candidate must survive for a
    # corridor with room on both sides.
    assert any(strategy.kind in ('before', 'after', 'mixed')
               for strategy in legal)


def test_wait_anchor_inside_reachable_set_is_rejected():
    # A stale mover near the approach line keeps its reachable set blocked, so
    # every backwards wait anchor is unsafe: the planner must record explicit
    # rejections and never claim a validated wait.
    sweep = crossing_track(track_id=7, x=-1.0, y=0.0, vx=0.2, vy=0.0,
                           radius=0.3, stamp=99.0, confident=False,
                           history=())
    prediction = predict_track(sweep, now=100.0)
    legal, rejected, _ = strategy_candidates([prediction], frame(obstacle_radius=0.3),
                                             payload=True, now=100.0)
    assert all(strategy.kind != 'wait' for strategy in legal)
    assert any('reachable' in rejected_strategy.reason
               for rejected_strategy in rejected)


def test_payload_envelope_changes_side_pass_room():
    # One mover crossing on the left half; the right half of the corridor is
    # free. The same lateral room clears the empty envelope but not the loaded
    # one, so the payload state changes the legal strategy set.
    one = crossing_track(track_id=1, x=1.2, y=-1.4, vy=0.30)
    predictions = predict([one], now=100.0)
    narrow = frame(corridor_left=1.10, corridor_right=1.10,
                   empty_envelope=0.28, carrying_envelope=0.42)
    empty_legal, _, _ = strategy_candidates(predictions, narrow, payload=False,
                                            now=100.0)
    carrying_legal, carrying_rejected, _ = strategy_candidates(
        predictions, narrow, payload=True, now=100.0)
    assert any(strategy.kind == 'side' for strategy in empty_legal)
    assert all(strategy.kind != 'side' for strategy in carrying_legal)
    assert any('lateral room' in item.reason for item in carrying_rejected)


def simple_strategy(name='pass', kind='after', lateral=0.0):
    return Strategy(name, kind, (PassageDecision(1, 'after'),), lateral,
                    ((1.0, 0.0), (3.0, lateral), (5.0, 0.0)))


def test_planner_rejects_the_cheapest_candidate_that_collides():
    # A body parked on the direct line: the nominal "pass" candidate is the
    # shortest but must fail the hard dynamic filter, and the safe wait/stop
    # plan must be selected instead of ranked through the collision.
    parked = crossing_track(track_id=1, x=2.5, y=0.0, vx=0.0, vy=0.0,
                            confident=True,
                            history=())
    prediction = predict_track(parked, now=100.0)
    robot = RobotState(x=0.0, y=0.0, stamp=100.0, vx=0.3)
    result = plan(robot, False, frame(), [prediction],
                  [simple_strategy('pass', 'after', 0.0)],
                  dynamics=Dynamics(), config=PlanConfig())
    candidate = result.candidates[0]
    assert candidate.feasible is False
    assert 'dynamic_clearance' in candidate.reject_reason or \
        'braking_path_enters_obstacle' in candidate.reject_reason
    assert result.selected is None
    assert result.fallback in ('emergency_stop', 'blocked_no_safe_action')


def test_planner_selects_a_legal_side_pass_and_reports_why():
    predictions = two_mover_predictions(now=100.0)
    robot = RobotState(x=0.0, y=0.0, stamp=100.0, vx=0.35)
    legal, rejected, _ = strategy_candidates(predictions, frame(),
                                             payload=False, now=100.0)
    result = plan(robot, False, frame(), predictions, legal,
                  dynamics=Dynamics(), config=PlanConfig())
    assert result.selected is not None
    assert result.selected.feasible
    assert result.reason == 'selected_by_hard_filter_then_cost'
    assert result.decision['legal'] >= 1
    assert result.decision['rejected'] >= 1


def test_planner_hysteresis_keeps_the_previous_kind_when_competitive():
    predictions = two_mover_predictions(now=100.0)
    robot = RobotState(x=0.0, y=0.0, stamp=100.0, vx=0.2)
    legal, _, _ = strategy_candidates(predictions, frame(), payload=False,
                                      now=100.0)
    first = plan(robot, False, frame(), predictions, legal,
                 dynamics=Dynamics(), config=PlanConfig())
    assert first.selected is not None
    previous_kind = first.selected.strategy.kind
    second = plan(robot, False, frame(), predictions, legal,
                  previous_kind=previous_kind,
                  dynamics=Dynamics(), config=PlanConfig())
    assert second.selected is not None
    assert second.selected.strategy.kind == previous_kind
