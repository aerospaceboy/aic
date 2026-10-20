"""Motion forecasts must use stationary-map geometry, never simulation truth."""
import math

import pytest

from tools_demo.dynamic_prediction import (
    Detection, MotionTracker, circle_detection, scan_detections, merge_cost_cells,
)


def arc(x, y, start, radius=.18):
    return [(x+radius*math.cos(start+i*.04),
             y+radius*math.sin(start+i*.04)) for i in range(45)]


def test_moving_body_plan_stays_outside_unchanged_final_guard():
    tracker = MotionTracker()
    for i in range(7):
        tracker.update(i*.2, [Detection(i*.2*.24, -5, .14)])
    hard = [p for p in tracker.cost_points(1.2) if p[3] == 254]
    assert hard
    assert max(math.hypot(p[0]-.288,p[1]+5) for p in hard)+.28 >= .78
    with pytest.raises(ValueError):
        tracker.cost_points(1.2, planning_clearance=.1)


def test_prediction_union_keeps_hard_cells_and_does_not_repeat_geometry():
    points = [(.01,.01,.25,200),(.02,.02,.25,254),(.06,.01,.25,200)]
    merged = merge_cost_cells(points)
    assert merged == [(.025,.025,.25,254),(.07500000000000001,.025,.25,200)]
    tracker = MotionTracker()
    for i in range(7):
        tracker.update(i*.2, [Detection(i*.2*.24,-5,.14)])
    forecast = tracker.cost_points(1.2)
    keys = [(round(p[0],3),round(p[1],3)) for p in forecast]
    assert len(keys) == len(set(keys))


def test_visible_arc_changes_do_not_move_static_world_center():
    tracker = MotionTracker()
    for i in range(8):
        detection = circle_detection(arc(2, 3, i*.2))
        assert detection.x == pytest.approx(2)
        assert detection.y == pytest.approx(3)
        tracker.update(i*.2, [detection])
    assert not tracker.forecast_points(1.4)


@pytest.mark.parametrize('direction', [-1, 1])
def test_future_marks_extend_in_observed_direction_only(direction):
    tracker = MotionTracker()
    for i in range(6):
        tracker.update(i*.2, [Detection(2, 3+direction*i*.04, .18)])
    track = tracker.snapshot(1)[0]
    assert track['confident']
    assert track['vy'] == pytest.approx(direction*.2)
    points = tracker.forecast_points(1)
    assert points
    assert sum(y for _, y, _ in points)/len(points) == pytest.approx(
        track['y']+direction*.2*.65/2, abs=.01)


def test_reversal_never_forecasts_old_direction():
    tracker = MotionTracker()
    for i in range(6):
        tracker.update(i*.2, [Detection(2, 3+i*.04, .18)])
    tracker.update(1.2, [Detection(2, 3.16, .18)])
    assert not tracker.forecast_points(1.2)


def test_observed_corner_retires_old_heading_and_reacquires_new_segment():
    tracker = MotionTracker()
    for i in range(6):
        tracker.update(i*.2,[Detection(2+i*.04,3,.14)])
    tracker.update(1.2,[Detection(2.2,3.04,.14)])
    assert not tracker.forecast_points(1.2)
    for i in range(1,5):
        tracker.update(1.2+i*.2,[Detection(2.2,3.04+i*.04,.14)])
    track = tracker.snapshot(2)[0]
    assert track['confident']
    assert track['vx'] == pytest.approx(0,abs=1e-10)
    assert track['vy'] == pytest.approx(.2)


def test_expired_and_duplicate_scans_cannot_retain_or_rewrite_forecast():
    tracker = MotionTracker(expiry=.65)
    for i in range(6):
        tracker.update(i*.2, [Detection(2, 3+i*.04, .18)])
    assert not tracker.update(.8, [Detection(30, 30, .18)])
    assert not tracker.forecast_points(1.7)
    tracker.update(1.8, [])
    assert not tracker.tracks


def test_occluded_track_cannot_make_fresh_forecasts():
    tracker = MotionTracker()
    for i in range(6):
        tracker.update(i*.2, [Detection(2, 3+i*.04, .18)])
    tracker.update(1.2, [])
    assert not tracker.forecast_points(1.2)


def test_collinear_board_and_too_short_arcs_are_not_given_fake_velocities():
    assert circle_detection([(i*.02, 0) for i in range(12)]) is None
    assert circle_detection(arc(0, 0, 0)[:3]) is None


def test_clusters_do_not_bridge_missing_rays_or_merge_objects():
    first = [(i, x, y) for i, (x, y) in enumerate(arc(2, 3, 0))]
    second = [(i+60, x, y) for i, (x, y) in enumerate(arc(4, 3, 0))]
    assert len(scan_detections(first+second)) == 2


def test_associations_are_one_to_one():
    tracker = MotionTracker()
    tracker.update(0, [Detection(0, 0, .18), Detection(.2, 0, .18)])
    tracker.update(.2, [Detection(.08, 0, .18), Detection(.12, 0, .18)])
    assert len(tracker.tracks) == 2
    assert all(len(t.samples) == 2 for t in tracker.tracks)


def test_unbounded_prediction_is_rejected():
    with pytest.raises(ValueError):
        MotionTracker(horizon=8)


def test_bounded_step_jitter_expands_occupancy_instead_of_losing_direction():
    tracker = MotionTracker()
    for i, error in enumerate([.00, .025, -.025, .02, -.02, .025, -.025]):
        tracker.update(i*.2,[Detection(2+i*.08+error,3,.14)])
    track = tracker.snapshot(1.2)[0]
    assert track['confident']
    assert track['uncertainty'] > .035
    assert track['uncertainty'] <= .07
    points = tracker.forecast_points(1.2)
    assert min(x for x,_,_ in points) <= track['x']-.25


def test_large_model_disagreement_does_not_become_directional_forecast():
    tracker = MotionTracker()
    for i, error in enumerate([.00, .08, -.08, .08, -.08, .08, -.08]):
        tracker.update(i*.3,[Detection(2+i*.12+error,3,.14)])
    assert not tracker.forecast_points(1.8)


def test_occlusion_retains_reachable_space_without_inventing_old_heading():
    tracker = MotionTracker()
    for i in range(6):
        tracker.update(i*.2,[Detection(2+i*.08,3,.14)])
    tracker.update(1.4,[])
    assert not tracker.forecast_points(1.4)
    points = tracker.occupancy_points(1.4)
    assert points
    # Both directions, including the reversal that a payload may conceal.
    assert min(x for x,_,_ in points) < 2.4-.65
    assert max(x for x,_,_ in points) > 2.4+.65
    assert tracker.occupancy_points(3.1) == []


def test_stationary_or_unconfirmed_circle_cannot_create_occlusion_risk():
    tracker = MotionTracker()
    for i in range(8):
        tracker.update(i*.2,[Detection(2,3,.14)])
    tracker.update(1.6,[])
    assert tracker.occupancy_points(1.6) == []


def test_current_body_remains_hard_but_occluded_future_has_egress_cost_only():
    tracker = MotionTracker()
    for i in range(6):
        tracker.update(i*.2,[Detection(2+i*.08,3,.14)])
    live = tracker.cost_points(1)
    assert any(p[3] == 254 for p in live)
    assert any(p[3] == 200 for p in live)
    tracker.update(1.4,[])
    hidden = tracker.cost_points(1.4)
    assert hidden and all(p[3] == 200 for p in hidden)
    assert tracker.cost_points(3.1) == []
