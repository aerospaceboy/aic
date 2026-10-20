"""Unit tests for phase-aware speed classification."""

from tools_demo.speed_profile_recorder import classify_motion, percentile


def classify(
    *,
    mission_state="navigating_destination",
    phase="nav2",
    linear=0.8,
    angular=0.0,
    clearance=2.0,
    smoother=0.8,
    collision=0.8,
    final=0.8,
):
    return classify_motion(
        mission_state,
        phase,
        linear,
        angular,
        clearance,
        smoother,
        collision,
        final,
    )


def test_motion_categories_keep_safety_and_turning_separate():
    assert classify() == "open_straight"
    assert classify(clearance=0.7) == "constrained_straight"
    assert classify(angular=0.35) == "ordinary_turn"
    assert classify(angular=1.1) == "sharp_turn"
    assert classify(collision=0.5) == "collision_slow"
    assert classify(collision=0.8, final=0.5) == "truth_safety_slow"
    assert classify(phase="direct_docking") == "docking_or_direct"
    assert classify(mission_state="picking") == "manipulation_or_wait"


def test_percentile_uses_nearest_rank_and_handles_empty_input():
    assert percentile([], 0.5) == 0.0
    assert percentile([0.1, 0.4, 0.2, 0.3], 0.5) == 0.2
    assert percentile([0.1, 0.4, 0.2, 0.3], 0.9) == 0.4


def test_delayed_samples_expose_unobserved_wall_time(tmp_path, monkeypatch):
    """A stalled executor must not silently lose its wall-clock delay."""
    from collections import defaultdict
    import json
    from types import SimpleNamespace
    from unittest.mock import Mock

    from tools_demo import speed_profile_recorder as profile

    recorder = SimpleNamespace(
        started_monotonic=0.0,
        last_sample_monotonic=0.0,
        maximum_sample_gap_sec=0.0,
        delayed_sample_count=0,
        finished=False,
        navigation={},
        mission={"state": "navigating_destination"},
        actual_linear=1.0,
        actual_angular=0.0,
        front_clearance=2.0,
        commands=defaultdict(float),
        samples=defaultdict(list),
        duration=defaultdict(float),
        distance=defaultdict(float),
        histogram=defaultdict(lambda: [0.0] * (len(profile.SPEED_BINS) - 1)),
        task_id="delayed_sample",
        output_path=tmp_path / "profile.json",
        get_logger=lambda: Mock(),
    )
    monkeypatch.setattr(profile.time, "monotonic", lambda: 2.0)
    profile.SpeedProfileRecorder.sample(recorder)
    monkeypatch.setattr(profile.time, "monotonic", lambda: 2.25)
    profile.SpeedProfileRecorder.write_report(recorder, "completed")
    report = json.loads(recorder.output_path.read_text())
    assert report["elapsed_sec"] == 2.25
    assert report["timing_coverage"] == {
        "sampled_wall_sec": 0.5,
        "unobserved_wall_sec": 1.75,
        "maximum_sample_gap_sec": 2.0,
        "delayed_sample_count": 1,
    }
    assert sum(item["distance_m"] for item in report["categories"].values()) == 0.5
