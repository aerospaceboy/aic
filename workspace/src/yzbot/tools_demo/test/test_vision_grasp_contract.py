"""Grasping must be driven by the wrist camera, not the pre-recorded slots."""

from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
DETECTOR = REPO / "tools_demo" / "tools_demo" / "color_target_detector.py"
COORDINATOR = REPO / "tools_demo" / "tools_demo" / "mission_coordinator.py"
MISSION = REPO / "tools_demo" / "config" / "mission.yaml"


def test_detector_never_blocks_the_frame_path():
    """A blocking synchroniser callback stopped publishing mid-mission."""
    source = DETECTOR.read_text(encoding="utf-8")
    assert "import message_filters" not in source
    assert "ApproximateTimeSynchronizer(" not in source
    assert "registerCallback" not in source
    assert "self.create_timer(0.05, self.process_latest_pair)" in source
    # The newest transform is used first and without waiting.
    assert "timeout=Duration(seconds=0.0)" in source
    assert "def process_latest_pair" in source
    assert "def publish_status" in source


def test_detector_publishes_a_liveness_heartbeat():
    """A silent detector must be visible instead of failing silently."""
    source = DETECTOR.read_text(encoding="utf-8")
    assert "processed_frames" in source
    assert "skipped_pairs" in source
    assert '"status"' in source


def test_stamp_seconds_round_trips():
    from types import SimpleNamespace

    from tools_demo.color_target_detector import stamp_seconds

    stamp = SimpleNamespace(sec=12, nanosec=500000000)
    assert stamp_seconds(stamp) == 12.5


def test_every_pick_requires_perception():
    yaml = MISSION.read_text(encoding="utf-8")
    assert "require_perception:" in yaml
    source = COORDINATOR.read_text(encoding="utf-8")
    assert 'self.declare_parameter("require_perception", True)' in source
    # The cached-slot shortcut is disabled while perception is required.
    cached = source[source.index("cached_is_graspable = ("):]
    cached = cached[:cached.index("if (", 200)]
    assert "cached_is_graspable = False" in cached


def test_perception_failure_is_reported_loudly():
    source = COORDINATOR.read_text(encoding="utf-8")
    assert "perception_required_but_falling_back_to_slot" in source
    assert "get_logger().error" in source


def test_perception_window_allows_several_camera_frames():
    """The camera runs at ~4 Hz, so a one second window was too tight."""
    yaml = MISSION.read_text(encoding="utf-8")
    assert "perception_timeout_sec:" in yaml
