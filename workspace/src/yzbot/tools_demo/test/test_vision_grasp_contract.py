"""Grasping must be driven by the wrist camera, not the pre-recorded slots."""

from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
import yaml

from tools_demo.mission_coordinator import MissionCoordinator

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
    config = yaml.safe_load(MISSION.read_text(encoding="utf-8"))
    assert config['competition_mission_coordinator']['ros__parameters'][
        'require_perception'] is True
    source = COORDINATOR.read_text(encoding="utf-8")
    assert 'self.declare_parameter("require_perception", True)' in source
    # The cached-slot shortcut is disabled while perception is required.
    cached = source[source.index("cached_is_graspable = ("):]
    cached = cached[:cached.index("if (", 200)]
    assert "cached_is_graspable = False" in cached


def test_perception_failure_is_reported_loudly():
    source = COORDINATOR.read_text(encoding="utf-8")
    assert "perception_required_slot_fallback_blocked" in source
    assert "get_logger().error" in source


def test_perception_window_allows_several_camera_frames():
    """The camera runs at ~4 Hz, so a one second window was too tight."""
    yaml = MISSION.read_text(encoding="utf-8")
    assert "perception_timeout_sec:" in yaml


def test_required_perception_cannot_fall_back_or_move_toward_a_cached_slot():
    node = SimpleNamespace(require_perception=True, get_logger=lambda: Mock(),
                           _active_job=Mock(), _realign_dock_to_cached_slot=Mock())
    assert not MissionCoordinator._attempt_slot_fallback(node)
    node._active_job.assert_not_called()
    node._realign_dock_to_cached_slot.assert_not_called()


@pytest.mark.parametrize('required', [True, False])
def test_camera_target_cannot_be_replaced_by_a_nearby_slot_in_required_mode(required):
    job = SimpleNamespace(cube_name='red_cube_3')
    node = SimpleNamespace(
        manipulation_active=False, require_perception=required,
        _active_job=lambda: job, _odom_pose=lambda: (0., 0., 0.),
        get_logger=lambda: Mock(), publish_error=Mock(),
        _expected_cube_base_position=Mock(return_value=(.380, .010)),
        _center_target_laterally=Mock(), pick_tool_offset=(.010, .001, .025),
        _send_manipulation_request=Mock())
    action = SimpleNamespace(kind='pick', job=job,
                             target={'position': {'x': .354, 'y': .014, 'z': .026}})
    MissionCoordinator.start_manipulation(node, action)
    target = node._send_manipulation_request.call_args.args[2]
    assert target['x'] == pytest.approx((.354 if required else .380) + .010)
    assert target['y'] == pytest.approx((.014 if required else .010) + .001)
    if required:
        assert target['z'] == pytest.approx(.026 + .025)
    node._center_target_laterally.assert_not_called()


def test_grasp_oriented_scan_keeps_the_cube_inside_the_wrist_camera_view():
    import math

    config = yaml.safe_load(MISSION.read_text())[
        'competition_mission_coordinator']['ros__parameters']
    assert config['scan_tool_orientation'] == config['pick_tool_orientation']
    sx, sy, sz = config['scan_pose']
    assert sy == 0.  # no separate side-axis turn from the upright start
    assert .25 <= sz <= .32
    # URDF: camera is (-.06, 0, -.03) in the downward-facing tool.
    camera = (sx + .06, sy, sz + .03)
    for x in (.285, .354, .430):
        for y in (-.025, 0., .025):
            radius = math.hypot(x - camera[0], y - camera[1])
            assert radius / (camera[2] - .026) < math.tan(math.pi / 6) * 480 / 640
