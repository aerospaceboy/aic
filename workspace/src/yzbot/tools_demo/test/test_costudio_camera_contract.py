"""Contract tests for the two physical camera views in CoStudio."""

import json
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LAYOUT_PATH = PACKAGE_ROOT / "config" / "costudio_reference_layout.json"
BRIDGE_PATH = PACKAGE_ROOT / "launch" / "costudio_bridge.launch.py"

BASE_CAMERA_TOPIC = "/camera/image_raw"
WRIST_RGBD_TOPIC = "/camera_rgbd/camera_rgbd/image_raw"
DERIVED_DETECTION_TOPIC = "/competition/perception/debug_image"


def _vision_configs():
    document = json.loads(LAYOUT_PATH.read_text(encoding="utf-8"))
    return {
        panel_id: config
        for panel_id, config in document["configById"].items()
        if ".vision-image!" in panel_id
    }


def test_two_vision_panels_show_two_distinct_physical_cameras():
    """Never show raw and annotated frames from the same camera as two cameras."""
    configs = _vision_configs()
    topics = {config["topic"] for config in configs.values()}
    assert len(configs) == 2
    assert topics == {BASE_CAMERA_TOPIC, WRIST_RGBD_TOPIC}
    assert DERIVED_DETECTION_TOPIC not in topics


def test_camera_panel_titles_identify_their_physical_mounts():
    """Operators must be able to tell the base and wrist views apart."""
    configs = _vision_configs()
    by_topic = {config["topic"]: config for config in configs.values()}
    assert "底盘" in by_topic[BASE_CAMERA_TOPIC]["title"]
    assert "腕部" in by_topic[WRIST_RGBD_TOPIC]["title"]


def test_bridge_exposes_both_physical_camera_topics():
    """The CoStudio bridge must forward both camera publishers."""
    source = BRIDGE_PATH.read_text(encoding="utf-8")
    assert r'^/camera/image_raw$' in source
    assert r'^/camera_rgbd/camera_rgbd/image_raw$' in source
