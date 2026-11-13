"""Only processed RGB-D pairs may increase the detector processing count."""

import json
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
from tools_demo.color_target_detector import ColorTargetDetector


def detector():
    return SimpleNamespace(
        target_frame='base_link', color_frames=8, depth_frames=8,
        processed_frames=3, skipped_pairs=0,
        targets_publisher=Mock(), debug_publisher=Mock(),
        warn_changed=Mock(), intrinsics=Mock(return_value=object()),
        bridge=SimpleNamespace(
            imgmsg_to_cv2=Mock(side_effect=[
                np.zeros((4, 4, 3), dtype=np.uint8),
                np.full((4, 4), .3, dtype=np.float32)]),
            cv2_to_imgmsg=Mock(return_value=SimpleNamespace(header=None))),
        lookup_rigid_transform=Mock(return_value=(
            (0., 0., 0.), (0., 0., 0., 1.))),
        hsv_ranges={}, maximum_targets_per_color=5,
        last_warning_key=None)


def images():
    header = SimpleNamespace(frame_id='camera', stamp=SimpleNamespace(sec=1, nanosec=0))
    return (SimpleNamespace(header=header),
            SimpleNamespace(header=header, encoding='32FC1'))


def test_heartbeats_preserve_the_actual_processing_count():
    node = detector()
    for _ in range(3):
        ColorTargetDetector.publish_status(node)
    assert node.processed_frames == 3
    messages = node.targets_publisher.publish.call_args_list
    assert all(json.loads(call.args[0].data)['status']['processed_frames'] == 3
               for call in messages)


def test_successful_pair_increments_once_and_heartbeat_reports_it():
    node = detector()
    ColorTargetDetector.image_callback(node, *images())
    assert node.processed_frames == 4
    ColorTargetDetector.publish_status(node)
    data = json.loads(node.targets_publisher.publish.call_args.args[0].data)
    assert data['status']['processed_frames'] == 4


def test_a_pair_without_intrinsics_does_not_count_as_processed():
    node = detector()
    node.intrinsics.return_value = None
    ColorTargetDetector.image_callback(node, *images())
    assert node.processed_frames == 3
    node.targets_publisher.publish.assert_not_called()
