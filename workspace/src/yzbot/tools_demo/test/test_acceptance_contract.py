"""Tests for pure competition acceptance contract helpers."""

from pathlib import Path

import pytest

from tools_demo.acceptance_contract import fully_qualified_node_names
from tools_demo.acceptance_contract import missing_typed_interfaces
from tools_demo.acceptance_contract import parse_transform_specification
from tools_demo.acceptance_contract import successful_full_mission
from tools_demo.acceptance_contract import typed_interface_map
from tools_demo.acceptance_contract import validated_avoidance
from tools_demo.acceptance_contract import validated_llm_plan


REPOSITORY = Path(__file__).parents[2]


def test_node_names_are_fully_qualified():
    assert fully_qualified_node_names(
        [
            ("gazebo", "/"),
            ("detector", "/competition"),
        ]
    ) == {"/gazebo", "/competition/detector"}


def test_typed_interfaces_require_exact_name_and_type():
    available = typed_interface_map(
        [
            ("/plan", ["nav_msgs/msg/Path"]),
            ("/clock", ["rosgraph_msgs/msg/Clock"]),
        ]
    )

    assert missing_typed_interfaces(
        [
            "/plan|nav_msgs/msg/Path",
            "/clock|std_msgs/msg/String",
            "/missing|std_msgs/msg/String",
        ],
        available,
    ) == [
        "/clock|std_msgs/msg/String",
        "/missing|std_msgs/msg/String",
    ]


def test_malformed_interface_contract_is_reported():
    assert missing_typed_interfaces(["/plan"], {}) == [
        "invalid:/plan"
    ]


def test_transform_specification_uses_target_source_order():
    assert parse_transform_specification("map:base_link") == (
        "map",
        "base_link",
    )


def test_malformed_transform_specification_is_rejected():
    with pytest.raises(ValueError):
        parse_transform_specification("map")


def test_llm_and_avoidance_evidence_are_strict():
    assert validated_llm_plan(
        {"available": True, "source": "llm"}
    )
    assert not validated_llm_plan(
        {"available": False, "source": "rules"}
    )
    assert validated_avoidance(
        {
            "obstacles_seen": 2,
            "sample_count": 100,
            "safe": True,
            "collision_detected": False,
            "avoidance_demonstrated": True,
            "avoidance_events": ["spawn_avoidance_1", "spawn_avoidance_2"],
        }
    )
    assert not validated_avoidance(
        {
            "obstacles_seen": 1,
            "sample_count": 100,
            "safe": True,
            "collision_detected": False,
            "avoidance_demonstrated": True,
            "avoidance_events": ["spawn_avoidance_1", "spawn_avoidance_2"],
        }
    )
    assert not validated_avoidance(
        {
            "obstacles_seen": 2,
            "sample_count": 100,
            "safe": False,
            "minimum_clearance": 0.043,
            "collision_detected": False,
            "avoidance_demonstrated": True,
            "avoidance_events": ["spawn_avoidance_1", "spawn_avoidance_2"],
        }
    )


def test_full_mission_rejects_partial_or_rule_based_passes():
    llm = {"available": True, "source": "llm"}
    avoidance = {
        "obstacles_seen": 2,
        "sample_count": 100,
        "safe": True,
        "collision_detected": False,
            "avoidance_demonstrated": True,
            "avoidance_events": ["spawn_avoidance_1", "spawn_avoidance_2"],
    }
    complete = {
        "state": "completed",
        "completed": 5,
        "total": 5,
        "skipped": 0,
    }
    assert successful_full_mission(
        complete,
        require_llm=True,
        llm_status=llm,
        require_avoidance=True,
        avoidance_status=avoidance,
    )
    assert not successful_full_mission(
        {**complete, "completed": 1},
        require_llm=True,
        llm_status=llm,
        require_avoidance=True,
        avoidance_status=avoidance,
    )
    assert not successful_full_mission(
        complete,
        require_llm=True,
        llm_status={"available": False, "source": "rules"},
        require_avoidance=True,
        avoidance_status=avoidance,
    )


def test_case_runner_uses_observable_rotation_only_safety_probe():
    """An all-zero Twist is suppressed and cannot verify the safety chain."""
    source = (
        REPOSITORY
        / "tools_demo"
        / "tools_demo"
        / "competition_case_runner.py"
    ).read_text(encoding="utf-8")

    assert "probe.angular.z = 0.01" in source
    assert "self.safe_velocity_publisher.publish(probe)" in source


def test_seen_actors_without_two_distinct_responses_do_not_pass():
    status = {"obstacles_seen": 4, "sample_count": 100,
              "safe": True, "collision_detected": False}
    assert not validated_avoidance(status)
    assert not validated_avoidance({**status, "avoidance_demonstrated": True,
                                   "avoidance_events": ["same", "same"]})
