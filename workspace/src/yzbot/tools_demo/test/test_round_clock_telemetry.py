"""Operator stopwatch endpoints are mission-scoped and immutable."""
import json
import time
from types import SimpleNamespace

import pytest

from tools_demo.mission_coordinator import MissionCoordinator
from tools_demo.legacy_costudio_status import LegacyCoStudioStatus


def coordinator(state, task_id="round-1"):
    """Construct only the telemetry surface, without ROS motion side effects."""
    messages = []
    node = SimpleNamespace(
        machine=SimpleNamespace(
            terminal=state in {"completed", "failed", "canceled"},
            snapshot=lambda: {"state": state, "task_id": task_id},
        ),
        active_task_id=task_id,
        _round_cube_name=None,
        mission_started_at=time.monotonic() - 2,
        mission_finished_at_unix_ms=None,
        state_publisher=SimpleNamespace(publish=messages.append),
        get_logger=lambda: SimpleNamespace(info=lambda _: None),
        get_parameter=lambda _: SimpleNamespace(value=False),
    )
    return node, messages


def test_running_and_idle_have_no_stop_endpoint():
    """Only a task terminal transition captures the wall-clock endpoint."""
    for state in ("idle", "navigating_pickup"):
        node, messages = coordinator(state)
        MissionCoordinator.publish_state(node)
        assert json.loads(messages[-1].data)["finished_at_unix_ms"] is None


@pytest.mark.parametrize("state", ["completed", "failed", "canceled"])
def test_terminal_endpoint_is_captured_once(state):
    """Duplicate snapshots and late dashboard reads preserve the first end."""
    node, messages = coordinator(state)
    before = time.time_ns() // 1_000_000
    MissionCoordinator.publish_state(node)
    finished = json.loads(messages[-1].data)["finished_at_unix_ms"]
    assert before <= finished <= time.time_ns() // 1_000_000
    node.mission_finished_at_unix_ms = finished - 500
    MissionCoordinator.publish_state(node)
    assert json.loads(messages[-1].data)["finished_at_unix_ms"] == finished - 500


def test_legacy_status_separates_parser_plan_from_actual_mission():
    """A new parsed plan cannot make an old final snapshot end the new clock."""
    messages = []
    node = SimpleNamespace(
        parse_result={"task_id": "new-plan", "plan": {"items": []}},
        mission={"task_id": "old-mission", "state": "completed",
                 "finished_at_unix_ms": 123456789},
        question="", mapping_rules="", navigation={}, gripper="idle",
        perception_at=0, perception={}, started_at=time.monotonic(),
        _track_round_progress=lambda _: [],
        publisher=SimpleNamespace(publish=messages.append),
    )
    LegacyCoStudioStatus.publish_status(node)
    payload = json.loads(messages[-1].data)
    assert payload["task_id"] == "new-plan"
    assert payload["mission_task_id"] == "old-mission"
    assert payload["mission_finished_at_unix_ms"] == 123456789


@pytest.mark.parametrize("enforced,wrong_zone,expected", [
    (False, False, "RULE_MATCH"),
    (True, False, "RULE_VIOLATION"),
    (False, True, "RULE_VIOLATION"),
])
def test_dynamic_order_does_not_relax_color_zone_rules(
    enforced, wrong_zone, expected
):
    """Only a mandatory sequence forbids legal dynamic reordering."""
    messages = []
    node = SimpleNamespace(
        parse_result={"plan": {"items": [
            {"color": "red", "count": 1, "target_zone": "A"},
            {"color": "blue", "count": 1, "target_zone": "B"},
        ]}},
        mission={"state": "completed", "total": 2, "completed": 2,
                 "enforce_color_order": enforced},
        question="", mapping_rules="", navigation={}, gripper="idle",
        perception_at=0, perception={}, started_at=time.monotonic(),
        _track_round_progress=lambda _: [
            {"category": "blue", "target_region": "C" if wrong_zone else "B"},
            {"category": "red", "target_region": "A"},
        ],
        publisher=SimpleNamespace(publish=messages.append),
    )
    LegacyCoStudioStatus.publish_status(node)
    payload = json.loads(messages[-1].data)
    assert payload["grasp_rule_compliance"] == expected
    assert payload["current_action"] == "completed"


def test_terminal_snapshot_cannot_erase_last_delivery_zone():
    """The last cube is still displayed after the active job becomes None."""
    node = SimpleNamespace(
        round_task_id="round-1", pending_cubes={5: ("red_cube_5", "C")},
        completed_steps=[], completed_seen=4,
        mission={"task_id": "round-1", "current_index": 5,
                 "cube_name": "red_cube_5", "target_zone": None,
                 "completed": 5},
    )
    steps = LegacyCoStudioStatus._track_round_progress(node, 5)
    assert steps[-1]["target_region"] == "C"
    assert steps[-1]["cube_name"] == "red_cube_5"
