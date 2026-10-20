"""Contract tests for the coStudio parse-result display evidence."""

import json
import time
from types import SimpleNamespace
from unittest.mock import Mock

from std_msgs.msg import String
from tools_demo.legacy_costudio_status import LegacyCoStudioStatus
from tools_demo.llm_client import LlmClientError
from tools_demo.problem_solver import problem_to_plan
from tools_demo.task_parser_node import TaskParserNode


QUESTION = (
    "每个衣柜有5件衣物;有T恤、裤子两种衣柜;小美需要2件T恤,1条裤子;"
    "小帅、小丽都需要7件T恤;小娜需要2件T恤,1条裤子;小杰需要1条裤子。"
    "设T恤衣柜数量为x,裤子衣柜数量为y,计算x、y分别为多少。"
)
RULES = "x代表红色,y代表蓝色,数量大于等于3的去A区,其他颜色去C区"
PLAN = problem_to_plan(QUESTION, RULES)
GOOD = json.dumps({"items": [item.to_dict() for item in PLAN.items]})


class RecordingPublisher:
    """Capture published payloads without a ROS graph."""

    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message.data)

    def last_json(self):
        return json.loads(self.messages[-1])


class ParserHarness:
    """Bind the parser callbacks to a node-free harness."""

    problem_callback = TaskParserNode.problem_callback
    parse_problem = TaskParserNode.parse_problem
    _validate_problem_llm = TaskParserNode._validate_problem_llm
    publish_plan = TaskParserNode.publish_plan
    begin_parse = TaskParserNode.begin_parse
    publish_parse_result = TaskParserNode.publish_parse_result

    def __init__(self, enabled=True, required=True):
        self.use_llm = enabled
        self.require_llm = required
        self.model = "test-model"
        self.base_url = "http://127.0.0.1:1/v1"
        self.last_problem = None
        self.last_question = None
        self.display_request = {}
        self.get_logger = Mock(return_value=Mock())
        self.publish_llm_status = Mock()
        self.plan_publisher = RecordingPublisher()
        self.counts_publisher = RecordingPublisher()
        self.order_publisher = RecordingPublisher()
        self.parse_result_publisher = RecordingPublisher()
        self.request_llm_problem = Mock(return_value=GOOD)

    def submit(self, task_id="evidence-1", question=QUESTION, rules=RULES):
        self.problem_callback(SimpleNamespace(data=json.dumps({
            "question": question, "mapping_rules": rules,
            "task_id": task_id,
        })))


def test_successful_parse_publishes_display_evidence():
    harness = ParserHarness()
    harness.submit("evidence-ok")
    evidence = harness.parse_result_publisher.last_json()
    assert evidence["state"] == "published"
    assert evidence["published"] is True
    assert evidence["source"] == "llm"
    assert evidence["request_id"] == "evidence-ok"
    assert evidence["task_id"] == "evidence-ok"
    assert evidence["question"] == QUESTION
    assert evidence["mapping_rules"] == RULES
    assert evidence["error"] == ""
    items = [(item["color"], item["count"], item["target_zone"])
             for item in evidence["plan"]["items"]]
    assert items == [("red", 4, "A"), ("blue", 1, "C")]


def test_parse_failure_publishes_no_instruction_and_a_reason():
    harness = ParserHarness()
    harness.request_llm_problem.side_effect = LlmClientError(
        "simulated timeout", kind="timeout")
    harness.submit("evidence-fail")
    evidence = harness.parse_result_publisher.last_json()
    assert evidence["state"] == "failed"
    assert evidence["published"] is False
    assert evidence["plan"] is None
    assert evidence["error"]
    assert harness.plan_publisher.messages == []


def test_retry_after_failure_publishes_the_recovered_instruction():
    harness = ParserHarness()
    harness.request_llm_problem.side_effect = LlmClientError(
        "offline", kind="connection")
    harness.submit("evidence-retry")
    harness.request_llm_problem.side_effect = None
    harness.submit("evidence-retry-new-click")
    evidence = harness.parse_result_publisher.last_json()
    assert evidence["state"] == "published"
    assert evidence["request_id"] == "evidence-retry-new-click"
    assert len(harness.plan_publisher.messages) == 1


def test_failed_click_id_cannot_publish_after_transport_replay():
    """A stopped failed timer must not acquire a later successful mission."""
    harness = ParserHarness()
    harness.request_llm_problem.side_effect = LlmClientError(
        "offline", kind="connection")
    harness.submit("failed-click")
    failed = harness.parse_result_publisher.last_json()
    harness.request_llm_problem.side_effect = None
    harness.submit("failed-click")
    assert harness.parse_result_publisher.last_json() == failed
    assert harness.request_llm_problem.call_count == 1
    assert harness.plan_publisher.messages == []


def test_old_failed_id_stays_inert_after_another_click_succeeds():
    """Keep earlier IDs deduplicated rather than only remembering the last."""
    harness = ParserHarness()
    harness.request_llm_problem.side_effect = LlmClientError(
        "offline", kind="connection")
    harness.submit("old-failed-click")
    harness.request_llm_problem.side_effect = None
    harness.submit("new-click")
    published = harness.parse_result_publisher.last_json()
    harness.submit("old-failed-click")
    assert harness.parse_result_publisher.last_json() == published
    assert harness.request_llm_problem.call_count == 2
    assert len(harness.plan_publisher.messages) == 1


def test_no_click_id_legacy_failure_remains_explicitly_retryable():
    """The click-specific rule must not break legacy ID-less callers."""
    harness = ParserHarness()
    harness.request_llm_problem.side_effect = LlmClientError(
        "offline", kind="connection")
    harness.submit(None)
    harness.request_llm_problem.side_effect = None
    harness.submit(None)
    assert len(harness.plan_publisher.messages) == 1


class StatusHarness:
    """Bind the legacy status publisher to a node-free harness."""

    parse_callback = LegacyCoStudioStatus.parse_callback
    targets_callback = LegacyCoStudioStatus.targets_callback
    _track_round_progress = LegacyCoStudioStatus._track_round_progress
    publish_status = LegacyCoStudioStatus.publish_status

    def __init__(self):
        self.parse_result = {}
        self.perception = {}
        self.perception_at = 0.0
        self.question = ""
        self.mapping_rules = ""
        self.plan = {}
        self.mission = {}
        self.navigation = {}
        self.llm = {}
        self.gripper = "idle"
        self.started_at = time.monotonic()
        self.round_task_id = ""
        self.pending_cubes = {}
        self.completed_steps = []
        self.completed_seen = 0
        self.publisher = RecordingPublisher()

    def apply(self, evidence):
        self.parse_callback(String(data=json.dumps(evidence, ensure_ascii=False)))
        self.publish_status()
        return self.publisher.last_json()

    def observe_mission(self, **fields):
        """Feed one mission message and return the rebuilt panel payload."""
        self.mission = {"task_id": "round-1", **fields}
        self.publish_status()
        return self.publisher.last_json()



def _evidence(state, plan):
    return {
        "schema_version": "1.0", "state": state,
        "published": state == "published", "source": "llm",
        "model": "test-model", "request_id": "evidence-1",
        "task_id": "evidence-1" if plan else "",
        "question": QUESTION, "mapping_rules": RULES,
        "plan": plan, "error": "" if plan else "模型解析失败",
    }


def test_panel_payload_carries_the_published_instruction():
    harness = StatusHarness()
    payload = harness.apply(_evidence("published", PLAN.to_dict()))
    assert payload["question"] == QUESTION
    assert payload["llm_state"] == "published"
    assert payload["resources"] == {"red": 4, "blue": 1}
    assert payload["target_zones"] == {"red": "A", "blue": "C"}
    assert payload["objects"] == [
        {"category": "red", "count": 4, "target_region": "A"},
        {"category": "blue", "count": 1, "target_region": "C"},
    ]
    assert [step["target_region"] for step in payload["planned_grasp_order"]] == [
        "A", "A", "A", "A", "C"]
    assert payload["parse_result"]["state"] == "published"


def test_panel_payload_clears_stale_instruction_after_a_failure():
    harness = StatusHarness()
    harness.apply(_evidence("published", PLAN.to_dict()))
    payload = harness.apply(_evidence("failed", None))
    assert payload["llm_state"] == "failed"
    assert payload["error"]
    assert payload["objects"] == []
    assert payload["planned_grasp_order"] == []
    assert payload["resources"] == {"red": 0, "blue": 0}
    assert payload["target_zones"] == {"red": "", "blue": ""}


ROUND = [
    dict(current_index=1, cube_name="red_cube_1", target_zone="A", completed=0),
    dict(current_index=2, cube_name="red_cube_2", target_zone="A", completed=1),
    dict(current_index=3, cube_name="red_cube_3", target_zone="A", completed=2),
    dict(current_index=4, cube_name="red_cube_4", target_zone="A", completed=3),
    dict(current_index=5, cube_name="blue_cube_1", target_zone="C", completed=4),
]


def test_panel_reports_each_finished_round_with_its_cube_name():
    harness = StatusHarness()
    harness.apply(_evidence("published", PLAN.to_dict()))
    for step in ROUND:
        payload = harness.observe_mission(total=5, **step)
    assert [item["cube_name"] for item in payload["actual_grasp_order"]] == [
        "red_cube_1", "red_cube_2", "red_cube_3", "red_cube_4"]
    assert [item["step"] for item in payload["actual_grasp_order"]] == [1, 2, 3, 4]
    assert payload["actual_grasp_order"][0]["target_region"] == "A"
    assert payload["current_grasp_step"] == 5
    assert payload["current_rule"]["category"] == "blue"
    assert payload["current_rule"]["target_region"] == "C"

    final = harness.observe_mission(
        total=5, current_index=5, cube_name="blue_cube_1",
        target_zone="C", completed=5, state="completed")
    assert len(final["actual_grasp_order"]) == 5
    assert final["grasp_rule_compliance"] == "RULE_MATCH"


def test_panel_flags_a_round_that_breaks_the_planned_rule():
    harness = StatusHarness()
    harness.apply(_evidence("published", PLAN.to_dict()))
    payload = harness.observe_mission(
        total=5, current_index=1, cube_name="blue_cube_1",
        target_zone="C", completed=0)
    assert payload["grasp_rule_compliance"] == "IN_PROGRESS"
    payload = harness.observe_mission(
        total=5, current_index=2, cube_name="red_cube_1",
        target_zone="A", completed=1)
    assert payload["actual_grasp_order"][0]["category"] == "blue"
    assert payload["grasp_rule_compliance"] == "RULE_VIOLATION"


def test_panel_shows_the_live_perception_colours():
    harness = StatusHarness()
    harness.apply(_evidence("published", PLAN.to_dict()))
    harness.targets_callback(String(data=json.dumps({
        "schema_version": "1.0",
        "targets": [{"color": "red"}, {"color": "blue"}, {"color": "red"}],
    })))
    payload = harness.publish_status() or harness.publisher.last_json()
    assert payload["visible_targets"] == ["red", "blue"]


def test_panel_progress_tracking_restarts_with_a_new_round():
    harness = StatusHarness()
    harness.apply(_evidence("published", PLAN.to_dict()))
    harness.observe_mission(total=5, **ROUND[0])
    harness.observe_mission(total=5, **ROUND[1])
    assert len(harness.completed_steps) == 1
    harness.mission = {
        "task_id": "round-2", "total": 5, "current_index": 1,
        "cube_name": "blue_cube_2", "target_zone": "A", "completed": 0,
    }
    payload = harness.publish_status() or harness.publisher.last_json()
    assert payload["actual_grasp_order"] == []
    assert payload["current_grasp_step"] == 1
