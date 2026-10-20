"""Behavioral checks for the official-generator submission boundary."""

import importlib.util
import json
from pathlib import Path

import pytest

from tools_demo.dashboard_state import DashboardState


spec = importlib.util.spec_from_file_location(
    "video_submit", Path(__file__).parents[1] / "scripts/video_submit.py"
)
video = importlib.util.module_from_spec(spec)
spec.loader.exec_module(video)


@pytest.mark.parametrize("encoding", ["utf-8", "utf-8-sig", "gb18030"])
def test_official_chinese_output_is_preserved(encoding):
    text = "题目: 每个宠物店有5只宠物;计算x、y分别为多少"
    assert video.decode_question(text.encode(encoding)) == text


def test_empty_generator_never_submits():
    with pytest.raises(ValueError):
        video.decode_question(b"\n")


def test_only_fresh_idle_scene_accepts_submission():
    state = {"mission": {"state": "idle"}, "robot_updated_at": 100}
    video.require_idle(state, 101)
    with pytest.raises(ValueError):
        video.require_idle(state, 110)
    state["mission"] = {"state": "completed", "task_id": "old"}
    with pytest.raises(ValueError):
        video.require_idle(state, 101)


def test_existing_parse_blocks_double_submit():
    state = {"mission": {"state": "idle"}, "robot_updated_at": 100,
             "parse_result": {"state": "parsing"}}
    with pytest.raises(ValueError):
        video.require_idle(state, 101)


def test_old_success_cannot_validate_new_round():
    result = {"request_id": "old", "task_id": "old",
              "state": "published", "source": "llm"}
    assert video.matching_result({"parse_result": result}, "new") is None
    result.update(request_id="new", task_id="new")
    assert video.matching_result({"parse_result": result}, "new") == result
    result["source"] = "rules"
    with pytest.raises(ValueError):
        video.matching_result({"parse_result": result}, "new")


def test_matching_api_failure_is_not_http_success():
    result = {"request_id": "new", "state": "failed", "error": "timeout"}
    with pytest.raises(ValueError, match="timeout"):
        video.matching_result({"parse_result": result}, "new")


def test_dashboard_retains_matching_parse_evidence():
    state = DashboardState()
    evidence = {"request_id": "current", "state": "failed", "error": "401"}
    state.update_json("parse_result", json.dumps(evidence))
    assert state.snapshot()["parse_result"] == evidence


def test_model_solves_without_receiving_computed_answer():
    from types import SimpleNamespace
    from unittest.mock import Mock
    from tools_demo.task_parser_node import TaskParserNode
    from tools_demo.task_parser_node import PROBLEM_SYSTEM_PROMPT

    client = Mock()
    node = SimpleNamespace(model="test", llm_max_tokens=256,
                           llm_enable_thinking=False, llm_client=client)
    # If the prompt accesses candidate items or serialization this object
    # has no such attributes, exposing accidental answer injection.
    TaskParserNode.request_llm_problem(node, "test question", "test rules", object())
    payload = client.complete.call_args.args[0]
    assert payload["messages"][0]["content"] == PROBLEM_SYSTEM_PROMPT
    assert "test question" in payload["messages"][1]["content"]
    assert "test rules" in payload["messages"][1]["content"]
