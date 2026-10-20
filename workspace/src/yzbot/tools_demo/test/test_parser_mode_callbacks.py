"""Competition parser callbacks must enforce LLM mode and allow retry."""
import json
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from tools_demo.llm_client import LlmClientError

from tools_demo.problem_solver import problem_to_plan
from tools_demo.task_parser_node import TaskParserNode
from tools_demo.task_parser_node import llm_error_is_retryable

QUESTION = (
    "每个衣柜有5件衣物;有T恤、裤子两种衣柜;小美需要2件T恤,1条裤子;"
    "小帅、小丽都需要7件T恤;小娜需要2件T恤,1条裤子;小杰需要1条裤子。"
    "设T恤衣柜数量为x,裤子衣柜数量为y,计算x、y分别为多少。")
RULES = "x代表红色,y代表蓝色,数量大于等于3的去A区,其他颜色去C区"
PLAN = problem_to_plan(QUESTION, RULES)
GOOD = json.dumps({"items": [item.to_dict() for item in PLAN.items]})
REVERSED = json.dumps({"items": [item.to_dict()
                                 for item in reversed(PLAN.items)]})


class Harness:
    begin_parse = TaskParserNode.begin_parse
    publish_parse_result = TaskParserNode.publish_parse_result
    problem_callback = TaskParserNode.problem_callback
    parse_problem = TaskParserNode.parse_problem
    _validate_problem_llm = TaskParserNode._validate_problem_llm
    question_callback = TaskParserNode.question_callback
    parse_question = TaskParserNode.parse_question

    def __init__(self, required=True, enabled=True):
        self.require_llm = required
        self.model = "test-model"
        self.parse_result_publisher = Mock()
        self.use_llm = enabled
        self.last_problem = None
        self.last_question = None
        self.get_logger = Mock(return_value=Mock())
        self.publish_plan = Mock()
        self.publish_llm_status = Mock()
        self.request_llm_problem = Mock(return_value=GOOD)
        self.request_llm = Mock()

    def submit(self, task_id="attempt-1"):
        self.problem_callback(SimpleNamespace(data=json.dumps({
            "question": QUESTION, "mapping_rules": RULES,
            "task_id": task_id})))


@pytest.mark.parametrize("failure", [
    LlmClientError("offline", kind="timeout"), "invalid json",
    '{"items":[{"color":"red","count":4,"target_zone":"C"},'
    '{"color":"blue","count":1,"target_zone":"A"}]}'])
def test_required_web_problem_never_publishes_rules_fallback(failure):
    harness = Harness()
    if isinstance(failure, Exception):
        harness.request_llm_problem.side_effect = failure
    else:
        harness.request_llm_problem.return_value = failure
    harness.submit()
    harness.publish_plan.assert_not_called()
    assert harness.publish_llm_status.call_args.kwargs["source"] == "error"


def test_required_web_problem_rejects_disabled_model():
    harness = Harness(enabled=False)
    harness.submit()
    harness.publish_plan.assert_not_called()


def test_explicit_practice_mode_allows_rules_fallback():
    harness = Harness(required=False, enabled=False)
    harness.submit()
    assert harness.publish_plan.call_args.args[0].source == "rules"


def test_valid_model_solution_preserves_order_and_task_identity():
    harness = Harness()
    harness.submit()
    plan = harness.publish_plan.call_args.args[0]
    assert plan.source == "llm"
    assert plan.items == PLAN.items
    assert plan.task_id == "attempt-1"


def test_correct_reversed_model_order_is_canonicalized_without_fallback():
    """Current goods scoring accepts both orders with exact zones/counts."""
    harness = Harness()
    harness.request_llm_problem.return_value = REVERSED
    harness.submit()
    plan = harness.publish_plan.call_args.args[0]
    assert plan.source == "llm"
    assert plan.items == PLAN.items


def test_problem_model_receives_deterministic_candidate_for_verification():
    """The model verifies the trusted plan rather than inventing arithmetic."""
    harness = Harness()
    harness.submit()

    candidate = harness.request_llm_problem.call_args.args[2]
    assert candidate.items == PLAN.items


def test_failed_web_submission_can_be_retried_after_model_recovers():
    """Recovery requires an explicit new click, not a replay of its ID."""
    harness = Harness()
    harness.request_llm_problem.side_effect = LlmClientError("offline", kind="timeout")
    harness.submit()
    harness.publish_plan.assert_not_called()
    harness.request_llm_problem.side_effect = None
    harness.submit("attempt-2-new-click")
    assert harness.publish_plan.call_count == 1
    assert harness.publish_plan.call_args.args[0].source == "llm"


def test_successful_duplicate_is_suppressed_but_new_round_is_accepted():
    harness = Harness()
    harness.submit("round-1")
    harness.submit("round-1")
    assert harness.publish_plan.call_count == 1
    harness.submit("round-2")
    assert harness.publish_plan.call_count == 2
    assert harness.publish_plan.call_args.args[0].task_id == "round-2"


def test_direct_question_can_retry_after_required_model_failure():
    harness = Harness()
    question = SimpleNamespace(
        data="Move 1 blue cube to zone B and 4 red cubes to zone A.")
    harness.request_llm.side_effect = LlmClientError("offline", kind="timeout")
    harness.question_callback(question)
    harness.publish_plan.assert_not_called()
    harness.request_llm.side_effect = None
    harness.request_llm.return_value = json.dumps({"items": [
        {"color": "blue", "count": 1, "target_zone": "B"},
        {"color": "red", "count": 4, "target_zone": "A"}]})
    harness.question_callback(question)
    assert harness.publish_plan.call_count == 1
    harness.question_callback(question)
    assert harness.publish_plan.call_count == 1


# The model table contradicts itself (rows sum to 2 while x_total says 3).
# Observed on a real round: "Problem rejected: Required LLM problem parsing
# failed: Model demand sum is inconsistent" (runs/manual-20261013T170042).
INCONSISTENT_CALCULATION = json.dumps({
    "items": [item.to_dict() for item in PLAN.items],
    "calculation": {
        "people": [{"name": "小美", "x_goods": 2, "y_goods": 1}],
        "capacity": 5,
        "x_total": 3,
        "y_total": 1,
        "x": 4,
        "y": 1,
    },
})


def test_invalid_model_sample_is_resampled_before_rejecting():
    """One bad sample must not cost the round; the retry must still verify."""
    harness = Harness()
    harness.llm_validate_attempts = 3
    harness.request_llm_problem.side_effect = [INCONSISTENT_CALCULATION, GOOD]

    harness.submit()

    plan = harness.publish_plan.call_args.args[0]
    assert plan.source == "llm"
    assert plan.items == PLAN.items
    assert harness.request_llm_problem.call_count == 2
    assert harness.publish_llm_status.call_args.kwargs["available"] is True


def test_invalid_model_sample_rejects_after_all_attempts():
    """Every attempt must be validated; exhaustion still fails loudly."""
    harness = Harness()
    harness.llm_validate_attempts = 3
    harness.request_llm_problem.return_value = INCONSISTENT_CALCULATION

    harness.submit()

    harness.publish_plan.assert_not_called()
    assert harness.request_llm_problem.call_count == 3
    assert harness.publish_llm_status.call_args.kwargs["source"] == "error"
    assert "Model demand sum is inconsistent" in (
        harness.publish_llm_status.call_args.kwargs["detail"]
    )


class RequestHarness:
    """Bare object exercising the real request builder with a mock client."""

    request_llm_problem = TaskParserNode.request_llm_problem

    def __init__(self):
        self.model = "test-model"
        self.llm_max_tokens = 4096
        self.llm_enable_thinking = False
        self.llm_retry_temperature = 0.3
        self.llm_client = Mock()
        self.llm_client.complete.return_value = GOOD


def test_first_attempt_is_greedy_and_has_no_retry_instruction():
    harness = RequestHarness()
    harness.request_llm_problem(QUESTION, RULES, PLAN)
    payload = harness.llm_client.complete.call_args.args[0]
    assert payload["temperature"] == 0.0
    assert len(payload["messages"]) == 2
    assert harness.llm_client.complete.call_args.kwargs["budget_sec"] is None


def test_resample_changes_prompt_and_temperature():
    """At temperature 0 a retry could repeat the rejected answer verbatim."""
    harness = RequestHarness()
    harness.request_llm_problem(QUESTION, RULES, PLAN, attempt=2,
                                budget_sec=12.0)
    payload = harness.llm_client.complete.call_args.args[0]
    assert payload["temperature"] == 0.3
    assert len(payload["messages"]) == 3
    assert "未通过校验" in payload["messages"][-1]["content"]
    assert harness.llm_client.complete.call_args.kwargs["budget_sec"] == 12.0


def test_only_protocol_failures_are_resampled():
    assert llm_error_is_retryable(
        LlmClientError("truncated", kind="protocol")) is True
    for kind in ("connection", "timeout", "auth", "config", "budget", "http"):
        assert llm_error_is_retryable(
            LlmClientError(kind, kind=kind)) is False


def test_truncated_completion_is_resampled():
    harness = Harness()
    harness.llm_validate_attempts = 3
    harness.request_llm_problem.side_effect = [
        LlmClientError("llm completion truncated by max_tokens",
                       kind="protocol"),
        GOOD,
    ]
    harness.submit()
    assert harness.publish_plan.call_args.args[0].source == "llm"
    assert harness.request_llm_problem.call_count == 2


def test_resamples_share_one_phase_budget():
    """Three samples must not each get a fresh full solve budget."""
    harness = Harness()
    harness.llm_validate_attempts = 3
    harness.solve_budget_sec = 40.0
    harness.request_llm_problem.side_effect = [INCONSISTENT_CALCULATION, GOOD]
    harness.submit()
    budgets = [call.kwargs["budget_sec"]
               for call in harness.request_llm_problem.call_args_list]
    assert len(budgets) == 2
    assert all(0.0 < budget <= 40.0 for budget in budgets)
    assert budgets[1] <= budgets[0]


def test_incomplete_chunk_set_is_reported_to_the_operator():
    """A lost chunk must not leave the round hanging without a message."""
    import time as time_module
    from types import SimpleNamespace

    harness = SimpleNamespace(
        _problem_chunks={"lost": {"total": 3, "parts": {0: "YQ=="},
                                  "received_at": time_module.monotonic() - 31.0}},
        _last_successful_parse_at=0.0,
        publish_parse_result=Mock(),
        get_logger=Mock(return_value=Mock()),
    )
    TaskParserNode._expire_problem_chunks(harness)
    assert harness._problem_chunks == {}
    harness.publish_parse_result.assert_called_once()
    assert "分片不完整" in harness.publish_parse_result.call_args.kwargs["error"]


def test_incomplete_chunk_set_does_not_overwrite_a_later_success():
    import time as time_module
    from types import SimpleNamespace

    harness = SimpleNamespace(
        _problem_chunks={"stale": {"total": 2, "parts": {0: "YQ=="},
                                   "received_at": time_module.monotonic() - 31.0}},
        _last_successful_parse_at=time_module.monotonic(),
        publish_parse_result=Mock(),
        get_logger=Mock(return_value=Mock()),
    )
    TaskParserNode._expire_problem_chunks(harness)
    assert harness._problem_chunks == {}
    harness.publish_parse_result.assert_not_called()
