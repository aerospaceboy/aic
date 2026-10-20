"""Tests for the competition task protocol."""

import json
import random

import pytest

from tools_demo.task_protocol import generate_task_question
from tools_demo.task_protocol import parse_llm_response
from tools_demo.task_protocol import parse_task_question
from tools_demo.task_protocol import parse_with_fallback
from tools_demo.task_protocol import TaskProtocolError


def test_chinese_question_preserves_order_counts_and_zones():
    """Parse two ordered Chinese instructions."""
    plan = parse_task_question(
        "请先将2个红色方块送到A区，再将一个蓝色方块送到C区。"
    )

    assert [item.color for item in plan.items] == ["red", "blue"]
    assert [item.count for item in plan.items] == [2, 1]
    assert [item.target_zone for item in plan.items] == ["A", "C"]
    assert plan.counts == {"red": 2, "blue": 1}
    assert plan.execution_order == ("red", "red", "blue")


def test_english_question_and_json_are_deterministic():
    """Support an English task and stable serialized derived fields."""
    plan = parse_task_question(
        "First move 1 blue cube to zone B, then 3 red blocks into A."
    )
    payload = json.loads(plan.to_json())

    assert [item.color for item in plan.items] == ["blue", "red"]
    assert payload["counts"] == {"blue": 1, "red": 3}
    assert payload["execution_order"] == [
        "blue",
        "red",
        "red",
        "red",
    ]
    assert len(payload["task_id"]) == 16


def test_generated_questions_are_always_rule_parseable():
    """Generate many seeded questions and parse every result."""
    randomizer = random.Random(20260730)

    for _ in range(50):
        question = generate_task_question(randomizer)
        plan = parse_task_question(question)
        assert len(plan.items) == 2
        assert {item.color for item in plan.items} == {"red", "blue"}
        assert len({item.target_zone for item in plan.items}) == 2
        assert sum(plan.counts.values()) == 5


def test_valid_fenced_llm_json_is_cross_checked():
    """Accept strict JSON when it agrees with explicit question values."""
    question = "先搬运1个蓝色方块到C区，然后搬运2个红色方块到A区。"
    content = """```json
{"schema_version":"1.0","items":[
  {"color":"blue","count":1,"target_zone":"C"},
  {"color":"red","count":2,"target_zone":"A"}
]}
```"""

    plan = parse_llm_response(content, question)

    assert plan.source == "llm"
    assert [item.color for item in plan.items] == ["blue", "red"]


@pytest.mark.parametrize(
    "content",
    (
        "not json",
        '{"items":[{"color":"green","count":1,"target_zone":"A"}]}',
        '{"items":[{"color":"red","count":0,"target_zone":"A"}]}',
        '{"items":[{"color":"red","count":1,"target_zone":"D"}]}',
        '{"items":[{"color":"red","count":1}]}',
        (
            '{"items":['
            '{"color":"red","count":3,"target_zone":"A"},'
            '{"color":"blue","count":3,"target_zone":"B"}'
            "]}"
        ),
    ),
)
def test_invalid_llm_json_is_rejected(content):
    """Reject malformed values, unsupported values, and missing fields."""
    with pytest.raises(TaskProtocolError):
        parse_llm_response(content, "请将1个红色方块送到A区。")


def test_llm_disagreement_falls_back_to_rules():
    """Never trust valid JSON that contradicts explicit task wording."""
    question = "请先将2个红色方块送到A区，再将1个蓝色方块送到B区。"
    hallucinated = (
        '{"items":['
        '{"color":"red","count":3,"target_zone":"A"},'
        '{"color":"blue","count":1,"target_zone":"B"}'
        "]}"
    )

    plan = parse_with_fallback(question, hallucinated)

    assert plan.source == "rules"
    assert plan.counts == {"red": 2, "blue": 1}


@pytest.mark.parametrize(
    "question",
    (
        "",
        "把红色方块拿走。",
        "请将两个蓝色方块送到未知区域。",
        "请将1个绿色方块送到A区。",
    ),
)
def test_invalid_question_is_rejected(question):
    """Reject questions without a complete supported instruction."""
    with pytest.raises(TaskProtocolError):
        parse_task_question(question)
