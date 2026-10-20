"""Check the published provincial mappings in both variable directions."""

import pytest
import json
import copy

from tools_demo.problem_solver import problem_to_plan
from tools_demo.problem_solver import solve_math_problem
from tools_demo.task_protocol import TaskProtocolError
from tools_demo.task_parser_node import TaskParserNode


RULES = (
    "x 代表红色,y 代表蓝色;数值大于等于 3 的去 A 区,其他颜色去 C 区",
    "xy 中,数值大于等于 3 值的代表红色，否则代表蓝色；x 去 B 区,y 去 A 区",
    "蓝色去 B 区,红色去 C 区; xy 中,数值大的是蓝色，数值少的是红色",
)


def question(x, y):
    """Produce a demand question with a known independently checked answer."""
    return (
        "每个宠物店有5只宠物;有猫、狗两种宠物店;"
        f"小爱领养{x * 5}只猫,{y * 5}只狗;"
        "设猫宠物店数量为x,狗宠物店数量为y,计算x、y分别为多少"
    )


@pytest.mark.parametrize("x,y", [(1, 4), (2, 3), (3, 2), (4, 1)])
@pytest.mark.parametrize("index", range(3))
def test_published_rules(index, x, y):
    """Do not confuse variable-based destinations with color destinations."""
    plan = problem_to_plan(question(x, y), RULES[index])
    if index == 0:
        expected = [
            ("red", x, "A" if x >= 3 else "C"),
            ("blue", y, "A" if y >= 3 else "C"),
        ]
    elif index == 1:
        expected = [
            ("red" if x >= 3 else "blue", x, "B"),
            ("red" if y >= 3 else "blue", y, "A"),
        ]
    else:
        expected = [
            ("blue" if x > y else "red", x, "B" if x > y else "C"),
            ("blue" if y > x else "red", y, "B" if y > x else "C"),
        ]
    assert [(i.color, i.count, i.target_zone) for i in plan.items] == expected


def test_relative_mapping_rejects_undefined_tie():
    """Equal values need a judge-defined rule, not an invented color."""
    with pytest.raises(TaskProtocolError, match="equal x/y"):
        problem_to_plan(question(2, 2), RULES[2])


def test_modified_relative_rule_is_not_silently_accepted():
    """An unannounced zone change cannot reuse the published preset."""
    with pytest.raises(TaskProtocolError):
        problem_to_plan(question(3, 2), RULES[2].replace("B", "A"))


@pytest.mark.parametrize("question_text,answer", [
    (
        "每个果篮有5个水果;有苹果、橙子两种果篮。"
        "小芳想要2个苹果,3个橙子;小强想要1个苹果,他是双胞胎所以需要双倍水果;"
        "小丽和小芳需求一样,但少一个苹果;小美想要3个橙子;"
        "小帅想要2个苹果,3个橙子。设苹果果篮数量为x,橙子果篮数量为y,计算x、y。",
        (2, 3),
    ),
    (
        "每个衣柜有5件衣物;有T恤、裤子两种衣柜。"
        "小美想要3件T恤,2条裤子;小帅想要2件T恤,他是双胞胎所以需要双倍T恤;"
        "小娜和小美需求一样,但少一件T恤;小杰想要2条裤子;"
        "小浩想要2件T恤,1条裤子。设T恤衣柜数量为x,裤子衣柜数量为y,计算x、y。",
        (3, 2),
    ),
    (
        "每个球框有5个运动器材;有篮球、足球两种球框。"
        "小壮想要1个篮球;小强想要5个足球,1个篮球;小勇说和小壮一样;"
        "小健要5个足球;小康也和小壮一样但是换成足球;小杰想要1个篮球,5个足球。"
        "设篮球球框数量为x,足球球框数量为y,计算x、y。",
        (1, 4),
    ),
])
def test_actual_competition_generator_questions(question_text, answer):
    """The two actual time-window questions include twins and references."""
    assert solve_math_problem(question_text) == answer


@pytest.mark.parametrize("clause", [
    "小乙和不存在的人需求一样,但少一个猫",
    "小乙和小甲需求一样,但少五只猫",
    "小乙和小甲需求一样,但多一只猫",
])
def test_unknown_or_invalid_relationship_is_rejected(clause):
    """Never silently omit an unverified person's relative demand."""
    text = (
        "每个宠物店有5只宠物;有猫、狗两种宠物店;"
        f"小甲想要2只猫,5只狗;{clause};"
        "设猫宠物店数量为x,狗宠物店数量为y,计算x、y分别为多少"
    )
    with pytest.raises(TaskProtocolError):
        solve_math_problem(text)


def test_correct_model_items_can_arrive_in_either_color_order():
    """New scoring accepts correct goods and destinations in either order."""
    text = question(3, 2)
    expected = problem_to_plan(text, RULES[2])
    payload = {"schema_version": "1.0", "items": expected.to_dict()["items"]}
    payload["items"].reverse()
    actual = TaskParserNode._validate_problem_llm(
        None, json.dumps(payload), text, RULES[2], expected
    )
    assert actual.items == expected.items
    assert actual.source == "llm"


def test_wrong_model_destination_still_fails_after_order_normalization():
    """Color-order relaxation never relaxes the requested delivery zone."""
    text = question(3, 2)
    expected = problem_to_plan(text, RULES[2])
    payload = {"schema_version": "1.0", "items": expected.to_dict()["items"]}
    payload["items"][0]["target_zone"] = "A"
    with pytest.raises(TaskProtocolError, match="disagrees"):
        TaskParserNode._validate_problem_llm(
            None, json.dumps(payload), text, RULES[2], expected
        )


CALCULATION = {
    "people": [{"name": "小爱", "x_goods": 15, "y_goods": 10}],
    "capacity": 5, "x_total": 15, "y_total": 10, "x": 3, "y": 2,
}


def test_valid_optional_arithmetic_trace_keeps_strict_plan_validation():
    """A model may show arithmetic without adding control-schema fields."""
    text = question(3, 2)
    expected = problem_to_plan(text, RULES[1])
    payload = {
        "items": expected.to_dict()["items"], "calculation": CALCULATION,
    }
    actual = TaskParserNode._validate_problem_llm(
        None, json.dumps(payload), text, RULES[1], expected
    )
    assert actual.items == expected.items


@pytest.mark.parametrize("field,value", [
    ("capacity", 4), ("x_total", 14), ("x", 2), ("y", True),
])
def test_inconsistent_model_arithmetic_is_rejected(field, value):
    """Optional arithmetic never permits a wrong capacity or count."""
    text = question(3, 2)
    expected = problem_to_plan(text, RULES[1])
    calculation = copy.deepcopy(CALCULATION)
    calculation[field] = value
    payload = {
        "items": expected.to_dict()["items"], "calculation": calculation,
    }
    with pytest.raises(TaskProtocolError):
        TaskParserNode._validate_problem_llm(
            None, json.dumps(payload), text, RULES[1], expected
        )
