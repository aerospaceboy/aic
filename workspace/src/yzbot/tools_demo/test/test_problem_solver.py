"""Tests for the competition problem-generator word-problem solver."""

import pytest

from tools_demo.problem_solver import is_generator_problem
from tools_demo.problem_solver import parse_mapping_rules
from tools_demo.problem_solver import problem_to_plan
from tools_demo.problem_solver import solve_math_problem
from tools_demo.task_protocol import TaskProtocolError


WARDROBE = (
    "每个衣柜有5件衣物;有T恤、裤子两种衣柜;小美需要2件T恤,1条裤子;"
    "小帅、小丽都需要7件T恤;小娜需要2件T恤,1条裤子;小杰需要1条裤子。"
    "设T恤衣柜数量为x,裤子衣柜数量为y,计算x、y分别为多少。"
)
PET = (
    "每个宠物店有5只宠物;有猫、狗两种宠物店;小爱领养2只猫,5只狗;"
    "小宠、小动都领养2只猫;小乐领养3只猫,5只狗;小欢领养4只狗。"
    "设猫宠物店数量为x,狗宠物店数量为y,计算x、y分别为多少"
)
RULES = "x代表红色,y代表蓝色,数量大于等于3的去A区,其他颜色去C区"
THREE_RED_TWO_BLUE_QUESTION = (
    "每个衣柜有5件衣物；有T恤、裤子两种衣柜；小美需要8件T恤、5条裤子；"
    "小帅需要7件T恤、5条裤子。设T恤衣柜数量为x，裤子衣柜数量为y，"
    "计算x、y分别为多少。"
)
THREE_RED_TWO_BLUE_RULES = (
    "x代表红色，y代表蓝色；数量大于等于3的去A区，其他颜色去B区。"
)


def test_wardrobe_variables():
    """The official wardrobe problem yields x=4, y=1."""
    assert solve_math_problem(WARDROBE) == (4, 1)


def test_pet_shop_variables():
    """The official pet-shop problem yields x=2, y=3."""
    assert solve_math_problem(PET) == (2, 3)


BOOKS = (
    "每个书箱有5本书籍;有故事书、科普书两种书箱;"
    "小文借阅3本故事书,2本科普书;小博、小雅都借阅6本故事书;"
    "小静借阅3本故事书,1本科普书;小豪借阅1本科普书。"
    "设故事书书箱数量为x,科普书书箱数量为y,计算x、y分别为多少。"
)


def test_official_book_generator_variables():
    """Seed 995447 has eighteen story books and four science books."""
    assert solve_math_problem(BOOKS) == (4, 1)


STATIONERY = (
    "每个文具盒有5个文具;有铅笔、橡皮两种文具盒;小明领取1个铅笔,5个橡皮;"
    "小红、小兰都领取1个铅笔;小刚领取1个铅笔,5个橡皮;小芳领取7个橡皮。"
    "设铅笔文具盒数量为x,橡皮文具盒数量为y,计算x、y分别为多少。"
)
TOYS = (
    "每个玩具箱有5个玩具;有小车、积木两种玩具箱;"
    "小杰想玩4个小车,1个积木;小乐、小欣都想玩5个小车;"
    "小琪想玩4个小车,1个积木;小浩想玩2个积木。"
    "设小车玩具箱数量为x,积木玩具箱数量为y,计算x、y分别为多少。"
)
BOOKS_SHORT_VAR_DECL = (
    "每个书箱有5本书籍;有故事书、科普书两种书箱;"
    "小文想要1本故事书,5本科普书;小雅想要1本故事书,"
    "他是双胞胎所以需要双倍的书;小静和小文需求一样,但少一本故事书;"
    "小豪想要5本科普书;小智想要1本故事书,5本科普书。"
    "设故事书箱数为x,科普书箱数为y,计算x、y。"
)
UNSUPPORTED_DEMAND = (
    "每个衣柜有5件衣物;有T恤、裤子两种衣柜;小美购得2件T恤,1条裤子。"
    "设T恤衣柜数量为x,裤子衣柜数量为y,计算x、y分别为多少。"
)


def test_official_stationery_generator_variables():
    """The 领取 wording yields four pencils and seventeen erasers -> (1, 4)."""
    assert solve_math_problem(STATIONERY) == (1, 4)


def test_official_toy_generator_want_to_play_variables():
    """The live toy wording counts both people in the grouped clause."""
    assert solve_math_problem(TOYS) == (4, 1)


def test_official_toy_generator_plan_with_threshold_mapping():
    """The exact rejected live question now reaches the expected task plan."""
    rules = "xy中,数值大于等于3值的代表红色，否则代表蓝色；x去B区,y去A区"
    plan = problem_to_plan(TOYS, rules)
    assert [(item.color, item.count, item.target_zone)
            for item in plan.items] == [("red", 4, "B"), ("blue", 1, "A")]


def test_official_short_variable_declaration_is_detected_and_solved():
    """Official book questions use 箱数为 as well as 数量为."""
    assert is_generator_problem(BOOKS_SHORT_VAR_DECL)
    assert solve_math_problem(BOOKS_SHORT_VAR_DECL) == (1, 4)


def test_official_stationery_plan_with_mapping_rules():
    """One red (under 3) goes to C and four blue (at least 3) go to A."""
    plan = problem_to_plan(STATIONERY, RULES)
    assert plan.counts == {"red": 1, "blue": 4}
    assert [item.target_zone for item in plan.items] == ["C", "A"]


def test_unseen_demand_wording_is_parsed_from_stable_structure():
    """A new action word must not block a structurally valid generator clause."""
    assert solve_math_problem(UNSUPPORTED_DEMAND) == (1, 1)


@pytest.mark.parametrize("action", ["购得", "准备使用", "希望得到", "打算拿走"])
def test_future_action_words_do_not_require_parser_code_changes(action):
    """Invented future wording still counts every grouped person exactly once."""
    question = (
        "每个宠物店有5只宠物;有猫、狗两种宠物店;"
        f"小甲、小乙都{action}2只猫,1只狗;"
        "设猫宠物店数量为x,狗宠物店数量为y,计算x、y。"
    )
    assert solve_math_problem(question) == (1, 1)


def test_new_wording_never_relaxes_item_validation():
    """Structural parsing must still reject quantities for an unknown item."""
    question = (
        "每个宠物店有5只宠物;有猫、狗两种宠物店;"
        "小甲准备带走2只兔;"
        "设猫宠物店数量为x,狗宠物店数量为y,计算x、y。"
    )
    with pytest.raises(TaskProtocolError, match="unknown item"):
        solve_math_problem(question)


@pytest.mark.parametrize("rule,expected", [
    ("x代表红色,y代表蓝色;数值大于等于3的去A区,其他颜色去C区",
     [("red", 4, "A"), ("blue", 1, "C")]),
    ("xy中,数值大于等于3值的代表红色，否则代表蓝色；x去B区,y去A区",
     [("red", 4, "B"), ("blue", 1, "A")]),
    ("蓝色去B区,红色去C区;xy中,数值大的是蓝色，数值少的是红色",
     [("blue", 4, "B"), ("red", 1, "C")]),
])
def test_official_books_obey_each_pdf_mapping(rule, expected):
    """Book units and borrow verbs must preserve all three PDF mappings."""
    plan = problem_to_plan(BOOKS, rule)
    assert [(item.color, item.count, item.target_zone)
            for item in plan.items] == expected


def test_wardrobe_plan_with_mapping_rules():
    """4 red (>=3) go to A and 1 blue goes to C."""
    plan = problem_to_plan(WARDROBE, RULES)
    assert plan.counts == {"red": 4, "blue": 1}
    assert [item.target_zone for item in plan.items] == ["A", "C"]


def test_pet_shop_plan_with_mapping_rules():
    """2 red (<3) go to C and 3 blue (>=3) go to A."""
    plan = problem_to_plan(PET, RULES)
    assert plan.counts == {"red": 2, "blue": 3}
    assert [item.target_zone for item in plan.items] == ["C", "A"]


def test_chinese_punctuation_three_red_to_a_two_blue_to_b():
    """Chinese punctuation preserves the intended three-red/two-blue plan."""
    plan = problem_to_plan(
        THREE_RED_TWO_BLUE_QUESTION,
        THREE_RED_TWO_BLUE_RULES,
    )
    assert plan.counts == {"red": 3, "blue": 2}
    assert [item.target_zone for item in plan.items] == ["A", "B"]


def test_json_mapping_rules_equivalent_to_text():
    """The structured JSON form produces the same plan as free text."""
    json_rules = (
        '{"color_map": {"x": "red", "y": "blue"}, '
        '"zone_rules": [{"op": "ge", "value": 3, "zone": "A"}, '
        '{"op": "else", "zone": "C"}]}'
    )
    text_plan = problem_to_plan(PET, RULES)
    json_plan = problem_to_plan(PET, json_rules)
    assert json_plan.items == text_plan.items


def test_detects_generator_problems():
    """Generator word problems are not direct rule questions."""
    assert is_generator_problem(WARDROBE)
    assert is_generator_problem(PET)
    assert not is_generator_problem("请先将2个红色方块送到A区")


def test_empty_rules_are_rejected():
    """No mapping rules must raise rather than silently guess."""
    with pytest.raises(TaskProtocolError):
        problem_to_plan(WARDROBE, "")


def test_incomplete_color_map_is_rejected():
    """Rules that omit y must raise a protocol error."""
    with pytest.raises(TaskProtocolError):
        parse_mapping_rules("x代表红色,数量大于等于3的去A区,其他颜色去C区")
