"""Keep symbolic comparison guidance separate from the current answer."""

from types import SimpleNamespace
from unittest.mock import Mock

from tools_demo.task_parser_node import PROBLEM_SYSTEM_PROMPT, TaskParserNode


def test_comparison_prompt_covers_both_variable_orderings():
    assert '若x>y：x项为blue、count=x、target_zone=B' in PROBLEM_SYSTEM_PROMPT
    assert '若x<y：x项为red、count=x、target_zone=C' in PROBLEM_SYSTEM_PROMPT
    assert '禁止为符合某个颜色而交换x、y已经算出的数值' in PROBLEM_SYSTEM_PROMPT
    assert '不得把“较大”理解为x' not in PROBLEM_SYSTEM_PROMPT


def test_prompt_treats_action_words_as_open_vocabulary():
    assert '动作词可能变化' in PROBLEM_SYSTEM_PROMPT
    assert '不能因为动作词陌生而忽略该人物' in PROBLEM_SYSTEM_PROMPT


def test_model_only_receives_problem_and_rule_not_trusted_answer():
    client = Mock()
    node = SimpleNamespace(model='qwen3.8-max', llm_max_tokens=4096,
                           llm_enable_thinking=False, llm_client=client)
    TaskParserNode.request_llm_problem(
        node, 'unknown question', 'unknown rule',
        {'SECRET_CANDIDATE': [777, 888]},
    )
    payload = client.complete.call_args.args[0]
    assert payload['model'] == 'qwen3.8-max'
    assert payload['enable_thinking'] is False
    assert 'SECRET_CANDIDATE' not in str(payload)
    assert '777' not in str(payload) and '888' not in str(payload)
    assert payload['messages'][1]['content'] == '映射规则：\nunknown rule\n\n题目：\nunknown question'
