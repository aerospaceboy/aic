"""Question seeds must remain attributable to this exact supplied question."""
import pytest

from tools_demo.question_metadata import supplied_question_seed


def test_seed_survives_parser_that_omits_auxiliary_metadata():
    original = {'task_id': 'round-1', 'question': '问题一', 'seed': 0}
    parsed = {'request_id': 'round-1', 'question': '问题一'}
    assert supplied_question_seed(parsed, original) == ('0', 'supplied_metadata')


@pytest.mark.parametrize('parsed', [
    {'task_id': 'round-2', 'question': '问题一'},
    {'task_id': 'round-1', 'question': '问题二'},
    {'question': '问题一'},
])
def test_another_request_cannot_inherit_old_seed(parsed):
    assert supplied_question_seed(parsed, {
        'task_id': 'round-1', 'question': '问题一', 'seed': 42}) == ('', 'not_supplied')


@pytest.mark.parametrize('seed', [True, False, -1, 2.5, None, '', '-1'])
def test_invalid_or_unconfigured_seed_is_explicitly_missing(seed):
    assert supplied_question_seed({'seed': seed}) == ('', 'not_supplied')


@pytest.mark.parametrize('question,expected', [
    ('题目种子：00123\n每个果篮有5个水果', '00123'),
    ('Seed = AIC-2026_03\n问题', 'AIC-2026_03'),
    ('每篮5个水果，共17个，编号123', ''),
])
def test_seed_is_extracted_only_from_an_explicit_label(question, expected):
    assert supplied_question_seed({'question': question})[0] == expected


def test_task_id_is_never_presented_as_seed():
    assert supplied_question_seed({'task_id': 'generated-task-123'}) == ('', 'not_supplied')
