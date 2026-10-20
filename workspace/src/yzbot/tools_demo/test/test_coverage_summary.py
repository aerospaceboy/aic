"""Coverage statistics must preserve column meaning and failed attempts."""
import importlib.util
import json
from pathlib import Path

spec = importlib.util.spec_from_file_location('summary48',
    Path(__file__).resolve().parents[1] / 'scripts' / 'summarize48_v2.py')
summary = importlib.util.module_from_spec(spec)
spec.loader.exec_module(summary)


def write_run(root, name, accepted, state='completed'):
    directory=root / name
    directory.mkdir()
    (directory / 'question.txt').write_text(summary.questions()['1rA_4bB'])
    (directory / 'result.log').write_text('MISSION_TERMINAL='+json.dumps({
        'state':state,'completed':5,'skipped':0,'elapsed_sec':250.0})+'\n'
        +'CASE_ACCEPTED='+str(accepted).lower()+' collision_faults=[]\n')
    return directory


def test_completed_but_rejected_does_not_count_as_accepted(tmp_path):
    write_run(tmp_path,'20260101-a',False)
    row=summary.rows_for(tmp_path)[0]
    assert row['five_of_five']
    assert not row['case_accepted']
    assert not row['under300']
    assert row['question'].startswith('Move ')
    assert row['state']=='completed'


def test_latest_failure_is_not_hidden_by_old_success(tmp_path):
    write_run(tmp_path,'20260101-a',True)
    write_run(tmp_path,'20260102-b',False,'failed')
    assert not summary.rows_for(tmp_path)[0]['case_accepted']


def test_latest_missing_result_is_visible(tmp_path):
    write_run(tmp_path,'20260101-a',True)
    directory=tmp_path/'20260102-b';directory.mkdir()
    (directory/'question.txt').write_text(summary.questions()['1rA_4bB'])
    rows=summary.rows_for(tmp_path)
    assert len(rows)==48
    assert rows[0]['state']=='NO_TERMINAL'
    assert not rows[0]['case_accepted']
