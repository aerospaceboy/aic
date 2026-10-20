"""Exercise nested evidence summaries and honest latest-attempt selection."""

import importlib.util
import json
from pathlib import Path

SCRIPT = Path(__file__).parents[1] / "scripts" / "summarize48_v2.py"
SPEC = importlib.util.spec_from_file_location("campaign_summary_test", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def write_attempt(root, stamp, functional=True):
    """Create one minimal terminal evidence directory beneath a case tag."""
    question = next(iter(MODULE.questions().values()))
    path = root / "case" / stamp
    path.mkdir(parents=True)
    (path / "question.txt").write_text(question)
    terminal = dict(state="completed", completed=5, skipped=0, elapsed_sec=250)
    (path / "result.log").write_text(
        "MISSION_TERMINAL=" + json.dumps(terminal)
        + "\nCASE_ACCEPTED=true collision_faults=[]\n")
    (path / "audit.json").write_text(json.dumps(
        dict(motion_functional_pass=functional)))
    return path


def test_nested_run48_directories_are_discovered(tmp_path):
    """The summary accepts run48_v2's tag/attempt directory structure."""
    path = write_attempt(tmp_path, "20261005T100000Z-a")
    row = next(row for row in MODULE.rows_for(tmp_path)
               if row["evidence"] == str(path))
    assert row["strict_motion_pass"] and row["under300"]


def test_latest_failure_is_not_hidden_by_an_older_success(tmp_path):
    """The current campaign verdict follows its newest attempt."""
    write_attempt(tmp_path, "20261005T100000Z-a")
    newest = write_attempt(tmp_path, "20261005T100100Z-b", functional=False)
    row = next(row for row in MODULE.rows_for(tmp_path)
               if row["evidence"] == str(newest))
    assert not row["strict_motion_pass"]
    assert not row["under300"]


def test_interrupted_attempt_cannot_be_counted_as_a_pass(tmp_path):
    """An interruption marker overrides apparently successful fragments."""
    path = write_attempt(tmp_path, "20261005T100000Z-a")
    (path / "INTERRUPTED.md").write_text("manually stopped")
    row = next(row for row in MODULE.rows_for(tmp_path)
               if row["evidence"] == str(path))
    assert row["state"] == "INTERRUPTED"
    assert not row["strict_motion_pass"] and not row["under300"]
