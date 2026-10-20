#!/usr/bin/env python3
"""Audit latest attempts without treating completion as safety acceptance."""
import contextlib
import csv
import io
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
import gen48_questions


def questions():
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        gen48_questions.main()
    return dict(line.split(' ', 1) for line in buf.getvalue().splitlines())


def rows_for(root):
    expected = questions()
    inverse = {q: tag for tag, q in expected.items()}
    latest = {}
    directories = {path.parent for path in Path(root).rglob('question.txt')}
    for directory in sorted(directories, key=lambda path: (path.name, str(path))):
        qpath = directory / 'question.txt'
        if not qpath.is_file():
            continue
        question = qpath.read_text().strip()
        tag = inverse.get(question)
        if not tag:
            continue
        terminal, accepted = {}, False
        rpath = directory / 'result.log'
        for line in rpath.read_text(errors='replace').splitlines() if rpath.exists() else []:
            if 'MISSION_TERMINAL=' in line:
                try:
                    terminal = json.loads(line.split('MISSION_TERMINAL=', 1)[1])
                except ValueError:
                    terminal = {}
            if 'CASE_ACCEPTED=' in line:
                accepted = line.split('CASE_ACCEPTED=', 1)[1].startswith('true collision_faults=[]')
        audit = {}
        try:
            audit = json.loads((directory / 'audit.json').read_text())
        except (OSError, ValueError):
            pass
        completed = (terminal.get('state') == 'completed'
                     and terminal.get('completed') == 5
                     and terminal.get('skipped') == 0)
        elapsed = terminal.get('elapsed_sec')
        functional = audit.get('motion_functional_pass') is True
        interrupted = (directory / 'INTERRUPTED.md').exists()
        target_290 = (functional and completed and accepted
                      and not interrupted
                      and isinstance(elapsed, (int, float))
                      and 0 < elapsed < 290)
        latest[tag] = dict(tag=tag, question=question,
            state='INTERRUPTED' if interrupted else terminal.get('state', 'NO_TERMINAL'),
            elapsed_sec=elapsed,
            completed=terminal.get('completed'), skipped=terminal.get('skipped'),
            five_of_five=completed, case_accepted=completed and accepted,
            strict_motion_pass=functional and completed and accepted and not interrupted,
                           requirements_verified=audit.get(
                               'requirements_verified'),
                           goods_score=audit.get('goods_score'),
                           under290=target_290,
            under300=functional and completed and accepted and not interrupted
                     and isinstance(elapsed, (int, float)) and 0 < elapsed < 300,
            evidence=str(directory),
            commit=(directory / 'commit.txt').read_text().strip() if (directory / 'commit.txt').exists() else 'unknown')
    return [latest.get(tag, dict(tag=tag, question=q, state='NO_EVIDENCE',
             elapsed_sec=None, completed=None, skipped=None, five_of_five=False,
             case_accepted=False, strict_motion_pass=False, under300=False,
                                 requirements_verified=None,
                                 goods_score=None, under290=False,
             evidence='', commit='unknown')) for tag, q in expected.items()]


def main():
    rows = rows_for(sys.argv[1])
    print(json.dumps({'tasks':len(rows),
        'five_of_five':sum(r['five_of_five'] for r in rows),
        'case_accepted':sum(r['case_accepted'] for r in rows),
        'strict_motion_pass':sum(r['strict_motion_pass'] for r in rows),
        'under300':sum(r['under300'] for r in rows),
                      'under290': sum(r['under290'] for r in rows),
        'retry':[r['tag'] for r in rows if not r['under290']]}, indent=2))
    if len(sys.argv)>2:
        with open(sys.argv[2], 'w', newline='') as handle:
            writer=csv.DictWriter(handle, fieldnames=list(rows[0]))
            writer.writeheader(); writer.writerows(rows)


if __name__ == '__main__':
    main()
