#!/usr/bin/env python3
"""Aggregate the legal 48-case matrix from campaign run directories.

``gen48_questions.py`` defines the legal space: six ordered zone pairs with
four count splits and two clause orders each. ``run_zone9_campaign.py``
records one JSON object per round under ``runs/<prefix>_<stamp>/results.jsonl``
with the parsed order and the acceptance counters. This audit joins the two by
order string so both tag spellings (``AB14`` / ``1rA_4bB``) land on the same
row, and reports the latest attempt plus the best passing attempt per case.

Unlike ``summarize48_v2.py`` (which reads ``experiments/coverage-48``
``result.log`` evidence), this reads campaign ``results.jsonl`` records.
Completion alone is not acceptance: a row counts as verified only when the
round also passed the campaign gate (avoidance, counters, 290 s budget).
"""
import argparse
import contextlib
import io
import json
import pathlib
import sys

RUNS = pathlib.Path("/home/ros/huxuyang/YZBot-224/runs")
WORDS = {"r": "red", "b": "blue"}


def generator():
    """Import the sibling generator script (not an installed package)."""
    sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
    import gen48_questions

    return gen48_questions


def order_string(tag):
    """Convert ``1rA_4bB`` to the campaign's ``1:redx1->A | 2:bluex4->B``."""
    parts = tag.split("_")
    rendered = []
    for index, part in enumerate(parts, start=1):
        count, colour, zone = part[0], part[1], part[2:]
        rendered.append(f"{index}:{WORDS[colour]}x{count}->{zone}")
    return " | ".join(rendered)


def legal_cases():
    """Map every legal order string to its canonical tag."""
    buffer = io.StringIO()
    with contextlib.redirect_stdout(buffer):
        generator().main()
    cases = {}
    for line in buffer.getvalue().splitlines():
        tag = line.split(" ", 1)[0]
        cases[order_string(tag)] = tag
    return cases


def records(root):
    for directory in sorted(root.glob("*/results.jsonl")):
        for line in directory.read_text(
            encoding="utf-8", errors="ignore"
        ).splitlines():
            if not line.strip():
                continue
            try:
                record = json.loads(line)
            except ValueError:
                continue
            record["_run"] = directory.parent.name
            yield record


def key_of(record):
    """Prefer the expected order, falling back to the parsed plan order."""
    value = record.get("expected_order")
    if not value:
        value = (record.get("metrics") or {}).get("plan_order")
    return value.strip() if value else None


def collect(root):
    cases = legal_cases()
    attempts = {}
    for record in records(root):
        key = key_of(record)
        tag = cases.get(key) if key else None
        if not tag:
            continue
        attempts.setdefault(tag, []).append(record)
    return cases, attempts


def summarize(tag, rows):
    if not rows:
        return {
            "tag": tag,
            "attempts": 0,
            "passes": 0,
            "latest_run": None,
            "latest_at": None,
            "latest_passed": False,
            "latest_elapsed": None,
            "latest_state": None,
            "latest_completed": None,
            "min_clearance": None,
            "plan_source": None,
            "best_elapsed": None,
            "reason": "no attempt recorded",
        }
    latest = rows[-1]
    mission = latest.get("mission") or {}
    avoidance = latest.get("avoidance") or {}
    passing = [row for row in rows if row.get("passed")]
    best = min(
        [
            (row.get("mission") or {}).get("elapsed_sec") or 0.0
            for row in passing
        ],
        default=None,
    )
    return {
        "tag": tag,
        "attempts": len(rows),
        "passes": len(passing),
        "latest_run": latest.get("_run"),
        "latest_at": latest.get("finished_at"),
        "latest_passed": bool(latest.get("passed")),
        "latest_elapsed": mission.get("elapsed_sec"),
        "latest_state": mission.get("state"),
        "latest_completed": mission.get("completed"),
        "min_clearance": avoidance.get("minimum_clearance"),
        "plan_source": (latest.get("metrics") or {}).get("plan_source"),
        "best_elapsed": best,
        "reason": latest.get("reason"),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runs", default=str(RUNS))
    parser.add_argument("--json", action="store_true")
    parser.add_argument(
        "--family",
        default=None,
        help="Only report zone pairs touching these zones, e.g. C,B",
    )
    options = parser.parse_args()

    cases, attempts = collect(pathlib.Path(options.runs))
    rows = [
        summarize(tag, attempts.get(tag, []))
        for tag in sorted(cases.values())
    ]
    if options.family:
        zones = {part.strip().upper() for part in options.family.split(",")}
        rows = [
            row
            for row in rows
            if {row["tag"][-1], row["tag"].split("_")[1][-1]} <= zones
        ]
    if options.json:
        print(json.dumps(rows, ensure_ascii=False, indent=1))
        return 0

    covered = [row for row in rows if row["passes"]]
    print(f"legal cases: {len(rows)}  verified: {len(covered)}")
    print(
        f"{'case':12} {'att':>3} {'pass':>4} {'elapsed':>8} "
        f"{'clear':>7} {'src':>4}  {'state':<9} run"
    )
    for row in rows:
        elapsed = row["latest_elapsed"]
        clearance = row["min_clearance"]
        shown = f"{elapsed:.1f}" if elapsed is not None else "-"
        clear = f"{clearance:.4f}" if clearance is not None else "-"
        print(
            f"{row['tag']:12} {row['attempts']:>3} {row['passes']:>4} "
            f"{shown:>8} {clear:>7} {str(row['plan_source']):>4}  "
            f"{str(row['latest_state']):<9} {row['latest_run'] or '-'}"
            + ("" if row["latest_passed"] else f"  !{row['reason']}")
        )
    missing = [row["tag"] for row in rows if not row["passes"]]
    print(f"unverified ({len(missing)}): {' '.join(missing) or '-'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
