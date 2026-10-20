#!/usr/bin/env python3
"""Check the 48-case evidence really matches the deployed policy table.

``coverage48_summary.py`` answers "does this legal task have a passing
round?". It does not answer "was that passing round produced by the policy
table that is deployed now": a case can pass, the table can change, and the
row still reads verified. This audit closes that gap by reading the actual
pick sequence and dock side out of each passing round's log and comparing
them with the current ``cube_order_policy.json`` entry.
"""
import argparse
import json
import pathlib
import re
import sys

RUNS = pathlib.Path("/home/ros/huxuyang/YZBot-224/runs")
PICK = re.compile(r"manip_start operation=pick cube=(\w+)")
EPOCH = re.compile(r"\[(\d{9,}\.\d+)\]")


def helpers():
    sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
    import coverage48_summary
    import leg_side_table

    return coverage48_summary, leg_side_table


def task_window(text, task_id):
    stamps = [
        float(EPOCH.search(line).group(1))
        for line in text.splitlines()
        if task_id in line and EPOCH.search(line)
    ]
    if not stamps:
        return None
    return min(stamps), max(stamps)


def observed(text, task_id, side_tool):
    """Ordered pick cubes and their measured dock sides for one mission."""
    window = task_window(text, task_id)
    if not window:
        return [], {}
    lines = text.splitlines()
    cubes = []
    for line in lines:
        match = PICK.search(line)
        stamp = EPOCH.search(line)
        if not match or not stamp:
            continue
        if window[0] <= float(stamp.group(1)) <= window[1]:
            cubes.append(match.group(1))
    groups = side_tool.goal_groups(text, window)
    sides = {}
    for zone, x, _y in groups:
        if zone.lower() in side_tool.DOCKS:
            sides.setdefault(zone.lower(), []).append(
                side_tool.side_of(zone, x)
            )
    return cubes, sides


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runs", default=str(RUNS))
    parser.add_argument(
        "--policy",
        default=str(
            pathlib.Path(__file__).resolve().parents[1]
            / "config"
            / "cube_order_policy.json"
        ),
    )
    options = parser.parse_args()
    coverage, side_tool = helpers()
    tasks = json.loads(
        pathlib.Path(options.policy).read_text(encoding="utf-8")
    )["tasks"]
    cases = coverage.legal_cases()

    passing = {}
    root = pathlib.Path(options.runs)
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
            if not record.get("passed"):
                continue
            key = coverage.key_of(record)
            tag = cases.get(key) if key else None
            if not tag:
                continue
            stamp = record.get("finished_at") or ""
            if tag not in passing or stamp > passing[tag][0]:
                passing[tag] = (stamp, record, directory.parent.name)

    mismatched, unverified = [], []
    for tag in sorted(cases.values()):
        if tag not in passing:
            unverified.append(tag)
            continue
        stamp, record, run = passing[tag]
        entry = tasks.get(tag)
        if not entry:
            mismatched.append((tag, "no policy entry", run, stamp))
            continue
        expected = [
            (item["cube"].lower(), item["side"])
            for item in entry["order"]
        ]
        log = record.get("log_path")
        if not log or not pathlib.Path(log).exists():
            mismatched.append((tag, "log missing", run, stamp))
            continue
        text = pathlib.Path(log).read_text(
            encoding="utf-8", errors="ignore"
        )
        task_id = (record.get("mission") or {}).get("task_id") or ""
        cubes, sides = observed(text, task_id, side_tool)
        problems = []
        if len(cubes) != len(expected):
            problems.append(f"picks={len(cubes)} expected={len(expected)}")
        else:
            for index, (cube, side) in enumerate(expected):
                if cubes[index] != cube:
                    problems.append(
                        f"#{index + 1} {cubes[index]} != {cube}"
                    )
                    break
                seen = sides.get(cube)
                if seen and side not in seen:
                    problems.append(f"{cube} side {seen} != {side}")
        if problems:
            mismatched.append((tag, "; ".join(problems), run, stamp))
        else:
            print(f"OK   {tag:10} {stamp} {run}")
    print()
    done = len(cases) - len(mismatched) - len(unverified)
    print(f"matched {done}/{len(cases)}")
    for tag, why, run, stamp in mismatched:
        print(f"MISMATCH {tag:10} {why}  ({run} {stamp})")
    for tag in unverified:
        print(f"NO-PASS  {tag}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
