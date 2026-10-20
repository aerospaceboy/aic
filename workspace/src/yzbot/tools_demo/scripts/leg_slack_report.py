#!/usr/bin/env python3
"""Per-leg slack for the rounds that never went under 280 s.

For every cube of a target case the passing round's pickup and delivery leg is
compared with the measured median for the same route (start zone / tray /
approach side), so the remaining seconds can be attributed either to the
route choice or to the run itself.
"""
import argparse
import json
import pathlib
import statistics
import sys

SCRIPTS = pathlib.Path("/home/ros/huxuyang/YZBot-224/workspace/src/yzbot/"
                       "tools_demo/scripts")
sys.path.insert(0, str(SCRIPTS))
import coverage48_summary as cov          # noqa: E402
import leg_cost_table as cost             # noqa: E402
import leg_side_table as side_table       # noqa: E402

ROOT = pathlib.Path("/home/ros/huxuyang/YZBot-224")
TARGETS = [
    "1bA_4rB", "1bB_4rC", "3rC_2bA", "1bA_4rC", "4bB_1rC",
    "2rC_3bA", "4rC_1bB", "1rB_4bA", "3rB_2bA", "4bB_1rA",
]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cases", nargs="*", default=TARGETS)
    options = parser.parse_args()

    cases = cov.legal_cases()
    pickup, delivery, _ = cost.collect(ROOT / "runs")
    side_pickup, side_delivery, _ = side_table.collect(ROOT / "runs")

    passing = {}
    for directory in sorted((ROOT / "runs").glob("*/results.jsonl")):
        for line in directory.read_text(encoding="utf-8",
                                        errors="ignore").splitlines():
            if not line.strip():
                continue
            try:
                record = json.loads(line)
            except ValueError:
                continue
            if not record.get("passed"):
                continue
            key = cov.key_of(record)
            tag = cases.get(key) if key else None
            if not tag:
                continue
            stamp = record.get("finished_at") or ""
            if tag not in passing or stamp > passing[tag][0]:
                passing[tag] = (stamp, record, directory.parent.name)

    analyze = cost.analyzer().analyze
    for tag in options.cases:
        stamp, record, run = passing[tag]
        log = pathlib.Path(record["log_path"])
        text = log.read_text(encoding="utf-8", errors="ignore")
        legs = analyze(log)["nav_legs"]
        window = side_table.mission_window(text)
        groups = side_table.goal_groups(text, window)
        sides = {}
        for zone, x, _y in groups:
            if zone.lower() in side_table.DOCKS:
                sides.setdefault(zone.lower(), x)
        print(f"== {tag}  {record['mission']['elapsed_sec']:.1f}s  {run}")
        for leg in legs:
            cube, where = leg["cube"], leg["location"]
            origin = side_table.nearest_anchor(leg["x"], leg["y"])
            if where.lower() in side_table.DOCKS:
                side = (
                    side_table.side_of(where, sides[where.lower()])
                    if where.lower() in sides else "-"
                )
                median = statistics.median(
                    side_pickup.get((origin, cube, side))
                    or pickup.get((origin, cube), [0])
                )
                print(f"   pick {cube:12} from {origin:5} {side:4} "
                      f"{leg['duration']:6.1f}  median {median:6.1f}  "
                      f"delta {leg['duration'] - median:+5.1f}")
            elif where.upper() in ("A", "B", "C"):
                median = statistics.median(
                    delivery.get((cube, where.upper()), [0])
                )
                print(f"   dest {cube:12} to   {where:5} ---- "
                      f"{leg['duration']:6.1f}  median {median:6.1f}  "
                      f"delta {leg['duration'] - median:+5.1f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
