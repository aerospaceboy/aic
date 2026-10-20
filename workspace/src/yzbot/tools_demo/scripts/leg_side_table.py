#!/usr/bin/env python3
"""Per-approach-side pickup costs from the campaign logs.

Every pickup navigation logs its staged dock approach as
``nav_goal zone=<CUBE> stage=k/n x=.. y=.. yaw=..``; the last stage is the
dock pose, and its x offset from the tray decides which side the base
stopped on. ``leg_cost_table.py`` aggregates legs without that dimension, so
a side change cannot be judged from it. This script pairs each staged goal
group with the navigation leg that follows it, attributes the side, and
reports medians per (start zone, cube, side) plus, for the delivery leg that
follows, per (cube, destination zone, pickup side) -- the two numbers a
side swap has to improve.
"""
import argparse
import re
import pathlib
import statistics
import sys

RUNS = pathlib.Path("/home/ros/huxuyang/YZBot-224/runs")
GOAL = re.compile(
    r"nav_goal zone=(\S+) stage=(\d+)/(\d+) x=(-?[\d.]+) y=(-?[\d.]+) "
    r"yaw=(-?[\d.]+)"
)
DOCKS = {
    "red_cube_1": (-4.36, 3.00),
    "red_cube_2": (-0.36, 3.00),
    "red_cube_3": (-6.36, -2.50),
    "red_cube_4": (-0.36, -1.00),
    "red_cube_5": (-4.36, -11.00),
    "blue_cube_1": (4.00, 1.91),
    "blue_cube_2": (-10.36, -3.00),
    "blue_cube_3": (-6.36, -7.00),
    "blue_cube_4": (1.04, -3.90),
    "blue_cube_5": (2.24, -5.80),
}
ANCHORS = {
    "spawn": (13.2, -0.95),
    "A": (-2.41, 2.99),
    "B": (-0.40, -12.00),
    "C": (-9.40, -7.00),
}
ZONES = ("A", "B", "C")


def analyzer():
    sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
    import mission_timing_analysis

    return mission_timing_analysis


EPOCH = re.compile(r"\[(\d{9,}\.\d+)\]")
TASK = re.compile(r'"task_id":"([0-9a-f]+)"')


def mission_window(text):
    """Epoch range of the last completed mission in one launch log.

    A campaign log can hold several missions (re-submitted questions), so the
    goal lines have to be scoped to the mission the analyzer picked.
    """
    lines = text.splitlines()
    completed = [
        TASK.search(line).group(1)
        for line in lines
        if '"state":"completed"' in line and TASK.search(line)
    ]
    if not completed:
        return None
    task_id = completed[-1]
    stamps = [
        float(EPOCH.search(line).group(1))
        for line in lines
        if task_id in line and EPOCH.search(line)
    ]
    if not stamps:
        return None
    return min(stamps), max(stamps)


def goal_groups(text, window=None):
    """Collapse each staged approach into its final dock pose."""
    groups = []
    for line in text.splitlines():
        match = GOAL.search(line)
        if not match:
            continue
        if window:
            stamp = EPOCH.search(line)
            if not stamp:
                continue
            value = float(stamp.group(1))
            if not window[0] <= value <= window[1]:
                continue
        zone, stage, _total, x, y, _yaw = match.groups()
        if stage == "1" or not groups or groups[-1][0] != zone:
            groups.append([zone, float(x), float(y)])
        else:
            groups[-1] = [zone, float(x), float(y)]
    return groups


def nearest_anchor(x, y):
    return min(
        ANCHORS,
        key=lambda name: (
            (x - ANCHORS[name][0]) ** 2 + (y - ANCHORS[name][1]) ** 2
        ),
    )


def side_of(zone, x):
    dock = DOCKS.get(zone.lower())
    if dock is None:
        return "-"
    return "east" if x > dock[0] else "west"


def collect(root):
    pickup, delivery, mismatch = {}, {}, 0
    analyze = analyzer().analyze
    for path in sorted(root.glob("*/results.jsonl")):
        for line in path.read_text(
            encoding="utf-8", errors="ignore"
        ).splitlines():
            if not line.strip():
                continue
            try:
                import json

                record = json.loads(line)
            except ValueError:
                continue
            log = record.get("log_path")
            if not log or not pathlib.Path(log).exists():
                continue
            log_path = pathlib.Path(log)
            try:
                legs = analyze(log_path)["nav_legs"]
            except Exception:  # noqa: BLE001 - unparsed round, keep going
                continue
            raw = log_path.read_text(encoding="utf-8", errors="ignore")
            window = mission_window(raw)
            if not window:
                continue
            groups = goal_groups(raw, window)
            named = [
                leg
                for leg in legs
                if leg["location"].lower() in DOCKS
                or leg["location"].upper() in ZONES
            ]
            if not groups or abs(len(groups) - len(named)) > 2:
                mismatch += 1
                continue
            previous_side = {}
            cursor = 0
            for leg in named:
                zone = leg["location"]
                while (
                    cursor < len(groups)
                    and groups[cursor][0].lower() != zone.lower()
                ):
                    cursor += 1
                if cursor >= len(groups):
                    mismatch += 1
                    continue
                group = groups[cursor]
                cursor += 1
                cube = leg["cube"]
                origin = nearest_anchor(leg["x"], leg["y"])
                if zone.lower() in DOCKS:
                    side = side_of(zone, group[1])
                    previous_side[cube] = side
                    pickup.setdefault((origin, cube, side), []).append(
                        leg["duration"]
                    )
                elif zone.upper() in ZONES:
                    key = (cube, zone.upper(), previous_side.get(cube, "-"))
                    delivery.setdefault(key, []).append(leg["duration"])
    return pickup, delivery, mismatch


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runs", default=str(RUNS))
    parser.add_argument("--cube", default=None)
    options = parser.parse_args()
    pickup, delivery, mismatch = collect(pathlib.Path(options.runs))
    print("pickup legs by side (median s, n)")
    for (origin, cube, side), samples in sorted(pickup.items()):
        if options.cube and options.cube not in cube:
            continue
        value = statistics.median(samples)
        print(
            f"  {origin:5} -> {cube:12} {side:4} {value:6.1f}"
            f"  n={len(samples)}"
        )
    print("delivery legs by pickup side (median s, n)")
    for (cube, zone, side), samples in sorted(delivery.items()):
        if options.cube and options.cube not in cube:
            continue
        value = statistics.median(samples)
        print(
            f"  {cube:12} -> {zone} via {side:4} {value:6.1f}"
            f"  n={len(samples)}"
        )
    if mismatch:
        print(f"unmatched goals: {mismatch}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
