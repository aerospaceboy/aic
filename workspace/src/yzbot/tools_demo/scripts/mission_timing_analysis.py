#!/usr/bin/env python3
"""Summarize competition mission timing from one or more ROS launch logs."""

from __future__ import annotations

import argparse
import json
import re
from collections import defaultdict
from pathlib import Path


JSON_RE = re.compile(r"\{.*\}")
NAV_RESULT_RE = re.compile(r"nav_result state=(\w+) elapsed=([0-9.]+)s")
NAV_START_RE = re.compile(
    r"nav_start location=(\S+).*robot=\(([-0-9.]+),([-0-9.]+),yaw=([-0-9.]+)\)"
)
PREALIGN_RE = re.compile(
    r"pre_navigation_alignment distance=([0-9.]+) error=([-0-9.]+) "
    r"target_yaw=([-0-9.]+)"
)


def mission_events(path: Path) -> list[dict]:
    events = []
    for line in path.read_text(encoding="utf-8", errors="ignore").splitlines():
        if "[competition_mission_coordinator]: {" not in line:
            continue
        match = JSON_RE.search(line)
        if not match:
            continue
        try:
            payload = json.loads(match.group(0))
        except json.JSONDecodeError:
            continue
        if "elapsed_sec" in payload and "task_id" in payload:
            events.append(payload)
    terminal_ids = [
        event["task_id"]
        for event in events
        if event.get("state") == "completed"
    ]
    if not terminal_ids:
        raise ValueError(f"no completed mission in {path}")
    task_id = terminal_ids[-1]
    selected = [event for event in events if event["task_id"] == task_id]
    deduped = []
    for event in selected:
        signature = (
            event.get("elapsed_sec"),
            event.get("state"),
            event.get("detail"),
            event.get("cube_name"),
        )
        if deduped and deduped[-1][0] == signature:
            continue
        deduped.append((signature, event))
    return [event for _, event in deduped]


def analyze(path: Path) -> dict:
    text = path.read_text(encoding="utf-8", errors="ignore")
    events = mission_events(path)
    stages = defaultdict(float)
    cubes = defaultdict(lambda: defaultdict(float))
    transitions = []
    for current, following in zip(events, events[1:]):
        gap = round(following["elapsed_sec"] - current["elapsed_sec"], 3)
        if gap < 0:
            continue
        stage = current.get("state", "unknown")
        # The coordinator deliberately keeps the previously completed cube in
        # the first navigating_pickup message.  The following recognition
        # event is the first payload carrying the newly selected cube.
        if stage == "navigating_pickup" and following.get("cube_name"):
            cube = following["cube_name"]
        else:
            cube = current.get("cube_name") or "startup"
        stages[stage] += gap
        cubes[cube][stage] += gap
        transitions.append(
            (current["elapsed_sec"], gap, cube, stage, current["detail"])
        )

    nav_legs = []
    sides = {}
    active = None
    current_cube = None
    for line in text.splitlines():
        match = (
            JSON_RE.search(line)
            if "[competition_mission_coordinator]: {" in line
            else None
        )
        if match:
            try:
                payload = json.loads(match.group(0))
            except json.JSONDecodeError:
                payload = {}
            current_cube = payload.get("cube_name") or current_cube
        if "cube_selection_override name=" in line:
            current_cube = line.split(
                "cube_selection_override name=", 1
            )[1].split()[0]
            if " side=" in line:
                sides[current_cube] = line.split(" side=", 1)[1].split()[0]
        start = NAV_START_RE.search(line)
        if start:
            active = {
                "cube": current_cube or "startup",
                "location": start.group(1),
                "x": float(start.group(2)),
                "y": float(start.group(3)),
                "yaw": float(start.group(4)),
                "near_slow": 0,
                "rpp_collision": 0,
                "prealign": None,
            }
            continue
        if active and "Robot to slowdown" in line and "NearSlow" in line:
            active["near_slow"] += 1
        if active and "detected collision ahead" in line:
            active["rpp_collision"] += 1
        prealign = PREALIGN_RE.search(line)
        if active and prealign:
            active["prealign"] = tuple(
                float(value) for value in prealign.groups()
            )
        result = NAV_RESULT_RE.search(line)
        if result and active:
            active["result"] = result.group(1)
            active["duration"] = float(result.group(2))
            nav_legs.append(active)
            active = None

    terminal = events[-1]
    order = []
    for event in events:
        cube = event.get("cube_name")
        if (
            event.get("state") == "waiting_perception"
            and cube
            and cube not in order
        ):
            order.append(cube)
    jobs = {}
    for event in events:
        cube = event.get("cube_name")
        if event.get("state") == "waiting_perception" and cube:
            jobs[cube] = (event.get("color"), event.get("target_zone"))
    return {
        "path": path,
        "total": terminal["elapsed_sec"],
        "completed": terminal.get("completed"),
        "skipped": terminal.get("skipped"),
        "order": order,
        "sides": sides,
        "jobs": jobs,
        "stages": stages,
        "cubes": cubes,
        "transitions": transitions,
        "nav_legs": nav_legs,
        "retries": text.count("nav_retry"),
        "slow_recoveries": text.count("slow_progress recovery"),
        "bounded_recoveries": text.count("Bounded recovery"),
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("logs", nargs="+", type=Path)
    parser.add_argument("--budget", type=float, default=290.0)
    parser.add_argument(
        "--task",
        choices=("AB", "AC", "BC"),
        help="keep only 3 red to the first zone and 2 blue to the second",
    )
    parser.add_argument("--details", action="store_true")
    args = parser.parse_args()
    runs = [analyze(path) for path in args.logs]
    if args.task:
        red_zone, blue_zone = args.task
        runs = [
            run
            for run in runs
            if sum(
                job == ("red", red_zone)
                for job in run["jobs"].values()
            ) == 3
            and sum(
                job == ("blue", blue_zone)
                for job in run["jobs"].values()
            ) == 2
        ]
    if not runs:
        raise SystemExit("no matching completed runs")
    baseline = min(run["total"] for run in runs)

    def run_name(run: dict) -> str:
        path = run["path"]
        return path.parent.name if path.name == "launch.log" else path.stem

    stage_names = [
        "navigating_pickup",
        "waiting_perception",
        "picking",
        "navigating_departure",
        "navigating_destination",
        "placing",
        "verifying",
        "navigating_zone_exit",
    ]
    header = [
        "run", "total", "delta", "result", *stage_names,
        "retry/slow/bounded",
    ]
    print("\t".join(header))
    for run in runs:
        values = [
            run_name(run),
            f'{run["total"]:.1f}',
            f'{run["total"] - baseline:+.1f}',
            "PASS" if run["total"] <= args.budget else "FAIL",
            *(f'{run["stages"].get(stage, 0.0):.1f}' for stage in stage_names),
            (
                f'{run["retries"]}/{run["slow_recoveries"]}/'
                f'{run["bounded_recoveries"]}'
            ),
        ]
        print("\t".join(values))

    print("\nPer-cube totals (stage transition timing):")
    for run in runs:
        parts = []
        for cube in run["order"]:
            total = sum(run["cubes"][cube].values())
            side = run["sides"].get(cube, "auto")
            parts.append(f"{cube}/{side}={total:.1f}")
        print(f'{run_name(run)}: ' + "  ".join(parts))

    print("\nNavigation legs >= 10s:")
    for run in runs:
        legs = [
            (
                f'{leg["cube"]}:{leg["location"]}={leg["duration"]:.1f}'
                f'@({leg["x"]:.1f},{leg["y"]:.1f},{leg["yaw"]:.2f})'
                f'[slow={leg["near_slow"]},rpp={leg["rpp_collision"]}]'
                + (
                    f'[align=d{leg["prealign"][0]:.1f},'
                    f'e{leg["prealign"][1]:.2f}]'
                    if leg["prealign"] else ""
                )
            )
            for leg in run["nav_legs"]
            if leg["duration"] >= 10.0
        ]
        print(f'{run_name(run)}: ' + "  ".join(legs))

    if args.details:
        print("\nPer-cube stage matrix:")
        cube_stage_names = [
            "navigating_pickup",
            "waiting_perception",
            "picking",
            "navigating_departure",
            "navigating_destination",
            "placing",
            "verifying",
            "navigating_zone_exit",
        ]
        for run in runs:
            print(f'== {run_name(run)}')
            print("cube\t" + "\t".join(cube_stage_names) + "\ttotal")
            for cube in run["order"]:
                values = [
                    run["cubes"][cube].get(stage, 0.0)
                    for stage in cube_stage_names
                ]
                print(
                    cube + "\t" + "\t".join(f"{value:.1f}" for value in values)
                    + f"\t{sum(values):.1f}"
                )

        print("\nTransitions >= 3s:")
        for run in runs:
            print(f'== {run_name(run)}')
            for start, gap, cube, stage, detail in run["transitions"]:
                if gap >= 3.0:
                    print(
                        f"{start:6.1f} +{gap:5.1f}  {cube:12s} "
                        f"{stage:24s} {detail}"
                    )


if __name__ == "__main__":
    main()
