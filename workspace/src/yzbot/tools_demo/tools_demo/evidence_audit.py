"""Audit motion benchmarks separately from full competition acceptance."""
from collections import Counter
import argparse
import json
import math
from tools_demo.acceptance_contract import validated_avoidance
from pathlib import Path
import re

from tools_demo.task_protocol import parse_task_question


def audit_run(question, result_text, launch_text, enforce_color_order=False):
    """Verify requested goods independently of the allowed delivery order."""
    plan = parse_task_question(question)
    expected = [(item.color, item.target_zone) for item in plan.items
                for _ in range(item.count)]
    mission, avoidance = {}, {}
    for line in result_text.splitlines():
        for marker, target in (("MISSION_TERMINAL=", mission),
                               ("AVOIDANCE_EVIDENCE=", avoidance)):
            if marker in line:
                target.clear()
                target.update(json.loads(line.split(marker, 1)[1]))
    picks = re.findall(r"manip_start operation=pick cube=(\w+)", launch_text)
    truth = re.findall(
        r"delivery_truth cube=(\w+) zone=([ABC]) .*?valid=(\w+) "
        r"slot_error=([\d.]+) precise=(\w+)", launch_text)
    delivered = [(cube.split("_", 1)[0], zone) for cube, zone, *_ in truth]
    pick_colors = [cube.split("_", 1)[0] for cube in picks]
    # Retries need manual timeline review; never silently discard attempts.
    ordered = (pick_colors == [color for color, _ in expected]
               and delivered == expected
               and picks == [row[0] for row in truth]
               and len(set(picks)) == len(expected))
    consistent = (picks == [row[0] for row in truth]
                  and len(set(picks)) == len(expected) == len(truth))
    requirements = consistent and Counter(delivered) == Counter(expected)
    remaining = Counter(expected)
    credited = set()
    for cube, zone, valid, error, exact in truth:
        identity = (cube.split("_", 1)[0], zone)
        if (cube in picks and cube not in credited and valid == "True"
                and exact == "True" and remaining[identity] > 0):
            credited.add(cube)
            remaining[identity] -= 1
    goods_score = 4 * len(credited)
    precise = bool(truth) and all(row[2] == "True" and row[4] == "True"
                                  for row in truth)
    responses = avoidance.get("response_kinds", {})
    dual_response = all(
        responses.get(f"competition_moving_obstacle_{i}") == "yield"
        for i in (1, 2))
    observed = (set(avoidance.get("encountered_obstacles", []))
                & set(avoidance.get("response_obstacles", [])))
    effective = {name for name, kind in responses.items()
                 if name in observed and kind in {"yield", "detour"}}
    two_responses = validated_avoidance(avoidance) and len(effective) >= 2
    clearance = avoidance.get("minimum_clearance")
    clearance_ok = (isinstance(clearance, (int, float))
                    and math.isfinite(clearance) and clearance > 0.1)
    functional = ("CASE_ACCEPTED=true collision_faults=[]" in result_text
                  and mission.get("state") == "completed"
                  and mission.get("completed") == len(expected) == 5
                  and mission.get("skipped") == 0 and requirements and precise
                  and (ordered or not enforce_color_order)
                  and two_responses and clearance_ok
                  and avoidance.get("safe") is True
                  and avoidance.get("collision_detected") is False)
    elapsed = mission.get("elapsed_sec")
    timed = (isinstance(elapsed, (int, float)) and math.isfinite(elapsed)
             and 0 < elapsed < 300)
    return {"question": question, "expected": expected, "pick_entities": picks,
            "delivery_truth": truth, "order_verified": ordered,
            "color_order_required": enforce_color_order,
            "requirements_verified": requirements,
            "pickup_delivery_consistent": consistent,
            "goods_score": goods_score,
            "truth_precise": precise, "dual_yield": dual_response,
            "two_dynamic_responses": two_responses,
            "minimum_clearance": clearance, "elapsed_sec": elapsed,
            "motion_functional_pass": functional,
            "motion_under_300": functional and timed,
            "motion_under_290": functional and timed and elapsed < 290,
            "full_competition_acceptance": "not_tested",
            "limits": ["LLM/question-generation/dashboard timing excluded",
                       "truth may follow existing entity-pose stabilization",
                       "repeated manipulation attempts require manual review"]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    parser.add_argument("question")
    args = parser.parse_args()
    report = audit_run(
        args.question,
        (args.directory / "result.log").read_text(errors="replace"),
        (args.directory / "launch.log").read_text(errors="replace"))
    (args.directory / "audit.json").write_text(
        json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
