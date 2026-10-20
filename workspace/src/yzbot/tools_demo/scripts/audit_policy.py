#!/usr/bin/env python3
"""Audit the pick policy against measured leg medians only.

Only two decisions change a clause's total: which cubes form the set, and
which member is picked first (every other member is a zone -> tray -> zone
round trip, so the tail order is irrelevant). This script enumerates those
decisions per task and reports the tasks where a *fully measured* alternative
beats the deployed entry by at least 3 s.
"""
import argparse
import json
import pathlib
import statistics
import sys

SCRIPTS = pathlib.Path("/home/ros/huxuyang/YZBot-224/workspace/src/yzbot/"
                       "tools_demo/scripts")
sys.path.insert(0, str(SCRIPTS))
import leg_cost_table as cost          # noqa: E402

ROOT = pathlib.Path("/home/ros/huxuyang/YZBot-224")
MIN_SAMPLES = 3


def med(table, key):
    values = table.get(key)
    if not values or len(values) < MIN_SAMPLES:
        return None
    return statistics.median(values)


def clause_total(order, clauses, pickup, delivery):
    """Measured cost of one full order, or None if a cell is unmeasured."""
    total, index, origin = 0.0, 0, "spawn"
    for count, colour, zone in clauses:
        for _position in range(count):
            cube = order[index]
            index += 1
            p = med(pickup, (origin, cube))
            d = med(delivery, (cube, zone))
            if p is None or d is None:
                return None
            total += p + d
            origin = zone
    return total


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--min-gain", type=float, default=3.0)
    options = parser.parse_args()
    pickup, delivery, _ = cost.collect(ROOT / "runs")
    tasks = json.loads(
        (ROOT / "workspace/src/yzbot/tools_demo/config/"
         "cube_order_policy.json").read_text(encoding="utf-8")
    )["tasks"]

    rows = []
    for tag in sorted(tasks):
        clauses = cost.task_clauses(tag)
        current = [item["cube"].lower() for item in tasks[tag]["order"]]
        cur_cost = clause_total(current, clauses, pickup, delivery)
        best, best_cost = None, None
        for order in cost.enumerate_orders(clauses):
            value = clause_total(order, clauses, pickup, delivery)
            if value is None:
                continue
            if best_cost is None or value < best_cost:
                best, best_cost = list(order), value
        if best is None:
            continue
        if cur_cost is None:
            rows.append((float("inf"), tag, "current has unmeasured cells",
                         None, best_cost, best))
            continue
        gain = cur_cost - best_cost
        if gain >= options.min_gain and sorted(best) != sorted(current):
            rows.append((gain, tag, "set", cur_cost, best_cost, best))
        elif gain >= options.min_gain:
            rows.append((gain, tag, "order", cur_cost, best_cost, best))
    rows.sort(reverse=True, key=lambda r: r[0])
    print(f"measured improvements >= {options.min_gain}s: {len(rows)}")
    for gain, tag, kind, cur, best, order in rows:
        short = " ".join(c.replace("_cube_", "") for c in order)
        shown = "unmeasured" if cur is None else f"{cur:.1f}"
        print(f"  {tag:10} {kind:6} cur={shown:>9} best={best:6.1f} "
              f"gain={gain:5.1f}  {short}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
