#!/usr/bin/env python3
"""E2a shadow evaluation on top of the snapshot-validated RouteModel.

Enumerates same-color subset/order/side choices for the two archived
representative tasks, scores them with measured per-class effective speeds
from the archived runs (chain_len/elapsed medians), and compares against
the actually-flown selection. Offline only; no control change.

Usage: python3 e2_shadow.py <sprint_dir> <out_json>
"""
import csv
import itertools
import json
import math
import os
import statistics
import sys

from e2_route_extract import RouteModel  # noqa: F401  (sets sys.path first)
import navigation_math as nm
import extract_e0 as e0


def load_measured_speeds(sprint_dir, out_dir):
    """Median effective speed per leg class from archived per-action CSVs."""
    import csv

    legs = {"inbound": [], "delivery": []}
    for name in os.listdir(out_dir):
        if not (name.startswith("per_action_") and name.endswith(".csv")):
            continue
        path = os.path.join(out_dir, name)
        run_id = name[len("per_action_"):-len(".csv")]
        with open(path, encoding="utf-8") as fh:
            for row in csv.DictReader(fh):
                if row["kind"] != "nav" or row["state"] != "succeeded":
                    continue
                if row["class"] not in legs:
                    continue
                if not row["wall_monotonic"] or not row["chain_len_m"]:
                    continue
                elapsed = float(row["wall_monotonic"])
                chain = float(row["chain_len_m"])
                if elapsed <= 0 or chain <= 0.5:
                    continue
                legs[row["class"]].append({
                    "run": run_id, "location": row["location"],
                    "side": row["side"], "chain": chain,
                    "elapsed": elapsed, "v": chain / elapsed})
    speeds = {}
    for cls, rows in legs.items():
        speeds[cls] = {
            "median_v": statistics.median(r["v"] for r in rows),
            "n": len(rows),
            "per_leg": rows,
        }
    return speeds


def chain_time(start_xy, poses, v):
    """Time estimate for one leg: start -> waypoints -> dock chain / v."""
    pts = [(float(start_xy[0]), float(start_xy[1]))]
    for p in poses:
        pt = (float(p.x), float(p.y))
        if pt != pts[-1]:
            pts.append(pt)
    if len(pts) < 2:
        return 0.0, 0.0
    total = 0.0
    for (x1, y1), (x2, y2) in zip(pts, pts[1:]):
        total += math.hypot(x2 - x1, y2 - y1)
    return total / v, total


def evaluate_task(model, v, sequence, start_pose):
    """sequence: list of (kind, name, side, zone).
    kind: 'cube' (inbound + its delivery) or 'origin' (inbound only)."""
    total = 0.0
    detail = []
    cur = start_pose
    last_pickup = None
    for kind, name, side, zone in sequence:
        if kind == "cube":
            side_chosen, entry, dock, in_poses = model.plan_inbound(
                name, cur, side)
            t_in, chain_in = chain_time(cur, in_poses, v)
            last_pickup = (
                (model.cube_docks[name].x + nm.CUBE_DOCK_STANDOFF,
                 model.cube_docks[name].y), side_chosen)
            d_poses, ddock = model.plan_delivery(
                zone, (entry.x, entry.y, entry.yaw), last_pickup[0],
                last_pickup[1])
            t_del, chain_del = chain_time((entry.x, entry.y), d_poses, v)
            total += t_in + t_del
            detail.append({"cube": name, "side": side_chosen, "zone": zone,
                           "in_chain": round(chain_in, 2),
                           "del_chain": round(chain_del, 2),
                           "in_s": round(t_in, 1), "del_s": round(t_del, 1)})
            cur = (entry.x, entry.y, entry.yaw)
        elif kind == "origin":
            # spawn -> universal staging, identical for every candidate
            side_chosen, entry, dock, in_poses = model.plan_inbound(
                name, cur, side)
            t_in, chain_in = chain_time(cur, in_poses, v)
            total += t_in
            detail.append({"cube": name, "side": side_chosen,
                           "origin_leg": True,
                           "in_chain": round(chain_in, 2),
                           "in_s": round(t_in, 1)})
            cur = (entry.x, entry.y, entry.yaw)
    return total, detail


def main():
    sprint_dir, out_json = sys.argv[1], sys.argv[2]
    out_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           "out")
    model = RouteModel()
    speeds = load_measured_speeds(sprint_dir, out_dir)
    v_in = speeds["inbound"]["median_v"]
    v_del = speeds["delivery"]["median_v"]
    print("measured v: inbound %.3f m/s (n=%d), delivery %.3f m/s (n=%d)"
          % (v_in, speeds["inbound"]["n"], v_del, speeds["delivery"]["n"]))

    results = {"v_inbound": v_in, "v_delivery": v_del,
               "speed_detail": {k: {"median_v": v["median_v"], "n": v["n"]}
                                for k, v in speeds.items()}}

    # ---- task 1bB4rA: 1 blue -> B, 4 red -> A ---------------------------
    # Origin staging pose after the universal gate leg (archived staging
    # anchor (0.00, 0.00, pi)); the spawn->staging cost is common.
    origin = (0.0, 0.0, math.pi)
    spawn = (15.02, -2.0, math.pi)
    spawn_cost, _ = evaluate_task(
        model, v_in, [("origin", "RED_CUBE_1", "east", "C")], spawn)

    blues = [c for c in nm.VALID_CUBES if c.startswith("BLUE")]
    reds = [c for c in nm.VALID_CUBES if c.startswith("RED")]

    def sides_for(cube):
        return model.approach_candidates(cube)

    best = []
    current = None
    current_sides = ("east", "west", "west", "east")
    for blue in blues:
        for blue_cand in sides_for(blue):
            blue_side = blue_cand.side
            # the task delivers exactly 4 of the 5 red entities: subset
            # and order both matter (5P4 = 120 ordered 4-subsets)
            for red_order in itertools.permutations(reds, 4):
                # sides per red cube from its allowed candidates
                side_lists = [[c.side for c in sides_for(r)]
                              for r in red_order]
                for red_sides in itertools.product(*side_lists):
                    seq = [("cube", blue, blue_side, "B")]
                    seq += [("cube", r, s, "A")
                            for r, s in zip(red_order, red_sides)]
                    try:
                        t, detail = evaluate_task(model, v_in, seq, origin)
                    except Exception:
                        continue
                    tag = "b:%s%s " % (blue.split("_")[-1], blue_side[0]) \
                        + " ".join(
                            "r:%s%s" % (d["cube"].split("_")[-1],
                                        d["side"][0])
                            for d in detail[1:])
                    if (blue, blue_side, red_order, red_sides) == (
                            "BLUE_CUBE_4", "east",
                            ("RED_CUBE_1", "RED_CUBE_2", "RED_CUBE_4",
                             "RED_CUBE_5"), current_sides):
                        current = {"tag": tag, "time": round(t, 1),
                                   "detail": detail}
                    best.append({"tag": tag, "time": round(t, 1),
                                 "blue": blue, "blue_side": blue_side,
                                 "red_order": red_order,
                                 "red_sides": red_sides, "detail": detail})
    best.sort(key=lambda d: d["time"])
    results["task_1bB4rA"] = {
        "spawn_leg_common_s": round(spawn_cost, 1),
        "current_from_log": current,
        "n_candidates": len(best),
        "top10": best[:10],
        "best": best[0],
    }
    if current:
        results["task_1bB4rA"]["saving_best_vs_current"] = round(
            current["time"] - best[0]["time"], 1)

    with open(out_json, "w", encoding="utf-8") as fh:
        json.dump(results, fh, indent=2, ensure_ascii=False, default=str)
    print("current:", json.dumps(current, default=str)[:300])
    print("top5:")
    for c in best[:5]:
        print("  %s  %.1f s" % (c["tag"], c["time"]))
    if current:
        print("saving best vs current: %.1f s"
              % results["task_1bB4rA"]["saving_best_vs_current"])


if __name__ == "__main__":
    main()
