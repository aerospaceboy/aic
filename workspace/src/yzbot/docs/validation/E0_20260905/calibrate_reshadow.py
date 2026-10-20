#!/usr/bin/env python3
"""Calibrate the E2 shadow cost model with actual /plan lengths.

Joins each rep bag's plan publications (in order) to the non-direct nav
segments of the matching per-action table (in order), yielding per-leg
actual path lengths and actual blended speeds. Reports the plan_len /
waypoint-chain inflation per class and re-ranks the 1bB4rA shadow with
calibrated speeds.

Usage: python3 calibrate_reshadow.py <exp_dir>
"""
import csv
import json
import math
import os
import statistics
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from e2_route_extract import RouteModel  # noqa: E402
import e2_shadow as sh  # noqa: E402


def load_nav_segments(exp_dir, run_id):
    for sub in ("e1_out", "out"):
        path = os.path.join(exp_dir, sub, "per_action_%s.csv" % run_id)
        if os.path.isfile(path):
            break
    else:
        return [], None
    rows = []
    with open(path, encoding="utf-8") as fh:
        for row in csv.DictReader(fh):
            if row["kind"] != "nav" or row["state"] != "succeeded":
                continue
            if not row["wall_monotonic"]:
                continue
            rows.append(row)
    import extract_e0 as e0
    result = e0.parse_result_log(os.path.join(
        exp_dir, "e1_runs", run_id, "result.log"))
    return rows, result.get("t_task")


def analyse(exp_dir, run_id):
    import analyse_bag as ab

    messages, _ = ab.read_bag(os.path.join(exp_dir, "e1_runs", run_id,
                                           "bag"))
    plans = messages["plan"]
    odom = messages["odom"]
    nav, t_task = load_nav_segments(exp_dir, run_id)
    if t_task is None:
        print("  %s: no TASK_PUBLISHED time" % run_id)
        return []
    joined = []
    for row in nav:
        if row["route"] not in ("batch", "single"):
            continue
        w0 = t_task + float(row["t_offset"])
        w1 = w0 + float(row["wall_event_delta"]
                        or row["wall_monotonic"])
        in_window = [p for p in plans if w0 - 1.0 <= p[0] <= w1]
        travelled = 0.0
        prev = None
        for o in odom:
            if w0 - 1.0 <= o[0] <= w1:
                pt = (o[3], o[4])
                if prev is not None:
                    step = math.hypot(pt[0] - prev[0], pt[1] - prev[1])
                    if step < 1.0:  # reject odom jumps
                        travelled += step
                prev = pt
            elif o[0] > w1:
                break
        joined.append({
            "run": run_id,
            "location": row["location"], "side": row["side"],
            "class": row["class"],
            "elapsed": float(row["wall_monotonic"]),
            "chain": float(row["chain_len_m"])
            if row["chain_len_m"] else None,
            "plan_len": max((p[2] for p in in_window), default=None),
            "n_plans": len(in_window),
            "odom_travelled": round(travelled, 2),
        })
    return joined


def main():
    exp_dir = sys.argv[1]
    all_joined = []
    for run_id in ("e1rep1", "e1rep2", "e1rep3"):
        joined = analyse(exp_dir, run_id)
        print("%s: joined %d legs" % (run_id, len(joined)))
        all_joined.extend(joined)

    inflation = {}
    speed = {}
    for cls in ("inbound", "delivery"):
        rows = [r for r in all_joined if r["class"] == cls
                and r["elapsed"] > 0]
        ratios = [r["odom_travelled"] / r["chain"] for r in rows
                  if r["chain"] and r["chain"] > 0.5
                  and r["odom_travelled"] > 0.5]
        speeds = [r["odom_travelled"] / r["elapsed"] for r in rows
                  if r["odom_travelled"] > 0.5]
        inflation[cls] = {
            "median_travelled_vs_chain": round(statistics.median(ratios), 3)
            if ratios else None,
            "ratio_max": round(max(ratios), 3) if ratios else None,
            "median_speed_mps": round(statistics.median(speeds), 3),
            "n": len(rows),
        }
        speed[cls] = statistics.median(speeds)
    print("calibration:", json.dumps(inflation, indent=1))

    # Re-rank the 1bB4rA shadow with calibrated speeds and chain
    # inflation (the enumeration only compares, so a consistent class
    # inflation preserves fairness while fixing the lower-bound bias).
    model = RouteModel()
    v_in = speed["inbound"]
    v_del = speed["delivery"]

    def evaluate_with_inflation(sequence, start):
        total = 0.0
        detail = []
        cur = start
        last_pickup = None
        for kind, name, side, zone in sequence:
            if kind == "cube":
                side_chosen, entry, dock, in_poses = model.plan_inbound(
                    name, cur, side)
                t_in, chain_in = sh.chain_time(cur, in_poses, 1.0)
                last_pickup = (
                    (model.cube_docks[name].x + 0.36,
                     model.cube_docks[name].y), side_chosen)
                d_poses, _ = model.plan_delivery(
                    zone, (entry.x, entry.y, entry.yaw),
                    last_pickup[0], last_pickup[1])
                t_del, chain_del = sh.chain_time((entry.x, entry.y),
                                                 d_poses, 1.0)
                t_in = t_in * inflation["inbound"][
                    "median_travelled_vs_chain"] / v_in
                t_del = t_del * inflation["delivery"][
                    "median_travelled_vs_chain"] / v_del
                total += t_in + t_del
                detail.append({"cube": name, "side": side_chosen,
                               "in_s": round(t_in, 1),
                               "del_s": round(t_del, 1)})
                cur = (entry.x, entry.y, entry.yaw)
        return total, detail

    import itertools
    import navigation_math as nm

    origin = (0.0, 0.0, math.pi)
    blues = [c for c in nm.VALID_CUBES if c.startswith("BLUE")]
    reds = [c for c in nm.VALID_CUBES if c.startswith("RED")]
    best = []
    current = None
    for blue in blues:
        for blue_cand in model.approach_candidates(blue):
            for red_order in itertools.permutations(reds, 4):
                side_lists = [[c.side for c in
                               model.approach_candidates(r)]
                              for r in red_order]
                for red_sides in itertools.product(*side_lists):
                    seq = [("cube", blue, blue_cand.side, "B")] + [
                        ("cube", r, s, "A")
                        for r, s in zip(red_order, red_sides)]
                    try:
                        t, detail = evaluate_with_inflation(seq, origin)
                    except Exception:
                        continue
                    tag = "b:%s%s " % (blue.split("_")[-1],
                                       blue_cand.side[0]) + " ".join(
                        "r:%s%s" % (d["cube"].split("_")[-1],
                                    d["side"][0]) for d in detail)
                    if (blue, blue_cand.side, red_order, red_sides) == (
                            "BLUE_CUBE_4", "east",
                            ("RED_CUBE_1", "RED_CUBE_2", "RED_CUBE_4",
                             "RED_CUBE_5"),
                            ("east", "west", "west", "east")):
                        current = {"tag": tag, "time": round(t, 1)}
                    best.append({"tag": tag, "time": round(t, 1),
                                 "detail": detail})
    best.sort(key=lambda d: d["time"])
    result = {"calibration": inflation, "per_leg": all_joined,
              "current": current,
              "top10": [{k: v for k, v in c.items() if k != "detail"}
                        for c in best[:10]],
              "saving_vs_current": round(current["time"] - best[0]["time"],
                                         1) if current else None}
    with open(os.path.join(exp_dir, "e1_out", "calibrated_shadow.json"),
              "w", encoding="utf-8") as fh:
        json.dump(result, fh, indent=2)
    print("current:", current)
    print("top5 with calibrated speeds:")
    for c in best[:5]:
        print("  %s  %.1f s" % (c["tag"], c["time"]))
    if current:
        print("saving vs current: %.1f s" % result["saving_vs_current"])


if __name__ == "__main__":
    main()
