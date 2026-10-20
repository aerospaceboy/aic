#!/usr/bin/env python3
"""E0 per-action timetable extraction from archived sprint logs.

Reads unpacked run directories of yzbot-sprint-20260905-evidence.tar.gz and
produces per-action CSV rows plus a summary markdown. Log wall timestamps are
used only for ordering/association; navigation durations prefer the
monotonic-based nav_result elapsed, and event-pair wall deltas are reported
separately so the two can be cross-checked. This is an offline diagnostic and
does not touch any control behavior.

Usage: python3 extract_e0.py <sprint_dir> <out_dir>
"""
import csv
import json
import math
import os
import re
import sys

LINE_RE = re.compile(
    r"^\[(?P<node>[\w]+)-\d+\] \[(?P<level>INFO|WARN|ERROR)\] "
    r"\[(?P<sec>\d+)\.(?P<nanos>\d+)\] \[(?P<logger>[\w_]+)\]: (?P<msg>.*)$")

# result.log lines have no node-pid prefix: [INFO] [sec.nanos] [logger]: msg
RESULT_LINE_RE = re.compile(
    r"^\[(?P<level>INFO|WARN|ERROR)\] "
    r"\[(?P<sec>\d+)\.(?P<nanos>\d+)\] \[(?P<logger>[\w_]+)\]: (?P<msg>.*)$")

NUM = r"[-\d.]+"

EVENT_PATTERNS = [
    ("nav_start", re.compile(
        r"nav_start location=(?P<location>\S+) side=(?P<side>\S+) "
        r"robot=\((?P<x>%s),(?P<y>%s),yaw=(?P<yaw>%s)\)" % (NUM, NUM, NUM))),
    ("nav_result", re.compile(
        r"nav_result state=(?P<state>\S+) elapsed=(?P<elapsed>[\d.]+)s")),
    ("nav_plan", re.compile(
        r"nav_plan zone=(?P<zone>\S+) poses=(?P<poses>.*?) "
        r"robot=\((?P<x>%s),(?P<y>%s),yaw=(?P<yaw>%s)\)" % (NUM, NUM, NUM))),
    ("nav_goal_batch", re.compile(
        r"nav_goal_batch zone=(?P<zone>\S+) stages=(?P<stages>\d+) "
        r"first=\((?P<fx>%s),(?P<fy>%s)\) final=\((?P<lx>%s),(?P<ly>%s)\)"
        % (NUM, NUM, NUM, NUM))),
    ("nav_goal", re.compile(r"nav_goal zone=(?P<zone>\S+)")),
    ("nav_handoff", re.compile(r"nav_handoff zone=(?P<zone>\S+)")),
    ("cube_entry_choice", re.compile(
        r"cube_entry_choice zone=(?P<zone>\S+) side=(?P<side>\S+) "
        r"entry=\((?P<ex>%s),(?P<ey>%s),yaw=(?P<eyaw>%s)\) "
        r"dock=\((?P<dx>%s),(?P<dy>%s)\)" % (NUM, NUM, NUM, NUM, NUM))),
    ("alignment_start", re.compile(r"alignment_start phase=(?P<phase>\S+)")),
    ("manip_start", re.compile(
        r"manip_start operation=(?P<op>\S+) cube=(?P<cube>\S+) "
        r"robot=\((?P<x>%s),(?P<y>%s),yaw=(?P<yaw>%s)\)" % (NUM, NUM, NUM))),
    ("manip_request", re.compile(r"manip_request id=(?P<id>\d+)")),
    ("persistent_request", re.compile(r"persistent_request id=(?P<id>\d+)")),
    ("persistent_result", re.compile(
        r"persistent_result id=(?P<id>\d+) success=(?P<success>\w+) "
        r"detail=(?P<detail>.*)")),
    ("delivery_truth", re.compile(
        r"delivery_truth cube=(?P<cube>\S+) zone=(?P<zone>\S+) x=(?P<x>%s) "
        r"y=(?P<y>%s) z=(?P<z>%s) valid=(?P<valid>\w+) "
        r"slot_error=(?P<slot_error>%s) precise=(?P<precise>\w+)"
        % (NUM, NUM, NUM, NUM))),
    ("world_slot_stabilized", re.compile(
        r"world_slot_stabilized error=(?P<err>%s)" % NUM)),
    ("batch_fallback_serial", re.compile(
        r"batch_fallback_serial reason=(?P<reason>\S+) remaining=(?P<remaining>\d+)")),
    ("tray_crossing_route", re.compile(r"tray_crossing_route zone=(?P<zone>\S+)")),
    ("pickup_entry_alignment", re.compile(
        r"pickup_entry_alignment zone=(?P<zone>\S+)")),
    ("waiting_perception", re.compile(r"waiting_perception")),
]


def parse_log(path):
    events = []
    with open(path, "r", encoding="utf-8", errors="replace") as fh:
        for line in fh:
            m = LINE_RE.match(line.rstrip("\n"))
            if not m:
                continue
            t = float(m.group("sec")) + float(m.group("nanos")) * 1e-9
            msg = m.group("msg")
            name, fields = None, {}
            for ev_name, pat in EVENT_PATTERNS:
                fm = pat.search(msg)
                if fm:
                    name, fields = ev_name, fm.groupdict()
                    break
            if name is None:
                tok = msg.split(" ", 1)[0]
                if re.fullmatch(r"[a-z_]+", tok):
                    name, fields = tok, {"raw": msg}
                else:
                    continue
            events.append({"t": t, "logger": m.group("logger"),
                           "name": name, "fields": fields, "msg": msg})
    events.sort(key=lambda e: e["t"])
    return events


def parse_result_log(path):
    out = {"task": None, "t_task": None, "terminal": None,
           "elapsed_sec": None, "avoidance": None, "min_center": None}
    with open(path, "r", encoding="utf-8", errors="replace") as fh:
        for line in fh:
            m = RESULT_LINE_RE.match(line.rstrip("\n"))
            if not m:
                continue
            t = float(m.group("sec")) + float(m.group("nanos")) * 1e-9
            msg = m.group("msg")
            if msg.startswith("TASK_PUBLISHED="):
                out["task"] = msg.split("=", 1)[1]
                out["t_task"] = t
            elif msg.startswith("MISSION_TERMINAL="):
                try:
                    out["terminal"] = json.loads(msg.split("=", 1)[1])
                except json.JSONDecodeError:
                    pass
            elif msg.startswith("AVOIDANCE_EVIDENCE="):
                try:
                    out["avoidance"] = json.loads(msg.split("=", 1)[1])
                except json.JSONDecodeError:
                    pass
            elif msg.startswith("MINIMUM_CENTER_DISTANCE="):
                out["min_center"] = msg.split("=", 1)[1]
    if out["terminal"]:
        out["elapsed_sec"] = out["terminal"].get("elapsed_sec")
    return out


def pose_stream(events):
    pts = []
    for e in events:
        f = e["fields"]
        if e["name"] in ("nav_start", "manip_start") and "x" in f:
            pts.append((e["t"], float(f["x"]), float(f["y"]),
                        float(f.get("yaw", 0.0)), e["name"]))
    return pts


def pose_at(pts, t, max_lag=10.0):
    """First pose reported at/after t, or None if it is much later (which
    would silently attribute a following action's movement to this one)."""
    for pt in pts:
        if t <= pt[0] <= t + max_lag:
            return pt
    return None


def dist(x1, y1, x2, y2):
    return math.hypot(x2 - x1, y2 - y1)


def chain_len(poses_str, start_xy=None):
    """Sum of consecutive waypoint distances (plus robot start -> first
    waypoint when given); a lower bound of the true path."""
    pts = re.findall(r"\((%s),(%s),yaw=%s\)" % (NUM, NUM, NUM),
                     poses_str or "")
    coords = [(float(a), float(b)) for a, b in pts]
    if start_xy is not None:
        coords = [start_xy] + coords
    if len(coords) < 2:
        return None
    total = 0.0
    for (x1, y1), (x2, y2) in zip(coords, coords[1:]):
        total += dist(x1, y1, x2, y2)
    return total


def classify_location(loc):
    if "CUBE" in loc:
        return "inbound"
    if loc.endswith("_EXIT"):
        return "exit"
    if loc in ("A", "B", "C"):
        return "delivery"
    return "other"


def extract_run(run_dir, run_id, out_dir):
    events = parse_log(os.path.join(run_dir, "launch.log"))
    result = parse_result_log(os.path.join(run_dir, "result.log"))
    t0 = result.get("t_task") or (events[0]["t"] if events else 0.0)
    pts = pose_stream(events)

    rows = []
    seq = 0
    i = 0
    n = len(events)
    while i < n:
        e = events[i]
        f = e["fields"]
        if e["name"] == "nav_start":
            seq += 1
            end = None
            detail = []
            j = i + 1
            while j < n:
                if events[j]["name"] in ("nav_start", "manip_start"):
                    break
                if events[j]["name"] == "nav_result":
                    end = events[j]
                    break
                detail.append(events[j])
                j += 1
            end_t = end["t"] if end else e["t"]
            end_pose = pose_at(pts, end_t)
            x0, y0 = float(f["x"]), float(f["y"])
            if end_pose:
                disp = dist(x0, y0, end_pose[1], end_pose[2])
            else:
                disp = None
            plans = [d for d in detail if d["name"] == "nav_plan"]
            batches = [d for d in detail if d["name"] == "nav_goal_batch"]
            entries = [d for d in detail if d["name"] == "cube_entry_choice"]
            fallbacks = [d for d in detail
                         if d["name"] == "batch_fallback_serial"]
            route = "direct"
            stages = 0
            if batches:
                route = "batch"
                stages = int(batches[0]["fields"]["stages"])
            elif any(d["name"] in ("nav_goal", "nav_handoff") for d in detail):
                route = "single"
            plan_pts = plans[0]["fields"]["poses"] if plans else ""
            cl = None
            if plan_pts:
                cl = chain_len(plan_pts, (x0, y0))
            if cl is None and route == "direct" and end_pose:
                cl = dist(x0, y0, end_pose[1], end_pose[2])
            loc = f["location"]
            rows.append({
                "run_id": run_id, "seq": seq, "kind": "nav",
                "class": classify_location(loc), "location": loc,
                "side": f.get("side", ""), "cube": "",
                "zone": (entries[0]["fields"]["zone"]
                         if entries else (plans[0]["fields"]["zone"]
                                          if plans else loc)),
                "t_offset": round(e["t"] - t0, 3),
                "wall_monotonic": float(end["fields"]["elapsed"])
                if end else "",
                "wall_event_delta": round(end_t - e["t"], 3)
                if end else "",
                "state": end["fields"]["state"] if end else "UNPAIRED",
                "route": route, "stages": stages,
                "chain_len_m": round(cl, 3) if cl is not None else "",
                "displacement_m": round(disp, 3) if disp is not None else "",
                "pose_start": "(%.3f,%.3f,%.3f)" % (x0, y0, float(f["yaw"])),
                "pose_end": ("(%.3f,%.3f,%.3f)" % end_pose[1:4])
                if end_pose else "",
                "detail_events": ",".join(
                    sorted(set(d["name"] for d in detail))),
                "fallbacks": ";".join(
                    "%s rem=%s" % (d["fields"]["reason"],
                                   d["fields"]["remaining"])
                    for d in fallbacks),
                "success": "",
                "precise": "", "slot_error": "", "notes": "",
            })
            i = j + 1 if end else j
            continue
        if e["name"] == "manip_start":
            seq += 1
            end = None
            j = i + 1
            while j < n:
                if events[j]["name"] in ("manip_start", "nav_start"):
                    break
                if events[j]["name"] == "persistent_result":
                    end = events[j]
                    break
                j += 1
            end_t = end["t"] if end else e["t"]
            end_pose = pose_at(pts, end_t)
            x0, y0 = float(f["x"]), float(f["y"])
            disp = dist(x0, y0, end_pose[1], end_pose[2]) if end_pose else None
            rows.append({
                "run_id": run_id, "seq": seq, "kind": "manip",
                "class": f["op"], "location": "", "side": "",
                "cube": f["cube"], "zone": "",
                "t_offset": round(e["t"] - t0, 3),
                "wall_monotonic": "",
                "wall_event_delta": round(end_t - e["t"], 3),
                "state": end["fields"]["success"] if end else "UNPAIRED",
                "route": "", "stages": "", "chain_len_m": "",
                "displacement_m": round(disp, 3) if disp is not None else "",
                "pose_start": "(%.3f,%.3f,%.3f)" % (x0, y0, float(f["yaw"])),
                "pose_end": ("(%.3f,%.3f,%.3f)" % end_pose[1:4])
                if end_pose else "",
                "detail_events": "",
                "fallbacks": "",
                "success": end["fields"]["detail"] if end else "",
                "precise": "", "slot_error": "", "notes": "",
            })
            i = j + 1 if end else j
            continue
        if e["name"] == "delivery_truth":
            seq += 1
            rows.append({
                "run_id": run_id, "seq": seq, "kind": "delivery_truth",
                "class": "delivery_truth", "location": "", "side": "",
                "cube": f["cube"], "zone": f["zone"],
                "t_offset": round(e["t"] - t0, 3),
                "wall_monotonic": "", "wall_event_delta": "", "state": "",
                "route": "", "stages": "", "chain_len_m": "",
                "displacement_m": "", "pose_start": "", "pose_end": "",
                "detail_events": "", "fallbacks": "",
                "success": f["valid"], "precise": f["precise"],
                "slot_error": f["slot_error"],
                "notes": "x=%s y=%s z=%s" % (f["x"], f["y"], f["z"]),
            })
            i += 1
            continue
        i += 1

    csv_path = os.path.join(out_dir, "per_action_%s.csv" % run_id)
    if rows:
        with open(csv_path, "w", newline="", encoding="utf-8") as fh:
            w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
            w.writeheader()
            w.writerows(rows)

    nav_rows = [r for r in rows if r["kind"] == "nav"]
    manip_rows = [r for r in rows if r["kind"] == "manip"]
    deliveries = [r for r in rows if r["kind"] == "delivery_truth"]

    def fnum(r, k):
        v = r[k]
        return float(v) if v != "" else None

    nav_mono = sum(v for r in nav_rows
                   if (v := fnum(r, "wall_monotonic")) is not None)
    nav_delta = sum(v for r in nav_rows
                    if (v := fnum(r, "wall_event_delta")) is not None)
    manip_delta = sum(v for r in manip_rows
                      if (v := fnum(r, "wall_event_delta")) is not None)
    total = result.get("elapsed_sec")
    summary = {
        "run_id": run_id,
        "task": result.get("task"),
        "terminal": result.get("terminal"),
        "total_elapsed_sec": total,
        "event_span_sec": round(events[-1]["t"] - t0, 2) if events else None,
        "nav_sum_monotonic": round(nav_mono, 2),
        "nav_sum_event_delta": round(nav_delta, 2),
        "manip_sum_event_delta": round(manip_delta, 2),
        "residual_vs_total": round(total - nav_mono - manip_delta, 2)
        if total else None,
        "nav_count": len(nav_rows),
        "manip_count": len(manip_rows),
        "delivery_count": len(deliveries),
        "nav_class_sums": {
            cls: round(sum(v for r in nav_rows
                           if r["class"] == cls
                           and (v := fnum(r, "wall_monotonic")) is not None), 2)
            for cls in ("inbound", "delivery", "exit", "other")
        },
        "top5_actions": sorted(
            [{"kind": r["kind"], "class": r["class"],
              "location": r["location"] or r["zone"], "cube": r["cube"],
              "sec": fnum(r, "wall_monotonic") or fnum(r, "wall_event_delta"),
              "displacement_m": fnum(r, "displacement_m"),
              "chain_len_m": fnum(r, "chain_len_m")}
             for r in rows],
            key=lambda d: d["sec"] or 0.0, reverse=True)[:5],
        "top5_low_translation_nav": sorted(
            [{"location": r["location"], "side": r["side"],
              "sec": fnum(r, "wall_monotonic"),
              "displacement_m": fnum(r, "displacement_m"),
              "chain_len_m": fnum(r, "chain_len_m"),
              "route": r["route"], "state": r["state"]}
             for r in nav_rows
             if fnum(r, "wall_monotonic") is not None
             and fnum(r, "displacement_m") is not None
             and fnum(r, "displacement_m") < 1.0],
            key=lambda d: d["sec"], reverse=True)[:5],
        "nav_segments_chain_vs_elapsed": [
            {"location": r["location"], "side": r["side"],
             "route": r["route"], "stages": r["stages"],
             "elapsed": fnum(r, "wall_monotonic"),
             "chain_len_m": fnum(r, "chain_len_m"),
             "disp_m": fnum(r, "displacement_m"),
             "eff_mps": round(fnum(r, "chain_len_m")
                              / fnum(r, "wall_monotonic"), 3)
             if fnum(r, "chain_len_m") and fnum(r, "wall_monotonic")
             else None}
            for r in nav_rows],
        "avoidance_minimum_clearance": (
            result.get("avoidance") or {}).get("minimum_clearance"),
        "min_center_distance": result.get("min_center"),
    }
    return summary, rows


def main():
    sprint_dir, out_dir = sys.argv[1], sys.argv[2]
    os.makedirs(out_dir, exist_ok=True)
    runs = sorted(d for d in os.listdir(sprint_dir)
                  if os.path.isdir(os.path.join(sprint_dir, d)))
    summaries = []
    for run_id in runs:
        run_dir = os.path.join(sprint_dir, run_id)
        if not os.path.isfile(os.path.join(run_dir, "launch.log")):
            continue
        summary, _ = extract_run(run_dir, run_id, out_dir)
        summaries.append(summary)
        print("parsed", run_id)
    with open(os.path.join(out_dir, "e0_summaries.json"), "w",
              encoding="utf-8") as fh:
        json.dump(summaries, fh, indent=2, ensure_ascii=False)
    print("wrote", os.path.join(out_dir, "e0_summaries.json"))


if __name__ == "__main__":
    main()
