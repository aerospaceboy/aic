#!/usr/bin/env python3
"""E2b step 1: offline route-description extractor, snapshot-validated.

Reimplements only the thin conditional glue of task_navigator route
construction on top of the existing pure navigation_math module, loads the
same yaml parameters, and validates itself against every archived
cube_entry_choice / nav_plan snapshot from the sprint 20260905 runs.
No production code is changed. When snapshots validate, the same extractor
provides shadow costs for candidate cube choices (E2a).

Usage: python3 e2_route_extract.py <sprint_dir> [out_json]
"""
import json
import math
import os
import re
import sys
from pathlib import Path

import yaml

REPO = Path("/home/ros/dev_ws/src/yzbot")
sys.path.insert(0, str(REPO / "tools_demo" / "tools_demo"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import navigation_math as nm  # pure module: math only

ZONES_YAML = REPO / "bot_navigation" / "config" / "competition_zones.yaml"
RISK_YAML = REPO / "bot_navigation" / "config" / "navigation_risk_zones.yaml"

# Mirrored from task_navigator.py:108-124 (snapshot validation below proves
# the mirror matches the running navigator).
CUBE_APPROACH_WAYPOINTS_MIRROR = {
    "BLUE_CUBE_3": [-7.0, -6.0, 0.0],
}
OBSTACLE_2_NORTH_SOUTH_GATES = (
    nm.ZonePose("obstacle_2_north_gate", -2.10, 1.80, -math.pi / 2.0),
    nm.ZonePose("obstacle_2_south_gate", -2.10, 0.00, -1.48),
)
OBSTACLE_2_LOWER_RIGHT_GATE = nm.ZonePose(
    "obstacle_2_lower_right", -1.40, -7.60, 3.03)
OBSTACLE_2_WEST_UPPER_GATE = nm.ZonePose(
    "obstacle_2_west_upper", -4.20, 0.00, -2.58)

POS_TOL = 0.02
YAW_TOL = 0.06


def load_params():
    payload = yaml.safe_load(ZONES_YAML.read_text(encoding="utf-8"))
    return (
        payload["competition_task_navigator"]["ros__parameters"],
        payload.get("competition_mission_coordinator", {})
        .get("ros__parameters", {}),
    )


def parse_pose(name, values):
    return nm.parse_location_pose(name, values)


class RouteModel:
    """Mirror of the navigator's route construction conditions."""

    def __init__(self):
        self.params, _ = load_params()
        risk = yaml.safe_load(RISK_YAML.read_text(encoding="utf-8"))
        self.profiles = risk.get("zone_strategies", {})
        self.zones = {
            z: parse_pose(z, self.params[f"zone_{z.lower()}"])
            for z in ("A", "B", "C")
        }
        self.cube_docks = {}
        self.cube_entries = {}
        self.cube_docks_inner = {}
        self.cube_entries_inner = {}
        for cube in nm.VALID_CUBES:
            key = cube.lower()
            if key in self.params:
                self.cube_docks[cube] = parse_pose(cube, self.params[key])
            if f"{key}_entry" in self.params:
                self.cube_entries[cube] = parse_pose(
                    f"{cube}_ENTRY", self.params[f"{key}_entry"])
            if f"{key}_dock_inner" in self.params:
                self.cube_docks_inner[cube] = parse_pose(
                    cube, self.params[f"{key}_dock_inner"])
            if f"{key}_entry_inner" in self.params:
                self.cube_entries_inner[cube] = parse_pose(
                    f"{cube}_ENTRY", self.params[f"{key}_entry_inner"])
        # task_navigator.py:223 declares zone_dock_standoff with default
        # 0.40; the yaml does not override it.
        self.zone_dock_standoff = float(self.params.get(
            "zone_dock_standoff", 0.40))

    # --- inbound ---------------------------------------------------------
    def approach_candidates(self, zone):
        west_dock = self.cube_docks[zone]
        cube_x = west_dock.x + nm.CUBE_DOCK_STANDOFF
        cube_y = west_dock.y
        cube_key = zone.lower()
        candidates = []
        for side in nm.CUBE_APPROACH_SIDES.get(cube_key, (nm.APPROACH_WEST,)):
            if side == nm.APPROACH_WEST:
                candidates.append(nm.ApproachCandidate(
                    side, cube_key, self.cube_entries[zone], west_dock))
            elif (side == nm.APPROACH_EAST and zone.startswith("BLUE")
                  and zone in self.cube_entries_inner
                  and zone in self.cube_docks_inner):
                candidates.append(nm.ApproachCandidate(
                    side, cube_key, self.cube_entries_inner[zone],
                    self.cube_docks_inner[zone]))
            else:
                candidates.append(nm.ApproachCandidate(
                    side, cube_key,
                    nm.approach_entry(cube_x, cube_y, side),
                    nm.approach_dock(cube_x, cube_y, side)))
        return candidates

    def approach_waypoints(self, zone, dock, start_x, start_y):
        obstacle_2 = []
        if (zone.startswith("BLUE_CUBE_") and start_x is not None
                and start_y >= 1.5 and dock.y < 0.5):
            obstacle_2 = list(OBSTACLE_2_NORTH_SOUTH_GATES)
            if zone == "BLUE_CUBE_2":
                obstacle_2.append(OBSTACLE_2_WEST_UPPER_GATE)
            elif zone in ("BLUE_CUBE_3", "BLUE_CUBE_4", "BLUE_CUBE_5"):
                obstacle_2.append(OBSTACLE_2_LOWER_RIGHT_GATE)
        gates = []
        if start_x is not None and start_y is not None:
            gates = list(nm.dynamic_gate_waypoints(
                (start_x, start_y), (dock.x, dock.y)))
        values = CUBE_APPROACH_WAYPOINTS_MIRROR.get(zone)
        if zone == "BLUE_CUBE_3" and obstacle_2:
            return obstacle_2 + gates
        if values is None:
            return obstacle_2 + gates
        if zone == "BLUE_CUBE_3" and start_y is not None and start_y <= 0.0:
            return obstacle_2 + gates
        return obstacle_2 + gates + [
            nm.ZonePose(f"{zone}_rock_avoid", float(values[0]),
                        float(values[1]), float(values[2]))]

    def plan_inbound(self, zone, start, side_hint=None):
        """Return (side, entry, dock, poses) for one cube inbound leg."""
        candidates = self.approach_candidates(zone)
        inside_side = nm.APPROACH_INSIDE_SIDE.get(
            "red" if zone.startswith("RED") else "blue", "")
        chosen = None
        if side_hint:
            chosen = next((c for c in candidates if c.side == side_hint),
                          None)
        if chosen is None:
            chosen = nm.choose_approach(candidates, start, (inside_side,))
        entry, dock = chosen.entry, chosen.dock
        poses = self.approach_waypoints(
            zone, entry, start[0], start[1]) + [entry]
        return chosen.side, entry, dock, poses

    # --- delivery --------------------------------------------------------
    def route_pose(self, zone, key):
        values = self.profiles.get(zone, {}).get(key)
        if not isinstance(values, list) or len(values) != 3:
            return None
        return nm.ZonePose(f"{zone}_{key}", float(values[0]),
                           float(values[1]), float(values[2]))

    def route_waypoints(self, zone, key):
        values = self.profiles.get(zone, {}).get(key, [])
        if not isinstance(values, list):
            return []
        out = []
        for index, point in enumerate(values):
            if not isinstance(point, list) or len(point) != 3:
                continue
            out.append(nm.ZonePose(f"{zone}_{key}_{index}", float(point[0]),
                                   float(point[1]), float(point[2])))
        return out

    def delivery_dock(self, zone, latest_x):
        place = self.route_pose(zone, "place_pose") or self.zones[zone]
        standoff = self.zone_dock_standoff
        if standoff <= 0.0 or latest_x is None:
            return place
        center_x = place.x + standoff
        if latest_x > center_x:
            return nm.ZonePose(f"{zone}_dock_east", center_x + standoff,
                               place.y, math.pi)
        return place

    def plan_delivery(self, zone, start, last_cube_xy=None, last_side=None):
        """Return poses for one zone delivery leg (mirrors glue order)."""
        latest_x, latest_y = start[0], start[1]
        dock = self.delivery_dock(zone, latest_x)
        poses = self.route_waypoints(zone, "primary_waypoints") + [dock]
        if zone == "C" and latest_y is not None and latest_y < 0.5:
            if latest_x is not None and latest_x > 0.1:
                poses = [nm.ZonePose("C_SOUTH_BYPASS_0", -6.0, -2.0, 0.0),
                         nm.ZonePose("C_SOUTH_BYPASS_1", -8.5, 0.5, 0.0)] \
                    + poses
            else:
                poses = [nm.ZonePose("C_WEST_GUIDE_0", -8.5, 0.5, 0.0)] \
                    + poses
        if last_cube_xy is not None and last_side is not None and poses:
            tray = list(nm.tray_crossing_waypoints(
                last_cube_xy, last_side, (poses[-1].x, poses[-1].y)))
            if tray:
                poses = tray + poses
        return poses, dock


# --- snapshot validation -------------------------------------------------
POSE_RE = re.compile(r"\((-?[\d.]+),(-?[\d.]+),yaw=(-?[\d.]+)\)")


def parse_logged_poses(poses_text):
    return [(float(a), float(b), float(c))
            for a, b, c in POSE_RE.findall(poses_text or "")]


def poses_close(computed, logged):
    if len(computed) != len(logged):
        return False, "len %d vs %d" % (len(computed), len(logged))
    for (cx, cy, cyaw), (lx, ly, lyaw) in zip(computed, logged):
        if math.hypot(cx - lx, cy - ly) > POS_TOL:
            return False, "(%.2f,%.2f) vs (%.2f,%.2f)" % (cx, cy, lx, ly)
        if abs(nm.angle_delta(cyaw, lyaw)) > YAW_TOL:
            return False, "yaw %.2f vs %.2f" % (cyaw, lyaw)
    return True, ""


def run_validation(model, sprint_dir):
    """Validate every logged inbound/delivery nav_plan against the model."""
    import extract_e0 as e0  # same directory when run from exp dir

    results = {"ok": 0, "mismatch": 0, "skipped": 0, "detail": []}

    def note(run_id, loc, kind, why):
        results["mismatch"] += 1
        results["detail"].append([run_id, loc, kind, why])

    for run_id in sorted(os.listdir(sprint_dir)):
        run_dir = os.path.join(sprint_dir, run_id)
        if not os.path.isfile(os.path.join(run_dir, "launch.log")):
            continue
        events = e0.parse_log(os.path.join(run_dir, "launch.log"))
        open_nav = None
        last_pickup = None  # (cube_xy, side) mirroring navigator state
        for ev in events:
            name, f = ev["name"], ev["fields"]
            if name == "nav_start":
                loc = f["location"]
                if "CUBE" in loc or loc in ("A", "B", "C"):
                    open_nav = {"location": loc,
                                "start": (float(f["x"]), float(f["y"]),
                                          float(f["yaw"])),
                                "side": f["side"], "choice": None}
                else:
                    open_nav = None
                continue
            if name == "cube_entry_choice" and open_nav is not None:
                open_nav["choice"] = f
                continue
            if name == "nav_plan" and open_nav is not None:
                loc, start = open_nav["location"], open_nav["start"]
                logged = parse_logged_poses(f["poses"])
                if "CUBE" in loc:
                    hint = (open_nav["side"]
                            if open_nav["side"] != "auto" else None)
                    try:
                        side, entry, dock, poses = model.plan_inbound(
                            loc, start, hint)
                    except Exception as exc:  # noqa: BLE001
                        results["skipped"] += 1
                        results["detail"].append([run_id, loc, "EXC",
                                                  str(exc)])
                        open_nav = None
                        continue
                    if open_nav["choice"]:
                        cf = open_nav["choice"]
                        e_ok = math.hypot(
                            entry.x - float(cf["ex"]),
                            entry.y - float(cf["ey"])) <= POS_TOL
                        d_ok = math.hypot(
                            dock.x - float(cf["dx"]),
                            dock.y - float(cf["dy"])) <= POS_TOL
                        if not (e_ok and d_ok):
                            note(run_id, loc, "CHOICE",
                                 "entry/dock differ from logged")
                            open_nav = None
                            continue
                    ok, why = poses_close(
                        [(p.x, p.y, p.yaw) for p in poses], logged)
                    if ok:
                        results["ok"] += 1
                        west = model.cube_docks[loc]
                        last_pickup = (
                            (west.x + nm.CUBE_DOCK_STANDOFF, west.y), side)
                    else:
                        note(run_id, loc, "PLAN", why)
                    open_nav = None
                    continue
                # delivery leg
                try:
                    cube_xy, cube_side = (last_pickup if last_pickup
                                          else (None, None))
                    poses, dock = model.plan_delivery(
                        loc, start, cube_xy, cube_side)
                except Exception as exc:  # noqa: BLE001
                    results["skipped"] += 1
                    results["detail"].append([run_id, loc, "EXC", str(exc)])
                    open_nav = None
                    continue
                ok, why = poses_close(
                    [(p.x, p.y, p.yaw) for p in poses], logged)
                if ok:
                    results["ok"] += 1
                else:
                    note(run_id, loc, "PLAN", why)
                open_nav = None
                continue
            if name == "nav_result" and open_nav is not None:
                open_nav = None
    return results


def main():
    sprint_dir = sys.argv[1]
    out_json = sys.argv[2] if len(sys.argv) > 2 else None
    model = RouteModel()
    validation = run_validation(model, sprint_dir)
    print("validation: ok=%d mismatch=%d skipped=%d"
          % (validation["ok"], validation["mismatch"], validation["skipped"]))
    for row in validation["detail"][:20]:
        print("  ", row)
    if out_json:
        with open(out_json, "w", encoding="utf-8") as fh:
            json.dump(validation, fh, indent=2, ensure_ascii=False)


if __name__ == "__main__":
    main()
