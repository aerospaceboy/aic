"""Whole-map reachability checks for every cube and delivery zone."""

from pathlib import Path

import cv2
import numpy as np
import yaml
import math

from tools_demo.navigation_math import approach_entry


REPO = Path(__file__).resolve().parents[2]
ROBOT_RADIUS_M = 0.28


def _load_navigation_grid():
    map_dir = REPO / "bot_navigation" / "maps"
    metadata = yaml.safe_load(
        (map_dir / "mapn3.yaml").read_text(encoding="utf-8")
    )
    image = cv2.imread(str(map_dir / metadata["image"]), cv2.IMREAD_GRAYSCALE)
    assert image is not None
    resolution = float(metadata["resolution"])
    clearance = cv2.distanceTransform(
        (image >= 89).astype(np.uint8), cv2.DIST_L2, 5
    ) * resolution
    navigable = (clearance >= ROBOT_RADIUS_M).astype(np.uint8)
    _, components = cv2.connectedComponents(navigable, connectivity=8)
    return image, clearance, components, metadata


def _cell(point, image, metadata):
    resolution = float(metadata["resolution"])
    origin_x, origin_y = metadata["origin"][:2]
    column = int((float(point[0]) - origin_x) / resolution)
    world_row = int((float(point[1]) - origin_y) / resolution)
    row = image.shape[0] - 1 - world_row
    assert 0 <= row < image.shape[0] and 0 <= column < image.shape[1], point
    return row, column


def _navigation_points():
    config = yaml.safe_load(
        (REPO / "bot_navigation" / "config" / "competition_zones.yaml")
        .read_text(encoding="utf-8")
    )["competition_task_navigator"]["ros__parameters"]
    risk = yaml.safe_load(
        (REPO / "bot_navigation" / "config" / "navigation_risk_zones.yaml")
        .read_text(encoding="utf-8")
    )["zone_strategies"]

    cubes = {}
    for color in ("red", "blue"):
        for index in range(1, 6):
            name = f"{color}_cube_{index}"
            # Red shelves have a dedicated tray-clear aisle point. Blue
            # shelves depart through the same validated entry used to dock.
            key = f"{name}_entry_aisle" if color == "red" else f"{name}_entry"
            allowed = config.get("cube_approach_allowlist", [])
            if allowed:
                dock = config[name]
                for side in ("west", "east"):
                    if f"{name}:{side}" in allowed:
                        entry = approach_entry(dock[0] + .36, dock[1], side)
                        cubes[f"{name}:{side}"] = [entry.x, entry.y, entry.yaw]
            else:
                cubes[name] = config[key]
    gateways = {zone: risk[zone]["retreat_pose"] for zone in ("A", "B", "C")}
    places = {zone: risk[zone]["place_pose"] for zone in ("A", "B", "C")}
    exit_waypoints = {
        zone: risk[zone]["exit_waypoints"] for zone in ("A", "B", "C")
    }
    return cubes, gateways, places, exit_waypoints


def _point_segment_distance(point, start, end):
    segment = np.asarray(end[:2], dtype=float) - np.asarray(start[:2], dtype=float)
    offset = np.asarray(point[:2], dtype=float) - np.asarray(start[:2], dtype=float)
    denominator = float(np.dot(segment, segment))
    fraction = 0.0 if denominator == 0.0 else float(np.dot(offset, segment) / denominator)
    closest = np.asarray(start[:2], dtype=float) + np.clip(fraction, 0.0, 1.0) * segment
    return float(np.linalg.norm(np.asarray(point[:2], dtype=float) - closest))


def test_all_cube_zone_round_trips_share_safe_map_components():
    """Validate 30 outbound and 30 return combinations on the deployed map."""
    image, clearance, components, metadata = _load_navigation_grid()
    cubes, gateways, _, _ = _navigation_points()
    failures = []

    for cube_name, cube_point in cubes.items():
        cube_cell = _cell(cube_point, image, metadata)
        for zone_name, gateway_point in gateways.items():
            gateway_cell = _cell(gateway_point, image, metadata)
            cube_component = int(components[cube_cell])
            gateway_component = int(components[gateway_cell])
            if (
                cube_component == 0
                or gateway_component == 0
                or cube_component != gateway_component
            ):
                failures.append(
                    (
                        cube_name,
                        zone_name,
                        round(float(clearance[cube_cell]), 3),
                        round(float(clearance[gateway_cell]), 3),
                    )
                )

    assert not failures, "unreachable cube/zone pairs: " + repr(failures)
    assert len(cubes) >= 10
    assert len(gateways) == 3


def test_zone_backoff_moves_away_from_the_placed_cube_before_nav2():
    """The direct first exit stage must reverse west, away from every cube."""
    image, clearance, _, metadata = _load_navigation_grid()
    _, _, places, _ = _navigation_points()
    failures = []

    for zone_name, place in places.items():
        # Every delivery pose faces east (yaw zero). The placed cube is in
        # front of base_link, so the required first motion is 0.45 m west.
        backoff = [float(place[0]) - 0.45 * math.cos(place[2]),
                   float(place[1]) - 0.45 * math.sin(place[2])]
        sampled = []
        for fraction in np.linspace(0.0, 1.0, 31):
            point = [
                float(place[0]) + fraction * (backoff[0] - float(place[0])),
                float(place[1]) + fraction * (backoff[1] - float(place[1])),
            ]
            sampled.append(float(clearance[_cell(point, image, metadata)]))
        if min(sampled) < ROBOT_RADIUS_M:
            failures.append((zone_name, round(min(sampled), 3)))
        assert (backoff[0] - place[0]) * math.cos(place[2]) < 0, zone_name

    assert not failures, "blocked zone backoff segments: " + repr(failures)


def test_complete_zone_exit_routes_do_not_cross_the_placed_cube():
    """After reversing, every staged route stays outside the cube envelope."""
    _, gateways, places, exit_waypoints = _navigation_points()
    required_separation = ROBOT_RADIUS_M + 0.10
    failures = []

    for zone_name, place in places.items():
        cube = [place[0] + 0.35 * math.cos(place[2]),
                place[1] + 0.35 * math.sin(place[2])]
        backoff = [place[0] - 0.45 * math.cos(place[2]),
                   place[1] - 0.45 * math.sin(place[2]), place[2]]
        route = [backoff] + exit_waypoints[zone_name] + [gateways[zone_name]]
        for start, end in zip(route, route[1:]):
            separation = _point_segment_distance(cube, start, end)
            if separation < required_separation:
                failures.append(
                    (zone_name, start[:2], end[:2], round(separation, 3))
                )

    assert not failures, "zone exit crosses placed cube: " + repr(failures)
