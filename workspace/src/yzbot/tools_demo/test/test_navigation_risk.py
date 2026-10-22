"""Tests for map risk zones and the cube group preference."""

from pathlib import Path
import importlib.util
import math

from tools_demo.mission_coordinator import MissionCoordinator
from tools_demo.mission_coordinator import cube_selection_cost

REPO = Path(__file__).resolve().parents[2]


def test_risk_generator_matches_the_checked_in_config():
    """Regeneration must preserve the validated routes exactly."""
    import yaml  # noqa: PLC0415

    script = REPO / "tools_demo" / "scripts" / "generate_navigation_risk_zones.py"
    spec = importlib.util.spec_from_file_location("risk_generator", script)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    generated = module.build_config()
    checked = yaml.safe_load(
        (REPO / "bot_navigation" / "config" / "navigation_risk_zones.yaml")
        .read_text(encoding="utf-8")
    )
    assert generated == checked


def test_cube_group_splits_red_by_actual_y():
    """Red cubes split on the median of their y coordinates."""
    colors_y = [-2.0, -5.0, 1.0, 3.0, -7.0]
    assert MissionCoordinator._cube_group(colors_y, 3.0) == "upper"
    assert MissionCoordinator._cube_group(colors_y, -2.0) == "upper"
    assert MissionCoordinator._cube_group(colors_y, -5.0) == "lower"
    assert MissionCoordinator._cube_group(colors_y, -7.0) == "lower"


def test_cube_group_splits_blue_by_actual_y():
    """Blue cubes split on the median of their y coordinates."""
    colors_y = [-2.0, -4.0, 1.0, 3.0, -6.0]
    assert MissionCoordinator._cube_group(colors_y, 1.0) == "upper"
    assert MissionCoordinator._cube_group(colors_y, -4.0) == "lower"
    assert MissionCoordinator._cube_group(colors_y, -6.0) == "lower"


def test_group_preference_is_a_soft_bonus():
    """The preference shifts cost but never dominates a long path."""
    base = cube_selection_cost(10.0, 0.0, 0, group_preference=1.0)
    no_pref = cube_selection_cost(10.0, 0.0, 0)
    assert base == no_pref - 1.0
    # A candidate 3 m farther away still loses despite the preference.
    assert cube_selection_cost(13.0, 0.0, 0, group_preference=1.0) > base


def test_risk_zones_config_exists_and_is_parseable():
    """The generated risk zones file exists and is valid YAML."""
    import yaml  # noqa: PLC0415
    path = REPO / "bot_navigation" / "config" / "navigation_risk_zones.yaml"
    assert path.exists(), path
    payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    zones = payload["risk_zones"]
    assert "large_rock" in zones
    assert set(zones) == {"large_rock"}
    assert zones["large_rock"]["escape_waypoint"]
    assert set(payload["zone_strategies"]) == {"A", "B", "C"}


def test_risk_zones_waypoints_are_off_the_hazards():
    """Escape waypoints must not sit inside the hazard they escape."""
    import yaml  # noqa: PLC0415
    path = REPO / "bot_navigation" / "config" / "navigation_risk_zones.yaml"
    payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    zones = payload["risk_zones"]
    rock_center = zones["large_rock"]["center"]
    rock_radius = zones["large_rock"]["radius"]
    escape = zones["large_rock"]["escape_waypoint"]
    import math  # noqa: PLC0415
    dist = math.hypot(escape[0] - rock_center[0],
                      escape[1] - rock_center[1])
    assert dist > rock_radius


def test_zone_strategies_define_independent_routes_and_escapes():
    """Every delivery zone owns its route, return, and escape poses."""
    import yaml  # noqa: PLC0415

    payload = yaml.safe_load(
        (REPO / "bot_navigation" / "config" / "navigation_risk_zones.yaml")
        .read_text(encoding="utf-8")
    )
    strategies = payload["zone_strategies"]
    assert set(strategies) == {"A", "B", "C"}
    required = {
        "approach_pose",
        "place_pose",
        "retreat_pose",
        "return_waypoint",
        "exit_waypoints",
        "primary_waypoints",
        "backup_waypoints",
        "escape_pose",
    }
    for strategy in strategies.values():
        assert required <= set(strategy)
        assert len(strategy["place_pose"]) == 3
        assert strategy["primary_waypoints"] == []
        assert strategy["backup_waypoints"] == []


def test_c_delivery_has_one_global_planner_goal():
    """Let Smac choose C's complete wall-aware route from the live start."""
    import yaml  # noqa: PLC0415

    strategy = yaml.safe_load(
        (REPO / "bot_navigation/config/navigation_risk_zones.yaml")
        .read_text(encoding="utf-8")
    )["zone_strategies"]["C"]
    route = strategy["primary_waypoints"] + [strategy["place_pose"]]
    assert route == [strategy["place_pose"]]


def test_delivery_route_stages_bound_the_pre_turn_before_nav2():
    """Allow only large heading changes to get a short direct pre-turn."""
    source = (REPO / "tools_demo" / "tools_demo" / "task_navigator.py").read_text(encoding="utf-8")
    assert "def _continue_staged_navigation" in source
    assert "if self.pending_poses:" in source
    assert "self.start_pre_navigation_alignment()" in source
    body = source.split(
        "    def start_pre_navigation_alignment(self) -> None:", 1
    )[1].split("\n    def ", 1)[0]
    assert "needs_pre_navigation_alignment" in body
    assert "self.pre_navigation_alignment_timeout_ns" in body
    assert "self._send_next_goal()" in body
    # All three completion paths keep the staged owner. A completed corner
    # delegates its outgoing heading to Nav2; physical wait-anchor capture
    # still invokes the ordinary pre-alignment contract.
    assert source.count("self._continue_staged_navigation(") == 3
    continuation = source.split("    def _continue_staged_navigation(", 1)[1].split(
        "\n    def ", 1)[0]
    assert "self.pending_poses and not path_alignment" in continuation


def test_zone_return_route_uses_its_own_retreat_and_return_waypoint():
    """B/C pickup returns leave the bay before entering the shared aisles."""
    source = (
        REPO / "tools_demo" / "tools_demo" / "task_navigator.py"
    ).read_text(encoding="utf-8")
    assert "def _start_zone_return_route" in source
    assert 'self._route_pose(zone, "return_waypoint")' in source
    assert 'self._route_pose(zone, "retreat_pose")' in source
    assert 'if nearest == "C" and self._start_zone_return_route("C")' in source
    assert 'if nearest == "B"' in source
    assert "self.active_return_zone" in source


def test_c_staged_handoff_is_tighter_than_final_docking_handoff():
    """C route turns cannot hand off at the global final-dock radius."""
    import yaml  # noqa: PLC0415

    payload = yaml.safe_load(
        (REPO / "bot_navigation" / "config" / "navigation_risk_zones.yaml")
        .read_text(encoding="utf-8")
    )
    assert payload["zone_strategies"]["C"]["staged_handoff_distance"] == 0.20
    source = (
        REPO / "tools_demo" / "tools_demo" / "task_navigator.py"
    ).read_text(encoding="utf-8")
    assert "def _route_handoff_distance" in source
    assert "self._route_handoff_distance()," in source


def test_every_risk_route_point_is_free_on_the_deployed_map():
    """Recovery routes must not encode waypoints inside mapped walls."""
    import cv2  # noqa: PLC0415
    import numpy as np  # noqa: PLC0415
    import yaml  # noqa: PLC0415

    map_yaml = yaml.safe_load(
        (REPO / "bot_navigation" / "maps" / "mapn3.yaml")
        .read_text(encoding="utf-8")
    )
    image = cv2.imread(
        str(REPO / "bot_navigation" / "maps" / map_yaml["image"]), 0
    )
    resolution = float(map_yaml["resolution"])
    origin_x, origin_y = map_yaml["origin"][:2]
    distance = cv2.distanceTransform(
        (image >= 89).astype(np.uint8), cv2.DIST_L2, 5
    ) * resolution
    payload = yaml.safe_load(
        (REPO / "bot_navigation" / "config" / "navigation_risk_zones.yaml")
        .read_text(encoding="utf-8")
    )

    points = []
    for zone_name, zone in payload["risk_zones"].items():
        for key in ("escape_waypoint", "exit_waypoint", "return_waypoint"):
            if key in zone:
                points.append((f"{zone_name}.{key}", zone[key]))
    for zone_name, strategy in payload["zone_strategies"].items():
        for key in (
            "approach_pose",
            "place_pose",
            "retreat_pose",
            "return_waypoint",
            "escape_pose",
        ):
            points.append((f"{zone_name}.{key}", strategy[key]))
        for key in (
            "exit_waypoints",
            "primary_waypoints",
            "backup_waypoints",
        ):
            points.extend(
                (f"{zone_name}.{key}[{index}]", point)
                for index, point in enumerate(strategy[key])
            )

    violations = []
    for name, point in points:
        col = int((float(point[0]) - origin_x) / resolution)
        world_row = int((float(point[1]) - origin_y) / resolution)
        row = image.shape[0] - 1 - world_row
        if not (
            0 <= row < image.shape[0]
            and 0 <= col < image.shape[1]
        ):
            violations.append((name, "outside", point))
            continue
        if image[row, col] < 205 or distance[row, col] < 0.45:
            violations.append(
                (name, int(image[row, col]), float(distance[row, col]))
            )
    assert not violations, "\n".join(str(item) for item in violations)


def test_zone_egress_routes_clear_the_deployed_map():
    """Direct backoff and staged exits must remain on free map cells."""
    import cv2  # noqa: PLC0415
    import numpy as np  # noqa: PLC0415
    import yaml  # noqa: PLC0415

    map_yaml = yaml.safe_load(
        (REPO / "bot_navigation" / "maps" / "mapn3.yaml")
        .read_text(encoding="utf-8")
    )
    image = cv2.imread(
        str(REPO / "bot_navigation" / "maps" / map_yaml["image"]), 0
    )
    resolution = float(map_yaml["resolution"])
    origin_x, origin_y = map_yaml["origin"][:2]
    distance = cv2.distanceTransform(
        (image >= 89).astype(np.uint8), cv2.DIST_L2, 5
    ) * resolution
    risk = yaml.safe_load(
        (REPO / "bot_navigation" / "config" / "navigation_risk_zones.yaml")
        .read_text(encoding="utf-8")
    )["zone_strategies"]

    for zone in ("A", "B", "C"):
        place = risk[zone]["place_pose"]
        backoff = [place[0] - 0.45 * math.cos(place[2]),
                   place[1] - 0.45 * math.sin(place[2]), place[2]]
        route = [place, backoff]
        route.extend(risk[zone]["exit_waypoints"])
        route.append(risk[zone]["retreat_pose"])
        for start, end in zip(route, route[1:]):
            clearances = []
            for fraction in np.linspace(0.0, 1.0, 101):
                x_value = start[0] + fraction * (end[0] - start[0])
                y_value = start[1] + fraction * (end[1] - start[1])
                col = int((x_value - origin_x) / resolution)
                world_row = int((y_value - origin_y) / resolution)
                row = image.shape[0] - 1 - world_row
                clearances.append(float(distance[row, col]))
            assert min(clearances) >= 0.45, (
                zone,
                start,
                end,
                min(clearances),
            )
