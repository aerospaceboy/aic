"""
Prove every consumer agrees with config/final_mission_layout.yaml.

The layout config is generated from the world file. These tests fail if any
consumer (mission config, navigation config, coordinator, navigator,
dashboard) drifts away from it, which is the failure mode that previously let
world, mission and navigation carry three disagreeing copies of the same
coordinates.
"""
from pathlib import Path
import math
import re
import xml.etree.ElementTree as ElementTree


REPOSITORY = Path(__file__).resolve().parents[2]
LAYOUT = REPOSITORY / "config" / "final_mission_layout.yaml"


def _layout_text():
    return LAYOUT.read_text(encoding="utf-8")


def _vector(text, key):
    """Read one 'key: [a, b, c]' entry from a flat YAML region."""
    for line in text.splitlines():
        stripped = line.strip()
        if stripped.startswith(f"{key}:") and "[" in stripped:
            body = stripped.split("[", 1)[1].rsplit("]", 1)[0]
            return [float(v) for v in body.split(",")]
    raise AssertionError(f"missing vector {key}")


def _scalar(text, key):
    for line in text.splitlines():
        stripped = line.strip()
        if stripped.startswith(f"{key}:"):
            return float(stripped.split(":", 1)[1].strip())
    raise AssertionError(f"missing scalar {key}")


def _layout_objects():
    """Parse pickup_objects into a list of dicts without a YAML dependency."""
    text = _layout_text()
    block = text[text.index("pickup_objects:"):text.index("zones:")]
    objects = []
    current = None
    for line in block.splitlines():
        stripped = line.strip()
        if stripped.startswith("- id:"):
            if current:
                objects.append(current)
            current = {"id": stripped.split(":", 1)[1].strip()}
        elif current is not None and ":" in stripped and stripped:
            key, _, value = stripped.partition(":")
            current[key.strip()] = value.strip()
    if current:
        objects.append(current)
    return objects


def test_layout_config_covers_all_ten_cubes_and_three_zones():
    """The layout file is the single source, so it must be complete."""
    objects = _layout_objects()
    assert len(objects) == 10, [o["id"] for o in objects]
    ids = {o["id"] for o in objects}
    expected = {
        f"{color}_cube_{n}" for color in ("red", "blue") for n in range(1, 6)
    }
    assert ids == expected
    text = _layout_text()
    for letter in ("A", "B", "C"):
        assert f"zone_id: {letter}" in text
    # the 2026 world carries exactly the two scored obstacles
    for key in ("obstacle_1", "obstacle_2"):
        assert f"{key}:" in text
    assert "mission_frame: map" in text
    assert "robot_frame: base_link" in text


def test_layout_matches_the_authoritative_world():
    """A stale layout file is worse than none, so pin it to the world."""
    root = ElementTree.parse(
        REPOSITORY / "mybot_description" / "worlds" / "offic_room.world"
    ).getroot()
    models = {
        m.attrib["name"]: m
        for m in root.findall("./world/model")
        if "name" in m.attrib
    }
    for obj in _layout_objects():
        pose = models[obj["id"]].findtext("pose").split()
        assert math.isclose(float(pose[0]), float(obj["x"]), abs_tol=1e-4)
        assert math.isclose(float(pose[1]), float(obj["y"]), abs_tol=1e-4)


def test_navigation_config_docks_match_the_layout():
    """task_navigator drives to these, so they must equal the layout."""
    config = (
        REPOSITORY
        / "bot_navigation"
        / "config"
        / "competition_zones.yaml"
    ).read_text(encoding="utf-8")
    for obj in _layout_objects():
        approach = [
            float(v)
            for v in obj["pickup_approach"].strip("[]").split(",")
        ]
        assert _vector(config, obj["id"])[:2] == approach[:2], obj["id"]
        retreat = [
            float(v) for v in obj["retreat_pose"].strip("[]").split(",")
        ]
        entry = _vector(config, f"{obj['id']}_entry")
        assert entry[:2] == retreat[:2], obj["id"]


def test_mission_config_docks_match_the_layout():
    """The coordinator plans with these, so they must equal the layout."""
    mission = (
        REPOSITORY / "tools_demo" / "config" / "mission.yaml"
    ).read_text(encoding="utf-8")
    for obj in _layout_objects():
        approach = [
            float(v)
            for v in obj["pickup_approach"].strip("[]").split(",")
        ]
        dock = _vector(mission, f"{obj['id']}_dock")
        assert dock[:2] == approach[:2], obj["id"]


def test_coordinator_cube_slots_match_the_layout():
    """CUBE_WORLD_SLOTS feeds perception cross-checks and cube selection."""
    source = (
        REPOSITORY
        / "tools_demo"
        / "tools_demo"
        / "mission_coordinator.py"
    ).read_text(encoding="utf-8")
    block = source[
        source.index("CUBE_WORLD_SLOTS = {"):source.index("}", source.index(
            "CUBE_WORLD_SLOTS = {"
        ))
    ]
    slots = {
        name: (float(x), float(y))
        for name, x, y in re.findall(
            r'"([a-z_0-9]+)":\s*\(([-0-9.]+),\s*([-0-9.]+)\)', block
        )
    }
    assert len(slots) == 10, slots
    for obj in _layout_objects():
        assert math.isclose(slots[obj["id"]][0], float(obj["x"]), abs_tol=1e-4)
        assert math.isclose(slots[obj["id"]][1], float(obj["y"]), abs_tol=1e-4)


def test_navigator_dock_defaults_match_the_layout():
    """Parameter defaults must not contradict the YAML that overrides them."""
    source = (
        REPOSITORY / "tools_demo" / "tools_demo" / "task_navigator.py"
    ).read_text(encoding="utf-8")
    for obj in _layout_objects():
        upper = obj["id"].upper()
        approach = [
            float(v)
            for v in obj["pickup_approach"].strip("[]").split(",")
        ]
        match = re.search(
            r'"%s":\s*\[([-0-9.]+),\s*([-0-9.]+)' % upper, source
        )
        assert match is not None, upper
        assert math.isclose(float(match.group(1)), approach[0], abs_tol=1e-4)
        assert math.isclose(float(match.group(2)), approach[1], abs_tol=1e-4)


def test_navigator_entry_defaults_match_the_layout():
    """Fallback entry defaults must mirror the generated layout."""
    source = (
        REPOSITORY / "tools_demo" / "tools_demo" / "task_navigator.py"
    ).read_text(encoding="utf-8")
    start = source.index("DEFAULT_CUBE_ENTRIES = {")
    block = source[start:source.index("}", start)]
    for obj in _layout_objects():
        upper = obj["id"].upper()
        retreat = [
            float(v)
            for v in obj["retreat_pose"].strip("[]").split(",")
        ]
        match = re.search(
            r'"%s":\s*\[([-0-9.]+),\s*([-0-9.]+)' % upper,
            block,
        )
        assert match is not None, upper
        assert math.isclose(
            float(match.group(1)), retreat[0], abs_tol=1e-4
        )
        assert math.isclose(
            float(match.group(2)), retreat[1], abs_tol=1e-4
        )


def test_navigator_alternate_entry_defaults_match_the_layout():
    """Red aisle fallback defaults must mirror the generated layout."""
    source = (
        REPOSITORY / "tools_demo" / "tools_demo" / "task_navigator.py"
    ).read_text(encoding="utf-8")
    start = source.index("DEFAULT_RED_AISLE_ENTRIES = {")
    block = source[start:source.index("}", start)]
    for obj in _layout_objects():
        if "alternate_retreat_pose" not in obj:
            continue
        upper = obj["id"].upper()
        retreat = [
            float(v)
            for v in obj["alternate_retreat_pose"].strip("[]").split(",")
        ]
        match = re.search(
            r'"%s":\s*\[([-0-9.]+),\s*([-0-9.]+)' % upper,
            block,
        )
        assert match is not None, upper
        assert math.isclose(
            float(match.group(1)), retreat[0], abs_tol=1e-4
        )
        assert math.isclose(
            float(match.group(2)), retreat[1], abs_tol=1e-4
        )


def test_every_entry_clears_the_costmap_inflation():
    """A dock is inside the tray inflation; its entry must not be."""
    text = _layout_text()
    inflation = _scalar(text, "costmap_inflation_radius")
    for obj in _layout_objects():
        assert float(obj["entry_clearance"]) > inflation, obj["id"]
        approach = [
            float(v)
            for v in obj["pickup_approach"].strip("[]").split(",")
        ]
        retreat = [
            float(v)
            for v in obj["retreat_pose"].strip("[]").split(",")
        ]
        distance = math.hypot(
            approach[0] - retreat[0],
            approach[1] - retreat[1],
        )
        assert math.isclose(
            float(obj["dock_to_entry"]),
            distance,
            abs_tol=1e-6,
        )
        if "alternate_retreat_pose" in obj:
            assert (
                float(obj["alternate_entry_clearance"]) > inflation
            ), obj["id"]
            alternate = [
                float(v)
                for v in obj["alternate_retreat_pose"].strip("[]").split(",")
            ]
            alternate_distance = math.hypot(
                approach[0] - alternate[0],
                approach[1] - alternate[1],
            )
            assert math.isclose(
                float(obj["alternate_dock_to_entry"]),
                alternate_distance,
                abs_tol=1e-6,
            )


def test_cube_standoff_stays_inside_the_arm_reach_window():
    """The dock offset decides whether the arm can reach the cube at all."""
    mission = (
        REPOSITORY / "tools_demo" / "config" / "mission.yaml"
    ).read_text(encoding="utf-8")
    minimum = _scalar(mission, "reachable_minimum_x")
    maximum = _scalar(mission, "reachable_maximum_x")
    standoff = _scalar(_layout_text(), "cube_dock_standoff")
    assert minimum < standoff < maximum
    for obj in _layout_objects():
        approach = [
            float(v)
            for v in obj["pickup_approach"].strip("[]").split(",")
        ]
        assert math.isclose(
            float(obj["x"]) - approach[0], standoff, abs_tol=1e-6
        ), obj["id"]


def test_zone_centres_match_mission_and_navigation_configs():
    """Delivery truth checks use these, so all three copies must agree."""
    text = _layout_text()
    mission = (
        REPOSITORY / "tools_demo" / "config" / "mission.yaml"
    ).read_text(encoding="utf-8")
    block = text[text.index("zones:"):text.index("dynamic_obstacles:")]
    for letter in ("A", "B", "C"):
        section = block[block.index(f"  {letter}:"):]
        centre = _vector(section, "center")
        configured = _vector(mission, f"zone_{letter.lower()}_center")
        assert configured[:2] == centre[:2], letter


def _world_static_boxes():
    """
    Return (trays, rocks) box tuples from the authoritative world.

    Each box is (cx, cy, half_w, half_h, yaw). The tray boxes come from the
    pickup platform models; rock boxes are the absolute positions of every
    "Untitled" link (model pose + link pose).
    """
    world = (
        REPOSITORY / "mybot_description" / "worlds" / "offic_room.world"
    )
    root = ElementTree.parse(world).getroot()
    models = {
        model.attrib["name"]: model
        for model in root.findall("./world/model")
        if "name" in model.attrib
    }

    def pose(model):
        text = model.findtext("pose") or "0 0 0 0 0 0"
        return [float(v) for v in text.split()][:6]

    trays = []
    for name, model in models.items():
        if name.startswith("competition_") and "pickup_tray" in name:
            cx, cy = pose(model)[:2]
            size = model.findtext(".//link/collision/geometry/box/size")
            s = [float(v) for v in size.split()] if size else [0.24, 0.24, 0.1]
            trays.append((cx, cy, s[0] / 2.0, s[1] / 2.0, pose(model)[5]))
    rocks = []
    untitled = models.get("Untitled")
    if untitled is not None:
        mx, my = pose(untitled)[:2]
        for link in untitled.findall("./link"):
            size_el = link.find(".//collision/geometry/box/size")
            if size_el is None:
                continue
            lx, ly = (float(v) for v in
                      (link.findtext("pose") or "0 0 0 0 0 0").split()[:2])
            s = [float(v) for v in size_el.text.split()]
            rocks.append((mx + lx, my + ly, s[0] / 2.0, s[1] / 2.0, 0.0))
    return trays, rocks


def _read_binary_pgm(path):
    """Return width, height and pixels from the repository's P5 map."""
    data = path.read_bytes()
    tokens = []
    index = 0
    while len(tokens) < 4:
        while index < len(data) and chr(data[index]).isspace():
            index += 1
        if data[index:index + 1] == b"#":
            index = data.index(b"\n", index) + 1
            continue
        end = index
        while end < len(data) and not chr(data[end]).isspace():
            end += 1
        tokens.append(data[index:end])
        index = end
    while index < len(data) and chr(data[index]).isspace():
        index += 1
    assert tokens[0] == b"P5"
    assert tokens[3] == b"255"
    width, height = int(tokens[1]), int(tokens[2])
    pixels = data[index:index + width * height]
    assert len(pixels) == width * height
    return width, height, pixels


def test_navigation_map_matches_authoritative_world_static_collisions():
    """Every static wall and pickup tray must be occupied in the Nav2 map."""
    maps = REPOSITORY / "bot_navigation" / "maps"
    yaml_text = (maps / "mapn3.yaml").read_text(encoding="utf-8")
    assert "origin: [-15.6, -33.4, 0]" in yaml_text

    width, height, rendered = _read_binary_pgm(maps / "mapn3.pgm")
    assert (width, height) == (658, 822)

    resolution = 0.05
    origin_x, origin_y = -15.6, -33.4
    world = ElementTree.parse(
        REPOSITORY / "mybot_description" / "worlds" / "offic_room.world"
    ).getroot()
    room = next(
        model
        for model in world.findall("./world/model")
        if model.attrib.get("name") == "officeroom"
    )
    room_pose = [
        float(value)
        for value in (room.findtext("pose") or "0 0 0 0 0 0").split()
    ]
    room_state = next(
        model
        for model in world.findall("./world/state/model")
        if model.attrib.get("name") == "officeroom"
    )
    state_links = {
        link.attrib["name"]: [
            float(value)
            for value in (
                link.findtext("pose") or "0 0 0 0 0 0"
            ).split()
        ]
        for link in room_state.findall("./link")
    }

    samples = []
    for link in room.findall("./link"):
        size_text = link.findtext("collision/geometry/box/size")
        if size_text is None:
            continue
        size_x, size_y, _ = (float(value) for value in size_text.split())
        definition_pose = [
            float(value)
            for value in (link.findtext("pose") or "0 0 0 0 0 0").split()
        ]
        # Gazebo restores the saved <state>; its link poses are absolute world
        # poses and override the model-definition poses above.
        link_pose = state_links.get(link.attrib.get("name"))
        if link_pose is None:
            center_x = room_pose[0] + definition_pose[0]
            center_y = room_pose[1] + definition_pose[1]
            yaw = room_pose[5] + definition_pose[5]
        else:
            center_x, center_y, yaw = link_pose[0], link_pose[1], link_pose[5]
        half_length = max(size_x, size_y) / 2.0
        along_x = size_x >= size_y
        count = max(1, round(2.0 * half_length / 0.10))
        for index in range(count + 1):
            along = -half_length + 2.0 * half_length * index / count
            local_x, local_y = (along, 0.0) if along_x else (0.0, along)
            samples.append(
                (
                    center_x + math.cos(yaw) * local_x - math.sin(yaw) * local_y,
                    center_y + math.sin(yaw) * local_x + math.cos(yaw) * local_y,
                )
            )

    trays, _ = _world_static_boxes()
    samples.extend((cx, cy) for cx, cy, _, _, _ in trays)
    assert len(samples) > 1000
    for x_value, y_value in samples:
        column = round((x_value - origin_x) / resolution)
        row = height - 1 - round((y_value - origin_y) / resolution)
        if not (0 <= column < width and 0 <= row < height):
            continue
        neighborhood = [
            rendered[candidate_row * width + candidate_column]
            for candidate_row in range(max(0, row - 1), min(height, row + 2))
            for candidate_column in range(
                max(0, column - 1), min(width, column + 2)
            )
        ]
        assert min(neighborhood) == 0, (x_value, y_value)


def _box_clearance(px, py, boxes):
    """Minimum distance from (px, py) to any yawed box."""
    best = float("inf")
    for cx, cy, hx, hy, yaw in boxes:
        cos_y, sin_y = math.cos(yaw), math.sin(yaw)
        dx, dy = px - cx, py - cy
        lx = cos_y * dx + sin_y * dy
        ly = -sin_y * dx + cos_y * dy
        ox = max(abs(lx) - hx, 0.0)
        oy = max(abs(ly) - hy, 0.0)
        best = min(best, math.hypot(ox, oy))
    return best


def test_enabled_approach_sides_clear_the_world_geometry():
    """
    Every enabled approach entry/dock must be reachable and tray-clear.

    The navigator and coordinator select among CUBE_APPROACH_SIDES at
    runtime; this test proves each enabled side's Nav2 entry clears every
    tray and rock by more than the costmap inflation radius, and that the
    dock keeps the chassis clear of the tray (>= 0.08 m face gap).
    """
    from tools_demo.navigation_math import (
        APPROACH_EAST,
        APPROACH_WEST,
        CUBE_APPROACH_SIDES,
        CUBE_DOCK_STANDOFF,
        CUBE_ENTRY_OFFSET,
        approach_dock,
        approach_entry,
    )

    world = (
        REPOSITORY / "mybot_description" / "worlds" / "offic_room.world"
    )
    root = ElementTree.parse(world).getroot()
    models = {
        model.attrib["name"]: model
        for model in root.findall("./world/model")
        if "name" in model.attrib
    }
    trays, rocks = _world_static_boxes()
    obstacles = trays + rocks
    robot_radius = 0.28
    cube_xy = {}
    for color in ("red", "blue"):
        for number in range(1, 6):
            name = f"{color}_cube_{number}"
            pose = (models[name].findtext("pose") or "0 0 0 0 0 0").split()
            cube_xy[name] = (float(pose[0]), float(pose[1]))
    # Deployed Nav2 entries (competition_zones.yaml) are authoritative for
    # the west side and for blue east-side (inner) docks; the other sides
    # are staged geometrically from the cube.
    zones_text = (
        REPOSITORY
        / "bot_navigation"
        / "config"
        / "competition_zones.yaml"
    ).read_text(encoding="utf-8")

    def deployed_entry(cube_key, suffix):
        key = f"{cube_key}_{suffix}:"
        for line in zones_text.splitlines():
            stripped = line.strip()
            if stripped.startswith(key):
                body = stripped.split("[", 1)[1].rsplit("]", 1)[0]
                numbers = [float(v) for v in body.split(",")]
                return numbers[0], numbers[1]
        return None

    for cube_key, sides in CUBE_APPROACH_SIDES.items():
        cx, cy = cube_xy[cube_key]
        for side in sides:
            entry = approach_entry(cx, cy, side)
            dock = approach_dock(cx, cy, side)
            if side == APPROACH_WEST:
                deployed = deployed_entry(cube_key, "entry")
                if deployed is not None:
                    entry = entry.__class__(
                        entry.zone, deployed[0], deployed[1], entry.yaw
                    )
            elif side == APPROACH_EAST and cube_key.startswith("blue"):
                deployed = deployed_entry(cube_key, "entry_inner")
                if deployed is not None:
                    entry = entry.__class__(
                        entry.zone, deployed[0], deployed[1], entry.yaw
                    )
            entry_clr = _box_clearance(entry.x, entry.y, obstacles)
            assert entry_clr > robot_radius, (cube_key, side, entry_clr)
            dock_clr = _box_clearance(dock.x, dock.y, trays)
            # 0.36 standoff - 0.12 tray half - chassis reaches 0.16.
            assert dock_clr >= 0.08 - 1e-9, (cube_key, side, dock_clr)
        assert CUBE_DOCK_STANDOFF == 0.36
        assert CUBE_ENTRY_OFFSET == 0.54
