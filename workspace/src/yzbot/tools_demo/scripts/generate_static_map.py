"""
Generate mapn3.pgm / mapn3.yaml from the authoritative Gazebo world.

The previous static map was empty inside the scene, so the global planner
drove straight lines through the office walls and stalled whenever the local
costmap (laser) blocked the path. This script rasterizes every static
collision box from offic_room.world (room walls, pickup trays, zones, cubes)
into an occupancy grid at the exact resolution/origin the navigation stack
already uses, so the global planner sees the same layout Gazebo spawns.
"""
from __future__ import annotations

import json
import math
import pathlib
import xml.etree.ElementTree as ElementTree

REPO = pathlib.Path(".")
WORLD = REPO / "mybot_description" / "worlds" / "offic_room.world"
MAP_PGM = REPO / "bot_navigation" / "maps" / "mapn3.pgm"
MAP_YAML = REPO / "bot_navigation" / "maps" / "mapn3.yaml"
ROCK_OVERRIDES = REPO / "config" / "rocks_settled.json"
ROCK_YAW_OVERRIDES = REPO / "config" / "rocks_settled_yaw.json"

RESOLUTION = 0.05
ORIGIN_X = -15.6
ORIGIN_Y = -33.4
# Keep the existing grid geometry: 658 x 822 px @ 0.05 m.
MAP_W = 658
MAP_H = 822

FREE = 254
OCCUPIED = 0


def world_to_grid_px(wx, wy):
    """
    Return the PGM pixel for a world point.

    nav2's map_io vertically flips the PGM (image rows go top-to-bottom
    while the map frame has +Y up), so a world +Y must be stored at the
    row mirrored around the image height.
    """
    px = int((wx - ORIGIN_X) / RESOLUTION)
    world_py = int((wy - ORIGIN_Y) / RESOLUTION)
    py = MAP_H - 1 - world_py
    return px, py


def rasterize_box(grid, cx, cy, size_x, size_y, yaw):
    """Mark a yawed axis-aligned box (interior included) as occupied."""
    half_x = size_x / 2.0
    half_y = size_y / 2.0
    cos_y = math.cos(yaw)
    sin_y = math.sin(yaw)
    steps_x = max(int(size_x / RESOLUTION) + 2, 3)
    steps_y = max(int(size_y / RESOLUTION) + 2, 3)
    for i in range(steps_x):
        lx = -half_x + i * (size_x / max(steps_x - 1, 1))
        for j in range(steps_y):
            ly = -half_y + j * (size_y / max(steps_y - 1, 1))
            wx = cx + cos_y * lx - sin_y * ly
            wy = cy + sin_y * lx + cos_y * ly
            px, py = world_to_grid_px(wx, wy)
            if 0 <= px < MAP_W and 0 <= py < MAP_H:
                grid[py * MAP_W + px] = OCCUPIED


def rasterize_oriented_rock_cells(grid, cx, cy, size_x, size_y, yaw):
    """Mark every grid cell overlapping the measured oriented rock box.

    Sampling points on a rotated 50mm lattice leaves holes. Use the four
    separating axes of the box and cell instead; walls and trays retain
    their existing rasterization unchanged.
    """
    hx, hy = size_x/2, size_y/2
    cosine, sine = math.cos(yaw), math.sin(yaw)
    ex = abs(cosine)*hx + abs(sine)*hy
    ey = abs(sine)*hx + abs(cosine)*hy
    pad = RESOLUTION/2 * (abs(cosine)+abs(sine))
    for ix in range(max(0, int((cx-ex-ORIGIN_X)/RESOLUTION)-1),
                    min(MAP_W, int((cx+ex-ORIGIN_X)/RESOLUTION)+2)):
        x = ORIGIN_X + (ix+.5)*RESOLUTION
        if abs(x-cx) > ex+RESOLUTION/2:
            continue
        for iy in range(max(0, int((cy-ey-ORIGIN_Y)/RESOLUTION)-1),
                        min(MAP_H, int((cy+ey-ORIGIN_Y)/RESOLUTION)+2)):
            y = ORIGIN_Y + (iy+.5)*RESOLUTION
            if abs(y-cy) > ey+RESOLUTION/2:
                continue
            dx, dy = x-cx, y-cy
            if (abs(cosine*dx+sine*dy) <= hx+pad
                    and abs(-sine*dx+cosine*dy) <= hy+pad):
                grid[(MAP_H-1-iy)*MAP_W+ix] = OCCUPIED


def main() -> None:
    """Rasterize the world and write the map."""
    root = ElementTree.parse(WORLD).getroot()
    grid = bytearray([FREE]) * (MAP_H * MAP_W)

    def pose_of(model):
        text = model.findtext("pose")
        values = [float(v) for v in text.split()] if text else []
        return (values + [0.0] * 6)[:6]

    room_state = next(
        (
            model
            for model in root.findall("./world/state/model")
            if model.attrib.get("name") == "officeroom"
        ),
        None,
    )
    state_link_poses = {
        link.attrib["name"]: pose_of(link)
        for link in room_state.findall("./link")
    } if room_state is not None else {}
    untitled_state = next(
        (
            model
            for model in root.findall("./world/state/model")
            if model.attrib.get("name") == "Untitled"
        ),
        None,
    )
    untitled_state_poses = {
        link.attrib["name"]: pose_of(link)
        for link in untitled_state.findall("./link")
    } if untitled_state is not None else {}
    rock_overrides = {}
    if ROCK_OVERRIDES.exists():
        rock_overrides = json.loads(ROCK_OVERRIDES.read_text(encoding="utf-8"))
    rock_yaws = {}
    if ROCK_YAW_OVERRIDES.exists():
        rock_yaws = json.loads(ROCK_YAW_OVERRIDES.read_text(encoding="utf-8"))

    # Room walls: the officeroom model's large collision boxes. Gazebo restores
    # the saved <state> when this world loads; those link poses are absolute
    # world poses and override the model-definition poses.
    for model in root.findall("./world/model"):
        if model.attrib.get("name") != "officeroom":
            continue
        model_pose = pose_of(model)
        mx, my, model_yaw = model_pose[0], model_pose[1], model_pose[5]
        for link in model.findall("./link"):
            size_el = link.find("collision/geometry/box/size")
            if size_el is None:
                continue
            size = [float(v) for v in size_el.text.split()]
            # Small blocks are real physical obstacles: the map must
            # mark them or RPP will plan straight through them.
            if max(size[:2]) < 0.05:
                continue
            link_pose = pose_of(link)
            state_pose = state_link_poses.get(link.attrib.get("name"))
            if state_pose is not None:
                center_x, center_y = state_pose[0], state_pose[1]
                world_yaw = state_pose[5]
            else:
                lx, ly, link_yaw = link_pose[0], link_pose[1], link_pose[5]
                center_x = (
                    mx + math.cos(model_yaw) * lx - math.sin(model_yaw) * ly
                )
                center_y = (
                    my + math.sin(model_yaw) * lx + math.cos(model_yaw) * ly
                )
                world_yaw = model_yaw + link_yaw
            rasterize_box(
                grid,
                center_x,
                center_y,
                size[0],
                size[1],
                world_yaw,
            )

    # Static scene furniture: the "Untitled" decorative rock/block boxes,
    # pickup trays and zones. Cubes are deliberately excluded (they are
    # picked up and moved, so marking their spawn poses would leave ghost
    # obstacles). The "Untitled" 1x1 m boxes were previously missing from
    # the map, which is why the global planner drove straight through the
    # large rock near (-2.7,-3.5) and deadlocked against its costmap
    # inflation.
    for model in root.findall("./world/model"):
        name = model.attrib.get("name", "")
        if not name:
            continue
        # Delivery zones are flat floor markers the base must stop ON to
        # place a cube. Rasterizing them as obstacles made the robot's own
        # delivery pose lethal, so the global planner intermittently failed
        # the next goal with "Starting point in lethal space" / "Path is
        # empty" after every delivery. They are excluded from the map.
        is_furniture = (
            name == "Untitled"
            or name.startswith("competition_red_pickup_tray")
            or name.startswith("competition_blue_pickup_tray")
        )
        if not is_furniture:
            continue
        cx, cy = pose_of(model)[:2]
        yaw = pose_of(model)[5]
        if name == "Untitled":
            # 1x1 m rock/block boxes: rasterize each link's collision box.
            # Gazebo restores the saved <state> on load, so the state
            # link poses (absolute world) override the model-definition
            # poses, exactly like the officeroom walls above. Use them
            # so a freshly generated map matches the live world.
            for link in model.findall(".//link"):
                size_el = link.find(".//collision/geometry/box/size")
                if size_el is None:
                    continue
                link_name = link.attrib.get("name")
                state_pose = rock_overrides.get(link_name)
                if state_pose is None:
                    state_pose = untitled_state_poses.get(link_name)
                if state_pose is not None:
                    center_x, center_y = state_pose[0], state_pose[1]
                else:
                    lx, ly = (float(v) for v in
                              (link.findtext("pose") or "0 0 0 0 0 0").split()[:2])
                    center_x, center_y = cx + lx, cy + ly
                size = [float(v) for v in size_el.text.split()]
                rasterize_oriented_rock_cells(
                    grid, center_x, center_y, size[0], size[1],
                    float(rock_yaws.get(link_name, 0.0)),
                )
            continue
        size_el = model.find(".//link/collision/geometry/box/size")
        if size_el is None:
            continue
        size = [float(v) for v in size_el.text.split()]
        rasterize_box(grid, cx, cy, size[0], size[1], yaw)

    # Moving actors are deliberately excluded from the static map. They make
    # two short, odometry-triggered transverse crossings in the public route
    # to the origin. Their live lidar returns create Nav2 costs without leaving
    # phase-dependent static ghosts in the corridor.

    # Write the PGM in the same format nav2 consumes (P5 binary).
    header = f"P5\n{MAP_W} {MAP_H}\n255\n"
    MAP_PGM.write_bytes(header.encode("ascii") + grid)

    yaml_text = f"""image: mapn3.pgm
mode: trinary
resolution: {RESOLUTION}
origin: [{ORIGIN_X}, {ORIGIN_Y}, 0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
"""
    MAP_YAML.write_text(yaml_text, encoding="utf-8")
    occupied = grid.count(OCCUPIED)
    print(f"wrote {MAP_PGM} and {MAP_YAML}")
    print(f"occupied pixels: {occupied} ({occupied / len(grid) * 100:.1f}%)")


if __name__ == "__main__":
    main()
