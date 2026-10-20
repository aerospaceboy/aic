#!/usr/bin/env python3
"""Overlay low pickup trays onto the untouched official v2 map."""

from pathlib import Path
import math
import xml.etree.ElementTree as ElementTree


REPOSITORY = Path(__file__).resolve().parents[2]
MAPS = REPOSITORY / "bot_navigation" / "maps"
WORLD = REPOSITORY / "mybot_description" / "worlds" / "offic_room.world"
RESOLUTION = 0.05
ORIGIN_X = -15.6
ORIGIN_Y = -33.4


def read_pgm(path):
    """Return mutable P5 bytes plus dimensions and pixel offset."""
    data = bytearray(path.read_bytes())
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
        tokens.append(bytes(data[index:end]))
        index = end
    while index < len(data) and chr(data[index]).isspace():
        index += 1
    if tokens[0] != b"P5" or tokens[3] != b"255":
        raise ValueError("mapn3_official.pgm must be an 8-bit P5 image")
    width, height = int(tokens[1]), int(tokens[2])
    if len(data) - index < width * height:
        raise ValueError("mapn3_official.pgm has incomplete pixel data")
    return data, width, height, index


def pickup_trays():
    """Yield tray centre, half-size and yaw from the authoritative world."""
    root = ElementTree.parse(WORLD).getroot()
    for model in root.findall("./world/model"):
        name = model.attrib.get("name", "")
        if not (name.startswith("competition_") and "pickup_tray" in name):
            continue
        pose = [
            float(value)
            for value in (model.findtext("pose") or "0 0 0 0 0 0").split()
        ]
        size_text = model.findtext(".//link/collision/geometry/box/size")
        size = [float(value) for value in size_text.split()]
        yield pose[0], pose[1], size[0] / 2.0, size[1] / 2.0, pose[5]


def main():
    """Write mapn3.pgm with only tray footprint cells added as occupied."""
    data, width, height, offset = read_pgm(MAPS / "mapn3_official.pgm")
    trays = list(pickup_trays())
    if len(trays) != 10:
        raise ValueError(f"expected 10 pickup trays, found {len(trays)}")
    marked = 0
    for row in range(height):
        y_value = ORIGIN_Y + (height - 1 - row + 0.5) * RESOLUTION
        for column in range(width):
            x_value = ORIGIN_X + (column + 0.5) * RESOLUTION
            for cx, cy, half_w, half_h, yaw in trays:
                dx = x_value - cx
                dy = y_value - cy
                local_x = math.cos(yaw) * dx + math.sin(yaw) * dy
                local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
                if abs(local_x) <= half_w and abs(local_y) <= half_h:
                    pixel = offset + row * width + column
                    if data[pixel] != 0:
                        data[pixel] = 0
                        marked += 1
                    break
    (MAPS / "mapn3.pgm").write_bytes(data)
    print(f"marked {marked} new occupied tray pixels")


if __name__ == "__main__":
    main()
