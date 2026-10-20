"""Generate the validated navigation risk and delivery-route config."""

from pathlib import Path

import yaml


REPOSITORY = Path(__file__).resolve().parents[2]
LAYOUT_FILE = REPOSITORY / "config" / "final_mission_layout.yaml"
OUTPUT_FILE = (
    REPOSITORY / "bot_navigation" / "config" / "navigation_risk_zones.yaml"
)


def _pose(x_value, y_value, yaw=0.0):
    return [float(x_value), float(y_value), float(yaw)]


def build_config():
    """Return the risk config synchronized with the authoritative layout."""
    layout = yaml.safe_load(LAYOUT_FILE.read_text(encoding="utf-8"))
    centers = {
        name: [float(value) for value in item["center"]]
        for name, item in layout["zones"].items()
    }
    nav = {
        name: [center[0] - 0.40, center[1]]
        for name, center in centers.items()
    }

    if (REPOSITORY / "config" / "complex_layout.json").exists():
        strategies = {}
        for name, center in centers.items():
            sign = 1.0 if name == "A" else -1.0
            yaw = 3.141592653589793 if name == "A" else 0.0
            place = _pose(center[0] + sign * 0.40, center[1], yaw)
            exit_pose = _pose(center[0] + sign * 1.40, center[1], yaw)
            if name == "C":
                exit_pose = _pose(center[0] - .85, center[1] + 1.10)
            strategies[name] = {
                "approach_pose": place, "place_pose": place,
                "retreat_pose": exit_pose, "escape_pose": exit_pose,
                "return_waypoint": exit_pose,
                "exit_waypoints": [], "primary_waypoints": [],
                "backup_waypoints": [], "staged_handoff_distance": 0.20,
            }
        return {
            "frame": "map", "robot_half_diagonal": 0.226,
            "inflation_radius": 0.45, "rotation_clearance": 0.68,
            "safe_wait_clearance": 0.90,
            "risk_zones": {"large_rock": {
                "center": [-2.70, -3.54], "radius": 1.08,
                "forbidden_turning": True,
                "escape_waypoint": [-1.90, -4.90],
            }},
            "zone_strategies": strategies,
            "high_frequency_routes": {},
            "recovery": {"reverse_speed": 0.25,
                         "reverse_max_distance": 1.5,
                         "reverse_check_radius": 0.8,
                         "max_recovery_level": 5,
                         "clear_costmap_before_replan": True},
        }

    return {
        "frame": "map",
        "robot_half_diagonal": 0.226,
        "inflation_radius": 0.45,
        "rotation_clearance": 0.68,
        "safe_wait_clearance": 0.90,
        "risk_zones": {
            "large_rock": {
                "center": [-2.70, -3.54],
                "radius": 1.08,
                "forbidden_turning": True,
                "escape_waypoint": [-1.90, -4.90],
            },
            "shelf_corridor": {
                "center": [-2.20, -2.20],
                "radius": 1.0,
                "forbidden_turning": True,
                "escape_waypoint": [-2.00, -0.50],
            },
            "zone_b_corner": {
                "center": centers["B"],
                "radius": 1.4,
                "forbidden_turning": True,
                "exit_waypoint": [-1.49, -12.00],
                "return_waypoint": [-1.99, -12.60],
            },
            "zone_a_bay": {
                "center": centers["A"],
                "nav_point": nav["A"],
                "forbidden_turning": True,
                "escape_waypoint": [-3.35, 3.54],
            },
            "zone_b_bay": {
                "center": centers["B"],
                "nav_point": nav["B"],
                "forbidden_turning": True,
                "escape_waypoint": [-1.49, -12.00],
            },
            "zone_c_bay": {
                "center": centers["C"],
                "nav_point": nav["C"],
                "forbidden_turning": True,
                "escape_waypoint": [-9.85, -5.80],
            },
        },
        "high_frequency_routes": {
            "to_A": {
                "primary_waypoints": [],
                "alternate_waypoints": [],
                "hazard_zones": ["large_rock"],
            },
            "to_B": {
                "primary_waypoints": [],
                "alternate_waypoints": [],
                "hazard_zones": ["zone_b_corner"],
            },
            "to_C": {
                "primary_waypoints": [],
                "alternate_waypoints": [],
                "hazard_zones": ["large_rock"],
            },
            "to_blocks": {
                "primary_waypoints": [],
                "alternate_waypoints": [],
                "hazard_zones": ["large_rock"],
            },
        },
        "zone_strategies": {
            "A": {
                "approach_pose": _pose(*nav["A"]),
                "place_pose": _pose(*nav["A"]),
                "retreat_pose": _pose(-3.35, 3.54),
                "return_waypoint": _pose(-3.35, 3.54),
                "exit_waypoints": [_pose(-2.86, 3.54)],
                "staged_handoff_distance": 0.20,
                "primary_waypoints": [],
                "backup_waypoints": [],
                "escape_pose": _pose(-3.35, 3.54),
            },
            "B": {
                "approach_pose": _pose(*nav["B"]),
                "place_pose": _pose(*nav["B"]),
                "staged_handoff_distance": 0.20,
                "retreat_pose": _pose(-1.49, -12.00),
                "return_waypoint": _pose(-1.99, -12.60),
                "exit_waypoints": [_pose(-1.49, -12.00)],
                "primary_waypoints": [],
                "backup_waypoints": [],
                "escape_pose": _pose(-1.49, -12.00),
            },
            "C": {
                "approach_pose": _pose(*nav["C"]),
                "place_pose": _pose(*nav["C"]),
                "retreat_pose": _pose(-9.85, -5.80),
                "return_waypoint": _pose(-6.60, -6.00),
                "exit_waypoints": [_pose(-9.85, -5.80)],
                "staged_handoff_distance": 0.20,
                "primary_waypoints": [],
                "backup_waypoints": [],
                "escape_pose": _pose(-9.85, -5.80),
            },
        },
        "recovery": {
            "reverse_speed": 0.25,
            "reverse_max_distance": 1.5,
            "reverse_check_radius": 0.8,
            "max_recovery_level": 5,
            "clear_costmap_before_replan": True,
        },
    }


def main():
    """Write a deterministic YAML representation of the validated routes."""
    header = (
        "# Generated from config/final_mission_layout.yaml by\n"
        "# tools_demo/scripts/generate_navigation_risk_zones.py.\n"
    )
    output = yaml.safe_dump(
        build_config(), sort_keys=False, allow_unicode=True
    )
    OUTPUT_FILE.write_text(header + output, encoding="utf-8")
    print(f"wrote {OUTPUT_FILE}")


if __name__ == "__main__":
    main()
