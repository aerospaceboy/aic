"""Costudio 300s guards: C-heavy policy order and red-C path costs."""
import json
from pathlib import Path
from types import SimpleNamespace

from tools_demo.mission_coordinator import red_c_static_path_cost
from tools_demo.pick_order_policy import ordered_task_key
from tools_demo.pick_order_policy import validate_pick_entries

REPOSITORY = Path(__file__).resolve().parents[2]
POLICY = json.loads(
    (Path(__file__).resolve().parents[1] / "config" / "cube_order_policy.json")
    .read_text(encoding="utf-8")
)["tasks"]


def _jobs(key):
    import re

    jobs = []
    for part in key.split("_"):
        count, color, zone = re.fullmatch(r"(\d+)([br])([ABC])", part).groups()
        jobs += [
            SimpleNamespace(
                color={"b": "blue", "r": "red"}[color], target_zone=zone
            )
            for _ in range(int(count))
        ]
    return jobs


def test_four_reds_to_c_prefer_adjacent_trays_not_cube_2():
    """4rC must not start at the far (0,3) tray; skip it when four remain."""
    row = POLICY["4rC_1bA"]["order"]
    cubes = [entry["cube"] for entry in row]
    # Far east tray (0,3) is skipped when four of five reds remain.
    assert "RED_CUBE_2" not in cubes[:4]
    assert len(cubes[:4]) == 4
    assert len(set(cubes[:4])) == 4
    assert cubes[4].startswith("BLUE_")
    jobs = _jobs("4rC_1bA")
    assert ordered_task_key(jobs) == "4rC_1bA"
    entries = [f"{c}:{s}" for c, s in zip(cubes, [e["side"] for e in row])]
    assert validate_pick_entries(entries, jobs)


def test_three_reds_to_c_start_at_closest_tray():
    for key in ("3rC_2bA", "3rC_2bB"):
        cubes = [e["cube"] for e in POLICY[key]["order"]]
        # Must not open the three-red C tour on the far east (0,3) tray.
        assert cubes[0] != "RED_CUBE_2", (key, cubes)
        assert cubes[0].startswith("RED_"), (key, cubes)


def test_bc_hybrid_policy_uses_short_red_and_blue_tours():
    """BC combines the historically shorter red and blue sub-tours."""
    row = POLICY["3rB_2bC"]["order"]
    assert len(row) == 5
    cubes = [entry["cube"] for entry in row]
    assert all(c.startswith("RED_") for c in cubes[:3])
    assert all(c.startswith("BLUE_") for c in cubes[3:])


def test_bc14_avoids_lethal_transition_and_long_blue1_tour():
    """Prefer blue3's measured 38s C tour to blue1's 68s tour."""
    row = POLICY["1rB_4bC"]["order"]
    assert len(row) == 5
    cubes = [entry["cube"] for entry in row]
    assert cubes[0].startswith("RED_")
    assert "BLUE_CUBE_1" not in cubes
    assert any(c.startswith("BLUE_") for c in cubes[1:])


def test_bc41_avoids_long_variable_blue1_tour():
    """Blue1 varied from 66.5 to 76.9s and exhausted BC41's time margin."""
    row = POLICY["4bC_1rB"]["order"]
    assert len(row) == 5
    cubes = {entry["cube"] for entry in row}
    assert cubes >= {"BLUE_CUBE_3", "BLUE_CUBE_5"}
    assert any(c.startswith("RED_") for c in cubes)


def test_bc32_uses_measured_short_blue_prefix():
    """Use short blue tour and proven B-to-red5 leg, avoiding lethal red2."""
    row = POLICY["3bC_2rB"]["order"]
    assert len(row) == 5
    cubes = {entry["cube"] for entry in row}
    assert {"BLUE_CUBE_5"} <= cubes
    assert any(c.startswith("RED_") for c in cubes)


def test_bc_reverse_timeout_routes_use_proven_short_b_to_red5_leg():
    """Replace each 59-64s B-to-red2 return with the measured 26s red5 leg."""
    for key in ("1bC_4rB", "4rB_1bC", "2bC_3rB"):
        row = POLICY[key]["order"]
        red_entries = [entry for entry in row if entry["cube"].startswith("RED_")]
        assert red_entries[-1]["cube"].startswith("RED_"), key
        entries = [f"{entry['cube']}:{entry['side']}" for entry in row]
        assert validate_pick_entries(entries, _jobs(key)), key


def test_red_c_static_path_cost_prefers_south_west_trays():
    near_c_exit = (-9.85, -5.80)
    assert red_c_static_path_cost(near_c_exit, "red_cube_3") < red_c_static_path_cost(
        near_c_exit, "red_cube_2"
    )
    assert red_c_static_path_cost(near_c_exit, "red_cube_5") < red_c_static_path_cost(
        near_c_exit, "red_cube_2"
    )
    # Far from any calibrated origin: fall back to Euclidean scoring.
    assert red_c_static_path_cost((15.0, 10.0), "red_cube_3") is None


def test_centring_threshold_stays_inside_grasp_window():
    from tools_demo import mission_coordinator as coord

    assert coord.CENTERING_LATERAL_THRESHOLD == 0.033
    assert coord.CENTERING_LATERAL_THRESHOLD < 0.035


def test_zone_c_skips_tray_crossing_nearslow_detour():
    """C long-haul must not insert local tray corners in the NearSlow band."""
    source = (
        Path(__file__).resolve().parents[1]
        / "tools_demo"
        / "task_navigator.py"
    ).read_text(encoding="utf-8")
    assert 'reason=c_long_haul' in source
    assert 'zone in ("C",)' in source


def test_blue4_skips_bay_lower_right_gate():
    """Blue 4/5 must not be forced through the south-bay (-1.4,-7.6) gate."""
    from tools_demo.navigation_math import ZonePose
    from tools_demo.task_navigator import TaskNavigator

    class Logger:
        def info(self, *_args):
            pass

    class Harness:
        dock_pose = ZonePose("blue_4_dock", 1.76, -3.90, 3.14)

        def get_logger(self):
            return Logger()

    poses = TaskNavigator._cube_approach_waypoints(
        Harness(), "BLUE_CUBE_4", -2.89, 2.96
    )
    ys = [pose.y for pose in poses]
    assert ys, "expected north-south gates for a y>=1.5 start"
    assert all(y > -6.0 for y in ys), ys
    assert not any(abs(pose.x - (-1.40)) < 1e-6 and pose.y < -7.0 for pose in poses)


def test_zone_route_approach_matches_deployed_zones():
    """A/B/C approach anchors must match competition_zones.yaml centers."""
    from tools_demo.navigation_math import ZONE_ROUTE_APPROACH

    assert abs(ZONE_ROUTE_APPROACH["A"][0] - (-2.41)) < 0.05
    assert abs(ZONE_ROUTE_APPROACH["A"][1] - 2.99) < 0.05
    assert abs(ZONE_ROUTE_APPROACH["B"][0] - (-0.40)) < 0.05
    assert abs(ZONE_ROUTE_APPROACH["B"][1] - (-12.00)) < 0.05
    assert abs(ZONE_ROUTE_APPROACH["C"][0] - (-9.40)) < 0.05
    assert abs(ZONE_ROUTE_APPROACH["C"][1] - (-7.00)) < 0.05
    # A must not share C's anchor.
    assert ZONE_ROUTE_APPROACH["A"] != ZONE_ROUTE_APPROACH["C"]


def test_b_central_south_corridor():
    """Central r4/r2 departures get a west-lane guide to B."""
    from tools_demo.b_delivery_route import b_central_south_waypoints
    from tools_demo.navigation_math import ZonePose

    dock = ZonePose("B", -0.40, -12.00, 0.0)
    poses = b_central_south_waypoints(-0.81, -0.99, dock)
    assert poses
    assert all(abs(p.x + 2.20) < 1e-6 for p in poses)
    # North-east start stays on the NE corridor instead.
    assert b_central_south_waypoints(4.0, 3.0, dock) == []
    # West shelf (r3) is outside the central band.
    assert b_central_south_waypoints(-5.1, -2.5, dock) == []


def test_b_central_south_corridor_stays_out_of_active_navigation():
    """The measured-slower central B experiment must remain disabled."""
    source = (
        REPOSITORY
        / "tools_demo"
        / "tools_demo"
        / "task_navigator.py"
    ).read_text(encoding="utf-8")

    assert "import b_central_south_waypoints" not in source
    assert "b_central_south_waypoints(" not in source
    assert "from tools_demo.b_delivery_route import b_central_north_waypoints" in source
    assert "b_central_north_waypoints(" in source


def test_campaign_relaunches_after_gzserver_crash():
    """A dead gzserver must relaunch the sim, not burn the round budget."""
    source = (
        REPOSITORY
        / "tools_demo"
        / "scripts"
        / "run_zone9_campaign.py"
    ).read_text(encoding="utf-8")

    # policy_20260917T143854 round 1rB_4bA: gzserver died with exit code
    # -11 while spawn_entity called the spawn service, so odom and
    # base_footprint ended up in two unconnected TF trees. The task
    # manager never accepted the question and the round idled for the
    # whole 900 s budget with completed=0.
    assert "def sim_dead(" in source
    # Both launch process names must be covered. gzserver-N is how ros2 launch
    # reports a direct gzserver bringup; the competition launcher starts the
    # "gazebo" wrapper instead, so its death is logged as [gazebo-1] and a
    # gzserver-only pattern misses it - which is why a 2026-10-04 startup
    # crash left the round re-publishing its question into a dead graph for
    # eight minutes. Exercised in test_campaign_sim_health.py.
    assert "gzserver|gazebo" in source
    assert "process has died" in source
    assert "SIM_HEALTH_PERIOD_SEC = 15.0" in source
    assert 'record["sim_relaunches"]' in source
    assert "re-submitted after relaunch: " in source


def test_coverage48_summary_maps_legal_cases():
    """The 48-case audit must key every legal task to a canonical tag."""
    import sys

    scripts = str(REPOSITORY / "tools_demo" / "scripts")
    sys.path.insert(0, scripts)
    import coverage48_summary

    cases = coverage48_summary.legal_cases()
    assert len(cases) == 48
    assert coverage48_summary.order_string("1rA_4bB") == (
        "1:redx1->A | 2:bluex4->B"
    )
    # Both tag spellings of one task collapse onto the same row.
    assert cases["1:redx1->A | 2:bluex4->B"] == "1rA_4bB"
    assert cases["1:bluex4->B | 2:redx1->A"] == "4bB_1rA"
    # The three zone pairs left untested at the 24/48 checkpoint.
    assert cases["1:redx1->B | 2:bluex4->A"] == "1rB_4bA"
    assert cases["1:redx1->C | 2:bluex4->A"] == "1rC_4bA"
    assert cases["1:redx1->C | 2:bluex4->B"] == "1rC_4bB"
