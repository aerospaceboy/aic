"""Static integration contracts for the competition mission runtime."""

import math
from pathlib import Path
import xml.etree.ElementTree as ElementTree


REPOSITORY = Path(__file__).resolve().parents[2]


def test_official_world_has_ordered_cubes_and_delivery_zones():
    """Keep the 2026 physical inventory aligned with deterministic names."""
    world = (
        REPOSITORY / "mybot_description" / "worlds" / "offic_room.world"
    )
    root = ElementTree.parse(world).getroot()
    models = {
        model.attrib["name"]: model
        for model in root.findall("./world/model")
        if "name" in model.attrib
    }
    expected_xy = {
        'red': [(-7.5, -2.8), (-0.8, 0.8), (0.0, 3.8),
                (11.4, -9.8), (-4.7, -12.4)],
        'blue': [(7.4, 3.6), (-11.0, -5.6), (-2.0, -6.0),
                 (-0.5, -8.8), (11.8, -13.0)],
    }
    for color in ("red", "blue"):
        for number in range(1, 6):
            suffix = "" if number == 1 else f"_{number}"
            tray = models[f"competition_{color}_pickup_tray{suffix}"]
            tray_pose = [float(value) for value in tray.findtext("pose").split()]
            tray_size = tray.findtext(".//collision/geometry/box/size")
            # Square pickup platforms (long side == narrow side) so the
            # robot can grasp from any direction.
            assert [float(value) for value in tray_size.split()] == [0.24, 0.24, 0.10]
            assert tuple(tray_pose[:2]) == expected_xy[color][number - 1]
            assert math.isclose(tray_pose[2], 0.05, abs_tol=1e-6)
            assert math.isclose(tray_pose[5], math.pi / 2.0, abs_tol=1e-6)
            assert f"{color}_cube_{number}" in models
            cube = models[f"{color}_cube_{number}"]
            pose = [float(value) for value in cube.findtext("pose").split()]
            assert tuple(pose[:2]) == expected_xy[color][number - 1]
            assert math.isclose(pose[2], 0.13, abs_tol=1e-6)
            size = cube.findtext(".//collision/geometry/box/size")
            assert [float(value) for value in size.split()] == [0.03] * 3
            inertia = cube.find(".//inertial/inertia")
            assert math.isclose(float(inertia.findtext("ixx")), 7.5e-7)
            assert math.isclose(float(inertia.findtext("iyy")), 7.5e-7)
            assert math.isclose(float(inertia.findtext("izz")), 7.5e-7)

    expected_zones = {'a': (-5.4, 3.8), 'b': (11.7, -5.0), 'c': (3.3, -12.1)}
    for zone in ("a", "b", "c"):
        delivery_table = models[f"zone_{zone}"]
        assert delivery_table.findtext("static") in ("1", "true")
        base = delivery_table.find(".//link[@name='base']")
        assert base is not None
        visual_size = [
            float(value)
            for value in base.findtext(
                "visual[@name='visual']/geometry/box/size"
            ).split()
        ]
        assert visual_size == [1.0, 0.5, 0.02]
        zone_pose = [
            float(value)
            for value in delivery_table.findtext("pose").split()
        ]
        assert tuple(zone_pose[:2]) == expected_zones[zone]
        assert math.isclose(zone_pose[2], 0.01, abs_tol=1.0e-6)


def test_robot_starts_at_the_east_spawn_gate():
    """Keep Gazebo spawn aligned with the mapped east-side start."""
    launch = (
        REPOSITORY / "mybot" / "launch" / "gazebo_world.launch.py"
    ).read_text(encoding="utf-8")
    assert "default_value='15.0'" in launch
    assert "default_value='-2.0'" in launch
    assert "default_value='3.14159'" in launch
    assert "offic_room.world" in launch


def test_amcl_initial_pose_matches_gazebo_spawn_pose():
    """Keep the navigation localization marker aligned with Gazebo."""
    nav = (
        REPOSITORY / "bot_navigation" / "param" / "originbot_nav2.yaml"
    ).read_text(encoding="utf-8")
    assert "x: 13.2" in nav
    assert "y: -0.95" in nav
    assert "yaw: 3.14159" in nav
    launch = (
        REPOSITORY
        / "tools_demo"
        / "launch"
        / "competition_system.launch.py"
    ).read_text(encoding="utf-8")
    assert '"robot_x",\n            default_value="13.2"' in launch
    assert '"robot_y",\n            default_value="-0.95"' in launch


def test_advanced_spawn_stays_clear_of_obstacle_trigger_zones():
    """The advanced spawn must not pre-trigger either moving obstacle."""
    import math

    import yaml

    config = yaml.safe_load(
        (
            REPOSITORY / "bot_navigation" / "config" / "competition_zones.yaml"
        ).read_text(encoding="utf-8")
    )
    spawn = (13.2, -0.95)
    for index in (1, 2):
        parameters = config["competition_moving_obstacle_%d" % index][
            "ros__parameters"
        ]
        center = (parameters["center_x"], parameters["center_y"])
        distance = math.hypot(spawn[0] - center[0], spawn[1] - center[1])
        assert distance > parameters["trigger_distance"]


def test_advanced_spawn_inside_universal_staging_box():
    """The advanced spawn still triggers the origin-staging gate route."""
    from tools_demo.navigation_math import (
        UNIVERSAL_SPAWN_MAX_Y,
        UNIVERSAL_SPAWN_MIN_X,
        UNIVERSAL_SPAWN_MIN_Y,
    )

    assert 13.2 >= UNIVERSAL_SPAWN_MIN_X
    assert UNIVERSAL_SPAWN_MIN_Y <= -0.95 <= UNIVERSAL_SPAWN_MAX_Y


def test_navigation_config_contains_pickups_and_delivery_approaches():
    """Require both source locations and all destination approaches."""
    config = (
        REPOSITORY
        / "bot_navigation"
        / "config"
        / "competition_zones.yaml"
    ).read_text(encoding="utf-8")
    assert "pickup_red: [-2.40, -1.00, 0.0]" in config
    assert "pickup_blue: [-0.40, -1.00, 0.0]" in config
    assert "pickup_red_exit_clear: [-1.50, 0.00, 0.0]" in config
    assert "pickup_blue_exit_clear: [-0.50, 0.00, 0.0]" in config
    assert "zone_a: [-5.0, 3.8, 3.141592653589793]" in config
    assert "zone_b: [11.3, -5.0, 0.0]" in config
    assert "zone_c: [2.9, -12.1, 0.0]" in config


def test_pick_place_exposes_scan_split_and_safe_release_operations():
    """Require independently invokable scan, pick, place, and release phases."""
    source = (
        REPOSITORY / "mybot" / "pick_place_moveit.py"
    ).read_text(encoding="utf-8")
    assert '"scan": node.run_scan_only' in source
    assert '"pick": node.run_pick_only' in source
    assert '"place": node.run_place_only' in source
    assert '"release": node.run_release_only' in source
    assert '"/competition/manipulation/state"' in source
    assert '"/cmd_vel"' in source
    assert "self.stop_base_motion()" in source
    assert "release_pose = self.get_entity_pose" in source
    assert "self.set_entity_pose(self.cube_name, release_pose)" in source
    assert 'request.get("release_world_pose")' in source
    assert "release_pose.position.x = self.release_world_pose[0]" in source
    assert "release_pose.orientation.w = 1.0" in source
    assert "attempts = 3 if self.release_world_pose is not None else 1" in source
    assert "Loaded placement extension failed; retaining cube" in source
    assert "physical_slot_place_proof" in source
    assert "falling back to world-slot stabilization" not in source
    assert "final_pose.orientation.w = 1.0" in source
    assert "for _ in range(3):" in source
    # World-slot placement now physically extends the loaded arm to the place
    # pose (real arm motion) before the set_entity_pose final alignment.
    assert "place_extend_loaded" in source
    assert "extended_arm_to_place_pose" in source
    assert "message is not previous_message" in source
    assert "settle_sec=0.4" in source


def test_delivery_geometry_matches_the_opposing_tables():
    """Keep arm targets and truth checks synchronized with table geometry."""
    config = (
        REPOSITORY / "tools_demo" / "config" / "mission.yaml"
    ).read_text(encoding="utf-8")
    source = (
        REPOSITORY
        / "tools_demo"
        / "tools_demo"
        / "mission_coordinator.py"
    ).read_text(encoding="utf-8")

    assert "place_pose: [0.350, 0.000, 0.160]" in config
    assert "place_safe_pose: [0.350, 0.000, 0.200]" in config
    assert "delivery_precision_tolerance: 0.06" in config
    assert "place_minimum_x: 0.290" in config
    assert "place_maximum_x: 0.350" in config
    assert "place_maximum_abs_y: 0.160" in config
    assert "world_target_in_base(" in source
    assert 'frame = "world_compensated"' in source
    assert "self.place_maximum_abs_y" in source
    assert "clamped={clamped}" in source
    assert "slot_error=" in source
    assert "precise=" in source
    assert "zone_a_center: [-5.4, 3.8]" in config
    assert "zone_b_center: [11.7, -5.0]" in config
    assert "zone_c_center: [3.3, -12.1]" in config
    assert "zone_half_extent: [0.485, 0.235]" in config
    assert "delivery_surface_z: 0.020" in config
    assert "delivery_cube_half_height: 0.015" in config
    assert "delivery_height_tolerance: 0.10" in config
    assert "abs(position.z - expected_z)" in source
    assert "0.0 <= position.z <= 0.50" not in source


def test_world_delivery_target_cancels_docking_pose_error():
    """World slots must remain fixed when Nav2 stops off its nominal pose."""
    from tools_demo.mission_coordinator import world_target_in_base

    local_x, local_y = world_target_in_base(
        (2.627, 1.039, -0.030),
        (3.000, 1.000),
    )
    cosine = math.cos(-0.030)
    sine = math.sin(-0.030)
    world_x = 2.627 + cosine * local_x - sine * local_y
    world_y = 1.039 + sine * local_x + cosine * local_y

    assert math.isclose(world_x, 3.000, abs_tol=1.0e-9)
    assert math.isclose(world_y, 1.000, abs_tol=1.0e-9)


def test_mission_uses_separate_delivery_slots_for_accuracy():
    """Prevent multiple requested cubes from being dropped into one pose."""
    source = (
        REPOSITORY
        / "tools_demo"
        / "tools_demo"
        / "mission_coordinator.py"
    ).read_text(encoding="utf-8")
    config = (
        REPOSITORY / "tools_demo" / "config" / "mission.yaml"
    ).read_text(encoding="utf-8")
    assert "def _place_slot_poses" in source
    assert "delivery_slot " in source
    assert "place_slot_spacing: 0.080" in config
    assert "spacing = max(" in source
    assert "y_offset = centered_index * spacing" in source


def test_mission_aims_wrist_rgbd_camera_at_pickup_inventory():
    """Use the validated shelf-facing tool quaternion only while scanning."""
    source = (
        REPOSITORY
        / "tools_demo"
        / "tools_demo"
        / "mission_coordinator.py"
    ).read_text(encoding="utf-8")
    config = (
        REPOSITORY / "tools_demo" / "config" / "mission.yaml"
    ).read_text(encoding="utf-8")
    assert (
        "scan_tool_orientation: [-0.2369, 0.9667, 0.0225, 0.0946]"
        in config
    )
    assert "pick_tool_orientation: [0.0, 1.0, 0.0, 0.0]" in config
    assert "manipulation_pick_transit_height: 0.200" in config
    assert "manipulation_pick_lift_duration: 1.1" in config
    assert "reachable_minimum_x: 0.285" in config
    assert "reachable_maximum_x: 0.430" in config
    assert "reachable_minimum_z: 0.005" in config
    assert "pickup_preferred_y: 0.0" in config
    assert "pick_tool_offset: [0.010, 0.001, 0.025]" in config
    assert "scan_pose: [0.300, -0.030, 0.390]" in config
    assert '"tool_orientation":' in source
    assert "self.scan_tool_orientation" in source
    assert "self.pick_tool_orientation" in source
    assert "self.manipulation_pick_transit_height" in source
    assert "self.manipulation_pick_lift_duration" in source


def test_moveit_accepts_measured_gazebo_joint_tracking_error():
    """Keep MoveIt start validation above measured simulation jitter."""
    launch = (
        REPOSITORY / "mybot" / "launch" / "move_group.launch.py"
    ).read_text(encoding="utf-8")
    assert "trajectory_execution.allowed_start_tolerance" in launch
    # Cover equivalent continuous-joint representations through +/- pi;
    # Cartesian paths are still rebuilt from fresh current feedback.
    assert "value=3.2" in launch


def test_mission_launch_and_setup_export_all_runtime_nodes():
    """Require parser, detector, coordinator, and installed entry point."""
    launch = (
        REPOSITORY / "tools_demo" / "launch" / "mission.launch.py"
    ).read_text(encoding="utf-8")
    setup = (
        REPOSITORY / "tools_demo" / "setup.py"
    ).read_text(encoding="utf-8")
    assert 'executable="task_parser"' in launch
    assert 'executable="color_target_detector"' in launch
    assert 'executable="mission_coordinator"' in launch
    assert 'executable="competition_dashboard"' in launch
    assert (
        '"mission_coordinator = tools_demo.mission_coordinator:main"'
        in setup
    )
    assert (
        '"competition_dashboard = "'
        in setup
    )


def test_mission_coordinator_consumes_departure_navigation_results():
    """Safe-exit completion must advance the mission to its destination."""
    source = (
        REPOSITORY / "tools_demo" / "tools_demo" / "mission_coordinator.py"
    ).read_text(encoding="utf-8")
    callback_start = source.index("def navigation_state_callback")
    callback_end = source.index("def perception_callback", callback_start)

    assert '"navigating_departure"' in source[callback_start:callback_end]


def test_mission_uses_validated_accurate_manipulation_profile():
    """Keep the precise motion timing validated by the reference revision."""
    config = (
        REPOSITORY / "tools_demo" / "config" / "mission.yaml"
    ).read_text(encoding="utf-8")

    assert "manipulation_velocity_scaling: 1.0" in config
    assert "manipulation_acceleration_scaling: 1.0" in config
    # Cartesian/gripper durations were shortened from 0.8/0.5 to 0.6/0.4 to
    # fit five rounds in the 300 s budget; grasp success is still gated by
    # validate_grasp_window before the gripper closes.
    assert "manipulation_cartesian_duration: 0.6" in config
    assert "manipulation_gripper_duration: 0.35" in config


def test_dashboard_contains_required_scoring_widgets():
    """Require all task, progress, state, and path scoring surfaces."""
    page = (
        REPOSITORY / "tools_demo" / "web" / "dashboard.html"
    ).read_text(encoding="utf-8")

    for element_id in (
        'id="question"',
        'id="order"',
        'id="redCount"',
        'id="blueCount"',
        'id="completed"',
        'id="missionState"',
        'id="navigation"',
        'id="route"',
        'id="recovery"',
        'id="gripper"',
        'id="llm"',
        'id="perception"',
        'id="avoidance"',
        'id="map"',
    ):
        assert element_id in page

    source = (
        REPOSITORY
        / "tools_demo"
        / "tools_demo"
        / "competition_dashboard.py"
    ).read_text(encoding="utf-8")
    assert source.count("self.question_callback,") == 2
    assert 'str(self.get_parameter("avoidance_topic").value)' in source
    assert '"/competition/obstacles/crossing_cart_2"' in source
    assert "self.create_timer(0.05, self.refresh_robot_from_tf)" in source
    assert "setInterval(updateRemainingTime, 100)" in page
    assert "syncMissionClock(mission)" in page
    assert "avoidance.obstacles" in page
    assert "障碍 1" in page
    assert "障碍 2" in page
    assert 'local_path_topic", "/local_plan"' in source
    assert "const trail = Array.isArray(state.trail)" not in page
    assert "const mapToScreen" in page
    assert "const localPath = Array.isArray(state.local_path)" in page
    assert "navigationState.command_path" not in page
    assert "window.setTimeout(refresh, 100)" in page
    assert "setInterval(refresh" not in page
    assert "实时剩余路径" in page
    assert "实际轨迹" not in page
    assert "规划/指令" not in page
    assert "const layout = state.layout || {}" in page
    assert "layout.pickups || []" in page
    assert "layout.zones || []" in page
    assert "layout.cubes || []" in page
    assert "load_world_layout()" in source
    assert 'not name.endswith("_anchor")' in source
    assert 'navigation.state === "active"' in page


def test_integrated_launch_stages_every_competition_subsystem():
    """Require one-command Gazebo, MoveIt, Nav2, mission, and checks."""
    launch = (
        REPOSITORY
        / "tools_demo"
        / "launch"
        / "competition_system.launch.py"
    ).read_text(encoding="utf-8")
    setup = (REPOSITORY / "tools_demo" / "setup.py").read_text(
        encoding="utf-8"
    )

    for package, filename in (
        ("mybot", "gazebo_world.launch.py"),
        ("mybot", "move_group.launch.py"),
        ("bot_navigation", "competition_navigation.launch.py"),
        ("tools_demo", "mission.launch.py"),
    ):
        assert f'"{package}"' in launch
        assert f'"{filename}"' in launch
    assert "TimerAction" in launch
    assert "_render_environment_actions()" in launch
    assert 'DeclareLaunchArgument(\n            "start_llm_server"' in launch
    assert 'LaunchConfiguration("llm_server_executable")' in launch
    assert 'LaunchConfiguration("llm_model_path")' in launch
    assert '"--port",\n            "8081"' in launch
    assert 'executable="competition_acceptance"' in launch
    assert '"competition_acceptance = "' in setup


def test_acceptance_requires_live_rgbd_publishers_and_core_interfaces():
    """Prevent graph-only checks from accepting dormant sensors."""
    config = (
        REPOSITORY / "tools_demo" / "config" / "competition.yaml"
    ).read_text(encoding="utf-8")
    source = (
        REPOSITORY
        / "tools_demo"
        / "tools_demo"
        / "competition_acceptance.py"
    ).read_text(encoding="utf-8")
    gazebo = (
        REPOSITORY / "mybot" / "launch" / "gazebo_world.launch.py"
    ).read_text(encoding="utf-8")

    assert "required_publishers:" in config
    assert "/camera_rgbd/camera_rgbd/image_raw" in config
    assert "/camera_rgbd/camera_rgbd/depth/image_raw" in config
    assert "self.count_publishers(topic) == 0" in source
    assert "COMPETITION_ACCEPTANCE=" in source
    assert "start_gzserver_cmd" in gazebo
    assert "IfCondition(LaunchConfiguration('headless'))" in gazebo


def test_blue_c_routes_prefer_b1_then_b2_with_current_rpp():
    """Use the current controller's measured safe B1/B2 route order."""
    from tools_demo.mission_coordinator import blue_c_static_path_cost

    at_a_exit = (-2.86, 3.0, 0.0)
    at_c_exit = (-9.85, -5.8, 0.0)
    assert blue_c_static_path_cost(at_a_exit, "blue_cube_1") < (
        blue_c_static_path_cost(at_a_exit, "blue_cube_2")
    )
    assert blue_c_static_path_cost(at_c_exit, "blue_cube_2") < (
        blue_c_static_path_cost(at_c_exit, "blue_cube_3")
    )
    # The physical zone-exit handoff finishes about 2.1 m from the nominal
    # route origin, but it is still the same calibrated C-exit route.
    actual_c_handoff = (-8.135, -6.942, -3.107)
    assert blue_c_static_path_cost(actual_c_handoff, "blue_cube_2") < (
        blue_c_static_path_cost(actual_c_handoff, "blue_cube_3")
    )
    assert blue_c_static_path_cost((0.0, -5.0, 0.0), "blue_cube_1") is None


def test_red_three_measured_corner_stall_is_deprioritised_for_zone_a():
    """Prefer another spare red slot over the observed 32-second stall."""
    from tools_demo.mission_coordinator import cube_route_risk_penalty

    assert cube_route_risk_penalty("A", "red_cube_3") == 40.0
    assert cube_route_risk_penalty("C", "red_cube_3") == 0.0
    # Far south-west red tray is deprioritised for every delivery zone so a
    # four-out-of-five red task keeps the round under 300 s under VM load.
    assert cube_route_risk_penalty("A", "red_cube_5") == 25.0
    assert cube_route_risk_penalty("B", "red_cube_5") == 25.0
    assert cube_route_risk_penalty("C", "red_cube_5") == 25.0


def test_blue_five_safe_obstacle_two_detour_is_deprioritised_for_zone_b():
    """A four-blue B task must leave the measured 93-second spare unused."""
    from tools_demo.mission_coordinator import cube_route_risk_penalty

    assert cube_route_risk_penalty("B", "blue_cube_5") == 30.0
    assert cube_route_risk_penalty("A", "blue_cube_5") == 0.0


def test_cube_selection_cost_penalises_lateral_offset_and_failures():
    """Centre-out selection: same nav base, nearest lateral wins."""
    from tools_demo.mission_coordinator import cube_selection_cost

    base = 30.0
    centre = cube_selection_cost(base, 0.0, 0)
    mid = cube_selection_cost(base, 0.12, 0)
    far = cube_selection_cost(base, 0.24, 0)
    failed = cube_selection_cost(base, 0.0, 1)
    assert centre < mid < far
    assert failed > centre + 4.0
    assert cube_selection_cost(10.0, 0.24, 2) > cube_selection_cost(
        10.0, 0.24, 0
    )

    outer_first = cube_selection_cost(
        base, 0.24, 0, hardest_first=True
    )
    middle_next = cube_selection_cost(
        base, 0.12, 0, hardest_first=True
    )
    centre_last = cube_selection_cost(
        base, 0.0, 0, hardest_first=True
    )
    assert outer_first < middle_next < centre_last


def test_cube_selection_scores_the_complete_delivery_route():
    """Prefer the shortest pickup-plus-delivery route, not nearest pickup."""
    from tools_demo.mission_coordinator import two_leg_navigation_cost

    config = (
        REPOSITORY / "tools_demo" / "config" / "mission.yaml"
    ).read_text(encoding="utf-8")
    source = (
        REPOSITORY / "tools_demo" / "tools_demo" / "mission_coordinator.py"
    ).read_text(encoding="utf-8")
    assert "cube_group_preference_bonus: 0.0" in config
    assert 'declare_parameter("cube_group_preference_bonus", 0.0)' in source

    # The nearer pickup must not win: the far blue cube sits beside the
    # moved C bay at (-9, -7), so its two-leg route is cheaper.
    robot = (-2.00, -1.00)
    zone_c = (-9.40, -7.00)
    near_cube = (4.00, 1.91)
    far_cube = (-10.36, -3.00)
    assert math.dist(robot, near_cube) < math.dist(robot, far_cube)
    assert two_leg_navigation_cost(robot, far_cube, zone_c) < (
        two_leg_navigation_cost(robot, near_cube, zone_c)
    )


def test_lateral_centering_uses_measured_forward_compensation():
    """Mid-slot centring must correct the observed tool-X undershoot."""
    from tools_demo.mission_coordinator import (
        CENTERED_PICK_FORWARD_COMPENSATION,
    )

    assert CENTERED_PICK_FORWARD_COMPENSATION == 0.045


def test_pick_tool_y_offset_centres_cube_before_gripper_closes():
    """Correct the measured 9 mm lateral miss without changing arm stow."""
    config = (
        REPOSITORY / "tools_demo" / "config" / "mission.yaml"
    ).read_text(encoding="utf-8")
    pick_source = (
        REPOSITORY / "mybot" / "pick_place_moveit.py"
    ).read_text(encoding="utf-8")

    assert "pick_tool_offset: [0.010, 0.001, 0.025]" in config
    assert "staging_arm = {name: 0.0 for name in ARM_JOINT_NAMES}" in (
        pick_source
    )


def test_late_cube_detection_never_falls_back_to_a_blind_slot():
    """A no-match frame must wait for RGB-D evidence instead of blind pick."""
    source = (
        REPOSITORY
        / "tools_demo"
        / "tools_demo"
        / "mission_coordinator.py"
    ).read_text(encoding="utf-8")
    config = (
        REPOSITORY / "tools_demo" / "config" / "mission.yaml"
    ).read_text(encoding="utf-8")
    assert "perception_fallback_slot" not in source
    assert "hardest_first = False" in source
    assert "perception_timeout_sec: 1.0" in config
    assert "maximum_perception_retries: 1" in config
    assert "perception_recovery_rescans: 5" in config
    assert "if candidate.index == job.index" in source
    assert "perception_cached_slot:" in source
    assert "self.inventory_scan_complete = False" in source
    assert "self.inventory_scan_complete = True" in source
    assert "speed = 0.20" in source


def test_reachable_cached_slot_is_not_backed_out_of_grasp_range():
    """After one realignment, a safe slot must reach guarded fallback."""
    source = (
        REPOSITORY
        / "tools_demo"
        / "tools_demo"
        / "mission_coordinator.py"
    ).read_text(encoding="utf-8")

    recovery = source.split(
        "    def _attempt_no_target_recovery", 1
    )[1].split("    def _backup_dock_for_rescan", 1)[0]
    assert "fallback_reachable = (" in recovery
    assert "if used >= 1 and fallback_reachable:" in recovery
    assert "perception_recovery_handoff_slot_fallback:" in recovery
    assert recovery.index("return False") < recovery.index(
        "self._backup_dock_for_rescan(job.cube_name)"
    )


def test_centering_time_budget_covers_observed_base_speed():
    """
    The lateral-centring budget must reach the outer shelf slots.

    Loaded v2 Gazebo can move at ~40% of the commanded lateral
    speed, so the wall-clock budget must cover the observed speed or the
    outer shelf slots keep a residual that fails the grasp window. The
    budget must cover every slot offset at the observed speed.
    """
    from tools_demo.mission_coordinator import centering_time_budget

    for target_y in (0.12, 0.21, 0.24, -0.24):
        budget = centering_time_budget(target_y, 0.20)
        assert budget * (0.20 * 0.40) >= abs(target_y) - 1e-9
        assert budget <= 4.0
    assert centering_time_budget(0.24, 0.20) > centering_time_budget(0.12, 0.20)


def test_cube_slot_base_z_matches_supported_inventory_height():
    """Cached cube z must match the deterministic pickup trays."""
    from tools_demo.mission_coordinator import CUBE_SLOT_BASE_Z

    # mybot/pick_place_moveit.py BASE_LINK_HEIGHT.
    base_link_height = 0.1034

    supported_cube_z = 0.13
    assert abs(CUBE_SLOT_BASE_Z - (supported_cube_z - base_link_height)) < 0.001


def test_cached_slot_fast_path_cannot_trigger_a_centring_move():
    """
    A cached slot accurate to 15 mm must not drive a centring compensation.

    Measured regression: the cached world-slot estimate reported y=0.0149 for
    red_cube_2, crossing the 0.012 centring threshold. The coordinator then
    drove a lateral centring move and added the +0.045 m forward compensation
    calibrated for real centring moves, putting the grasp at tool x=0.0595
    against a 0.035 limit. The pick failed, forced a rescan, and cost about
    10 s per affected round. A fresh scan measured y=-0.0098 and succeeded.
    """
    source = (
        REPOSITORY
        / "tools_demo"
        / "tools_demo"
        / "mission_coordinator.py"
    ).read_text(encoding="utf-8")

    # The threshold must be a single named constant, not repeated literals,
    # so the cached-path gate and the centring decision cannot diverge.
    assert "CENTERING_LATERAL_THRESHOLD = 0.033" in source
    assert "abs(raw_y) > 0.012" not in source
    assert source.count("abs(raw_y) > CENTERING_LATERAL_THRESHOLD") >= 2

    # The compensation must still be tied to the same threshold.
    assert "CENTERED_PICK_FORWARD_COMPENSATION = 0.045" in source

    # The cached fast path must be gated on the cached estimate being inside
    # the no-centring band.
    assert "cached_is_graspable" in source
    gate_index = source.index("cached_is_graspable = (")
    gate = source[gate_index:gate_index + 200]
    assert "abs(expected[1]) <= 0.025" in gate
    # And the scan branch must actually be reachable when the gate rejects.
    condition_index = source.index("self.inventory_scan_complete", gate_index)
    condition = source[condition_index:condition_index + 200]
    assert "cached_is_graspable" in condition


def test_centring_threshold_sits_inside_the_arm_grasp_window():
    """
    Centring must only fire for offsets the arm cannot already tolerate.

    Two measured failure modes bound this value. Too tight and a small offset
    triggers a centring move plus the +0.045 m forward compensation calibrated
    for a much larger correction: at 0.012, offsets of 12.4-16.0 mm drove
    34-37 mm centring moves that overshot to tool x = 0.0587-0.0596 against the
    0.035 grasp limit, failing every time. Too loose and the arm would be asked
    to grasp outside its safe window.
    """
    coordinator = (
        REPOSITORY
        / "tools_demo"
        / "tools_demo"
        / "mission_coordinator.py"
    ).read_text(encoding="utf-8")
    arm = (
        REPOSITORY / "mybot" / "pick_place_moveit.py"
    ).read_text(encoding="utf-8")

    def scalar(text, key):
        for line in text.splitlines():
            stripped = line.strip()
            if stripped.startswith(f"{key} = "):
                return float(stripped.split("=", 1)[1].split("#")[0].strip())
        raise AssertionError(f"missing {key}")

    threshold = scalar(coordinator, "CENTERING_LATERAL_THRESHOLD")
    compensation = scalar(coordinator, "CENTERED_PICK_FORWARD_COMPENSATION")
    grasp_limit = scalar(arm, "GRASP_XY_LIMIT")

    # An un-centred grasp at the threshold must still be inside the window,
    # otherwise skipping the centring move would put the cube out of reach.
    assert threshold < grasp_limit, (threshold, grasp_limit)

    # The compensation is only safe to apply when the resulting forward offset
    # cannot by itself exceed the window. This is the invariant that was
    # violated: 0.045 compensation against a 0.035 limit means any centring
    # move that does not genuinely need the full compensation fails the grasp.
    assert compensation > grasp_limit, (
        "compensation exceeds the grasp limit, so it must only ever fire for "
        "offsets large enough to actually need it",
        compensation,
        grasp_limit,
    )
    # Measured successful un-centred grasps sit at y up to 0.0137, so the
    # threshold must be above that or those grasps get needlessly centred.
    assert threshold >= 0.014, threshold


def test_first_pick_failure_retries_the_same_cube():
    """Keep a cube eligible after its first failure instead of switching.

    Measured 2026-09-12: about one pick in four ended with the cube slipping
    out of the gripper onto the tray even though the grasp window, the close
    and the attachment were all clean. Switching cubes costs 25-50 s of extra
    navigation; a second approach to the same cube costs a few seconds and
    succeeds about three times in four.
    """
    source = (
        REPOSITORY / "tools_demo" / "tools_demo" / "mission_coordinator.py"
    ).read_text(encoding="utf-8")
    marker = 'if operation == "pick" and not success:'
    block = source.split(marker, 1)[1].split(
        "action = self.machine.manipulation_finished(success)", 1
    )[0]
    assert "if failures >= 2:" in block
    assert "self._cube_excluded.add(cube)" in block
    assert 'detail = f"retry {cube}"' in block
    # The exclusion still happens on the second failure.
    assert block.index("if failures >= 2:") < block.index(
        "self._cube_excluded.add(cube)"
    )


def _recovery_harness(cube_x, cube_y, used=0, rescans=5):
    """Bare object exercising the real recovery method with mocks."""
    from types import SimpleNamespace
    from unittest.mock import Mock

    actions = []
    harness = SimpleNamespace(
        machine=SimpleNamespace(attempts={"perception": 1},
                                maximum_perception_retries=1),
        _perception_recovery_used={"red_cube_2": used},
        perception_recovery_rescans=rescans,
        reachable_minimum_x=0.15,
        reachable_maximum_x=0.60,
        reachable_maximum_abs_y=0.30,
        _active_job=lambda: SimpleNamespace(cube_name="red_cube_2"),
        _expected_cube_base_position=lambda _name: (cube_x, cube_y),
        _realign_dock_to_cached_slot=lambda name: actions.append("realign"),
        _backup_dock_for_rescan=lambda name: actions.append("backup"),
        start_manipulation=lambda _action: actions.append("scan"),
        get_logger=lambda: Mock(),
    )
    return harness, actions


def test_large_dock_error_keeps_realigning_toward_the_fallback_band():
    """A 0.19 m dock error must keep closing, not back away or give up.

    Measured dock errors: 234 samples, median 0.038, p90 0.073, max 0.192 m,
    while one bounded realign moves the base ~0.023 m and the fallback grasp
    band is 0.10 m.
    """
    from tools_demo.mission_coordinator import MissionCoordinator

    harness, actions = _recovery_harness(0.361, 0.192, used=1)
    assert MissionCoordinator._attempt_no_target_recovery(harness) is True
    assert actions == ["realign", "scan"]


def test_in_band_dock_error_hands_off_to_the_guarded_slot_fallback():
    from tools_demo.mission_coordinator import MissionCoordinator

    harness, actions = _recovery_harness(0.361, 0.06, used=1)
    assert MissionCoordinator._attempt_no_target_recovery(harness) is False
    assert actions == []


def test_too_close_cube_still_backs_up_for_a_rescan():
    """The depth mitigation keeps its own trigger, independent of the band."""
    from tools_demo.mission_coordinator import MissionCoordinator

    harness, actions = _recovery_harness(0.22, 0.15, used=1)
    assert MissionCoordinator._attempt_no_target_recovery(harness) is True
    assert actions == ["backup", "scan"]


def test_in_band_cube_hands_off_even_when_it_sits_close():
    from tools_demo.mission_coordinator import MissionCoordinator

    harness, actions = _recovery_harness(0.22, 0.04, used=1)
    assert MissionCoordinator._attempt_no_target_recovery(harness) is False
    assert actions == []


def _stuck_cancel_harness(now, pending=True, deadline=None):
    from types import SimpleNamespace
    from unittest.mock import Mock

    harness = SimpleNamespace(
        cancel_pending=pending,
        cancel_deadline=deadline,
        cancel_timeout=15.0,
        finish_cancel=Mock(),
        get_logger=lambda: Mock(),
    )
    return harness


def test_stuck_cancel_cannot_gate_the_mission_watchdog_forever():
    """cancel_pending must not disable the deadline without a bound.

    Evidence 20261013T180611: after a MoveIt 'Cannot push a new trajectory'
    pick failure the round stayed at navigating_departure for >6 minutes with
    no five-minute-deadline message, because timer_callback skips the deadline
    while cancel_pending is set.
    """
    from tools_demo.mission_coordinator import MissionCoordinator

    # Arming tick: the deadline starts, the cancel is not finished yet.
    harness = _stuck_cancel_harness(now=100.0)
    assert MissionCoordinator._expire_stuck_cancel(harness, 100.0) is False
    assert harness.cancel_deadline == 115.0
    harness.finish_cancel.assert_not_called()

    # Still inside the window: keep waiting for the child result.
    assert MissionCoordinator._expire_stuck_cancel(harness, 110.0) is False
    harness.finish_cancel.assert_not_called()

    # Past the window: finish the cancel instead of hanging the round.
    assert MissionCoordinator._expire_stuck_cancel(harness, 115.5) is True
    harness.finish_cancel.assert_called_once()


def test_finished_cancel_clears_its_deadline():
    from tools_demo.mission_coordinator import MissionCoordinator

    harness = _stuck_cancel_harness(now=1.0, pending=False, deadline=99.0)
    assert MissionCoordinator._expire_stuck_cancel(harness, 100.0) is False
    assert harness.cancel_deadline is None
    harness.finish_cancel.assert_not_called()


def test_repeated_cancel_cannot_extend_the_hard_deadline():
    """Keep the first cancellation deadline and release request."""
    from types import SimpleNamespace

    from tools_demo.mission_coordinator import MissionCoordinator

    harness = SimpleNamespace(
        machine=SimpleNamespace(terminal=False),
        cancel_pending=True,
        cancel_deadline=115.0,
    )
    MissionCoordinator.cancel_callback(harness, SimpleNamespace(data=True))
    assert harness.cancel_deadline == 115.0


def test_runtime_mission_deadline_matches_competition_limit():
    import yaml

    mission = yaml.safe_load((
        REPOSITORY / "tools_demo" / "config" / "mission.yaml"
    ).read_text(encoding="utf-8"))
    assert mission["competition_mission_coordinator"]["ros__parameters"][
        "mission_timeout_sec"
    ] == 300.0
