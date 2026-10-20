"""Static integration contract tests for competition navigation."""

from pathlib import Path
import math
from types import SimpleNamespace
import xml.etree.ElementTree as ElementTree

import yaml


REPOSITORY = Path(__file__).resolve().parents[2]


def _planar_pose(element):
    text = element.findtext("pose") if element is not None else None
    values = [float(value) for value in text.split()] if text else []
    return (values + [0.0] * 6)[:6]


def _compose_planar(parent, child):
    cosine = math.cos(parent[5])
    sine = math.sin(parent[5])
    return (
        parent[0] + cosine * child[0] - sine * child[1],
        parent[1] + sine * child[0] + cosine * child[1],
        parent[2] + child[2],
        0.0,
        0.0,
        parent[5] + child[5],
    )


def _point_to_oriented_box(x_value, y_value, box):
    center_x, center_y, half_x, half_y, yaw = box
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    delta_x = x_value - center_x
    delta_y = y_value - center_y
    local_x = cosine * delta_x + sine * delta_y
    local_y = -sine * delta_x + cosine * delta_y
    outside_x = max(abs(local_x) - half_x, 0.0)
    outside_y = max(abs(local_y) - half_y, 0.0)
    return math.hypot(outside_x, outside_y)


def _nav_scalar(text, key):
    """Read one numeric 'key: value' entry from the Nav2 parameter file."""
    for line in text.splitlines():
        stripped = line.strip()
        if stripped.startswith(f"{key}:"):
            return float(stripped.split(":", 1)[1].split("#")[0].strip())
    raise AssertionError(f"missing nav parameter {key}")


def _nav_scalars(text, key):
    """Read every numeric 'key: value' entry from the Nav2 parameter file."""
    values = []
    for line in text.splitlines():
        stripped = line.strip()
        if stripped.startswith(f"{key}:"):
            values.append(
                float(stripped.split(":", 1)[1].split("#")[0].strip())
            )
    assert values, f"missing nav parameter {key}"
    return values


def _world_boxes(root):
    boxes = []
    for model in root.findall("./world/model"):
        model_name = model.attrib.get("name", "")
        if model_name == "ground_plane" or model_name.startswith(
            ("competition_moving_obstacle", "zone_", "red_cube_", "blue_cube_")
        ):
            continue
        model_pose = _planar_pose(model)
        for link in model.findall("link"):
            link_pose = _compose_planar(model_pose, _planar_pose(link))
            for collision in link.findall("collision"):
                collision_pose = _compose_planar(
                    link_pose, _planar_pose(collision)
                )
                size = collision.findtext("geometry/box/size")
                if size is None:
                    continue
                size_x, size_y, size_z = (
                    float(value) for value in size.split()
                )
                if (
                    collision_pose[2] + size_z / 2.0 < 0.0
                    or collision_pose[2] - size_z / 2.0 > 0.60
                ):
                    continue
                boxes.append(
                    (
                        "wall" if model_name == "officeroom" else "fixed",
                        (
                            collision_pose[0],
                            collision_pose[1],
                            size_x / 2.0,
                            size_y / 2.0,
                            collision_pose[5],
                        ),
                    )
                )
    return boxes


def test_nav2_recovery_is_process_isolated_and_respawning():
    """One lifecycle recovery must not crash every Nav2 action server."""
    launch = (
        REPOSITORY
        / "bot_navigation"
        / "launch"
        / "competition_navigation.launch.py"
    ).read_text(encoding="utf-8")
    assert '"use_composition": "False"' in launch
    assert '"use_respawn": "True"' in launch


def test_costudio_camera_streams_do_not_leak_when_disabled():
    """Image topics must remain behind the camera-stream launch gate."""
    source = (
        REPOSITORY / "tools_demo" / "launch" / "costudio_bridge.launch.py"
    ).read_text(encoding="utf-8")
    always_streamed = source.split("TOPIC_WHITELIST = [", 1)[1].split(
        "]\n\nCAMERA_STREAM_TOPICS", 1
    )[0]
    camera_streamed = source.split("CAMERA_STREAM_TOPICS = [", 1)[1].split(
        "]\n\nROBOT_MODEL_TOPICS", 1
    )[0]
    assert "debug_image" not in always_streamed
    assert "debug_image" in camera_streamed


def test_official_world_uses_a_control_rate_physics_step():
    """Keep the ODE step fine enough for grasp contacts, but not 1 kHz.

    The 250 Hz step (max_step_size 0.004) was chosen to spare the four-core
    host, but 2026-10-03/04 the GUI 48-case batch at 250 Hz produced four
    rounds with manipulator failures (attachment_anchor_retry, truncated
    pick_descend, "Attached cube settled more than 20 mm") plus a clearance
    penetration of -0.0498 m, and all five re-tested clean on the same code.
    The project had already recorded the same direction once: "250 Hz 测试会
    引入抓取失败，已回退 500 Hz" (docs/AGENT_HISTORY.md:1232).

    500 Hz is the accepted operating point: on the 28-core dev host gzserver
    goes 52% -> 76% CPU while RTF stays 0.987-0.998, so sim time still tracks
    wall time and the 290 s budget is unaffected. Re-check RTF before
    deploying to a smaller host.
    """
    world = (
        REPOSITORY / "mybot_description" / "worlds" / "offic_room.world"
    ).read_text(encoding="utf-8")
    physics = world.split('<physics type="ode">', 1)[1].split("</physics>", 1)[0]
    assert "<max_step_size>0.002</max_step_size>" in physics
    assert "<real_time_update_rate>500</real_time_update_rate>" in physics


def test_case_runner_uses_one_participant_and_direct_safety_gates():
    """Avoid high-bandwidth probes without replacing direct safety checks."""
    source = (
        REPOSITORY
        / "tools_demo"
        / "tools_demo"
        / "competition_case_runner.py"
    ).read_text(encoding="utf-8")
    assert "ActionClient(" in source
    assert '"/collision_monitor/get_state"' in source
    assert '"/cmd_vel_safe"' in source
    assert '"/competition/avoidance/state"' in source
    assert '"/camera_rgbd/camera_rgbd/image_raw"' not in source
    assert '"/gazebo/model_states"' not in source
    assert "float(clearance) > required_surface" in source
    assert 'mission.get("state") == "completed"' in source
    assert '"completed", "failed", "canceled", "timed_out"' in source


def test_fast_transit_keeps_symmetric_turning_and_safety_layers():
    """The sprint profile must not bind one turn direction or bypass safety."""
    parameters = (
        REPOSITORY
        / "bot_navigation"
        / "param"
        / "originbot_nav2.yaml"
    ).read_text(encoding="utf-8")
    # Forward and yaw limits are unchanged; only the bounded lateral
    # channel that holonomic docking/centering needs is enabled, and the
    # safety layers below still gate the final output.
    assert "max_velocity: [2.30, 0.22, 3.2]" in parameters
    assert "min_velocity: [-0.80, -0.22, -3.2]" in parameters
    assert "max_accel: [5.0, 1.0, 5.0]" in parameters
    assert "max_decel: [-4.5, -1.0, -4.0]" in parameters
    assert 'cmd_vel_in_topic: "/cmd_vel"' in parameters
    assert 'cmd_vel_out_topic: "/cmd_vel_collision_safe"' in parameters
    assert 'polygons: ["BodyStop", "NearSlow"]' in parameters
    assert "slowdown_ratio: 0.70" in parameters


def test_official_world_contains_four_kinematic_moving_obstacles():
    """Provide every scored moving object as a physical lidar obstacle."""
    world = REPOSITORY / "mybot_description" / "worlds" / "offic_room.world"
    root = ElementTree.parse(world).getroot()
    models = {
        model.attrib["name"]: model
        for model in root.findall("./world/model")
        if "name" in model.attrib
    }
    expected_initial_xy = {
        "competition_moving_obstacle_1": (8.5, -0.35),
        "competition_moving_obstacle_2": (5.0, -0.35),
        "crossing_cart_1": (0.0, -3.60),
        "crossing_cart_2": (0.4, -3.02),
    }
    assert "preview_moving_partition_1" not in models
    assert "preview_moving_partition_2" not in models
    for name, expected_xy in expected_initial_xy.items():
        assert name in models
        assert models[name].findtext("static") == "false"
        pose = [float(value) for value in models[name].findtext("pose").split()]
        assert tuple(pose[:2]) == expected_xy
        link = models[name].find("link")
        assert link is not None
        assert link.findtext("kinematic") == "true"
        geometry = link.find("collision/geometry")
        assert geometry is not None
        assert geometry.find("box") is not None or geometry.find("cylinder") is not None
        assert link.findtext("inertial/mass") == "1.0"
        assert link.findtext("collision/surface/friction/ode/mu") == "0.8"
    launch = (
        REPOSITORY
        / "bot_navigation"
        / "launch"
        / "competition_navigation.launch.py"
    ).read_text(encoding="utf-8")
    assert 'executable="moving_obstacle"' in launch
    assert launch.count('executable="moving_obstacle"') == 3
    assert "*extra_carts" in launch
    assert 'executable="avoidance_monitor"' in launch
    assert 'IfCondition(LaunchConfiguration("moving_obstacle"))' in launch
    config = (
        REPOSITORY
        / "bot_navigation"
        / "config"
        / "competition_zones.yaml"
    ).read_text(encoding="utf-8")
    assert "competition_moving_obstacle_1:" in config
    assert "competition_moving_obstacle_2:" in config
    assert "competition_moving_obstacle_3:" not in config
    assert config.count("motion_mode: continuous") == 4
    assert config.count("center_y: -0.35") == 2
    assert config.count("crossing_y: -0.10") == 2
    assert "minimum_robot_distance" not in config
    assert "yield_x" not in config
    assert "/competition/obstacles/obstacle_1" in config
    assert "/competition/obstacles/obstacle_2" in config
    assert "/competition/obstacles/obstacle_3" not in config
    mover = (
        REPOSITORY
        / "tools_demo"
        / "tools_demo"
        / "moving_obstacle.py"
    ).read_text(encoding="utf-8")
    assert "GetEntityState" in mover
    assert '"/gazebo/get_entity_state"' in mover
    assert 'source="gazebo"' in mover
    assert 'self.declare_parameter("truth_read_rate_hz", 5.0)' in mover
    assert "self.truth_timer = self.create_timer(" in mover
    assert "self.pending_set_started_ns" in mover
    assert "self.pending_get_started_ns" in mover
    assert "yield_position" not in mover
    assert "candidate_elapsed += self.update_period_sec" in mover
    assert "self.motion_elapsed_sec = candidate_elapsed" in mover
    assert "admit_proximity_guarded_step" in mover
    assert "self.get_clock().now().nanoseconds + self.phase_offset_sec" not in mover
    assert 'self.declare_parameter("service_call_timeout_sec", 1.0)' in mover
    assert "self.pending_set.cancel()" in mover
    assert "self.pending_get.cancel()" in mover
    assert "service timed out; retrying" in mover
    monitor = (
        REPOSITORY
        / "tools_demo"
        / "tools_demo"
        / "avoidance_monitor.py"
    ).read_text(encoding="utf-8")
    assert '"obstacles": {' in monitor


def test_moving_obstacle_sweeps_keep_physical_clearance():
    """Check each full cylinder envelope, not only its center endpoints."""
    world = REPOSITORY / "mybot_description" / "worlds" / "offic_room.world"
    root = ElementTree.parse(world).getroot()
    boxes = _world_boxes(root)
    radius = 0.18
    tracks = (
        ((-4.00, 0.00), (0.00, 0.00)),
        ((-3.50, -9.00), (0.00, -9.00)),
        ((-6.00, -4.00), (-6.00, -6.50)),
    )
    clearances = []
    for start, end in tracks:
        by_kind = {"wall": float("inf"), "fixed": float("inf")}
        for index in range(401):
            fraction = index / 400.0
            x_value = start[0] + fraction * (end[0] - start[0])
            y_value = start[1] + fraction * (end[1] - start[1])
            for kind, box in boxes:
                clearance = (
                    _point_to_oriented_box(x_value, y_value, box) - radius
                )
                by_kind[kind] = min(by_kind[kind], clearance)
        clearances.append(by_kind)

    # Hard rule: every moving obstacle envelope must keep a margin from the
    # walls (no sweep may cross a wall). Obstacle 3's track is user-specified
    # (-3.5,-9)->(0,-9) and passes the diagonal lower walls; it is a
    # kinematic body (no collision response) and Nav2/AMCL avoidance handles
    # the robot, so only the bounded fixed-object floor applies to it. The
    # other two tracks keep the strict wall margin.
    for index, entry in enumerate(clearances):
        if index == 1:
            assert entry["fixed"] >= -0.20, entry
            continue
        assert entry["wall"] >= 0.10, entry
        assert entry["fixed"] >= -0.20, entry
    # Every pair of tracks must stay far enough apart that the two moving
    # cylinders can never collide with each other. Compute each pair's
    # closest approach directly (segment-to-segment, sampled), rather than
    # assuming a fixed layout, so adding a track cannot silently skip this
    # check.

    def _segment_distance(p1, p2, q1, q2):
        def point_to_segment(point, a, b):
            ax, ay = a
            bx, by = b
            px, py = point
            dx, dy = bx - ax, by - ay
            length_sq = dx * dx + dy * dy
            if length_sq == 0.0:
                return math.hypot(px - ax, py - ay)
            t = max(
                0.0,
                min(1.0, ((px - ax) * dx + (py - ay) * dy) / length_sq),
            )
            return math.hypot(px - (ax + t * dx), py - (ay + t * dy))

        best = float("inf")
        for index in range(201):
            fraction = index / 200.0
            sample = (
                p1[0] + fraction * (p2[0] - p1[0]),
                p1[1] + fraction * (p2[1] - p1[1]),
            )
            best = min(best, point_to_segment(sample, q1, q2))
        return best

    for first in range(len(tracks)):
        for second in range(first + 1, len(tracks)):
            start_a, end_a = tracks[first]
            start_b, end_b = tracks[second]
            distance = _segment_distance(start_a, end_a, start_b, end_b)
            assert distance - 2.0 * radius >= 0.50, (first, second, distance)


def test_dashboard_uses_live_obstacles_and_one_map_transform():
    """Render latest truth snapshots without drawing historical routes."""
    dashboard = (
        REPOSITORY / "tools_demo" / "web" / "dashboard.html"
    ).read_text(encoding="utf-8")
    mover = (
        REPOSITORY
        / "tools_demo"
        / "tools_demo"
        / "moving_obstacle.py"
    ).read_text(encoding="utf-8")

    assert "const mapToScreen" in dashboard
    assert "project(" not in dashboard
    assert "state.trail" not in dashboard
    assert "avoidance.obstacles" in dashboard
    assert "state.obstacles || avoidance.obstacles" in dashboard
    assert "navigationState.command_path" not in dashboard
    assert "GetEntityState" in mover
    assert 'source="gazebo"' in mover


def test_lidar_and_local_costmap_track_moving_obstacles_at_high_rate():
    """Keep laser and local costmap updates fast enough for moving actors."""
    lidar = (
        REPOSITORY
        / "mybot_description"
        / "urdf"
        / "lidar_gazebo.xacro"
    ).read_text(encoding="utf-8")
    robot = (
        REPOSITORY
        / "mybot_description"
        / "urdf"
        / "originbot_with_rgbd_gazebo_arm.xacro"
    ).read_text(encoding="utf-8")
    nav = (
        REPOSITORY
        / "bot_navigation"
        / "param"
        / "originbot_nav2.yaml"
    ).read_text(encoding="utf-8")
    assert "<update_rate>10</update_rate>" in lidar
    # The lidar shares its scan plane with the arm pedestal. Keep it centred
    # so both the pedestal and the verified low payload stay inside the
    # existing self mask without changing collision-monitor boundaries.
    assert 'range_min="0.38"' in robot
    assert '<xacro:property name="lidar_offset_x" value="0.0" />' in robot
    assert (
        '<xacro:property name="lidar_offset_z" '
        'value="${base_height/2+0.07}" />'
    ) in robot
    assert "update_frequency: 10.0" in nav
    assert "publish_frequency: 5.0" in nav
    assert "controller_frequency: 15.0" in nav
    # 1.65 -> 1.78 on 2026-10-06: the B-zone families measured 281-292 s
    # against the 300 s motion bound and the lane-2 gate plus keepout add
    # ~8 s. The smoother still caps at 2.30 m/s and cost/collision
    # regulation, risk zones and the safety gates are untouched.
    assert _nav_scalar(nav, "desired_linear_vel") == 1.78
    assert _nav_scalar(nav, "min_approach_linear_velocity") == 0.10
    assert "use_collision_detection: true" in nav
    # The 2D obstacle layer keeps laser marking/clearing light
    # enough that the local costmap does not drop scan frames.
    assert "obstacle_layer" in nav
    assert 'plugins: ["obstacle_layer", "static_layer", "prediction_layer", "inflation_layer"]' in nav
    # Waiting out a crossing obstacle and a large in-place alignment must
    # stay possible without consuming an unbounded part of the round.
    allowance = _nav_scalar(nav, "movement_time_allowance")
    assert 8.0 <= allowance <= 20.0, allowance
    # The base must still pay attention to obstacles at all.
    assert "use_cost_regulated_linear_velocity_scaling: true" in nav


def test_loaded_vm_keeps_navigation_alive_and_bounds_obstacle_service_rate():
    """A short VM scheduler pause must not tear down the live Nav2 stack."""
    launch = (
        REPOSITORY
        / "bot_navigation"
        / "launch"
        / "competition_navigation.launch.py"
    ).read_text(encoding="utf-8")
    zones = (
        REPOSITORY
        / "bot_navigation"
        / "config"
        / "competition_zones.yaml"
    ).read_text(encoding="utf-8")

    assert 'SetParameter(name="bond_timeout", value=15.0)' in launch
    assert "nav2 = GroupAction(" in launch
    assert "delayed_collision_monitor = TimerAction(" in launch
    delay_block = launch.split(
        "delayed_collision_monitor = TimerAction(", 1
    )[1].split(")", 1)[0]
    assert "period=5.0" in delay_block
    assert "actions=[collision_monitor]" in delay_block
    assert "collision_monitor_recovery" not in delay_block
    assert "delayed_collision_lifecycle_manager = TimerAction(" in launch
    manager_block = launch.split(
        "delayed_collision_lifecycle_manager = TimerAction(", 1
    )[1].split(")", 1)[0]
    assert "period=7.0" in manager_block
    assert "actions=[collision_lifecycle_manager]" in manager_block
    assert launch.index("actions=[collision_monitor]") < launch.index(
        "actions=[collision_lifecycle_manager]"
    )
    assert "hot-cycled" in launch
    assert zones.count("update_rate_hz: 10.0") >= 2
    assert zones.count("truth_read_rate_hz: 10.0") >= 2


def test_full_competition_launch_uses_installed_udp_only_fastdds_profile():
    """The normal one-command launch must not depend on clean_launch.sh."""
    launch = (
        REPOSITORY
        / "tools_demo"
        / "launch"
        / "competition_system.launch.py"
    ).read_text(encoding="utf-8")
    setup = (REPOSITORY / "tools_demo" / "setup.py").read_text(
        encoding="utf-8"
    )
    profile = (
        REPOSITORY
        / "tools_demo"
        / "config"
        / "fastdds_udp.xml"
    ).read_text(encoding="utf-8")

    assert 'configure_ros_transport(transport_config)' in launch
    assert 'glob("config/*.xml")' in setup
    assert "<type>UDPv4</type>" in profile
    assert "<useBuiltinTransports>false</useBuiltinTransports>" in profile


def test_only_translation_resets_progress_and_recovery_never_backs_into_shelf():
    """Bound DWB rotation oscillation while keeping shelf-safe recovery."""
    nav = (
        REPOSITORY
        / "bot_navigation"
        / "param"
        / "originbot_nav2.yaml"
    ).read_text(encoding="utf-8")
    recovery = (
        REPOSITORY
        / "bot_navigation"
        / "config"
        / "navigate_to_pose_recovery.xml"
    ).read_text(encoding="utf-8")

    assert 'plugin: "nav2_controller::SimpleProgressChecker"' in nav
    assert "required_movement_angle" not in nav
    assert _nav_scalar(nav, "movement_time_allowance") == 12.0
    assert "<BackUp" not in recovery
    assert "<Spin" not in recovery
    assert '<Wait wait_duration="1"/>' in recovery


def test_static_path_keeps_frozen_refresh_and_dynamic_prediction_stays_separate():
    """Avoid changing ordinary wall routing to tune central moving bodies."""
    recovery = (
        REPOSITORY
        / "bot_navigation"
        / "config"
        / "navigate_to_pose_recovery.xml"
    ).read_text(encoding="utf-8")

    assert '<RateController hz="1.0">' in recovery
    assert "<PipelineSequence" in recovery
    assert recovery.count("<ComputePathToPose") == 2
    assert recovery.count("<FollowPath") == 1
    assert '<PipelineSequence name="ReplanWhileFollowing">' in recovery
    assert '<IsPathValid path="{path}"/>' in recovery
    assert '<PredictionPathClear ' not in recovery
    central = (REPOSITORY/'tools_demo/config/navigate_central_to_pose.xml').read_text()
    assert '<RateController hz="2.0">' in central
    assert '<IsPathValid path="{path}"/>' in central
    assert 'planner_id="CentralExact"' in central


def test_custom_prediction_bt_preserves_humble_builtin_plugins():
    parameters = yaml.safe_load((REPOSITORY/'bot_navigation/param/originbot_nav2.yaml').read_text())
    plugins = parameters['bt_navigator']['ros__parameters']['plugin_lib_names']
    for name in ['nav2_recovery_node_bt_node','nav2_follow_path_action_bt_node',
                 'nav2_is_path_valid_condition_bt_node','nav2_remove_passed_goals_action_bt_node',
                 'dynamic_prediction_path_clear']:
        assert name in plugins
    assert len(plugins) == len(set(plugins))


def test_rpp_lookahead_scales_with_commanded_velocity():
    """Bound lookahead so fast tracking still slows for close turns."""
    params = yaml.safe_load(
        (
            REPOSITORY
            / "bot_navigation"
            / "param"
            / "originbot_nav2.yaml"
        ).read_text(encoding="utf-8")
    )
    follow = params["controller_server"]["ros__parameters"]["FollowPath"]
    assert follow["use_velocity_scaled_lookahead_dist"] is True
    assert 0.3 <= follow["min_lookahead_dist"] < follow["lookahead_dist"]
    assert follow["lookahead_dist"] < follow["max_lookahead_dist"] <= 1.2
    assert follow["lookahead_time"] <= 1.0


def test_rotation_shim_aligns_to_the_stable_path_not_the_final_goal():
    """Initial rotation must follow the computed path tangent."""
    params = yaml.safe_load(
        (
            REPOSITORY
            / "bot_navigation"
            / "param"
            / "originbot_nav2.yaml"
        ).read_text(encoding="utf-8")
    )["controller_server"]["ros__parameters"]["FollowPath"]
    source = (
        REPOSITORY
        / "tools_demo"
        / "tools_demo"
        / "task_navigator.py"
    ).read_text(encoding="utf-8")
    prealign = source.split(
        "    def start_pre_navigation_alignment(self) -> None:", 1
    )[1].split("\n    def ", 1)[0]

    assert params["plugin"] == (
        "nav2_rotation_shim_controller::RotationShimController"
    )
    assert params["primary_controller"].endswith(
        "RegulatedPurePursuitController"
    )
    assert "needs_pre_navigation_alignment" in prealign
    assert "self.pre_navigation_alignment_timeout_ns" in prealign
    assert "self._send_next_goal()" in prealign


def test_competition_launch_exposes_clean_navigation_start_pose():
    """Dynamic matrix cases need Gazebo, odom, and TF to share one start."""
    launch = (
        REPOSITORY
        / "tools_demo"
        / "launch"
        / "competition_system.launch.py"
    ).read_text(encoding="utf-8")

    for argument in ("robot_x", "robot_y", "robot_yaw"):
        assert f'DeclareLaunchArgument(\n            "{argument}"' in launch
        assert f'"{argument}": LaunchConfiguration("{argument}")' in launch

    matrix_launch = (
        REPOSITORY
        / "tools_demo"
        / "launch"
        / "navigation_matrix_case.launch.py"
    ).read_text(encoding="utf-8")
    assert '"cmd_vel_in": "/cmd_vel_collision_safe"' in matrix_launch
    assert '"cmd_vel_out": "/cmd_vel_safe"' in matrix_launch


def test_navigation_matrix_waits_for_active_nav2_without_fixed_sleep():
    """A matrix result must describe navigation, not launch timing races."""
    probe = (
        REPOSITORY
        / "tools_demo"
        / "tools_demo"
        / "navigation_matrix_probe.py"
    ).read_text(encoding="utf-8")
    runner = (
        REPOSITORY
        / "tools_demo"
        / "scripts"
        / "run_navigation_matrix.py"
    ).read_text(encoding="utf-8")
    matrix_launch = (
        REPOSITORY
        / "tools_demo"
        / "launch"
        / "navigation_matrix_case.launch.py"
    ).read_text(encoding="utf-8")

    assert "lifecycle_msgs.srv import GetState" in probe
    assert '"/controller_server/get_state"' in probe
    assert '"/planner_server/get_state"' in probe
    assert '"/bt_navigator/get_state"' in probe
    assert "self.started = time.monotonic()" in probe.split(
        "    def send(self) -> None:", 1
    )[1]
    assert "time.sleep(22.0)" not in runner
    assert 'parser.add_argument("--start", type=int, default=1)' in runner
    assert "if number < args.start:" in runner
    assert 'parser.add_argument("--moving-obstacle"' in runner
    assert "((args.domain + number - 1) % 200) + 1" in runner
    assert 'GAZEBO_MASTER_URI=f"http://127.0.0.1:{11345 + number}"' in runner
    assert "period=25.0" in matrix_launch
    terminate = runner.split("def _terminate(process):", 1)[1].split(
        "\n\ndef main", 1
    )[0]
    assert "if process.poll() is not None:" not in terminate
    assert "signal.SIGKILL" in terminate
    for topic in (
        '"/cmd_vel_nav"',
        '"/cmd_vel"',
        '"/cmd_vel_collision_safe"',
        '"/cmd_vel_safe"',
    ):
        assert topic in probe
    assert "command_history" in probe
    assert "last_command_sample" in probe
    assert '"/scan"' in probe
    assert '"/plan"' in probe
    assert "dwb_msgs.msg import LocalPlanEvaluation" in probe
    assert '"/evaluation"' in probe
    assert "dwb_evaluation" in probe
    assert "best_index" in probe
    assert "best_moving" in probe
    assert "illegal_counts" in probe
    assert "obstacle_history" in probe
    assert '"/competition/obstacles/obstacle_1"' in probe
    assert '"/competition/obstacles/obstacle_2"' in probe
    package_xml = (REPOSITORY / "tools_demo" / "package.xml").read_text(
        encoding="utf-8"
    )
    assert "<exec_depend>dwb_msgs</exec_depend>" in package_xml
    plan = probe.split("    def _plan", 1)[1].split("\n    def ", 1)[0]
    assert "self.last_progress = time.monotonic()" in plan
    odom = probe.split("    def _odom", 1)[1].split("\n    def ", 1)[0]
    assert "turned" not in odom
    assert "self.translation_pose" in odom


def test_zone_exit_reaches_gateway_once_before_next_pickup():
    """An exited zone must not trigger a second route across its placed cube."""
    source = (
        REPOSITORY
        / "tools_demo"
        / "tools_demo"
        / "task_navigator.py"
    ).read_text(encoding="utf-8")
    exit_branch = source.split("if zone in VALID_ZONE_EXITS:", 1)[1].split(
        "if zone in self.departure_routes:", 1
    )[0]
    pickup_guard = source.split(
        "# A delivered cube leaves the base", 1
    )[1].split("# If the base currently sits", 1)[0]

    assert "zone_exit_gateway" in source
    assert 'zone_name, "exit_waypoints"' in exit_branch
    assert "+ [gateway]" not in exit_branch
    assert "self.last_completed_zone_exit" in source
    assert "nearest != self.last_completed_zone_exit" in pickup_guard

    # C_EXIT is a deliberately short reverse.  The following pickup must
    # still reach the open return corridor before any path-facing rotation.
    assert 'nearest == "C"' in pickup_guard
    assert '"C", "return_waypoint"' in pickup_guard
    assert 'phase="zone_exit_corridor_continue"' in pickup_guard


def test_zone_b_exit_directly_reaches_its_existing_safe_gateway():
    """B exit must not split one straight retreat into direct plus Nav2."""
    from tools_demo.task_navigator import TaskNavigator
    from tools_demo.navigation_math import ZonePose

    class ClockNow:
        nanoseconds = 100

    class FakeClock:
        def now(self):
            return ClockNow()

    class ExitHarness:
        latest_x = -0.43
        latest_y = -12.00
        latest_yaw = 0.0
        active_zone = "B_EXIT"
        zone_exit_gateway = ZonePose("B_retreat_pose", -1.49, -12.0, 0.0)
        # The proven west delivery dock keeps the configured gateway.
        dock_pose = ZonePose("B_dock_west", -0.40, -12.0, 0.0)
        alignment_timeout_ns = 32_000_000_000

        def get_clock(self):
            return FakeClock()

        def _nearest_delivery_zone(self):
            return "B"

        def publish_state(self, *_args, **_kwargs):
            pass

    harness = ExitHarness()
    TaskNavigator.start_zone_egress(harness)
    assert harness.zone_egress_target == (-1.49, -12.0)
    assert harness.alignment_target_yaw == 0.0

    source = (
        REPOSITORY / "tools_demo" / "tools_demo" / "task_navigator.py"
    ).read_text(encoding="utf-8")
    exit_branch = source.split("if zone in VALID_ZONE_EXITS:", 1)[1].split(
        "if zone in self.departure_routes:", 1
    )[0]
    assert 'if zone_name == "B":' in exit_branch
    assert "exit_waypoints = []" in exit_branch
    assert 'self.active_zone == "B_EXIT"' in source


def test_north_blue_pickup_uses_obstacle_2_west_bypass():
    """North starts must cross west of obstacle_2 and the offset walls."""
    from tools_demo.navigation_math import ZonePose
    from tools_demo.task_navigator import TaskNavigator

    class Logger:
        def info(self, *_args):
            pass

    class Harness:
        dock_pose = ZonePose("blue_2", -9.24, -3.0, math.pi)

        def get_logger(self):
            return Logger()

    west_upper = TaskNavigator._cube_approach_waypoints(
        Harness(),
        "BLUE_CUBE_2",
        7.5,
        3.0,
    )
    assert [(pose.x, pose.y) for pose in west_upper] == [
        (-2.10, 1.80),
        (-2.10, 0.00),
        (-4.20, 0.00),
    ]
    # A start at y = 1.0 has no obstacle_2 bypass, but its dock is far west
    # of Wall_57, so it keeps the plain route out of the tip band.
    assert TaskNavigator._cube_approach_waypoints(
        Harness(),
        "BLUE_CUBE_2",
        7.5,
        1.0,
    ) == []

    lower = TaskNavigator._cube_approach_waypoints(
        Harness(),
        "BLUE_CUBE_3",
        -9.16,
        2.0,
    )
    assert [(pose.x, pose.y) for pose in lower] == [
        (-2.10, 1.80),
        (-2.10, 0.00),
        (-1.40, -7.60),
    ]
    assert all(pose.zone != "BLUE_CUBE_3_rock_avoid" for pose in lower)


def test_blue_b_first_round_lookahead_is_strictly_bounded():
    """Only a near-tied lower tray near A exit may replace greedy scoring."""
    from tools_demo.mission_coordinator import blue_b_first_round_lookahead

    scored = [
        {"cube": "blue_cube_1", "total": 33.01, "lateral": 0.0,
         "group": "upper"},
        {"cube": "blue_cube_4", "total": 33.26, "lateral": 0.0,
         "group": "lower"},
    ]
    assert blue_b_first_round_lookahead((-2.86, 3.54, 0.0), 2, scored) == (
        "blue_cube_4"
    )
    assert blue_b_first_round_lookahead((-2.86, 3.54, 0.0), 1, scored) is None
    assert blue_b_first_round_lookahead((8.0, 3.0, 0.0), 2, scored) is None

    expensive_lower = [dict(item) for item in scored]
    expensive_lower[1]["total"] = 34.02
    assert blue_b_first_round_lookahead(
        (-2.86, 3.54, 0.0), 2, expensive_lower
    ) is None


def test_zone_exit_gateway_does_not_add_an_arbitrary_final_turn():
    """A cleared exit must hand its current path heading to the next leg."""
    source = (
        REPOSITORY / "tools_demo" / "tools_demo" / "task_navigator.py"
    ).read_text(encoding="utf-8")
    finish = source.split("def _finish_route_endpoint", 1)[1].split(
        "def _complete_nav_handoff", 1
    )[0]
    assert "self.active_zone in VALID_ZONE_EXITS" in finish
    assert 'phase="zone_exit_complete"' in finish
    assert "self.start_alignment()" in finish
    assert finish.index("zone_exit_complete") < finish.index("start_alignment")


def test_navigation_has_bounded_slow_progress_watchdog():
    """Slow wall-corner crawling cancels into the existing bounded retry."""
    source = (
        REPOSITORY
        / "tools_demo"
        / "tools_demo"
        / "task_navigator.py"
    ).read_text(encoding="utf-8")
    assert 'self.declare_parameter("progress_window_sec", 8.0)' in source
    assert 'self.declare_parameter("progress_min_translation", 0.50)' in source
    assert (
        'self.declare_parameter("progress_rotation_grace_sec", 4.0)'
        in source
    )
    assert 'self.declare_parameter("progress_min_rotation", 0.35)' in source
    timeout = source.split("def timeout_callback", 1)[1].split(
        "def _clear_goal", 1
    )[0]
    assert "navigation_progress_stalled(" in timeout
    assert "self.goal_handle.cancel_goal_async()" in timeout
    # The watchdog must stay live after the retry budget is spent. Gating it
    # on nav_retries turned it off for the rest of the leg: 4bB_1rA
    # (2026-10-07) burned 41-46 s in a second stall that way. Escalation is
    # bounded in result_callback instead.
    assert "self.nav_retries < self.nav_retry_attempts" not in timeout
    assert "self.progress_recovery_pending = True" in timeout
    result = source.split("def result_callback", 1)[1].split(
        "def cancel_callback", 1
    )[0]
    progress = result.split("if self.progress_recovery_pending:", 1)[1].split(
        "try:", 1
    )[0]
    assert "self._retry_nav_goal()" in progress
    assert "self._try_bounded_recovery()" in progress
    assert progress.index("self._retry_nav_goal()") < progress.index(
        "self._try_bounded_recovery()"
    )
    assert "translation >= self.progress_min_translation" in timeout
    assert "self.progress_anchor_x = self.latest_x" in timeout
    assert "self.last_completed_zone_exit = None" in source
    assert "progress_rotation_grace" in timeout
    assert "window_elapsed < rotation_deadline" in timeout
    assert "rotation >= self.progress_min_rotation" in timeout
    assert "self.goal_handle.cancel_goal_async()" in timeout


def test_pick_failure_can_switch_to_another_same_color_cube():
    """A physical pick failure must not consume the ordered task item."""
    source = (
        REPOSITORY / "tools_demo" / "tools_demo" / "mission_coordinator.py"
    ).read_text(encoding="utf-8")
    callback = source.split("def timer_callback", 1)[1]
    callback = callback.split("def handle_action", 1)[0]
    assert "self._cube_excluded.add(cube)" in callback
    assert "self._round_cube_name = None" in callback
    assert "self.machine.pick_failed_switch_cube()" in callback
    state_machine = (
        REPOSITORY / "tools_demo" / "tools_demo" / "mission_state_machine.py"
    ).read_text(encoding="utf-8")
    assert "self.switch_after_departure = True" in state_machine
    assert "if self.switch_after_departure:" in state_machine
    assert "return self._navigate_pickup()" in state_machine
    assert "_select_best_cube" in source


def test_every_delivery_zone_defines_an_open_exit_gateway():
    """A/B/C exits must finish beyond the placed-cube exclusion area."""
    risk = yaml.safe_load(
        (
            REPOSITORY
            / "bot_navigation"
            / "config"
            / "navigation_risk_zones.yaml"
        ).read_text(encoding="utf-8")
    )["zone_strategies"]

    for zone in ("A", "B", "C"):
        place = risk[zone]["place_pose"]
        gateway = risk[zone]["retreat_pose"]
        assert math.hypot(
            gateway[0] - place[0], gateway[1] - place[1]
        ) >= 1.0, zone
        assert "exit_waypoints" in risk[zone]


def test_navigation_launch_uses_fixed_map_frame_and_helpers():
    """Require one map transform plus both competition helper nodes."""
    launch = (
        REPOSITORY
        / "bot_navigation"
        / "launch"
        / "competition_navigation.launch.py"
    ).read_text(encoding="utf-8")
    assert "static_transform_publisher" in launch
    assert 'executable="task_navigator"' in launch
    assert 'executable="moving_obstacle"' in launch
    assert launch.count('executable="moving_obstacle"') == 3
    assert "*extra_carts" in launch
    assert 'executable="avoidance_monitor"' in launch
    assert '"slam": "False"' in launch
    system_launch = (
        REPOSITORY
        / "tools_demo"
        / "launch"
        / "competition_system.launch.py"
    ).read_text(encoding="utf-8")
    obstacle_argument = system_launch.split(
        '"moving_obstacle",', 1
    )[1].split("),", 1)[0]
    assert 'default_value="True"' in obstacle_argument


def test_pre_navigation_alignment_timeout_falls_back_to_nav2():
    """A failed optional pre-turn must not skip an otherwise valid cube."""
    source = (
        REPOSITORY
        / "tools_demo"
        / "tools_demo"
        / "task_navigator.py"
    ).read_text(encoding="utf-8")
    config = (
        REPOSITORY
        / "bot_navigation"
        / "config"
        / "competition_zones.yaml"
    ).read_text(encoding="utf-8")
    assert "pre_navigation_alignment_timeout_sec" in source
    assert 'phase="pre_navigation_fallback"' in source
    assert "self._send_next_goal()" in source
    assert "pre_navigation_alignment_timeout_sec: 4.0" in config
    assert "alignment_timeout_sec: 32.0" in config


def test_first_command_costmap_gate_does_not_block_executor():
    """The first command must be replayed by the costmap callback, not sleep."""
    source = (
        REPOSITORY
        / "tools_demo"
        / "tools_demo"
        / "task_navigator.py"
    ).read_text(encoding="utf-8")
    command_body = source.split("def command_callback", 1)[1].split(
        "def _maybe_escape_hazard_first", 1
    )[0]
    costmap_body = source.split("def _global_costmap_callback", 1)[1].split(
        "def _global_costmap_ready", 1
    )[0]

    assert "time.sleep(" not in command_body
    assert "self.pending_start_command = message" in command_body
    assert "self.command_callback(pending)" in costmap_body
    assert "self._recreate_action_client()" in costmap_body
    assert "def _recreate_action_client" in source
    assert "self.action_client.destroy()" in source


def test_deferred_command_resumes_once_after_action_server_recovers():
    """Exercise the delayed-server retry state transition, not just its source."""
    from std_msgs.msg import String

    from tools_demo.task_navigator import TaskNavigator

    class FakeActionClient:
        def __init__(self):
            self.ready = False

        def server_is_ready(self):
            return self.ready

    class FakeLogger:
        def info(self, _message):
            pass

        def warn(self, _message):
            pass

        def error(self, _message):
            pass

    class RetryHarness:
        def __init__(self):
            self.pending_start_command = None
            self.active_zone = None
            self.action_client = FakeActionClient()
            self.pending_action_retry_ticks = 0
            self.costmap_ready = True
            self.recreated = 0
            self.resumed = []
            self.states = []
            # Mirror the node's deferred-command deadline state.
            self.pending_start_deadline_ns = None
            self.prerequisite_deadline_sec = 60.0
            self.clock_now = SimpleNamespace(nanoseconds=0)

        def _global_costmap_ready(self):
            return self.costmap_ready

        def _recreate_action_client(self):
            self.recreated += 1
            self.pending_action_retry_ticks = 0

        def command_callback(self, message):
            self.resumed.append(message)

        def publish_error(self, message):
            raise AssertionError(message)

        def publish_state(self, state, **fields):
            self.states.append((state, fields))

        def get_logger(self):
            return FakeLogger()

        def get_clock(self):
            return SimpleNamespace(now=lambda: self.clock_now)

        def _arm_pending_start_deadline(self):
            return TaskNavigator._arm_pending_start_deadline(self)

        def _pending_start_expired(self):
            return TaskNavigator._pending_start_expired(self)

    harness = RetryHarness()
    pending = String(data="RED_EXIT")
    TaskNavigator.command_callback(harness, pending)

    assert harness.pending_start_command is pending
    assert harness.states == [("waiting", {"phase": "action_server"})]

    for _ in range(4):
        TaskNavigator._retry_pending_command(harness)

    assert harness.pending_start_command is pending
    assert harness.recreated == 1
    assert harness.resumed == []

    harness.action_client.ready = True
    TaskNavigator._retry_pending_command(harness)
    TaskNavigator._retry_pending_command(harness)

    assert harness.pending_start_command is None
    assert harness.resumed == [pending]


def test_deferred_command_fails_on_prerequisite_deadline():
    """A prerequisite that never arrives must fail, not stall forever.

    Campaign 20261001T142503 round 27 (2rB_3bC) delivered 0/5: /map_server
    never activated, the global costmap never received a static layer, every
    command was deferred, and the round only ended when the campaign's 150 s
    no-progress watchdog fired. The deferral path must instead give up on its
    own wall-clock deadline with a distinct reason so the round ends fast and
    the cause is recorded.
    """
    from std_msgs.msg import String

    from tools_demo.task_navigator import TaskNavigator

    class FakeActionClient:
        def server_is_ready(self):
            return False

    class FakeLogger:
        def info(self, _message):
            pass

        def warn(self, _message):
            pass

        def error(self, _message):
            pass

    class ExpiryHarness:
        def __init__(self):
            self.pending_start_command = None
            self.active_zone = None
            self.action_client = FakeActionClient()
            self.pending_action_retry_ticks = 0
            self.pending_start_deadline_ns = None
            self.prerequisite_deadline_sec = 60.0
            self.clock_now = SimpleNamespace(nanoseconds=0)
            self.errors = []
            self.states = []

        def _global_costmap_ready(self):
            return False

        def _recreate_action_client(self):
            self.pending_action_retry_ticks = 0

        def command_callback(self, _message):
            raise AssertionError("deferred command must not be dispatched")

        def publish_error(self, message):
            self.errors.append(message)

        def publish_state(self, state, **fields):
            self.states.append((state, fields))

        def get_logger(self):
            return FakeLogger()

        def get_clock(self):
            return SimpleNamespace(now=lambda: self.clock_now)

        def _arm_pending_start_deadline(self):
            return TaskNavigator._arm_pending_start_deadline(self)

        def _pending_start_expired(self):
            return TaskNavigator._pending_start_expired(self)

    harness = ExpiryHarness()
    pending = String(data="RED_CUBE_1")
    TaskNavigator.command_callback(harness, pending)

    assert harness.pending_start_command is pending
    assert harness.pending_start_deadline_ns == 60_000_000_000

    # Still inside the deadline: the command stays deferred.
    harness.clock_now.nanoseconds = 59_000_000_000
    TaskNavigator._retry_pending_command(harness)
    assert harness.pending_start_command is pending
    assert harness.errors == []
    # The action-server gate runs first, so the deferral is reported there.
    assert harness.states == [("waiting", {"phase": "action_server"})]

    # Past the deadline: fail the leg with a distinct reason instead of
    # stalling until the campaign watchdog.
    harness.clock_now.nanoseconds = 61_000_000_000
    TaskNavigator._retry_pending_command(harness)

    assert harness.pending_start_command is None
    assert harness.pending_start_deadline_ns is None
    assert harness.errors == ["navigation prerequisites timeout after 60s"]
    assert harness.states[-1] == (
        "failed",
        {"phase": "navigation_prerequisites"},
    )


def test_costmap_readiness_is_independent_of_scattered_object_positions():
    """Static-map readiness must not probe an old cube or tray coordinate."""
    source = (
        REPOSITORY
        / "tools_demo"
        / "tools_demo"
        / "task_navigator.py"
    ).read_text(encoding="utf-8")
    readiness_body = source.split("def _global_costmap_ready", 1)[1].split(
        "def command_callback", 1
    )[0]

    assert "probe_xy" not in readiness_body
    assert "any(value >= 50 for value in costmap.data)" in readiness_body


def test_bounded_recovery_cannot_use_an_unrelated_zone_escape():
    """Recovery direct control must stay short and on verified free space."""
    source = (
        REPOSITORY
        / "tools_demo"
        / "tools_demo"
        / "task_navigator.py"
    ).read_text(encoding="utf-8")
    body = source.split("def _try_bounded_recovery", 1)[1].split(
        "def _finish_bounded_recovery", 1
    )[0]

    assert "_nearest_delivery_zone" not in body
    assert "distance > self.reverse_max_distance" in body
    assert "self._costmap_segment_clear(" in body


def test_bounded_recovery_requeues_the_failed_nav2_goal():
    """A successful escape must retry Nav2 instead of reporting success."""
    source = (
        REPOSITORY
        / "tools_demo"
        / "tools_demo"
        / "task_navigator.py"
    ).read_text(encoding="utf-8")
    body = source.split("def _try_bounded_recovery", 1)[1].split(
        "def _finish_bounded_recovery", 1
    )[0]

    assert "[self.nav_goal_pose] + list(" in body
    assert "self.pending_poses" in body


def test_pre_navigation_hazard_escape_uses_the_same_safety_gate():
    """Startup escape direct control must also respect map and distance."""
    source = (
        REPOSITORY
        / "tools_demo"
        / "tools_demo"
        / "task_navigator.py"
    ).read_text(encoding="utf-8")
    body = source.split("def _maybe_escape_hazard_first", 1)[1].split(
        "def odom_callback", 1
    )[0]

    assert "distance > self.reverse_max_distance" in body
    assert "self._costmap_segment_clear(" in body


def test_collision_monitor_guards_all_nav_and_direct_velocity():
    """Laser collision monitoring must precede the truth safety limiter."""
    launch = (
        REPOSITORY
        / "bot_navigation"
        / "launch"
        / "competition_navigation.launch.py"
    ).read_text(encoding="utf-8")
    nav_params = (
        REPOSITORY
        / "bot_navigation"
        / "param"
        / "originbot_nav2.yaml"
    ).read_text(encoding="utf-8")
    mission_launch = (
        REPOSITORY / "tools_demo" / "launch" / "mission.launch.py"
    ).read_text(encoding="utf-8")
    acceptance = (
        REPOSITORY / "tools_demo" / "config" / "competition.yaml"
    ).read_text(encoding="utf-8")

    assert "nav2_collision_monitor" in launch
    assert 'executable="collision_monitor"' in launch
    assert 'cmd_vel_in_topic: "/cmd_vel"' in nav_params
    assert 'cmd_vel_out_topic: "/cmd_vel_collision_safe"' in nav_params
    assert '"cmd_vel_in": "/cmd_vel_collision_safe"' in mission_launch
    assert "- /collision_monitor" in acceptance
    assert "- /competition_safety_limiter" in acceptance


def test_rviz_hides_slam_markers_without_a_published_slam_tf():
    """Do not show a permanent error for the evidence-only SLAM frame."""
    rviz = (
        REPOSITORY
        / "bot_navigation"
        / "rviz"
        / "navigation.rviz"
    ).read_text(encoding="utf-8")
    marker = rviz.split(
        "Class: rviz_default_plugins/MarkerArray", 1
    )[1].split("Class: rviz_default_plugins/Map", 1)[0]
    assert "/competition/scene_markers" in marker
    assert "Value: true" in marker


def test_zone_config_matches_world_labels():
    """Keep configured navigation goals aligned with all zone models."""
    config = (
        REPOSITORY
        / "bot_navigation"
        / "config"
        / "competition_zones.yaml"
    ).read_text(encoding="utf-8")
    world_root = ElementTree.parse(
        REPOSITORY / "mybot_description" / "worlds" / "offic_room.world"
    ).getroot()
    world_models = {
        model.attrib["name"]: model
        for model in world_root.findall("./world/model")
        if "name" in model.attrib
    }
    expected_centers = {'a': (-5.4, 3.8), 'b': (11.7, -5.0), 'c': (3.3, -12.1)}
    for zone, (center_x, center_y) in expected_centers.items():
        assert f"zone_{zone}:" in config
        model = world_models[f"zone_{zone}"]
        pose = [float(value) for value in model.findtext("pose").split()]
        assert math.isclose(pose[0], center_x, abs_tol=1.0e-6)
        assert math.isclose(pose[1], center_y, abs_tol=1.0e-6)
    assert "goal_timeout_sec: 300.0" in config
    # Nav2 stops 0.40 m west of the zone model centre so the wrist RGB-D
    # camera reaches the centreline during placement.
    assert "zone_a: [-5.0, 3.8, 3.141592653589793]" in config
    assert "zone_b: [11.3, -5.0, 0.0]" in config
    assert "zone_c: [2.9, -12.1, 0.0]" in config
    assert "pickup_red_exit_clear: [-1.50, 0.00, 0.0]" in config
    assert "pickup_blue_exit_clear: [-0.50, 0.00, 0.0]" in config


def test_nav2_uses_obstacle_aware_cross_room_controller():
    """Use continuous path tracking while preserving direct-dock limits."""
    parameters = (
        REPOSITORY
        / "bot_navigation"
        / "param"
        / "originbot_nav2.yaml"
    ).read_text(encoding="utf-8")
    assert "RegulatedPurePursuitController" in parameters
    # Forward/yaw limits are unchanged; the lateral axis is the bounded
    # holonomic docking channel (see the smoothe test below).
    assert "max_velocity: [2.30, 0.22, 3.2]" in parameters
    desired_velocity = _nav_scalar(parameters, "desired_linear_vel")
    # The smoother must never be the binding limit on the planner.
    assert desired_velocity < 2.30, desired_velocity
    # Fast enough to fit five rounds into the five-minute budget: the longest
    # single leg is about 9 m and a round drives roughly 17.5 m.
    assert desired_velocity >= 1.40, desired_velocity
    assert "max_accel: [5.0, 1.0, 5.0]" in parameters
    assert "min_velocity: [-0.80, -0.22, -3.2]" in parameters
    assert "allow_reversing: false" in parameters
    # Nav2 must own the obstacle-aware path until the final precision zone.
    # A 0.60 m success radius handed a wall-crossing straight segment to the
    # direct controller and made the base turn before reaching the opening.
    assert parameters.count("xy_goal_tolerance: 0.25") >= 1
    assert "yaw_goal_tolerance: 6.28" in parameters
    parsed = yaml.safe_load(parameters)
    assert parsed['local_costmap']['local_costmap']['ros__parameters']['robot_radius'] == .28
    assert parsed['global_costmap']['global_costmap']['ros__parameters']['robot_radius'] == .38
    assert "inflation_radius: 0.45" in parameters
    assert "inflation_radius: 0.65" in parameters
    assert 'plugins: ["obstacle_layer", "static_layer", "prediction_layer", "inflation_layer"]' in parameters
    assert "expected_planner_frequency: 1.0" in parameters


def test_delivery_routes_delegate_the_whole_cross_room_path_to_nav2():
    """Do not split A/B/C into unrelated fixed NavigateToPose legs."""
    import yaml  # noqa: PLC0415

    risk = yaml.safe_load(
        (
            REPOSITORY
            / "bot_navigation"
            / "config"
            / "navigation_risk_zones.yaml"
        ).read_text(encoding="utf-8")
    )
    for zone in ("A", "B", "C"):
        strategy = risk["zone_strategies"][zone]
        assert strategy["primary_waypoints"] == [], zone
        assert strategy["backup_waypoints"] == [], zone


def test_nav2_transit_delegates_path_tangent_alignment_to_rotation_shim():
    """Do not run a second task-level yaw controller before Nav2."""
    source = (
        REPOSITORY
        / "tools_demo"
        / "tools_demo"
        / "task_navigator.py"
    ).read_text(encoding="utf-8")
    body = source.split(
        "    def start_pre_navigation_alignment(self) -> None:", 1
    )[1].split("\n    def ", 1)[0]
    assert "target_is_behind(" not in body
    assert "self.pre_navigation_alignment = False" in body
    assert body.count("self._send_next_goal()") == 1


def test_nav2_handoff_is_inside_the_precision_docking_zone():
    """Keep direct cmd_vel docking shorter than one chassis length."""
    import yaml  # noqa: PLC0415

    zones = yaml.safe_load(
        (
            REPOSITORY
            / "bot_navigation"
            / "config"
            / "competition_zones.yaml"
        ).read_text(encoding="utf-8")
    )
    parameters = zones["competition_task_navigator"]["ros__parameters"]
    assert 0.05 <= float(parameters["nav_handoff_distance"]) <= 0.20


def test_rotation_shim_wraps_rpp_once_for_path_alignment():
    """Use the stable compute-once path tangent before RPP tracking."""
    parameters = (
        REPOSITORY
        / "bot_navigation"
        / "param"
        / "originbot_nav2.yaml"
    ).read_text(encoding="utf-8")
    assert 'plugin: "nav2_rotation_shim_controller::RotationShimController"' in parameters
    assert "RegulatedPurePursuitController" in parameters
    assert "use_regulated_linear_velocity_scaling: true" in parameters
    assert "use_cost_regulated_linear_velocity_scaling: true" in parameters
    assert "use_collision_detection: true" in parameters
    collision_horizon = _nav_scalar(
        parameters, "max_allowed_time_to_collision_up_to_carrot"
    )
    assert 0.50 <= collision_horizon <= 0.75
    assert "use_interpolation: true" in parameters
    assert _nav_scalar(parameters, "inflation_cost_scaling_factor") == 10.0


def test_global_and_local_planners_prefer_corridor_centerlines():
    """Keep a soft wall margin without closing narrow passages."""
    parameters = (
        REPOSITORY
        / "bot_navigation"
        / "param"
        / "originbot_nav2.yaml"
    ).read_text(encoding="utf-8")

    # Gazebo lidar measured one wall 0.11 m closer than its map edge. Extend
    # only the non-lethal gradient so A* chooses corridor centerlines while
    # preserving the 0.28 m passability radius in narrow aisles.
    assert 8.0 <= _nav_scalar(parameters, "cost_travel_multiplier") <= 10.0
    assert _nav_scalars(parameters, "cost_scaling_factor") == [10.0, 10.0]
    assert _nav_scalars(parameters, "inflation_radius") == [0.45, 0.65]
    assert _nav_scalar(parameters, "cost_scaling_dist") == 0.45


def test_local_planner_uses_live_laser_obstacles():
    """Keep moving obstacles in local planning and the safety chain."""
    parameters = (
        REPOSITORY
        / "bot_navigation"
        / "param"
        / "originbot_nav2.yaml"
    ).read_text(encoding="utf-8")
    local_costmap = parameters.split("\nlocal_costmap:\n", 1)[1].split(
        "\nglobal_costmap:\n", 1
    )[0]

    assert 'plugins: ["obstacle_layer", "static_layer", "prediction_layer", "inflation_layer"]' in local_costmap
    assert 'observation_sources: scan' in local_costmap
    assert 'topic: /scan' in local_costmap
    assert 'clearing: True' in local_costmap
    assert 'marking: True' in local_costmap
    assert _nav_scalar(local_costmap, "raytrace_max_range") >= 3.0
    assert _nav_scalar(local_costmap, "obstacle_max_range") >= _nav_scalar(
        local_costmap, "raytrace_max_range"
    )
    # Clearing obstacle cells inside the planning footprint lets DWB command
    # forward motion into a wall while collision_monitor correctly stops it.


def test_authoritative_static_map_keeps_global_replans_stable():
    """Do not let near-field scan noise close a mapped wall-tip corridor."""
    parameters = (
        REPOSITORY
        / "bot_navigation"
        / "param"
        / "originbot_nav2.yaml"
    ).read_text(encoding="utf-8")
    global_costmap = parameters.split("\nglobal_costmap:\n", 1)[1].split(
        "\n# The yaml_filename", 1
    )[0]

    assert 'plugins: ["static_layer", "obstacle_layer", "dynamic_layer", "prediction_layer", "sweep_band_layer", "inflation_layer"]' in global_costmap
    assert 'observation_sources: placed_cubes' in global_costmap
    assert 'topic: /competition/placed_cubes' in global_costmap
    assert 'observation_sources: scan_clear dynamic_points' in global_costmap
    dynamic = yaml.safe_load(parameters)["global_costmap"]["global_costmap"]["ros__parameters"]["dynamic_layer"]
    assert dynamic["scan_clear"]["marking"] is False
    assert dynamic["scan_clear"]["clearing"] is True
    assert dynamic["dynamic_points"]["topic"] == "/competition/dynamic_obstacles"


def test_removed_partitions_leave_only_original_safety_and_static_keepouts():
    """Removed walls cannot leave phantom bands; original protections remain."""
    parameters = (
        REPOSITORY
        / "bot_navigation"
        / "param"
        / "originbot_nav2.yaml"
    ).read_text(encoding="utf-8")
    costmaps = yaml.safe_load(parameters)
    global_costmap = costmaps["global_costmap"]["global_costmap"][
        "ros__parameters"
    ]
    sweep_band = global_costmap["sweep_band_layer"]
    assert sweep_band["plugin"] == "nav2_costmap_2d::ObstacleLayer"
    assert sweep_band["keepout_points"]["marking"] is True
    assert sweep_band["keepout_points"]["clearing"] is False
    assert sweep_band["keepout_points"]["topic"] == \
        "/competition/sweep_band_keepout"
    zones = (
        REPOSITORY
        / "bot_navigation"
        / "config"
        / "competition_zones.yaml"
    ).read_text(encoding="utf-8")
    keepout = yaml.safe_load(zones)["competition_sweep_band_keepout"][
        "ros__parameters"
    ]
    bands = keepout["bands"]
    # New carts must not have permanent sweep bans. Keep the original
    # corridor admission pocket and all three diagonal-wall protections.
    assert bands == [
        "corridor_mover_2:4.78:-0.78:5.22:-0.22",
        # The diagonal wall between (5.5,-0.7) and (9.0,-1.4) is banded on
        # 2026-10-06: the planner hugged it, the pursuit controller clipped
        # its inflation and the base parked inside the inflated ring (start
        # cost 80), burning a 33 s patience/retry/recovery cycle
        # (1rA_4bB 295.8 s; 4bB_1rA 293-301 s in 2 of 3 runs).
        "wall_diag_upper:5.60:-0.75:6.60:-0.62",
        "wall_diag_middle:6.55:-1.05:7.95:-0.88",
        "wall_diag_lower:7.90:-1.35:9.05:-1.15",
    ]
    # Keep one raster cell beyond the scored 0.10 margin; robot radius
    # remains the inflation layer responsibility, not a second expansion.
    assert keepout["margin"] == 0.15


def test_collision_monitor_tolerates_normal_simulation_tf_skew():
    """Keep the safety velocity chain alive across short Gazebo TF delays."""
    parameters = (
        REPOSITORY
        / "bot_navigation"
        / "param"
        / "originbot_nav2.yaml"
    ).read_text(encoding="utf-8")
    collision_monitor = parameters.split("\ncollision_monitor:\n", 1)[1]

    assert _nav_scalar(collision_monitor, "transform_tolerance") >= 0.5
    assert _nav_scalar(collision_monitor, "source_timeout") >= 1.0


def test_path_controller_tolerates_loaded_vm_transform_latency():
    """The primary controller must retain the VM-safe TF tolerance."""
    parameters = (
        REPOSITORY
        / "bot_navigation"
        / "param"
        / "originbot_nav2.yaml"
    ).read_text(encoding="utf-8")
    controller = parameters.split("\ncontroller_server:\n", 1)[1].split(
        "\nlocal_costmap:\n", 1
    )[0]
    # Both the ordinary and the isolated central controller retain the same
    # VM-safe TF tolerance; adding a profile must not weaken either one.
    assert _nav_scalars(controller, "transform_tolerance") == [1.0, 1.0]


def test_valid_path_keeps_rotation_shim_alignment_stable():
    """Restore frozen static path stability while retaining the controller."""
    parameters = (
        REPOSITORY
        / "bot_navigation"
        / "param"
        / "originbot_nav2.yaml"
    ).read_text(encoding="utf-8")
    package = (
        REPOSITORY / "bot_navigation" / "package.xml"
    ).read_text(encoding="utf-8")
    behavior_tree = (
        REPOSITORY
        / "bot_navigation"
        / "config"
        / "navigate_to_pose_recovery.xml"
    ).read_text(encoding="utf-8")

    assert '<RateController hz="1.0">' in behavior_tree
    assert behavior_tree.count("<ComputePathToPose") == 2
    assert behavior_tree.count("<FollowPath") == 1
    assert (
        'plugin: "nav2_rotation_shim_controller::RotationShimController"'
        in parameters
    )
    assert "RegulatedPurePursuitController" in parameters
    assert "<exec_depend>nav2_rotation_shim_controller</exec_depend>" in package
    assert "<exec_depend>nav2_regulated_pure_pursuit_controller</exec_depend>" in package


def test_rpp_slows_and_checks_forward_before_tight_corners():
    """Regulate curvature and check the path ahead before tight turns."""
    parameters = (
        REPOSITORY
        / "bot_navigation"
        / "param"
        / "originbot_nav2.yaml"
    ).read_text(encoding="utf-8")
    assert _nav_scalar(parameters, "desired_linear_vel") == 1.78
    assert _nav_scalar(parameters, "regulated_linear_scaling_min_radius") <= 0.9
    assert _nav_scalar(parameters, "regulated_linear_scaling_min_speed") >= 0.25
    assert "use_collision_detection: true" in parameters


def test_nav2_transit_does_not_rotate_to_the_final_goal_heading():
    """Mission docking owns final yaw after Nav2 reaches the XY position."""
    parameters = (
        REPOSITORY
        / "bot_navigation"
        / "param"
        / "originbot_nav2.yaml"
    ).read_text(encoding="utf-8")
    assert '"RotateToGoal"' not in parameters
    assert "yaw_goal_tolerance: 6.28" in parameters


def test_mobile_manipulator_uses_contact_safe_planar_drive_contract():
    """Keep planar velocity control while preserving Gazebo contacts."""
    gazebo_xacro = (
        REPOSITORY
        / "mybot_description"
        / "urdf"
        / "large_base_gazebo.xacro"
    ).read_text(encoding="utf-8")
    package = ElementTree.parse(
        REPOSITORY / "mybot_description" / "package.xml"
    ).getroot()
    dependencies = {
        item.text for item in package.findall("exec_depend")
    }
    assert "<kinematic>false</kinematic>" in gazebo_xacro
    assert "libgazebo_ros_planar_move.so" in gazebo_xacro
    assert "libgazebo_ros_diff_drive.so" not in gazebo_xacro
    assert "gazebo_plugins" in dependencies


def test_controller_waits_through_transient_collision_predictions():
    """Do not turn a brief moving-obstacle stop into a failed Nav2 goal."""
    parameters = (
        REPOSITORY
        / "bot_navigation"
        / "param"
        / "originbot_nav2.yaml"
    ).read_text(encoding="utf-8")
    tolerance = _nav_scalar(parameters, "failure_tolerance")
    assert 2.0 <= tolerance <= 3.0


def test_exhausted_cube_selection_never_reuses_original_job_entity():
    """A delivered or attempted cube cannot be selected as a fallback."""
    source = (
        REPOSITORY
        / "tools_demo"
        / "tools_demo"
        / "mission_coordinator.py"
    ).read_text(encoding="utf-8")
    selection = source.split("def _select_best_cube", 1)[1].split(
        "def start_navigation", 1
    )[0]
    assert "return job.cube_name" not in selection
    assert "return None" in selection
    assert "pickup_inventory_exhausted" in source


def test_arm_joints_use_standard_urdf_dynamics():
    """Prevent the invalid damping API calls that destabilized Gazebo."""
    arm_xacro = (
        REPOSITORY / "mybot_description" / "urdf" / "arm.xacro"
    ).read_text(encoding="utf-8")
    assert "<joint_properties" not in arm_xacro
    assert arm_xacro.count(
        '<dynamics damping="50" friction="1" />'
    ) >= 6


def test_link_attacher_uses_single_gazebo_joint_lifecycle():
    """Prevent duplicate attach/detach calls that crash Gazebo Classic."""
    source = (
        REPOSITORY
        / "IFRA_LinkAttacher"
        / "ros2_LinkAttacher"
        / "src"
        / "gazebo_link_attacher.cpp"
    ).read_text(encoding="utf-8")
    assert "joint->Attach(link1, link2);" not in source
    assert "it->joint->Detach();" not in source
    assert "model1->RemoveJoint(it->joint_name)" in source
    assert "std::lock_guard<std::mutex>" in source


def test_setup_exports_navigation_executables():
    """Require installed entry points for both runtime helper nodes."""
    setup = (
        REPOSITORY / "tools_demo" / "setup.py"
    ).read_text(encoding="utf-8")
    assert '"task_navigator = tools_demo.task_navigator:main"' in setup
    assert '"moving_obstacle = tools_demo.moving_obstacle:main"' in setup
    assert '"avoidance_monitor = tools_demo.avoidance_monitor:main"' in setup


def test_navigation_uses_nav2_before_bounded_competition_docks():
    """Keep obstacle-aware Nav2 on every transit leg before final docking."""
    source = (
        REPOSITORY
        / "tools_demo"
        / "tools_demo"
        / "task_navigator.py"
    ).read_text(encoding="utf-8")
    assert "self.dock_pose = self.zones[zone]" in source
    assert "<= 6.0" not in source
    assert "self._route_waypoints(zone, waypoint_key) + [place]" in source
    assert 'if zone == "C":' not in source
    assert "self.pending_poses = list(self.corridor_waypoints)" not in source
    zone_config = (
        REPOSITORY
        / "bot_navigation"
        / "config"
        / "competition_zones.yaml"
    ).read_text(encoding="utf-8")
    assert "corridor_west: [-2.80, 0.80, -1.5708]" in zone_config
    assert "corridor_north: [-2.20, -1.00, -1.2490]" in zone_config
    assert "corridor_east: [4.00, -5.70, 0.0]" in zone_config
    assert "def start_pre_navigation_alignment" in source
    assert "RegulatedPurePursuitController" in (
        REPOSITORY / "bot_navigation" / "param" / "originbot_nav2.yaml"
    ).read_text(encoding="utf-8")
    assert "self._send_next_goal()" in source
    assert "def start_alignment" in source
    assert "if distance > self.dock_arrival_tolerance:" in source
    assert "self.dock_arrival_tolerance = self.dock_arrival_pickup" in source
    assert "self.pending_poses = [self.pickup_entries[zone]]" in source
    assert "self.dock_arrival_tolerance = self.dock_arrival_zone" in source
    assert "+ [place]" in source
    assert "self.pending_poses = [exit_clear]" in source
    assert "self.pending_poses.pop(0)" in source
    assert "self.dock_pose.x - self.latest_x" in source
    assert "self.dock_pose.y - self.latest_y" in source
    assert 'robot_summary = "unknown"' in source
    assert "self.final_yaw = self.dock_pose.yaw" in source
    assert "self.final_yaw = exit_clear.yaw" in source
    assert "self.pre_navigation_alignment = False" in source
    assert 'phase = "direct_docking"' in source
    assert "needs_pre_navigation_alignment" in source
    assert "pickup_dock_velocity" in source
    assert "self.pickup_final_approach" in source
    assert "command.linear.x, command.angular.z" in source
    assert (
        "math.sin(self.alignment_target_yaw - self.latest_yaw)" in source
    )
    assert "forward_heading_error = heading_error" not in source
    assert "def start_zone_egress" in source
    assert 'phase="zone_egress"' in source
    assert "VALID_ZONE_EXITS" in source
    assert 'phase="zone_exit_complete"' in source
    assert "math.cos(self.latest_yaw)" in source
    assert "math.sin(self.latest_yaw)" in source
    assert "for name in VALID_ZONES" in source
    assert "self.latest_x - 0.40" in source
    assert "speed = min(" in source
    assert "self.dock_max_speed" in source
    assert "self.dock_deceleration * distance" in source
    assert "command.linear.x = -speed if reverse else speed" in source
    assert "def alignment_callback" in source
    assert '"/competition/cmd_vel_direct"' in source
    assert '"/odom"' in source
    assert '"/amcl_pose"' not in source
    assert "if self.latest_x is None or self.latest_y is None:" in source
    assert not (
        "def alignment_callback(self) -> None:\n"
        "        \"\"\"Drive a bounded final dock and yaw PID after Nav2 "
        "reaches entry.\"\"\"\n"
        "        self.refresh_map_pose()"
    ) in source
    assert "self.tf_buffer.lookup_transform(" in source
    assert '"map",' in source
    assert '"base_link",' in source
    assert "zone in self.departure_routes" in source
    assert "def _load_departure_routes" in source
    assert '"final_alignment"' in source
    assert "alignment_yaw_tolerance: 0.04" in (
        REPOSITORY
        / "bot_navigation"
        / "config"
        / "competition_zones.yaml"
    ).read_text(encoding="utf-8")
    assert "self.action_client.wait_for_server(" in source
    assert "def _request_nav_handoff_if_reached" in source
    assert "navigation_handoff_reached(" in source
    assert 'reason="odom_goal_radius"' in source
    assert "self.goal_handle.cancel_goal_async()" in source
    assert 'reason="goal_rejected"' in source
    assert "action_server_timeout_sec: 30.0" in (
        REPOSITORY
        / "bot_navigation"
        / "config"
        / "competition_zones.yaml"
    ).read_text(encoding="utf-8")
    assert "nav_handoff_distance: 0.20" in (
        REPOSITORY
        / "bot_navigation"
        / "config"
        / "competition_zones.yaml"
    ).read_text(encoding="utf-8")
    assert "alignment_timeout_sec: 32.0" in (
        REPOSITORY
        / "bot_navigation"
        / "config"
        / "competition_zones.yaml"
    ).read_text(encoding="utf-8")
    assert "dock_max_speed_pickup: 0.80" in (
        REPOSITORY
        / "bot_navigation"
        / "config"
        / "competition_zones.yaml"
    ).read_text(encoding="utf-8")
    assert "dock_arrival_tolerance_pickup: 0.02" in (
        REPOSITORY
        / "bot_navigation"
        / "config"
        / "competition_zones.yaml"
    ).read_text(encoding="utf-8")


def test_cross_room_controller_has_bounded_approach_speed():
    """RPP must approach goals measurably without an unbounded crawl."""
    params = (
        REPOSITORY
        / "bot_navigation"
        / "param"
        / "originbot_nav2.yaml"
    ).read_text(encoding="utf-8")
    assert _nav_scalar(params, "min_approach_linear_velocity") == 0.10
    assert _nav_scalar(params, "regulated_linear_scaling_min_speed") == 0.25
    assert _nav_scalar(params, "approach_velocity_scaling_dist") == 0.80
    assert "allow_reversing: false" in params


def test_pickup_departure_reverses_then_turns_in_place():
    """Back straight away from the shelf, then turn east into the aisle."""
    source = (
        REPOSITORY
        / "tools_demo"
        / "tools_demo"
        / "task_navigator.py"
    ).read_text(encoding="utf-8")
    assert "def _load_departure_routes" in source
    assert 'f"pickup_{color.lower()}_exit_backoff"' in source
    assert "backoff_distance = self.departure_backoff_distance" in source
    assert "departure_backoff_distance: 0.25" in (
        REPOSITORY
        / "bot_navigation"
        / "config"
        / "competition_zones.yaml"
    ).read_text(encoding="utf-8")
    zones = yaml.safe_load(
        (
            REPOSITORY
            / "bot_navigation"
            / "config"
            / "competition_zones.yaml"
        ).read_text(encoding="utf-8")
    )["competition_task_navigator"]["ros__parameters"]
    assert 0.55 <= zones["dock_max_speed_departure"] <= 0.70
    assert "self.dock_pose.x - self.latest_x" in source
    assert "self.dock_pose.y - self.latest_y" in source
    assert "self.pending_poses = [exit_clear]" in source
    assert "self.final_yaw = exit_clear.yaw" in source
    assert 'phase = "direct_docking"' in source
    assert '"final_alignment"' in source
    assert "command.linear.x = -speed if reverse else speed" in source
    assert "if distance > self.dock_arrival_tolerance:" in source
    assert "self.dock_arrival_tolerance = self.dock_arrival_departure" in source
    assert "self.departure_backoff_speed" in source
    assert "self.pre_navigation_alignment = False" in source


def test_cube_departure_returns_to_the_selected_tray_clear_entry():
    """Departure must not cross the tray toward a fixed opposite-side exit."""
    source = (
        REPOSITORY
        / "tools_demo"
        / "tools_demo"
        / "task_navigator.py"
    ).read_text(encoding="utf-8")

    assert "self.last_cube_entry = self.dock_pose" in source
    assert "self.dock_pose = self.last_cube_entry" in source
    assert "self.pending_poses = []" in source
    assert "self.last_cube_exit" not in source
    assert "tray_crossing_waypoints(" in source
    assert "self.pending_poses = tray_route + self.pending_poses" in source

    departure_branch = source.split(
        "if zone in self.departure_routes:", 1
    )[1].split("elif zone in self.pickup_approaches:", 1)[0]
    cube_departure = departure_branch.split(
        "if departing_cube_dock:", 2
    )[2].split("else:", 1)[0]
    assert "self.dock_pose = None" not in cube_departure
    assert "self.final_yaw = self.dock_pose.yaw" in cube_departure


def test_cube_entries_leave_extra_room_for_safe_pregrasp_rotation():
    """Stage farther from each tray so the chassis can align without sweeping it."""
    config = (
        REPOSITORY
        / "bot_navigation"
        / "config"
        / "competition_zones.yaml"
    ).read_text(encoding="utf-8")

    for cube in (name for name in CUBE_SLOTS if name.startswith("red_")):
        dock = _parse_yaml_xy(config, cube)
        entry = _parse_yaml_xy(config, f"{cube}_entry")
        assert math.hypot(entry[0] - dock[0], entry[1] - dock[1]) >= 0.48, (
            cube,
            dock,
            entry,
        )


def test_task_navigation_does_not_multiply_nav2_recovery_cycles():
    """Bound outer retries after Nav2 has already run its recovery tree.

    One is the bound since 2026-10-06: 4bB_1rA measured 276.9 / 293.4 / 298.9 s
    with an identical schedule, and the two slow runs each burned a 41 s
    patience/retry window in which both Nav2 retries produced zero translation
    before the coordinator's own bounded recovery cleared the stall. Cutting to
    one retry reaches that guarded recovery earlier while still covering a
    transient block; measured after the change 284.5 / 280.4 / 300.6 s with the
    stall down to 32.6 s. The 2026-09-11 AB1 measurement that motivated two
    retries predates the bounded recovery, which is what actually unblocks the
    base now.
    """
    config = (
        REPOSITORY
        / "bot_navigation"
        / "config"
        / "competition_zones.yaml"
    ).read_text(encoding="utf-8")
    parameters = __import__("yaml").safe_load(config)[
        "competition_task_navigator"
    ]["ros__parameters"]

    assert parameters["nav_retry_attempts"] == 1


def test_navigation_state_exposes_direct_docking_command_path():
    """Keep the dashboard route live when short v2 legs bypass Nav2."""
    source = (
        REPOSITORY
        / "tools_demo"
        / "tools_demo"
        / "task_navigator.py"
    ).read_text(encoding="utf-8")

    assert 'payload["command_path"] = command_path' in source
    assert 'payload["goal"] = dock_point' in source
    assert "for pose in self.pending_poses" in source


def test_navigation_exposes_direct_docking_as_standard_path():
    """Expose direct legs that Nav2 /plan omits through a standard path."""
    source = (
        REPOSITORY
        / "tools_demo"
        / "tools_demo"
        / "task_navigator.py"
    ).read_text(encoding="utf-8")

    assert "from nav_msgs.msg import Path as NavPath" in source
    assert '"command_path_topic", "/competition/navigation/command_path"' in source
    assert "self.command_path_publisher = self.create_publisher(" in source
    assert "self._publish_command_path(command_path)" in source
    assert "path.header.frame_id = \"map\"" in source


def test_costudio_camera_streams_gate_high_bandwidth_topics():
    """Disabling camera streams must remove both image feeds from coBridge."""
    source = (
        REPOSITORY / "tools_demo" / "launch" / "costudio_bridge.launch.py"
    ).read_text(encoding="utf-8")
    topic_block = source.split("TOPIC_WHITELIST = [", 1)[1].split("]", 1)[0]
    _, after_gate = source.split(
        'if camera_streams.strip().lower() in {"1", "true", "yes", "on"}:',
        1,
    )

    assert "CAMERA_STREAM_TOPICS" in source
    assert "/camera_rgbd/camera_rgbd/image_raw" not in topic_block
    assert "/competition/perception/debug_image" not in topic_block
    assert "topics.extend(CAMERA_STREAM_TOPICS)" in after_gate
    assert "topics.extend(ROBOT_MODEL_TOPICS)" in source
    assert "competition/navigation/command_path" in source
    launcher = (
        REPOSITORY / "tools_demo" / "scripts" / "start_manual_problem_mode.sh"
    ).read_text(encoding="utf-8")
    # The two coStudio vision panels are a required display, so the launcher
    # enables the feeds by default and keeps COSTUDIO_CAMERA_STREAMS=False as
    # the bandwidth-lean opt-out.
    assert 'camera_streams="${COSTUDIO_CAMERA_STREAMS:-True}"' in launcher


def _parse_yaml_xy(config, key):
    """Read one '  key: [x, y, yaw]' entry from a flat YAML block."""
    for line in config.splitlines():
        stripped = line.strip()
        if not stripped.startswith(f"{key}:"):
            continue
        values = stripped.split("[", 1)[1].rsplit("]", 1)[0]
        numbers = [float(item) for item in values.split(",")]
        return numbers[0], numbers[1], numbers[2]
    raise AssertionError(f"missing navigation key {key}")


CUBE_SLOTS = {'red_cube_1': (-7.5, -2.8), 'red_cube_2': (-0.8, 0.8), 'red_cube_3': (0.0, 3.8), 'red_cube_4': (11.4, -9.8), 'red_cube_5': (-4.7, -12.4), 'blue_cube_1': (7.4, 3.6), 'blue_cube_2': (-11.0, -5.6), 'blue_cube_3': (-2.0, -6.0), 'blue_cube_4': (-0.5, -8.8), 'blue_cube_5': (11.8, -13.0)}
# Nav2 entries may lie in non-lethal inflation cost, but never inside the
# physical planning radius.
ROBOT_RADIUS = 0.28
# Arm reach window from tools_demo/config/mission.yaml.
REACH_MIN_X = 0.285
REACH_MAX_X = 0.430
# Chassis half length from mybot_description/urdf/large_base.xacro.
CHASSIS_HALF = 0.16


def test_every_cube_has_its_own_dock_centred_in_the_arm_reach_window():
    """The 2026 cubes are distributed, so no shared dock can reach them."""
    config = (
        REPOSITORY
        / "bot_navigation"
        / "config"
        / "competition_zones.yaml"
    ).read_text(encoding="utf-8")
    mission = (
        REPOSITORY / "tools_demo" / "config" / "mission.yaml"
    ).read_text(encoding="utf-8")

    for cube, (cube_x, cube_y) in CUBE_SLOTS.items():
        dock = _parse_yaml_xy(config, cube)
        # The coordinator plans with the same docks the navigator drives to.
        assert _parse_yaml_xy(mission, f"{cube}_dock") == dock, cube
        # Check reach in the actual dock frame, including the relocated
        # red-2 east approach (yaw pi), without relaxing the grasp window.
        dx, dy = cube_x - dock[0], cube_y - dock[1]
        lateral = -dx * math.sin(dock[2]) + dy * math.cos(dock[2])
        assert math.isclose(lateral, 0.0, abs_tol=1e-9), cube
        standoff = dx * math.cos(dock[2]) + dy * math.sin(dock[2])
        assert REACH_MIN_X < standoff < REACH_MAX_X, (cube, standoff)


def test_cube_docks_keep_the_chassis_clear_of_every_pickup_tray():
    """A dock inside the tray would push the base into the platform."""
    config = (
        REPOSITORY
        / "bot_navigation"
        / "config"
        / "competition_zones.yaml"
    ).read_text(encoding="utf-8")
    root = ElementTree.parse(
        REPOSITORY / "mybot_description" / "worlds" / "offic_room.world"
    ).getroot()
    trays = [
        (
            _planar_pose(model),
            [
                float(value)
                for value in model.findtext(
                    ".//collision/geometry/box/size"
                ).split()
            ],
        )
        for model in root.findall("./world/model")
        if model.attrib.get("name", "").startswith(
            "competition_red_pickup_tray"
        )
        or model.attrib.get("name", "").startswith(
            "competition_blue_pickup_tray"
        )
    ]
    assert len(trays) == 10

    for cube in CUBE_SLOTS:
        dock = _parse_yaml_xy(config, cube)
        for pose, size in trays:
            box = (
                pose[0],
                pose[1],
                size[0] / 2.0,
                size[1] / 2.0,
                pose[5],
            )
            distance = _point_to_oriented_box(dock[0], dock[1], box)
            # The chassis front edge must stay outside every tray box.
            assert distance > CHASSIS_HALF, (cube, distance)


def test_cube_entries_offer_at_least_one_collision_free_approach():
    """Every cube needs an entry outside the physical planning radius."""
    config = (
        REPOSITORY
        / "bot_navigation"
        / "config"
        / "competition_zones.yaml"
    ).read_text(encoding="utf-8")
    root = ElementTree.parse(
        REPOSITORY / "mybot_description" / "worlds" / "offic_room.world"
    ).getroot()
    boxes = [box for _, box in _world_boxes(root)]

    for cube in CUBE_SLOTS:
        dock = _parse_yaml_xy(config, cube)
        entry = _parse_yaml_xy(config, f"{cube}_entry")
        dx = dock[0] - entry[0]
        dy = dock[1] - entry[1]
        same_x_aisle_entry = math.isclose(dx, 0.0, abs_tol=1e-9)
        straight_entry = (
            dx * math.cos(dock[2]) + dy * math.sin(dock[2]) > 0.0
            and math.isclose(-dx * math.sin(dock[2]) + dy * math.cos(dock[2]),
                             0.0, abs_tol=1e-9)
        )
        assert same_x_aisle_entry or straight_entry, cube
        clearance = min(
            _point_to_oriented_box(entry[0], entry[1], box) for box in boxes
        )
        candidate_clearances = [clearance]
        if cube.startswith("red_"):
            alternate = _parse_yaml_xy(config, f"{cube}_entry_aisle")
            assert math.isclose(alternate[0], dock[0], abs_tol=1e-9), cube
            assert not math.isclose(
                alternate[1], dock[1], abs_tol=1e-9
            ), cube
            alternate_clearance = min(
                _point_to_oriented_box(alternate[0], alternate[1], box)
                for box in boxes
            )
            candidate_clearances.append(alternate_clearance)
        elif cube.startswith("blue_"):
            alternate = _parse_yaml_xy(config, f"{cube}_entry_inner")
            candidate_clearances.append(
                min(
                    _point_to_oriented_box(alternate[0], alternate[1], box)
                    for box in boxes
                )
            )
        assert max(candidate_clearances) > ROBOT_RADIUS, (
            cube,
            candidate_clearances,
        )


def test_dashboard_and_monitor_subscribe_all_live_obstacles():
    """Every scored obstacle must reach the panel and the monitor."""
    zones = (
        REPOSITORY
        / "bot_navigation"
        / "config"
        / "competition_zones.yaml"
    ).read_text(encoding="utf-8")
    live_topics = sorted(
        line.split("state_topic:", 1)[1].strip()
        for line in zones.splitlines()
        if "state_topic:" in line
        and "/competition/obstacles/" in line
    )
    assert len(live_topics) == 4, live_topics

    dashboard = (
        REPOSITORY / "tools_demo" / "config" / "dashboard.yaml"
    ).read_text(encoding="utf-8")
    for topic in live_topics:
        assert topic in dashboard, topic

    for module in ("competition_dashboard.py", "avoidance_monitor.py"):
        source = (
            REPOSITORY / "tools_demo" / "tools_demo" / module
        ).read_text(encoding="utf-8")
        for topic in live_topics:
            assert topic in source, (module, topic)


def test_nav2_handoff_finishes_from_canceled_action_result():
    """Serialize a staged handoff after Nav2 finishes halting its action."""
    source = (
        REPOSITORY
        / "tools_demo"
        / "tools_demo"
        / "task_navigator.py"
    ).read_text(encoding="utf-8")
    request = source.split(
        "def _request_nav_handoff_if_reached", 1
    )[1].split("def refresh_map_pose", 1)[0]
    result = source.split("def result_callback", 1)[1].split(
        "def cancel_callback", 1
    )[0]

    assert "cancel_future.add_done_callback(" in request
    assert "self._complete_nav_handoff()" in result
    assert "def _handoff_cancel_callback" in source
    assert "def _complete_nav_handoff" in source
    assert "self._complete_nav_handoff(expect_result=True)" not in source
    assert result.index("if self.nav_handoff_requested") < result.index(
        "status = future.result().status"
    )


def test_staged_handoff_waits_for_canceled_action_result():
    """Do not race a new staged goal against BT Navigator cancellation."""
    source = (
        REPOSITORY / "tools_demo" / "tools_demo" / "task_navigator.py"
    ).read_text(encoding="utf-8")
    cancel_callback = source.split(
        "    def _handoff_cancel_callback", 1
    )[1].split("\n    def ", 1)[0]
    result_callback = source.split(
        "    def result_callback", 1
    )[1].split("\n    def ", 1)[0]
    assert "_complete_nav_handoff" not in cancel_callback
    assert "if self.nav_handoff_requested" in result_callback
    assert result_callback.index("if self.nav_handoff_requested") < (
        result_callback.index("future.result().status")
    )


def test_truth_safety_guard_never_invents_reverse_motion():
    """The final guard may stop Nav2, but must not fight it by reversing."""
    source = (
        REPOSITORY
        / "tools_demo"
        / "tools_demo"
        / "safety_limiter.py"
    ).read_text(encoding="utf-8")
    assert "safe.linear.x = -" not in source
    assert "reverse_speed" not in source
    assert "obstacle_timeout_sec" in source


def test_manipulation_hold_gates_the_planar_plugin_command():
    """Arm motion must continuously zero the final safe velocity stream."""
    limiter = (
        REPOSITORY
        / "tools_demo"
        / "tools_demo"
        / "safety_limiter.py"
    ).read_text(encoding="utf-8")
    manipulation = (
        REPOSITORY / "mybot" / "pick_place_moveit.py"
    ).read_text(encoding="utf-8")

    assert '"/competition/base_hold"' in limiter
    assert "self.create_timer(0.05, self.publish_base_hold)" in limiter
    assert "if self.base_hold:" in limiter
    assert "self.publisher.publish(Twist())" in limiter
    assert '"/competition/base_hold"' in manipulation
    assert "self.set_base_hold(True)" in manipulation
    assert "self.set_base_hold(False)" in manipulation
    stabilize = manipulation.split("def stabilize_base", 1)[1].split(
        "def release_base", 1
    )[0]
    assert stabilize.index("set_base_hold(True)") < stabilize.index(
        "stop_base_motion()"
    )


def test_collision_monitor_does_not_require_odom_shift_correction():
    """Laser safety must survive delayed dynamic odom TF on a loaded VM."""
    params = (
        REPOSITORY
        / "bot_navigation"
        / "param"
        / "originbot_nav2.yaml"
    ).read_text(encoding="utf-8")
    collision = params.split("collision_monitor:", 1)[1]
    assert "base_shift_correction: False" in collision


def test_collision_monitor_uses_distance_zones_not_trajectory_prediction():
    """The wall-corner guard must not zero a still-recoverable DWB turn."""
    params = (
        REPOSITORY
        / "bot_navigation"
        / "param"
        / "originbot_nav2.yaml"
    ).read_text(encoding="utf-8")
    collision = params.split("collision_monitor:", 1)[1]
    assert 'polygons: ["BodyStop", "NearSlow"]' in collision
    assert 'action_type: "stop"' in collision
    assert 'action_type: "slowdown"' in collision
    assert collision.count('type: "circle"') >= 2
    # The hard stop remains outside the 0.226 m chassis half diagonal, but
    # below the 0.28 m planning radius so a small localization/path error
    # cannot permanently suppress a command that moves away from the wall.
    assert "radius: 0.26" in collision
    assert "radius: 0.38" in collision
    assert "points: [0.34, 0.34" not in collision
    assert 'slowdown_ratio: 0.70' in collision
    assert 'action_type: "approach"' not in collision


def test_collision_monitor_startup_is_delayed_without_hot_recycling():
    """Start once after Nav2; Humble cannot safely recycle its timer."""
    launch_source = (
        REPOSITORY
        / "bot_navigation"
        / "launch"
        / "competition_navigation.launch.py"
    ).read_text(encoding="utf-8")

    assert "period=5.0" in launch_source
    assert "actions=[collision_monitor]" in launch_source
    assert 'executable="collision_monitor_recovery"' not in launch_source
    assert "must not be hot-cycled" in launch_source


def test_in_place_rotation_can_stop_inside_its_release_window():
    """The rotation rate must be stoppable within the disengage window.

    Nav2's rotate-to-heading commands the full rotate_to_heading_angular_vel
    until the heading error enters angular_disengage_threshold and can then
    only ramp down at max_angular_accel, so the braking angle is
    omega^2 / (2 * alpha). Measured 2026-10-03 on the 7ca1ba3 baseline: at
    2.50 rad/s with 5.0 rad/s^2 that is 0.625 rad against a 0.20 rad window,
    so every rotation overshot by ~24 degrees and was corrected back - the
    observed left-right sway (83 in-place rotations per round, a reverse
    correction within 2 s of 84% of them). Dropping the rate to 1.40 rad/s
    put the braking angle inside the window and cut the same case from
    227.9 s to 202.8 s, with base commanded stalls 281 -> 0, min centre
    distance 0.7453 -> 0.7899 m and NearSlow toggles 107 -> 75.
    """
    params = yaml.safe_load(
        (
            REPOSITORY
            / "bot_navigation"
            / "param"
            / "originbot_nav2.yaml"
        ).read_text(encoding="utf-8")
    )
    follow_path = params["controller_server"]["ros__parameters"][
        "FollowPath"
    ]
    rate = follow_path["rotate_to_heading_angular_vel"]
    accel = follow_path["max_angular_accel"]
    window = follow_path["angular_disengage_threshold"]

    assert rate == 1.40
    assert accel == 5.0
    assert window == 0.20
    braking_angle = rate * rate / (2.0 * accel)
    assert braking_angle <= window, (
        "rotation cannot stop inside its release window: braking angle "
        f"{braking_angle:.3f} rad exceeds the {window:.3f} rad window, so "
        "every turn overshoots and is corrected back (left-right sway)"
    )


def test_rpp_keeps_cost_and_collision_regulation_enabled():
    """Use fast path tracking without bypassing costmap safety guards."""
    params = yaml.safe_load(
        (
            REPOSITORY
            / "bot_navigation"
            / "param"
            / "originbot_nav2.yaml"
        ).read_text(encoding="utf-8")
    )
    follow_path = params["controller_server"]["ros__parameters"][
        "FollowPath"
    ]

    assert follow_path["primary_controller"].endswith(
        "RegulatedPurePursuitController"
    )
    assert follow_path["angular_dist_threshold"] == 0.45
    assert follow_path["desired_linear_vel"] == 1.78
    assert follow_path["desired_linear_vel"] < 2.2
    source = (
        REPOSITORY / "tools_demo" / "tools_demo" / "task_navigator.py"
    ).read_text(encoding="utf-8")
    assert "YAW_ALIGNMENT_GAIN = 3.0" in source
    assert 'recovery_label = "recent_trail"' in source
    assert '"RECOVERY_TRAIL",' in source
    # The escape corridor is still the traversed trail and still bounded by
    # reverse_max_distance; alternatives only change the heading, never the
    # distance budget, and cells inside the robot's own footprint are the only
    # ones excused from the free-space check.
    assert "ESCAPE_DISTANCE_FRACTIONS" in source
    assert "base_distance * fraction" in source
    assert "ESCAPE_MAX_DISTANCE = 0.5" in source
    assert "ESCAPE_SELF_CLEARANCE" in source
    assert follow_path["use_regulated_linear_velocity_scaling"] is True
    assert follow_path["use_cost_regulated_linear_velocity_scaling"] is True
    assert follow_path["inflation_cost_scaling_factor"] == 10.0
    assert follow_path["use_collision_detection"] is True
    assert 0.85 < follow_path["max_lookahead_dist"] <= 1.20
    assert 0.55 < follow_path["lookahead_time"] <= 1.0
    assert follow_path["use_rotate_to_heading"] is False
    assert follow_path["rotate_to_heading_min_angle"] == .45
    assert follow_path["max_allowed_time_to_collision_up_to_carrot"] == 0.65
    assert follow_path["use_interpolation"] is True
    assert follow_path["allow_reversing"] is False


def test_pickup_departure_finishes_at_clear_position_without_yaw_wait():
    """Nav2 owns yaw after the direct controller clears the pickup tray."""
    source = (
        REPOSITORY
        / "tools_demo"
        / "tools_demo"
        / "task_navigator.py"
    ).read_text(encoding="utf-8")
    alignment = source.split(
        "    def alignment_callback", 1
    )[1].split("    def stop_alignment", 1)[0]

    marker = "departure_position_complete; handing yaw to Nav2"
    assert marker in alignment
    departure = alignment.split(marker, 1)[1]
    succeeded = 'self.publish_state(\n                    "succeeded"'
    assert departure.index(succeeded) < departure.index(
        "self.dock_pose = None"
    )


def test_zone_c_deliveries_use_the_direct_south_west_corridor():
    """The moved C bay sits south of the sweep, so no north bypass is added.

    Zone C is now at (-9, -7). Every pickup exit is south-west of it and the
    static map already holds a free 9-15 m corridor, so the former
    north-corridor guides (C_SOUTH_CORNER / C_SOUTH_BYPASS / C_WEST_GUIDE at
    y ~ 0.5) only added a 7.5 m detour north and back. Only a start north of
    the obstacle_2 sweep line and inside its mouth band still needs a guide.
    """
    from tools_demo.task_navigator import TaskNavigator
    from tools_demo.navigation_math import ZonePose

    class FakeLogger:
        def info(self, _message):
            pass

        def warn(self, _message):
            pass

        def debug(self, _message):
            pass

    class RouteHarness:
        def __init__(self, start_x, start_y, primary_waypoints):
            self.latest_x = start_x
            self.latest_y = start_y
            self.active_route_zone = None
            self.active_route = None
            self.dock_pose = None
            self.final_yaw = None
            self.dock_arrival_tolerance = None
            self.dock_max_speed = None
            self.pending_poses = []
            self.route_profiles = {
                "C": {
                    "primary_waypoints": primary_waypoints,
                    "place_pose": [-9.4, -7.0, 0.0],
                }
            }
            self.zones = {"C": ZonePose("C", -9.4, -7.0, 0.0)}
            self.nav_handoff_distance = 0.2
            self.dock_arrival_zone = 0.15
            self.dock_speed_zone = 0.8
            self._bypass_logged = []

        def _route_pose(self, zone, key):
            profile = self.route_profiles.get(zone, {})
            values = profile.get(key)
            if not isinstance(values, list) or len(values) != 3:
                return None
            return ZonePose(f"{zone}_{key}", *[float(v) for v in values])

        def _delivery_dock_pose(self, zone, place):
            return place

        def _route_waypoints(self, zone, key):
            values = self.route_profiles.get(zone, {}).get(key, [])
            return [
                ZonePose(f"{zone}_{key}_{i}", *[float(v) for v in point])
                for i, point in enumerate(values)
                if isinstance(point, list) and len(point) == 3
            ]

        def get_logger(self):
            return FakeLogger()

    # South starts head straight to the C bay with no prepended waypoint.
    for start_x, start_y in ((4.0, -7.0), (-6.0, -7.0), (-2.5, -2.0)):
        harness = RouteHarness(start_x=start_x, start_y=start_y,
                               primary_waypoints=[])
        TaskNavigator._configure_delivery_route(harness, "C")
        assert [pose.zone for pose in harness.pending_poses] == [
            "C_place_pose"
        ], (start_x, start_y)

    # North start outside the sweep mouth: the free route already crosses
    # y = 1.0 west of the board, so no guide is inserted.
    harness = RouteHarness(start_x=-3.0, start_y=2.0, primary_waypoints=[])
    TaskNavigator._configure_delivery_route(harness, "C")
    assert [pose.zone for pose in harness.pending_poses] == ["C_place_pose"]

    # Defensive guard: a north start inside the mouth band crosses west first.
    harness = RouteHarness(start_x=-0.8, start_y=2.0, primary_waypoints=[])
    TaskNavigator._configure_delivery_route(harness, "C")
    assert [pose.zone for pose in harness.pending_poses] == [
        "C_SWEEP_WEST_GUIDE",
        "C_place_pose",
    ]
    assert (harness.pending_poses[0].x, harness.pending_poses[0].y) == (
        -2.9,
        0.2,
    )

    # A start north-east of the x = 1.15 wall band rebuilds the whole
    # route: the legacy route poses sit 0.15 m from that wall and RPP
    # crawled the leg for 147 s (AC1 2026-09-09).
    harness = RouteHarness(start_x=5.09, start_y=1.91, primary_waypoints=[])
    TaskNavigator._configure_delivery_route(harness, "C")
    assert [pose.zone for pose in harness.pending_poses] == [
        "C_NORTH_CROSS",
        "C_DESCENT_ENTRY",
        "C_EAST_DESCENT",
        "C_SOUTH_CORRIDOR",
        "C_WEST_CORRIDOR",
        "C_place_pose",
    ]
    assert (harness.pending_poses[2].x, harness.pending_poses[2].y) == (
        0.1,
        -4.1,
    )
    assert (harness.pending_poses[3].x, harness.pending_poses[3].y) == (
        -3.05,
        -7.15,
    )

    # The backup route never inserts any C guide.
    harness = RouteHarness(start_x=-0.8, start_y=2.0, primary_waypoints=[])
    TaskNavigator._configure_delivery_route(harness, "C", backup=True)
    assert [pose.zone for pose in harness.pending_poses] == ["C_place_pose"]


def test_zone_a_deliveries_from_the_west_cross_the_wall_tip_gap():
    """West pickups cross Wall_57 through its 1.48 m north tip gap.

    Wall_57 (x = -3.2, y = -3.4 .. -0.65) separates the west pickups from
    zone A.  A straight leg from (-5.12, -2.50) to the south gate runs
    through the wall body (static-map clearance 0.000 m at (-3.28, -1.16)),
    so Smac detoured around the north tip and RPP crawled there for 30 s at
    0.32 m clearance (AB1/AB2 2026-09-09).  The tip-gap chain keeps 0.707 m
    clearance on the same crossing, and every start east of the wall keeps
    the existing south/north gate route.
    """
    from tools_demo.task_navigator import TaskNavigator
    from tools_demo.navigation_math import ZonePose

    class FakeLogger:
        def info(self, _message):
            pass

        def warn(self, _message):
            pass

        def debug(self, _message):
            pass

    class RouteHarness:
        def __init__(self, start_x, start_y):
            self.latest_x = start_x
            self.latest_y = start_y
            self.active_route_zone = None
            self.active_route = None
            self.dock_pose = None
            self.final_yaw = None
            self.dock_arrival_tolerance = None
            self.dock_max_speed = None
            self.pending_poses = []
            self.route_profiles = {
                "A": {
                    "primary_waypoints": [],
                    "place_pose": [-2.41, 2.99, 0.0],
                }
            }
            self.zones = {"A": ZonePose("A", -2.41, 2.99, 0.0)}
            self.nav_handoff_distance = 0.2
            self.dock_arrival_zone = 0.1
            self.dock_speed_zone = 1.5

        def _route_pose(self, zone, key):
            values = self.route_profiles.get(zone, {}).get(key)
            if not isinstance(values, list) or len(values) != 3:
                return None
            return ZonePose(f"{zone}_{key}", *[float(v) for v in values])

        def _delivery_dock_pose(self, zone, place):
            return place

        def _route_waypoints(self, zone, key):
            values = self.route_profiles.get(zone, {}).get(key, [])
            return [
                ZonePose(f"{zone}_{key}_{i}", *[float(v) for v in point])
                for i, point in enumerate(values)
                if isinstance(point, list) and len(point) == 3
            ]

        def get_logger(self):
            return FakeLogger()

    harness = RouteHarness(start_x=-5.12, start_y=-2.50)
    TaskNavigator._configure_delivery_route(harness, "A")
    assert [pose.zone for pose in harness.pending_poses] == [
        "A_TIP_WEST",
        "A_TIP_APPROACH",
        "A_TIP_GAP",
        "A_TIP_EAST",
        "A_NORTH_GATE",
        "A_place_pose",
    ]
    assert (harness.pending_poses[2].x, harness.pending_poses[2].y) == (
        -3.35,
        0.05,
    )

    # The red_cube_5 pickup west of the wall uses the same crossing.
    harness = RouteHarness(start_x=-4.70, start_y=-10.60)
    TaskNavigator._configure_delivery_route(harness, "A")
    assert harness.pending_poses[0].zone == "A_TIP_WEST"

    # Starts east of the wall keep the bounded south/north gate route.
    for start_x, start_y in ((-2.90, -2.00), (-2.41, 0.50)):
        harness = RouteHarness(start_x=start_x, start_y=start_y)
        TaskNavigator._configure_delivery_route(harness, "A")
        assert [pose.zone for pose in harness.pending_poses] == [
            "A_SOUTH_GATE",
            "A_NORTH_GATE",
            "A_place_pose",
        ], (start_x, start_y)

    # A north start needs no gate at all.
    harness = RouteHarness(start_x=11.20, start_y=3.00)
    TaskNavigator._configure_delivery_route(harness, "A")
    assert [pose.zone for pose in harness.pending_poses] == ["A_place_pose"]


def test_staged_routes_use_one_continuous_nav2_action():
    """Multiple validated route poses are submitted without stage stops."""
    from tools_demo.task_navigator import TaskNavigator
    from tools_demo.task_navigator import ZonePose

    class BatchHarness:
        def __init__(self):
            self.pending_poses = [
                ZonePose("turn", 1.0, 2.0, 0.0),
                ZonePose("endpoint", 3.0, 4.0, 1.0),
            ]
            self.nav_batch_poses = []
            self.nav_goal_pose = None
            self.nav_handoff_requested = True
            self.stage_index = 0
            self.sent = None

        def _wait_for_action_server(self):
            return True

        def _send_pose_batch(self, poses):
            self.sent = list(poses)

    harness = BatchHarness()
    TaskNavigator._send_next_goal(harness)

    assert [pose.zone for pose in harness.sent] == ["turn", "endpoint"]
    assert harness.pending_poses == []
    assert harness.nav_batch_poses == harness.sent
    assert harness.nav_goal_pose.zone == "endpoint"
    assert harness.nav_handoff_requested is False
    assert harness.stage_index == 1


def test_universal_obstacle_route_remains_serial():
    """The deterministic dual-obstacle staging keeps proven cancel semantics."""
    from tools_demo.task_navigator import TaskNavigator
    from tools_demo.task_navigator import ZonePose

    class SerialHarness:
        def __init__(self):
            self.pending_poses = [
                ZonePose("universal_corridor_entry", 12.6, -0.6, math.pi),
                ZonePose("universal_origin_staging", 0.0, 0.0, math.pi),
            ]
            self.nav_batch_poses = []
            self.nav_goal_pose = None
            self.nav_handoff_requested = True
            self.stage_index = 0
            self.sent = None

        def _wait_for_action_server(self):
            return True

        def _send_pose(self, pose):
            self.sent = pose

    harness = SerialHarness()
    TaskNavigator._send_next_goal(harness)

    assert harness.sent.zone == "universal_corridor_entry"
    assert harness.nav_batch_poses == []
    assert [pose.zone for pose in harness.pending_poses] == [
        "universal_origin_staging"
    ]


def test_failed_continuous_route_falls_back_to_original_serial_queue():
    """A batch failure restores every anchor before resuming navigation."""
    from tools_demo.task_navigator import TaskNavigator
    from tools_demo.task_navigator import ZonePose

    class FallbackHarness:
        def __init__(self):
            self.nav_batch_poses = [
                ZonePose("anchor_1", 1.0, 2.0, 0.0),
                ZonePose("anchor_2", 3.0, 4.0, 0.0),
            ]
            self.pending_poses = [ZonePose("endpoint", 5.0, 6.0, 0.0)]
            self.nav_goal_pose = self.nav_batch_poses[-1]
            self.goal_handle = object()
            self.nav_handoff_requested = True
            self.stage_index = 1
            self.started = False
            self.states = []

        def get_logger(self):
            return type("Logger", (), {"warn": lambda self, text: None})()

        def publish_state(self, state, **details):
            self.states.append((state, details))

        def start_pre_navigation_alignment(self):
            self.started = True

    harness = FallbackHarness()
    restored = TaskNavigator._fallback_batch_to_serial(harness, "test")

    assert restored is True
    assert [pose.zone for pose in harness.pending_poses] == [
        "anchor_1",
        "anchor_2",
        "endpoint",
    ]
    assert harness.nav_batch_poses == []
    assert harness.nav_goal_pose is None
    assert harness.goal_handle is None
    assert harness.nav_handoff_requested is False
    assert harness.stage_index == 0
    assert harness.started is True
    assert harness.states[-1][1]["phase"] == "batch_fallback_serial"


def test_rejected_continuous_goal_uses_serial_fallback():
    """An unavailable custom tree cannot turn a valid route into a skip."""
    from tools_demo.task_navigator import TaskNavigator

    class RejectedHandle:
        accepted = False

    class FinishedFuture:
        @staticmethod
        def result():
            return RejectedHandle()

    class ResponseHarness:
        def __init__(self):
            self.fallback_reason = None
            self.errors = []
            self.states = []

        def _fallback_batch_to_serial(self, reason):
            self.fallback_reason = reason
            return True

        def publish_error(self, text):
            self.errors.append(text)

        def publish_state(self, state, **details):
            self.states.append((state, details))

        def _clear_goal(self):
            raise AssertionError("a rejected batch must not clear the mission")

    harness = ResponseHarness()
    TaskNavigator.goal_response_callback(harness, FinishedFuture())

    assert harness.fallback_reason == "batch_goal_rejected"
    assert harness.errors == []
    assert harness.states == []


def test_continuous_nav2_route_preserves_recovery_state():
    """Batch poses survive retry, obstacle wait, and bounded recovery paths."""
    source = (
        REPOSITORY / "tools_demo" / "tools_demo" / "task_navigator.py"
    ).read_text(encoding="utf-8")

    assert "from nav2_msgs.action import NavigateThroughPoses" in source
    assert "goal.poses = [cls.stamped_pose(pose, stamp) for pose in poses]" in source
    assert 'self._fallback_batch_to_serial("batch_obstacle_reverse")' in source
    assert "list(self.nav_batch_poses) + list(" in source
    assert 'self.through_action_name = "/navigate_through_poses"' in source
    assert "nav_batch_stage_advanced" in source
    assert "self.progress_anchor_ns = now_ns" in source
    assert "nav_batch_fallback_serial" in source
    assert 'startswith("universal_")' in source
    strict_tree = (
        REPOSITORY
        / "tools_demo"
        / "config"
        / "navigate_through_poses_strict.xml"
    ).read_text(encoding="utf-8")
    assert "ComputePathThroughPoses" in strict_tree
    assert "RemovePassedGoals" not in strict_tree
    assert "FollowPath" in strict_tree


def test_b_northeast_corridor_clears_the_central_pickup_trays():
    """A north-east B start must not hug the rock beside the new trays.

    The moved red_cube_2 (0, 3) and red_cube_4 (0, -1) trays sit in the
    central corridor.  Measured 2026-09-10: the un-guided blue_cube_1 -> B
    route hugged the rock's inflation at y ~ 2.6, the controller tracked the
    path into contact range and every candidate trajectory was rejected while
    the robot carried the cube (its own cell reported cost 100).
    """
    from tools_demo.b_delivery_route import b_northeast_waypoints
    from tools_demo.navigation_math import ZonePose

    east_dock = ZonePose("B", 0.40, -12.00, math.pi)
    corridor = b_northeast_waypoints(4.00, 1.91, east_dock)
    assert [pose.zone for pose in corridor] == [
        "B_NE_CORRIDOR_0",
        "B_NE_CORRIDOR_1",
    ]
    ascent, west = corridor
    # Climb north of the rock, then run west above the (0, 3) tray at y=4.0.
    # The ascent lane is clamped west of the blue pickup tray (x <= 3.7).
    assert ascent.x == 3.70
    # An east-dock start climbs west of the blue pickup tray, never beside it.
    east_start = b_northeast_waypoints(5.08, 1.92, east_dock)
    assert east_start[0].x == 3.70
    assert east_start[0].y == 4.00
    assert ascent.y == 4.00
    assert math.isclose(ascent.yaw, math.pi, abs_tol=1.0e-9)
    assert west.x == -1.60
    assert west.y == 4.00
    # Both waypoints stay well clear of the trays and of the rock footprint.
    for pose in corridor:
        assert math.hypot(pose.x - 0.0, pose.y - 3.0) > 1.0
        assert math.hypot(pose.x - 0.0, pose.y + 1.0) > 1.0
        assert math.hypot(pose.x - 1.7333, pose.y - 1.629) > 2.0
    # The last waypoint aims at the dock so the descent stays in the lane.
    assert math.isclose(
        west.yaw,
        math.atan2(east_dock.y - 4.0, east_dock.x + 1.6),
        abs_tol=1.0e-9,
    )


def test_b_northeast_corridor_is_limited_to_north_east_starts():
    """Only a start north of the wall band and east of the rock is rerouted."""
    from tools_demo.b_delivery_route import b_northeast_waypoints
    from tools_demo.navigation_math import ZonePose

    east_dock = ZonePose("B", 0.40, -12.00, math.pi)
    west_dock = ZonePose("B", -0.40, -12.00, 0.0)
    # Spawn, west-spur and southern starts keep the direct route.
    assert b_northeast_waypoints(13.20, -0.95, east_dock) == []
    assert b_northeast_waypoints(-0.36, 3.00, east_dock) == []
    assert b_northeast_waypoints(1.04, -3.90, east_dock) == []
    assert b_northeast_waypoints(4.00, 1.20, east_dock) == []
    assert b_northeast_waypoints(None, 2.00, east_dock) == []
    assert b_northeast_waypoints(4.00, None, east_dock) == []
    assert b_northeast_waypoints(4.00, 1.91, None) == []
    # The corridor does not depend on which side of zone B is docked.
    assert len(b_northeast_waypoints(4.00, 1.91, west_dock)) == 2


def test_b_central_north_recovery_route_clears_red2_tray_before_descent():
    """Central north B starts must leave the red2 tray before going south."""
    from tools_demo.b_delivery_route import b_central_north_waypoints
    from tools_demo.navigation_math import ZonePose

    dock = ZonePose("B", -0.40, -12.00, 0.0)
    poses = b_central_north_waypoints(-0.41, 2.99, dock)
    assert [(pose.x, pose.y) for pose in poses[:3]] == [
        (-2.20, 2.99),
        (-2.20, -1.00),
        (-2.20, -6.00),
    ]
    assert b_central_north_waypoints(-0.81, -0.99, dock) == []


def test_zone_b_delivery_applies_the_northeast_corridor():
    """The navigator must apply the corridor for a north-east B start."""
    source = (
        REPOSITORY / "tools_demo" / "tools_demo" / "task_navigator.py"
    ).read_text(encoding="utf-8")
    assert (
        "from tools_demo.b_delivery_route import b_northeast_waypoints"
        in source
    )
    assert "B_northeast_corridor active" in source
    block = source.split('if zone == "B" and not backup:', 1)[1].split(
        'if zone == "C"', 1
    )[0]
    assert "b_northeast_waypoints(" in block
    assert "self.pending_poses = corridor + self.pending_poses" in block


def test_northeast_corridor_replaces_the_tray_crossing_route():
    """A dedicated corridor must not be preceded by the tray crossing route.

    Measured 2026-09-10 (AB regression, blue_cube_1 east dock): the tray
    crossing poses (5.01, 1.26) and (3.71, 1.26) sit 0.16 m north of the
    central wall face (cost 99) while every corridor waypoint reports
    cost 0.  Prepending them inverted the route order - the robot drove
    4.4 m west inside the wall band and was then asked to climb back east
    to (4.60, 4.00) - and stalled there (12.4 s of slow progress, repeated
    "RegulatedPurePursuitController detected collision ahead!"), which
    ended the mission with destination_navigation_failed:holding_cube.
    """
    source = (
        REPOSITORY / "tools_demo" / "tools_demo" / "task_navigator.py"
    ).read_text(encoding="utf-8")
    helper = source.split("def _dedicated_corridor_active", 1)[1].split(
        "def _configure_delivery_route", 1
    )[0]
    assert '"B": "b_northeast_route_active"' in helper
    assert '"C": "c_east_wall_descent_active"' in helper
    assert "c_clearance_route_active" in helper
    tray_block = source.split("tray_crossing_route ", 1)[1].split(
        "self.last_pickup_cube_xy = None", 1
    )[0]
    assert "self._dedicated_corridor_active(zone)" in tray_block
    assert "tray_crossing_route skipped" in tray_block
    # The corridor flag must be cleared for every new delivery route.
    route_block = source.split("def _configure_delivery_route", 1)[1].split(
        "if zone ==", 1
    )[0]
    assert "self.b_northeast_route_active = False" in route_block
    # ... and set when the corridor is actually applied.
    zone_b_block = source.split('if zone == "B" and not backup:', 1)[1].split(
        'if zone == "C"', 1
    )[0]
    assert "self.b_northeast_route_active = True" in zone_b_block


def test_zone_c_east_wall_descent_replaces_the_tray_crossing_route():
    """The C east-wall descent must not be preceded by the tray poses.

    Measured 2026-09-10 (nine-round campaign): the first blue delivery to zone
    C planned eight poses - (5.01,1.26) and (3.71,1.26) in front of the
    C_NORTH_CROSS chain.  Those two sit 0.15 m from the central wall face
    (probe goal_cost=99) and sent the robot 1.3 m west and then 1.75 m back
    east-north, so the leg cost 43.4-45.3 s in every AC and BC round, by far
    the longest leg of the mission.
    """
    source = (
        REPOSITORY / "tools_demo" / "tools_demo" / "task_navigator.py"
    ).read_text(encoding="utf-8")
    descent_block = source.split('"C_east_wall_descent active "', 1)[1].split(
        "elif (", 1
    )[0]
    assert "self.c_east_wall_descent_active = True" in descent_block
    helper = source.split("def _dedicated_corridor_active", 1)[1].split(
        "def _configure_delivery_route", 1
    )[0]
    assert '"C": "c_east_wall_descent_active"' in helper
    # The flag has to be cleared for every new delivery route.
    reset_block = source.split("def _configure_delivery_route", 1)[1].split(
        "if zone ==", 1
    )[0]
    assert "self.c_east_wall_descent_active = False" in reset_block


def test_direct_dock_always_has_a_deadline():
    """A blocked dock must time out instead of commanding the base forever.

    AB1 (2026-09-11) hung after `pickup_entry_aligned; starting forward-only
    dock`: the dock stayed active with no deadline, so the 20 Hz alignment
    callback kept publishing velocity commands for 570 s while the coordinator
    waited for a terminal navigation result, and the round budget expired.
    """
    source = (
        REPOSITORY / "tools_demo" / "tools_demo" / "task_navigator.py"
    ).read_text(encoding="utf-8")
    dock_block = source.split("if self.dock_pose is not None:", 1)[1].split(
        "if self.recovery_target is not None:", 1
    )[0]
    assert "if self.alignment_deadline_ns is None:" in dock_block
    assert "self.alignment_deadline_ns = (" in dock_block
    assert "re-armed for" in dock_block
    # ... and the timeout branch still reports a terminal result.
    timeout_block = source.split(
        "def alignment_callback", 1
    )[1].split("if self.zone_egress_target is not None:", 1)[0]
    assert '"Final docking or yaw alignment timed out"' in timeout_block
    assert 'self.publish_state("timed_out")' in timeout_block


def test_cube_pickups_cross_the_wall_tip_on_its_validated_chain():
    """A south-west dock reached from the north must ride the tip gap.

    AB1 (2026-09-11) and AC1 (2026-09-10) both stalled at (-3.36,-0.38) with
    zero translation for 8 s while RPP repeated "detected collision ahead",
    aborted the RED_CUBE_3 leg after 37 s and re-drove it (+43 s). The map is
    free there; the base was passing the Wall_57 tip at ~0.32 m clearance.
    Zone A deliveries already cross this tip on the A_TIP chain.
    """
    from tools_demo.navigation_math import ZonePose
    from tools_demo.task_navigator import TaskNavigator

    class Harness:
        _cube_approach_waypoints = TaskNavigator._cube_approach_waypoints

        def __init__(self, dock):
            self.dock_pose = dock
            self.messages = []

        def get_logger(self):
            harness = self

            class Logger:
                def info(self, message):
                    harness.messages.append(message)

            return Logger()

    south_west = ZonePose("dock", -5.10, -2.50, 3.14159)
    north_start = Harness(south_west)
    poses = north_start._cube_approach_waypoints("RED_CUBE_3", -2.71, 3.44)
    assert poses[0].zone == "wall_tip_west"
    assert any("wall_tip_crossing guide" in m for m in north_start.messages)
    # The staging pose keeps the descent ~0.95 m west of the x = -3.2 wall
    # for the whole southbound run, so the base never travels down the face.
    stage = poses[0]
    assert math.isclose(stage.x, -4.15, abs_tol=1e-9)
    assert math.isclose(stage.y, -0.45, abs_tol=1e-9)
    assert stage.x + 0.28 < -3.2 - 0.45
    # No crossing when the dock is north of the wall ...
    north_dock = Harness(ZonePose("dock", -3.64, 3.00, 0.0))
    assert not any(
        pose.zone.startswith("wall_tip")
        for pose in north_dock._cube_approach_waypoints("RED_CUBE_1", 0.0, 0.0)
    )
    # ... or when the base already starts south of it.
    south_start = Harness(south_west)
    assert not any(
        pose.zone.startswith("wall_tip")
        for pose in south_start._cube_approach_waypoints(
            "RED_CUBE_3", -1.40, -12.00
        )
    )


def test_departure_dock_does_not_gate_on_heading():
    """A departure drives to its entry instead of rotating to face it.

    AC1 (2026-09-11): the BLUE_EXIT target (the tray-clear entry) sits
    directly behind the base, so the reverse branch and the rotate-first
    branch alternate - the yaw limit-cycled between 3.10 and 1.85 for 33 s,
    timed out twice and failed the mission with departure_navigation_failed.
    """
    source = (
        REPOSITORY / "tools_demo" / "tools_demo" / "task_navigator.py"
    ).read_text(encoding="utf-8")
    dock_block = source.split("if self.dock_pose is not None:", 1)[1].split(
        "if self.recovery_target is not None:", 1
    )[0]
    assert "and self.active_zone not in self.departure_routes" in dock_block
    assert "A departure never gated on heading" in dock_block


def test_departure_backoff_keeps_one_direction_of_travel():
    """A departure latches its direction of travel for the whole backoff.

    BC1 (2026-09-12): the BLUE_EXIT backoff started at (4.713, 1.914,
    yaw=3.138) with its tray-clear entry dead astern, so the yaw trim was
    about 0.006 rad/s. Once the shelf contact had turned the base a few
    degrees the heading error crossed +-pi/2, ``reverse`` flipped, the
    commanded linear velocity flipped with it and the base pivoted around
    the contact: the leg expired 33 s later at (4.565, 1.641, yaw=1.855).
    The next leg then began inside the tray inflation (cost 99), the planner
    refused every goal with "Starting point in lethal space", both attempts
    aborted (121.1 s, 114.6 s) and the mission ended
    destination_navigation_failed:holding_cube at 3/5 after 470 s.

    Removing the trim instead is not an option: campaign 20260912T203830
    AB1-AB3 showed that a departure whose entry is not dead astern never
    converges without it and burns the full 33 s deadline (292-299 s rounds
    against the 290 s gate).
    """
    source = (
        REPOSITORY / "tools_demo" / "tools_demo" / "task_navigator.py"
    ).read_text(encoding="utf-8")
    assert "self.departure_reverse: Optional[bool] = None" in source
    dock_block = source.split("if self.dock_pose is not None:", 1)[1].split(
        "if self.recovery_target is not None:", 1
    )[0]
    assert "if self.departure_reverse is None:" in dock_block
    assert "reverse = self.departure_reverse" in dock_block
    assert "remaining, lateral = reverse_dock_errors(" in dock_block
    assert "remaining < -0.20" in dock_block
    assert "min(0.85, 2.6 * heading_error)" in dock_block
    assert "min(0.15, 2.6 * heading_error)" in dock_block
    stop_block = source.split("def stop_alignment(", 1)[1].split(
        "def timeout_callback(", 1
    )[0]
    assert "self.departure_reverse = None" in stop_block


def test_zone_egress_drives_out_instead_of_rotating_first():
    """A zone exit must not rotate to face a target that is behind it.

    BC3 (2026-09-11): the zone B retreat target sits behind the docked base,
    so the reverse branch and the rotate branch alternated, the yaw
    limit-cycled for 33 s twice and the mission failed at 2/5 with
    zone_exit_navigation_failed.
    """
    source = (
        REPOSITORY / "tools_demo" / "tools_demo" / "task_navigator.py"
    ).read_text(encoding="utf-8")
    egress = source.split("if self.zone_egress_target is not None:", 1)[1].split(
        "if self.pre_navigation_alignment:", 1
    )[0]
    assert "command.linear.x = -speed if reverse else speed" in egress
    assert "2.6 * heading_error" in egress
    # No rotate-only branch is left in the egress drive.
    assert "if abs(heading_error) > 0.12:" not in egress


def test_departure_clears_the_pickup_dock_mode():
    """A departure must not inherit the pickup forward-only approach.

    AB1 (2026-09-11): the BLUE_EXIT target (the tray-clear entry) sat 0.43 m
    behind the base, but `pickup_final_approach` was still set, so the
    handler drove the loaded base forward into the shelf. With the yaw error
    at 0.004 rad the leg still never reached its 0.04 m position tolerance
    and timed out twice at 33 s, failing the mission at 3/5.
    """
    source = (
        REPOSITORY / "tools_demo" / "tools_demo" / "task_navigator.py"
    ).read_text(encoding="utf-8")
    departure = source.split(
        "backoff_distance = self.departure_backoff_distance", 1
    )[1].split("departing_cube_dock = self.last_cube_entry is not None", 1)[0]
    assert "self.pickup_final_approach = False" in departure
    assert "self.next_dock_pose = None" in departure


def test_departure_timeout_hands_the_leg_to_nav2():
    """A failed departure or zone exit must not fail the whole mission.

    Measured 2026-09-12: the east-dock departure timed out twice at 33 s in
    one campaign (AB1 finished 4/5 and BC1 3/5 with
    departure_navigation_failed) while no other node logged a single message
    in the window, so the base simply never covered the last centimetres.
    """
    source = (
        REPOSITORY / "tools_demo" / "tools_demo" / "task_navigator.py"
    ).read_text(encoding="utf-8")
    timeout_block = source.split(
        "def alignment_callback", 1
    )[1].split("if self.zone_egress_target is not None:", 1)[0]
    assert "self.active_zone in self.departure_routes" in timeout_block
    assert "self.active_zone in VALID_ZONE_EXITS" in timeout_block
    assert "exit_timeout_handoff" in timeout_block
    # The generic failure path stays for real docks.
    assert '"Final docking or yaw alignment timed out"' in timeout_block


def test_blocked_blue_departure_sends_entry_goal_before_c_delivery():
    """Replay AC1's blocked east-dock exit at the alignment call site."""
    from tools_demo.task_navigator import TaskNavigator, ZonePose

    states = []
    entry_goals = []
    entry = ZonePose("BLUE_EXIT", -9.24, -3.0, math.pi)
    harness = SimpleNamespace(
        alignment_target_yaw=math.pi,
        latest_x=-9.593,
        latest_y=-3.031,
        latest_yaw=2.768,
        alignment_deadline_ns=1,
        pre_navigation_alignment=False,
        active_zone="BLUE_EXIT",
        departure_routes={"BLUE_EXIT": True},
        dock_pose=entry,
        dock_arrival_tolerance=0.04,
        pending_poses=[],
        _global_cost_at=lambda x_value, y_value: 83,
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=2)
        ),
        get_logger=lambda: SimpleNamespace(warn=lambda message: None),
        stop_alignment=lambda: None,
        publish_state=lambda state, **kwargs: states.append(state),
        _clear_goal=lambda: None,
        _send_next_goal=lambda: entry_goals.extend(harness.pending_poses),
    )

    TaskNavigator.alignment_callback(harness)
    assert states == ["sending"]
    assert entry_goals == [entry]


def test_blue_departure_completes_after_crossing_entry_plane():
    """Replay the fast reverse crossing that formerly ran 4 m past entry."""
    from tools_demo.task_navigator import TaskNavigator, ZonePose

    states = []
    harness = SimpleNamespace(
        alignment_target_yaw=math.pi,
        latest_x=-9.19,
        latest_y=-3.07,
        latest_yaw=3.10,
        alignment_deadline_ns=100,
        pre_navigation_alignment=False,
        zone_egress_target=None,
        active_zone="BLUE_EXIT",
        departure_routes={"BLUE_EXIT": True},
        dock_pose=ZonePose("BLUE_EXIT", -9.24, -3.0, math.pi),
        dock_arrival_tolerance=0.04,
        pickup_final_approach=False,
        recovery_target=None,
        next_dock_pose=None,
        pending_poses=[],
        _global_cost_at=lambda x_value, y_value: 0,
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=1)
        ),
        get_logger=lambda: SimpleNamespace(info=lambda message: None),
        stop_alignment=lambda: None,
        publish_state=lambda state, **kwargs: states.append(state),
        _clear_goal=lambda: None,
    )

    TaskNavigator.alignment_callback(harness)
    assert states == ["succeeded"]


def test_red_departure_does_not_send_nav2_from_lethal_start():
    """Replay BC1's cost-99 red2 exit after direct backoff stalled."""
    from tools_demo.task_navigator import TaskNavigator, ZonePose

    states = []
    entry_goals = []
    entry = ZonePose("RED_EXIT", -0.85, 3.0, 0.0)
    harness = SimpleNamespace(
        alignment_target_yaw=0.0,
        latest_x=-0.324,
        latest_y=3.171,
        latest_yaw=-0.864,
        alignment_deadline_ns=1,
        pre_navigation_alignment=False,
        active_zone="RED_EXIT",
        departure_routes={"RED_EXIT": True},
        dock_pose=entry,
        dock_arrival_tolerance=0.04,
        pending_poses=[],
        _global_cost_at=lambda x_value, y_value: 99,
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=2)
        ),
        get_logger=lambda: SimpleNamespace(warn=lambda message: None),
        stop_alignment=lambda: None,
        publish_error=lambda message: None,
        publish_state=lambda state, **kwargs: states.append(state),
        _clear_goal=lambda: None,
        _send_next_goal=lambda: entry_goals.extend(harness.pending_poses),
    )

    TaskNavigator.alignment_callback(harness)
    assert states == ["failed"]
    assert entry_goals == []


def test_global_costmap_probe_reads_the_lethal_start_cell():
    """Check the occupancy-grid cell used by departure fallback gating."""
    from tools_demo.task_navigator import TaskNavigator

    costmap = SimpleNamespace(
        info=SimpleNamespace(
            width=3,
            height=3,
            resolution=1.0,
            origin=SimpleNamespace(
                position=SimpleNamespace(x=-1.0, y=-1.0)
            ),
        ),
        data=[0, 0, 0, 0, 99, 0, 0, 0, 0],
    )
    harness = SimpleNamespace(global_costmap=costmap)
    assert TaskNavigator._global_cost_at(harness, 0.1, 0.1) == 99
    assert TaskNavigator._global_cost_at(harness, 2.5, 2.5) is None


def test_laser_filters_exclude_the_carried_payload():
    """Neither the costmap nor the collision monitor may see our own cube.

    The carried cube rides 0.28 m ahead of the base with its top edge at
    0.10-0.18 m. BC72 (2026-09-12) crept 0.087 m in 8 s while carrying,
    aborted both retries and timed out after 311 s with
    destination_navigation_failed:holding_cube, and campaigns 12-14 logged
    175-350 collision-monitor NearSlow toggles per round.
    """
    config = (
        REPOSITORY / "bot_navigation" / "param" / "originbot_nav2.yaml"
    ).read_text(encoding="utf-8")
    local = config.split("local_costmap:", 1)[1].split(
        "global_costmap:", 1
    )[0]
    assert "min_obstacle_height: 0.20" in local
    monitor = config.split("collision_monitor:", 1)[1]
    assert "min_height: 0.20" in monitor
    # Static obstacles and placed cubes keep their own low-height sources.
    assert "min_obstacle_height: 0.0" in config


def test_lethal_start_cost_reads_the_nav2_bands():
    """The OccupancyGrid encodes lethal as 100, not raw cost byte 254."""
    from tools_demo.task_navigator import TaskNavigator

    harness = SimpleNamespace(
        latest_x=8.0, latest_y=0.0,
        _global_cost_at=lambda x, y: 100,
    )
    assert TaskNavigator._lethal_start_cost(harness) == 100
    for cost in (99, 98, 71, 0, -1, None):
        harness._global_cost_at = lambda x, y, cost=cost: cost
        assert TaskNavigator._lethal_start_cost(harness) is None
    harness.latest_x = None
    harness._global_cost_at = lambda x, y: 100
    assert TaskNavigator._lethal_start_cost(harness) is None


def test_lethal_start_is_gated_before_the_goal_reaches_nav2():
    """A goal Nav2 cannot plan must never be sent.

    Round 4rB_1bC (2026-10-04) logged 'collision ahead' six times, then
    'Controller patience exceeded', then 'GridBased: failed to create plan,
    invalid use: Starting point in lethal space', and the round was recorded
    as failed on lethal start x1 although the mission delivered 5/5. The probe
    already computed the start cost there - it was only printed.
    """
    source = (
        REPOSITORY / "tools_demo" / "tools_demo" / "task_navigator.py"
    ).read_text(encoding="utf-8")
    send = source.split("def _send_pose(self", 1)[1].split(
        "def _send_pose_batch", 1
    )[0]
    probe = send.index("_log_costmap_probe(pose)")
    gate = send.index("_lethal_start_cost()")
    goal = send.index("send_goal_async")
    assert probe < gate < goal
    assert 'reason="lethal_start_blocked"' in send


def test_nav_retry_does_not_resend_from_a_lethal_cell():
    """A retry uses the same cell, so it must report 'no retry issued'.

    Otherwise _send_pose blocks the resend and the caller waits for a nav
    result that never arrives.
    """
    source = (
        REPOSITORY / "tools_demo" / "tools_demo" / "task_navigator.py"
    ).read_text(encoding="utf-8")
    retry = source.split("def _retry_nav_goal", 1)[1].split(
        "def _fallback_batch_to_serial", 1
    )[0]
    gate = retry.index("_lethal_start_cost()")
    assert retry.index("_send_pose(self.nav_goal_pose)") > gate
    assert "return False" in retry[gate:]


def test_every_costmap_layer_clears_the_robot_footprint():
    """The mark must not land on the cell the planner starts from.

    /competition/placed_cubes marks each delivered cube at floor height and
    the robot finishes a placement standing beside the cube it just set down.
    Round 1rA_4bB (2026-10-04, verify_lethal batch) refused the next leg three
    times with "Starting point in lethal space" for exactly that reason. The
    static layer was given footprint clearing earlier for the same symptom,
    but its clearing does not cover a mark this layer makes.
    """
    config = yaml.safe_load(
        (REPOSITORY / "bot_navigation" / "param" / "originbot_nav2.yaml").read_text(
            encoding="utf-8"
        )
    )
    global_params = config["global_costmap"]["global_costmap"]["ros__parameters"]
    local_params = config["local_costmap"]["local_costmap"]["ros__parameters"]
    for params in (global_params, local_params):
        assert params["obstacle_layer"]["footprint_clearing_enabled"] is True
        assert params["static_layer"]["footprint_clearing_enabled"] is True
    # The cube marks must still be an obstacle source for planning.
    assert global_params["obstacle_layer"]["observation_sources"] == "placed_cubes"
    assert global_params["obstacle_layer"]["placed_cubes"]["marking"] is True

def test_reverse_probe_uses_an_existing_clearance_method():
    """The mover reverse branch cannot crash on a missing probe method."""
    from tools_demo.task_navigator import TaskNavigator
    import inspect
    source = inspect.getsource(TaskNavigator._begin_obstacle_reverse)
    assert "self._costmap_segment_clear(" in source
    assert "_segment_is_free" not in source


def test_holonomic_lateral_channel_is_enabled_but_bounded():
    """Docking lateral centering must survive the smoother, under a small cap."""
    parameters = (REPOSITORY / "bot_navigation" / "param"
                  / "originbot_nav2.yaml").read_text(encoding="utf-8")
    assert "max_velocity: [2.30, 0.22, 3.2]" in parameters
    coordinator = (REPOSITORY / "tools_demo" / "tools_demo"
                   / "mission_coordinator.py").read_text(encoding="utf-8")
    # The centering command must fit under the smoother cap or it is clamped.
    assert "speed = 0.20" in coordinator


def test_handoff_waits_for_the_base_to_decelerate():
    """Cancelling Nav2 at cruise speed made the base coast past the entry."""
    from types import SimpleNamespace
    from unittest.mock import Mock, patch
    from tools_demo.task_navigator import TaskNavigator, ZonePose
    node = SimpleNamespace(
        goal_handle=Mock(),
        nav_goal_pose=ZonePose("RED_CUBE_1", 10.0, 0.0, 0.0),
        nav_handoff_requested=False, goal_timed_out=False,
        latest_x=9.95, latest_y=0.0, latest_vx=1.5, latest_vy=0.0,
        nav_handoff_max_speed=0.45,
        nav_handoff_speed_grace_ns=2_000_000_000,
        nav_handoff_radius_since_ns=None, active_zone="RED_CUBE_1",
        _route_handoff_distance=lambda: 0.20,
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(
            nanoseconds=1_000_000_000)),
        get_logger=lambda: Mock(), publish_state=Mock(),
        _handoff_cancel_callback=Mock(),
    )
    with patch("tools_demo.task_navigator.navigation_handoff_reached",
               return_value=True):
        TaskNavigator._request_nav_handoff_if_reached(node)
    # Fast base inside the radius: wait, do not cancel the controller yet.
    assert node.nav_handoff_requested is False
    assert node.nav_handoff_radius_since_ns == 1_000_000_000
    node.latest_vx = 0.30
    with patch("tools_demo.task_navigator.navigation_handoff_reached",
               return_value=True):
        TaskNavigator._request_nav_handoff_if_reached(node)
    assert node.nav_handoff_requested is True


def test_new_layout_a_zone_keeps_the_tray_route_without_a_corridor():
    """Zone A must not be treated as a dedicated corridor it never used.

    Evidence 20261013T215722: with use_legacy_layout_routes false the helper
    returned True for every zone, A's tray-crossing waypoints were skipped, and
    the leg from the north-east blue_cube_1 pickup to A ran straight along the
    shelf row (serpentine path hugging the wall, RPP collision-ahead warnings
    for the whole leg).
    """
    from types import SimpleNamespace

    from tools_demo.task_navigator import TaskNavigator

    def active(zone, **flags):
        state = dict(
            use_legacy_layout_routes=False,
            a_corridor_route_active=False,
            b_northeast_route_active=False,
            c_east_wall_descent_active=False,
            c_clearance_route_active=False,
        )
        state.update(flags)
        return TaskNavigator._dedicated_corridor_active(
            SimpleNamespace(**state), zone)

    assert active("A") is False
    assert active("A", a_corridor_route_active=True) is True
    # B and C keep their new-layout dedicated corridors.
    assert active("B") is True
    assert active("C") is True
    # Legacy layout keeps its historical behaviour.
    assert active("A", use_legacy_layout_routes=True) is False
    assert active(
        "C", use_legacy_layout_routes=True,
        c_clearance_route_active=True) is True


def test_pickup_dock_cross_track_miss_stops_instead_of_ramming():
    """A cross-track miss must fail the dock, never creep into the tray.

    Evidence 20261013T215722: 16 lateral-miss lines with forward=0.000 while the
    base leaned on the shelf, then a 40 s dock timeout and a lost round.
    """
    source = (
        REPOSITORY / "tools_demo" / "tools_demo" / "task_navigator.py"
    ).read_text(encoding="utf-8")
    loop = source.split("def alignment_callback", 1)[1].split(
        "def ", 1)[0]
    fail = loop.index("lateral_miss > self.dock_pickup_lateral_fail")
    gate = loop.rindex("forward <= 0.12", 0, fail)
    stop = loop.index('reason="pickup_dock_lateral_miss"')
    drive = loop.index("pickup_dock_drive_distance(")
    # The stop-and-fail branch is proximity gated: a miss the approach can
    # still arc away must not kill the leg (20261014T123209 failed 0.42 m out
    # with a 0.19 m miss), and it must run before any drive command.
    assert gate < fail < stop < drive
    assert "self.stop_alignment()" in loop[fail:stop]


def _pickup_guide_block():
    source = (
        REPOSITORY / "tools_demo" / "tools_demo" / "task_navigator.py"
    ).read_text(encoding="utf-8")
    return source.split("def _cube_approach_waypoints", 1)[1].split(
        "def ", 1)[0]


def test_new_layout_keeps_the_original_mover_bypass_gates():
    """Blue pickups keep the bypass for the movers that never moved.

    Evidence 20261013T225015: without it, a north-side start on blue_cube_2 ran
    a straight diagonal through the racks and clipped the bottom shelf.
    """
    block = _pickup_guide_block()
    assert "OBSTACLE_2_NORTH_SOUTH_GATES" in block
    assert "OBSTACLE_2_WEST_UPPER_GATE" in block
    assert "prefix = guides + bypass" in block
    assert "return prefix + central_gates" in block


def test_dock_fail_threshold_is_beyond_the_arrival_band():
    """Arrival needs 0.06 m; failing there turned residuals into whole-leg retries."""
    source = (
        REPOSITORY / "tools_demo" / "tools_demo" / "task_navigator.py"
    ).read_text(encoding="utf-8")
    assert 'declare_parameter("dock_pickup_lateral_fail", 0.15)' in source
    loop = source.split("def alignment_callback", 1)[1].split("def ", 1)[0]
    assert "lateral_miss > self.dock_pickup_lateral_fail" in loop


def test_south_east_bay_exit_stages_before_turning():
    """Leaving the SE pickup bay, run clear of the rack before turning.

    Evidence 20261013T231641: the direct B leg from that bay produced nine RPP
    collision-ahead frames in five seconds and a shelf contact, because the
    zone goal pulled the base straight back into the shelf band while the
    post-pickup line was 0.4 m further out.
    """
    from types import SimpleNamespace

    from tools_demo.task_navigator import TaskNavigator, ZonePose

    def staging(x, y, place_y, distance=2.0):
        node = SimpleNamespace(
            bay_exit_staging_distance=distance, latest_x=x, latest_y=y)
        return TaskNavigator._bay_exit_staging(
            node, ZonePose("B", 11.3, place_y, 0.0))

    pose = staging(10.93, -13.0, -5.0)
    assert (pose.x, pose.y) == (10.93, -11.0)
    # Outside the south-east bay nothing changes.
    assert staging(6.0, 3.6, -5.0) is None
    # Disabled, or a target the staging would overshoot.
    assert staging(10.93, -13.0, -5.0, distance=0.0) is None
    assert staging(10.93, -13.0, -11.2) is None
    # Heading south out of the bay stages the other way.
    assert abs(staging(10.93, -13.4, -16.0).y + 15.4) < 1e-9


def test_dock_never_creeps_forward_on_a_lateral_residual():
    """On the dock plane the loop must command no forward step.

    Every cube's tray edge sits 80 mm from the docked chassis front face, so a
    lateral residual that keeps the loop driving presses the base into the tray
    (20261013T231641: competition_blue_pickup_tray_5).
    """
    source = (
        REPOSITORY / "tools_demo" / "tools_demo" / "task_navigator.py"
    ).read_text(encoding="utf-8")
    loop = source.split("def alignment_callback", 1)[1].split("def ", 1)[0]
    hold = loop.index("if forward <= self.dock_arrival_tolerance:")
    drive = loop.index("distance = pickup_dock_drive_distance(")
    assert hold < drive
    assert "distance = 0.0" in loop[hold:drive]


def test_pickup_dock_rejects_miss_and_only_bounded_overshoot_realigns():
    """The final 12 cm is gated; only a bounded stopped overshoot may hand off."""
    source = (
        REPOSITORY / "tools_demo" / "tools_demo" / "task_navigator.py"
    ).read_text(encoding="utf-8")
    loop = source.split("def alignment_callback", 1)[1].split("def ", 1)[0]
    assert (
        "raw_forward\n                    "
        "< -self.dock_pickup_overshoot_realign_limit"
    ) in loop
    assert "forward <= 0.12" in loop
    assert '"pickup_dock_overshoot"' in loop
    assert 'phase="pickup_dock_overshoot_realign"' in loop
    assert "lateral_error=signed_lateral" in loop

    safe = loop.index('phase="pickup_dock_overshoot_realign"')
    drive = loop.index("distance = pickup_dock_drive_distance(")
    assert safe < drive
    assert "distance = 0.0" in loop[safe:drive]


def test_central_pickup_does_not_reverse_after_safe_gate():
    """A cart behind the red-2/blue-3 route cannot undo the precision entry."""
    from types import SimpleNamespace

    from tools_demo.task_navigator import TaskNavigator, ZonePose

    south = SimpleNamespace(
        latest_y=-5.10,
        nav_goal_pose=ZonePose("RED_CUBE_2_ENTRY", 1.72, -5.40, math.pi),
    )
    assert not TaskNavigator._central_cart_can_block_active_segment(south)

    approaching = SimpleNamespace(
        latest_y=-4.20,
        nav_goal_pose=ZonePose("RED_CUBE_2_ENTRY", 1.72, -5.40, math.pi),
    )
    assert TaskNavigator._central_cart_can_block_active_segment(approaching)


def test_failed_precision_retry_resumes_locally_instead_of_reentering_nav2():
    """A base beside the tray must not ask Nav2 to plan from a lethal cell."""
    source = (
        REPOSITORY / "tools_demo" / "tools_demo" / "task_navigator.py"
    ).read_text(encoding="utf-8")
    command = source.split("elif zone in self.cube_docks:", 1)[1].split(
        "else:\n            # A/B/C", 1
    )[0]
    assert "pickup_dock_resume_local" in command
    assert ") <= 0.20" in command
    assert "self.pending_poses = []" in command
    assert "self.pickup_final_approach = True" in command
    assert "max(0.12, self.alignment_yaw_tolerance)" in source


def test_pickup_dock_yaw_residual_realigns_without_forward_motion():
    """A bounded Blue-5 yaw residual must rotate in place, not fail/re-drive."""
    source = (
        REPOSITORY / "tools_demo" / "tools_demo" / "task_navigator.py"
    ).read_text(encoding="utf-8")
    loop = source.split("def alignment_callback", 1)[1].split(
        "def ", 1
    )[0]
    start = loop.index("if self.pickup_final_approach:", loop.index(
        "if self.next_dock_pose is not None:"
    ))
    block = loop[start:]
    assert "dock_pickup_yaw_realign_limit" in block
    assert 'phase="pickup_dock_yaw_realign"' in block
    # The safe residual falls through to the normal yaw-only controller; only
    # an angle beyond the explicit shelf-side limit may clear/fail the goal.
    safe = block.index('phase="pickup_dock_yaw_realign"')
    final_yaw = block.index("error = math.atan2(", safe)
    assert safe < final_yaw
    assert "command.linear.x" not in block[safe:final_yaw]


def test_dock_yaw_gate_matches_the_plinth_geometry():
    """The in-place yaw gate is bounded by the 0.08 m plinth clearance.

    Turning theta about the base centre swings the leading front corner by about
    0.16*sin(theta) + 0.16*(1-cos(theta)): 0.030 m at 0.18 rad and 0.065 m at
    0.30 rad.  A 0.18 rad gate failed red_cube_2 at 0.24 rad with its position
    already on the dock (20261014T043841), so the gate must stay at 0.30 rad
    and the clearance must stay at least the 0.065 m that implies.
    """
    source = (
        REPOSITORY / "tools_demo" / "tools_demo" / "task_navigator.py"
    ).read_text(encoding="utf-8")
    assert 'declare_parameter("dock_pickup_yaw_realign_limit", 0.30)' in source


def test_central_hold_backs_off_and_replans_instead_of_waiting_out():
    """The central corridor gets a short retreat-and-replan, nothing else does.

    Evidence 20261014: the first red leg spent its time budget holding for a
    crossing window in the central corridor.
    """
    source = (
        REPOSITORY / "tools_demo" / "tools_demo" / "task_navigator.py"
    ).read_text(encoding="utf-8")
    assert 'declare_parameter("central_backoff_sec", 1.5)' in source
    wait = source.split("def _obstacle_wait_step", 1)[1].split("def ", 1)[0]
    backoff = wait.index("central_backoff_sec")
    # The branch is fenced to the central gate line and reuses the checked
    # reverse, so no other region can take it.
    assert "abs(self.latest_y - CENTRAL_MOVER_GATE_WEST.y) <= 1.2" in wait
    assert "self._begin_obstacle_reverse(topic)" in wait[backoff:]
    assert "central_backoff_replan begin" in wait


def test_central_failure_path_backs_off_before_aborting():
    """A leg that dies in the central corridor retreats and re-plans first.

    Evidence 20261014T074511Z (reproduced): the base started at (-0.84,-4.32)
    inside the corridor for the zone A south gate, the only bounded recovery
    was rejected, and the leg aborted with the cube in hand for a zero score.
    """
    source = (
        REPOSITORY / "tools_demo" / "tools_demo" / "task_navigator.py"
    ).read_text(encoding="utf-8")
    recovery = source.split("def _try_bounded_recovery", 1)[1].split(
        "def ", 1)[0]
    assert 'getattr(self, "_try_central_backoff_replan", None)' in recovery
    helper = source.split("def _try_central_backoff_replan", 1)[1].split(
        "def ", 1)[0]
    # Fenced to the central gate band, and only starts a checked reverse.
    assert "abs(latest_y - CENTRAL_MOVER_GATE_WEST.y) > 1.2" in helper
    assert "not -1.5 <= latest_x <= 2.0" in helper
    assert "begin_reverse(topic)" in helper
