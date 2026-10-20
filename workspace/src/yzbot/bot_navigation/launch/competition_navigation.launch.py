# Copyright 2026 aerospaceboy
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch Nav2 and competition navigation helpers for Gazebo."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import SetParameter
from launch_ros.actions import SetRemap
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    """Create the Gazebo competition navigation launch description."""
    navigation_share = get_package_share_directory("bot_navigation")
    tools_share = get_package_share_directory("tools_demo")
    nav2_share = get_package_share_directory("nav2_bringup")
    # Resolve the same project BT through the installed package, so a new
    # Linux user/workspace does not inherit the original VM source path.
    portable_nav2_params = RewrittenYaml(
        source_file=LaunchConfiguration("params_file"),
        param_rewrites={
            "default_nav_to_pose_bt_xml": (
                f"{navigation_share}/config/navigate_to_pose_recovery.xml"
            ),
            # Match TaskNavigator's existing strict through-path tree. Once
            # two goal checkers are available, every default tree must name
            # the unchanged ordinary checker explicitly as well.
            "default_nav_through_poses_bt_xml": (
                f"{tools_share}/config/navigate_through_poses_strict.xml"
            ),
        },
        convert_types=True,
    )

    map_argument = DeclareLaunchArgument(
        "map",
        default_value=f"{navigation_share}/maps/mapn3.yaml",
    )
    params_argument = DeclareLaunchArgument(
        "params_file",
        default_value=f"{navigation_share}/param/originbot_nav2.yaml",
    )
    zones_argument = DeclareLaunchArgument(
        "zones_file",
        default_value=f"{navigation_share}/config/competition_zones.yaml",
    )
    risk_zones_argument = DeclareLaunchArgument(
        "risk_zones_file",
        default_value=f"{navigation_share}/config/navigation_risk_zones.yaml",
    )
    sim_time_argument = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
    )
    obstacle_argument = DeclareLaunchArgument(
        "moving_obstacle",
        default_value="True",
    )

    # Humble's bringup launch does not pass the project parameter file to its
    # lifecycle managers. Inject the timeout into this launch scope so a short
    # loaded-VM DDS pause cannot turn into a permanent Nav2 shutdown.
    nav2 = GroupAction(
        actions=[
            SetParameter(name="bond_timeout", value=15.0),
            SetRemap(src="behavior_server:cmd_vel", dst="/cmd_vel_nav"),
            SetRemap(src="velocity_smoother:cmd_vel",
                     dst="/competition/cmd_vel_selected"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    f"{nav2_share}/launch/bringup_launch.py"
                ),
                launch_arguments={
                    "map": LaunchConfiguration("map"),
                    "params_file": portable_nav2_params,
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "autostart": "True",
                    "slam": "False",
                    "use_composition": "False",
                    "use_respawn": "True",
                }.items(),
            ),
        ],
    )
    collision_monitor = Node(
        package="nav2_collision_monitor",
        executable="collision_monitor",
        name="collision_monitor",
        parameters=[
            LaunchConfiguration("params_file"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        output="screen",
        emulate_tty=True,
    )
    collision_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="collision_monitor_lifecycle_manager",
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            {"autostart": True},
            {"node_names": ["collision_monitor"]},
            {"bond_timeout": 15.0},
        ],
        output="screen",
        emulate_tty=True,
    )
    # Start the lifecycle node first, then its manager. The stock Humble
    # launch starts both concurrently and can ask for configure before the
    # service exists on a loaded VM. This remains a single cold activation;
    # the monitor must not be hot-cycled because reactivation cancels its timer.
    delayed_collision_monitor = TimerAction(
        period=5.0,
        actions=[collision_monitor],
    )
    delayed_collision_lifecycle_manager = TimerAction(
        period=7.0,
        actions=[collision_lifecycle_manager],
    )
    map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="competition_map_to_odom",
        arguments=[
            "0",
            "0",
            "0",
            "0",
            "0",
            "0",
            "map",
            "odom",
        ],
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        output="screen",
    )
    # The two support wheels are continuous joints in large_base.xacro but are
    # not part of ros2_control and have no joint states, so robot_state_publisher
    # never publishes base_link -> {left,right}_wheel_link and RViz reports
    # "No transform from [left_wheel_link] to [map]". They never articulate (the
    # base moves through the planar controller), so static transforms taken from
    # the URDF origins are exact: x = wheel_joint_x, y = +-(base_width +
    # wheel_length)/2, z = -(base_height/2 + caster_radius*2 - wheel_radius).
    left_wheel_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="competition_left_wheel_tf",
        arguments=[
            "0.14", "0.185", "-0.0384", "0", "0", "0",
            "base_link", "left_wheel_link",
        ],
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        output="screen",
    )
    right_wheel_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="competition_right_wheel_tf",
        arguments=[
            "0.14", "-0.185", "-0.0384", "0", "0", "0",
            "base_link", "right_wheel_link",
        ],
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        output="screen",
    )
    velocity_selector = Node(
        package="tools_demo", executable="velocity_selector",
        name="competition_velocity_selector", output="screen",
    )
    navigator = Node(
        package="tools_demo",
        executable="task_navigator",
        name="competition_task_navigator",
        parameters=[
            LaunchConfiguration("zones_file"),
            {"risk_zones_file": LaunchConfiguration("risk_zones_file")},
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        output="screen",
    )
    dynamic_obstacles = Node(
        package="tools_demo", executable="dynamic_obstacles",
        name="competition_dynamic_obstacles",
        parameters=[LaunchConfiguration("zones_file"),
                    {"use_sim_time": LaunchConfiguration("use_sim_time")}],
        output="screen",
    )
    # The costmaps and the collision monitor read /scan_payload_masked (see
    # originbot_nav2.yaml): while the mission holds a cube this node removes
    # the carried payload's own returns, which otherwise marked the robot's
    # own box, stalled RPP with "collision ahead" and left a lethal trail
    # behind every carrying leg. Every real obstacle source (static map,
    # placed-cubes layer, mover truth for the safety limiter, returns outside
    # the payload box) stays untouched.
    payload_scan_mask = Node(
        package="tools_demo", executable="payload_scan_mask",
        name="competition_payload_scan_mask",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        output="screen",
    )
    # Keep the original mover-2 unsafe stripe and static diagonal-wall bands.
    # New central carts have no permanent swept-band keepout: lidar and its
    # short prediction must leave their actually vacated gap available.
    sweep_band_keepout = Node(
        package="tools_demo", executable="sweep_band_keepout",
        name="competition_sweep_band_keepout",
        parameters=[
            LaunchConfiguration("zones_file"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        output="screen",
    )
    extra_carts = [Node(
        package="tools_demo", executable="moving_obstacle", name=name,
        parameters=[LaunchConfiguration("zones_file"),
                    {"use_sim_time": LaunchConfiguration("use_sim_time")}],
        condition=IfCondition(LaunchConfiguration("moving_obstacle")),
        output="screen",
    ) for name in ["crossing_cart_1", "crossing_cart_2"]]
    obstacle_1 = Node(
        package="tools_demo",
        executable="moving_obstacle",
        name="competition_moving_obstacle_1",
        parameters=[
            LaunchConfiguration("zones_file"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        condition=IfCondition(LaunchConfiguration("moving_obstacle")),
        output="screen",
    )
    obstacle_2 = Node(
        package="tools_demo",
        executable="moving_obstacle",
        name="competition_moving_obstacle_2",
        parameters=[
            LaunchConfiguration("zones_file"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        condition=IfCondition(LaunchConfiguration("moving_obstacle")),
        output="screen",
    )
    avoidance_monitor = Node(
        package="tools_demo",
        executable="avoidance_monitor",
        name="competition_avoidance_monitor",
        parameters=[
            LaunchConfiguration("zones_file"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        condition=IfCondition(LaunchConfiguration("moving_obstacle")),
        output="screen",
    )

    return LaunchDescription(
        [
            map_argument,
            params_argument,
            zones_argument,
            risk_zones_argument,
            sim_time_argument,
            obstacle_argument,
            map_to_odom,
            left_wheel_tf,
            right_wheel_tf,
            nav2,
            delayed_collision_lifecycle_manager,
            delayed_collision_monitor,
            navigator,
            velocity_selector,
            dynamic_obstacles,
            payload_scan_mask,
            sweep_band_keepout,
            *extra_carts,
            obstacle_1,
            obstacle_2,
            avoidance_monitor,
        ]
    )
