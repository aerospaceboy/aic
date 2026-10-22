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

"""Launch task parsing, perception, and the competition mission coordinator."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Create the non-blocking mission orchestration launch description."""
    share = FindPackageShare("tools_demo")
    task_parameters = PathJoinSubstitution(
        [share, "config", "task_parser.yaml"]
    )
    perception_parameters = PathJoinSubstitution(
        [share, "config", "perception.yaml"]
    )
    mission_parameters = PathJoinSubstitution(
        [share, "config", "mission.yaml"]
    )
    dashboard_parameters = PathJoinSubstitution(
        [share, "config", "dashboard.yaml"]
    )
    generate_question = DeclareLaunchArgument(
        "generate_question",
        default_value="False",
        description="Generate one random task after launch",
    )
    use_llm = DeclareLaunchArgument(
        "use_llm",
        default_value="True",
        description="Try the configured OpenAI-compatible task parser",
    )
    require_llm = DeclareLaunchArgument(
        "require_llm",
        default_value="True",
        description="Reject a round unless a validated model plan is used",
    )
    dashboard = DeclareLaunchArgument(
        "dashboard",
        default_value="True",
        description="Serve the local competition scoring dashboard",
    )
    question = Node(
        package="tools_demo",
        executable="task_question",
        name="competition_task_question",
        parameters=[task_parameters, {"use_sim_time": True}],
        condition=IfCondition(LaunchConfiguration("generate_question")),
        output="screen",
    )
    parser = Node(
        package="tools_demo",
        executable="task_parser",
        name="competition_task_parser",
        parameters=[
            task_parameters,
            {
                "use_llm": ParameterValue(
                    LaunchConfiguration("use_llm"),
                    value_type=bool,
                ),
                "require_llm": ParameterValue(
                    LaunchConfiguration("require_llm"),
                    value_type=bool,
                ),
                "use_sim_time": True,
            },
        ],
        output="screen",
    )
    legacy_costudio_status = Node(
        package="tools_demo",
        executable="legacy_costudio_status",
        name="legacy_costudio_status",
        parameters=[{"use_sim_time": False}],
        output="screen",
    )
    safety_limiter = Node(
        package="tools_demo",
        executable="safety_limiter",
        name="competition_safety_limiter",
        parameters=[
            {"use_sim_time": True},
            {
                "cmd_vel_in": "/cmd_vel_collision_safe",
                "cmd_vel_out": "/cmd_vel_safe",
                # This truth guard only covers short scan dropouts. Nav2 and
                # Collision Monitor retain ownership of route avoidance.
                "danger_radius": 0.70,
                "caution_radius": 1.60,
                "cone_half_angle_deg": 50.0,
                "obstacle_timeout_sec": 0.75,
            },
        ],
        output="screen",
    )
    detector = Node(
        package="tools_demo",
        executable="color_target_detector",
        name="competition_color_target_detector",
        parameters=[perception_parameters, {"use_sim_time": True}],
        output="screen",
    )
    coordinator = Node(
        package="tools_demo",
        executable="mission_coordinator",
        name="competition_mission_coordinator",
        remappings=[("/cmd_vel", "/competition/cmd_vel_manipulation")],
        parameters=[mission_parameters, {"use_sim_time": True}],
        output="screen",
    )
    # One persistent MoveIt pick/place worker serves scan/pick/place/release
    # requests from the coordinator, removing the per-operation startup cost.
    pick_place = Node(
        package="mybot",
        executable="pick_place_moveit",
        name="competition_pick_place",
        remappings=[("/cmd_vel", "/competition/cmd_vel_arm_stop")],
        parameters=[
            {
                "use_sim_time": True,
                "persistent": True,
                "prepare_scene": False,
                "verify_placement": False,
                "defer_place_stow": True,
            }
        ],
        output="screen",
    )
    dashboard_node = Node(
        package="tools_demo",
        executable="competition_dashboard",
        name="competition_dashboard",
        parameters=[dashboard_parameters, {"use_sim_time": True}],
        condition=IfCondition(LaunchConfiguration("dashboard")),
        output="screen",
    )
    return LaunchDescription(
        [
            generate_question,
            use_llm,
            require_llm,
            dashboard,
            question,
            parser,
            legacy_costudio_status,
            detector,
            safety_limiter,
            coordinator,
            pick_place,
            dashboard_node,
        ]
    )
