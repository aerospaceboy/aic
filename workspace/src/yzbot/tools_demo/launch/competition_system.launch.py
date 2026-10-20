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

"""Start the complete Gazebo competition system with staged readiness."""

from glob import glob
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from tools_demo.runtime_environment import configure_ros_transport


def _launch_file(package: str, filename: str) -> PythonLaunchDescriptionSource:
    """Resolve one installed Python launch file."""
    return PythonLaunchDescriptionSource(
        f"{get_package_share_directory(package)}/launch/{filename}"
    )


def _render_environment_actions() -> list:
    """Reuse the logged-in Ubuntu desktop for Gazebo camera rendering."""
    actions = []
    if not os.environ.get("DISPLAY") and Path(
        "/tmp/.X11-unix/X0"
    ).exists():
        actions.append(SetEnvironmentVariable("DISPLAY", ":0"))
    if not os.environ.get("XAUTHORITY"):
        candidates = sorted(
            glob(
                f"/run/user/{os.getuid()}"
                "/.mutter-Xwaylandauth.*"
            )
        )
        if candidates:
            actions.append(
                SetEnvironmentVariable("XAUTHORITY", candidates[-1])
            )
    return actions


def generate_launch_description():
    """Create a one-command, safely staged competition launch."""
    transport_config = (
        Path(get_package_share_directory("tools_demo"))
        / "config"
    )
    configure_ros_transport(transport_config)
    arguments = [
        DeclareLaunchArgument(
            "headless",
            default_value="False",
            description="Run Gazebo without its graphical client when true",
        ),
        DeclareLaunchArgument(
            "debug_gui",
            default_value="True",
            description="Use the bounded low-refresh Gazebo client GUI",
        ),
        DeclareLaunchArgument(
            "robot_x",
            default_value="13.2",
            description="Robot initial world X for navigation tests",
        ),
        DeclareLaunchArgument(
            "robot_y",
            default_value="-0.95",
            description="Robot initial world Y for navigation tests",
        ),
        DeclareLaunchArgument(
            "robot_yaw",
            default_value="3.14159",
            description="Robot initial world yaw for navigation tests",
        ),
        DeclareLaunchArgument(
            "moving_obstacle",
            default_value="True",
            description="Enable both scored competition obstacles",
        ),
        DeclareLaunchArgument(
            "slam_evidence",
            default_value="True",
            description="Build a live SLAM map on an isolated topic",
        ),
        DeclareLaunchArgument(
            "generate_question",
            default_value="True",
            description="Generate one task after all subsystems start",
        ),
        DeclareLaunchArgument(
            "use_llm",
            default_value="True",
            description="Prefer LLM parsing with deterministic fallback",
        ),
        DeclareLaunchArgument(
            "start_llm_server",
            default_value="True",
            description="Start the bundled llama.cpp OpenAI-compatible server",
        ),
        DeclareLaunchArgument(
            "llm_server_executable",
            default_value="/home/ros/llama.cpp/build/bin/llama-server",
            description="Path to llama.cpp's server executable",
        ),
        DeclareLaunchArgument(
            "llm_model_path",
            default_value=(
                "/home/ros/llama.cpp/"
                "qwen2.5-coder-0.5b-instruct-q4_k_m.gguf"
            ),
            description="Local GGUF model used to parse competition tasks",
        ),
        DeclareLaunchArgument(
            "require_llm",
            default_value="True",
            description="Require a validated model plan for competition",
        ),
        DeclareLaunchArgument(
            "dashboard",
            default_value="True",
            description="Serve the read-only browser dashboard",
        ),
        DeclareLaunchArgument(
            "run_acceptance",
            default_value="True",
            description="Run the non-destructive startup acceptance check",
        ),
        DeclareLaunchArgument(
            "speed_diagnostics",
            default_value="True",
            description="Record phase-aware speed statistics for each task",
        ),
        DeclareLaunchArgument(
            "subsystem_delay_sec",
            default_value="15.0",
            description="Delay before MoveIt startup",
        ),
        DeclareLaunchArgument(
            "slam_delay_sec",
            default_value="30.0",
            description="Delay before isolated SLAM evidence startup",
        ),
        DeclareLaunchArgument(
            "navigation_delay_sec",
            default_value="45.0",
            description="Delay before Nav2 and collision monitor startup",
        ),
        DeclareLaunchArgument(
            "mission_delay_sec",
            default_value="65.0",
            description="Delay before task and mission startup",
        ),
        DeclareLaunchArgument(
            "acceptance_delay_sec",
            default_value="70.0",
            description="Delay before the startup acceptance check",
        ),
    ]

    gazebo = IncludeLaunchDescription(
        _launch_file("mybot", "gazebo_world.launch.py"),
        launch_arguments={
            "headless": LaunchConfiguration("headless"),
            "debug_gui": LaunchConfiguration("debug_gui"),
            "robot_x": LaunchConfiguration("robot_x"),
            "robot_y": LaunchConfiguration("robot_y"),
            "robot_yaw": LaunchConfiguration("robot_yaw"),
        }.items(),
    )
    llm_server = ExecuteProcess(
        cmd=[
            LaunchConfiguration("llm_server_executable"),
            "--model",
            LaunchConfiguration("llm_model_path"),
            "--host",
            "127.0.0.1",
            "--port",
            "8081",
            "--ctx-size",
            "2048",
            "--threads",
            "4",
            "--n-gpu-layers",
            "0",
        ],
        condition=IfCondition(LaunchConfiguration("start_llm_server")),
        output="screen",
    )
    moveit = IncludeLaunchDescription(
        _launch_file("mybot", "move_group.launch.py"),
    )
    navigation = IncludeLaunchDescription(
        _launch_file(
            "bot_navigation",
            "competition_navigation.launch.py",
        ),
        launch_arguments={
            "moving_obstacle": LaunchConfiguration("moving_obstacle"),
            "use_sim_time": "True",
        }.items(),
    )
    slam_evidence = IncludeLaunchDescription(
        _launch_file(
            "bot_navigation",
            "competition_slam_evidence.launch.py",
        ),
        launch_arguments={"use_sim_time": "True"}.items(),
        condition=IfCondition(LaunchConfiguration("slam_evidence")),
    )
    mission = IncludeLaunchDescription(
        _launch_file("tools_demo", "mission.launch.py"),
        launch_arguments={
            "generate_question": LaunchConfiguration(
                "generate_question"
            ),
            "use_llm": LaunchConfiguration("use_llm"),
            "require_llm": LaunchConfiguration("require_llm"),
            "dashboard": LaunchConfiguration("dashboard"),
        }.items(),
    )
    acceptance = Node(
        package="tools_demo",
        executable="competition_acceptance",
        name="competition_acceptance",
        parameters=[
            f"{get_package_share_directory('tools_demo')}"
            "/config/competition.yaml",
            {"use_sim_time": True},
        ],
        condition=IfCondition(LaunchConfiguration("run_acceptance")),
        output="screen",
    )
    speed_diagnostics = Node(
        package="tools_demo",
        executable="speed_profile_recorder",
        name="competition_speed_profile",
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(LaunchConfiguration("speed_diagnostics")),
        output="screen",
    )

    return LaunchDescription(
        arguments
        + _render_environment_actions()
        + [
            # The standalone launch uses the same local transport as CoStudio.
            SetEnvironmentVariable("RMW_IMPLEMENTATION", os.environ["RMW_IMPLEMENTATION"]),
            llm_server,
            gazebo,
            TimerAction(
                period=LaunchConfiguration("subsystem_delay_sec"),
                actions=[moveit],
            ),
            TimerAction(
                period=LaunchConfiguration("slam_delay_sec"),
                actions=[slam_evidence],
            ),
            TimerAction(
                period=LaunchConfiguration("navigation_delay_sec"),
                actions=[navigation],
            ),
            TimerAction(
                period=LaunchConfiguration("mission_delay_sec"),
                actions=[mission, speed_diagnostics],
            ),
            TimerAction(
                period=LaunchConfiguration("acceptance_delay_sec"),
                actions=[acceptance],
            ),
        ]
    )
