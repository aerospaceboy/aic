"""Start one clean Gazebo/Nav2 process for a dynamic matrix case."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch(package: str, filename: str) -> PythonLaunchDescriptionSource:
    return PythonLaunchDescriptionSource(
        f"{get_package_share_directory(package)}/launch/{filename}"
    )


def generate_launch_description():
    """Create an isolated navigation-only case with a coherent start pose."""
    arguments = [
        DeclareLaunchArgument("robot_x"),
        DeclareLaunchArgument("robot_y"),
        DeclareLaunchArgument("robot_yaw", default_value="0.0"),
        DeclareLaunchArgument("moving_obstacle", default_value="False"),
    ]
    gazebo = IncludeLaunchDescription(
        _launch("mybot", "gazebo_world.launch.py"),
        launch_arguments={
            "headless": "True",
            "robot_x": LaunchConfiguration("robot_x"),
            "robot_y": LaunchConfiguration("robot_y"),
            "robot_yaw": LaunchConfiguration("robot_yaw"),
        }.items(),
    )
    navigation = IncludeLaunchDescription(
        _launch("bot_navigation", "competition_navigation.launch.py"),
        launch_arguments={
            "moving_obstacle": LaunchConfiguration("moving_obstacle"),
            "use_sim_time": "True",
        }.items(),
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
                "danger_radius": 0.55,
                "caution_radius": 1.20,
                "cone_half_angle_deg": 50.0,
                "obstacle_timeout_sec": 0.75,
            },
        ],
        output="screen",
    )
    return LaunchDescription(
        arguments
        + [
            gazebo,
            TimerAction(
                # Let the large world and robot finish spawning before Nav2
                # composable nodes compete for CPU on the two-core VM.
                period=25.0,
                actions=[navigation, safety_limiter],
            ),
        ]
    )
