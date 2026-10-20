"""Launch the competition RGB-D color target detector."""

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Create the perception launch description."""
    parameters = PathJoinSubstitution(
        [
            FindPackageShare("tools_demo"),
            "config",
            "perception.yaml",
        ]
    )
    detector = Node(
        package="tools_demo",
        executable="color_target_detector",
        name="competition_color_target_detector",
        parameters=[parameters, {"use_sim_time": True}],
        output="screen",
    )
    return LaunchDescription([detector])
