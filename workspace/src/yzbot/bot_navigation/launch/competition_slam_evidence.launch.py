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

"""Build an independent live SLAM map without disturbing Nav2's fixed map."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Create Cartographer nodes remapped to competition evidence topics."""
    share = get_package_share_directory("bot_navigation")
    use_sim_time = LaunchConfiguration("use_sim_time")
    configuration_directory = f"{share}/config"

    cartographer = Node(
        package="cartographer_ros",
        executable="cartographer_node",
        name="competition_slam_cartographer",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=[
            "-configuration_directory",
            configuration_directory,
            "-configuration_basename",
            "competition_slam.lua",
        ],
    )
    occupancy = Node(
        package="cartographer_ros",
        executable="cartographer_occupancy_grid_node",
        name="competition_slam_occupancy_grid",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=[
            "-resolution",
            "0.05",
            "-publish_period_sec",
            "1.0",
        ],
        remappings=[("map", "/competition/slam/map")],
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="True",
            ),
            cartographer,
            occupancy,
        ]
    )
