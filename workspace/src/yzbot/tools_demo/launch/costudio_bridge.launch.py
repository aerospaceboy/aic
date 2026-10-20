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

"""Expose current competition telemetry and legacy manual input to coStudio."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
from tools_demo.runtime_environment import configure_ros_transport


TOPIC_WHITELIST = [
    r"^/competition/(scene_markers|map)$",
    r"^/clock$", r"^/map$", r"^/amcl_pose$", r"^/plan$", r"^/local_plan$", r"^/received_global_plan$", r"^/global_costmap/costmap$", r"^/local_costmap/costmap$",
    r"^/competition/navigation/command_path$",
    r"^/competition/navigation/current_goal$",
    r"^/competition/(mission/state|navigation/state|status)$",
    r"^/competition/task/(question|problem|plan|counts|order|llm_status|parse_result)$",
    r"^/competition/(perception/targets|avoidance/state)$",
    r"^/competition/(gripper/state|placed_cubes)$",
    r"^/competition/obstacles/(obstacle_[12]|preview_moving_partition_[12])$",
    r"^/competition/dynamic_obstacles$",
]

CAMERA_STREAM_TOPICS = [
    r"^/camera/image_raw$",
    r"^/camera_rgbd/camera_rgbd/image_raw$",
    r"^/competition/perception/debug_image$",
]

ROBOT_MODEL_TOPICS = [
    r"^/tf$", r"^/tf_static$", r"^/joint_states$", r"^/robot_description$",
]


def _bridge_node(context):
    """Create a narrow coBridge endpoint for visualization and one input."""
    topics = list(TOPIC_WHITELIST)
    topics.extend(ROBOT_MODEL_TOPICS)
    camera_streams = LaunchConfiguration("camera_streams").perform(context)
    if camera_streams.strip().lower() in {"1", "true", "yes", "on"}:
        topics.extend(CAMERA_STREAM_TOPICS)
    return [
        Node(
            package="cobridge",
            executable="cobridge",
            name="cobridge",
            parameters=[
                {
                    "address": LaunchConfiguration("address"),
                    "port": ParameterValue(
                        LaunchConfiguration("port"), value_type=int
                    ),
                    "topic_whitelist": topics,
                    "param_whitelist": [r"^$"],
                    "service_whitelist": [r"^$"],
                    "client_topic_whitelist": [
                        r"^/competition/task/problem_chunk$",
                        r"^/competition/mission/start$",
                        r"^/competition/mission/cancel$",
                    ],
                    "capabilities": [
                        "connectionGraph", "assets", "clientPublish",
                    ],
                    "min_qos_depth": 1,
                    "max_qos_depth": 2,
                    "num_threads": 2,
                    "send_buffer_limit": 32_000_000,
                    "include_hidden": False,
                    "use_sim_time": ParameterValue(
                        LaunchConfiguration("use_sim_time"), value_type=bool
                    ),
                }
            ],
            output="screen",
        )
    ]


def generate_launch_description():
    configure_ros_transport(Path(get_package_share_directory("tools_demo")) / "config")
    return LaunchDescription(
        [
            DeclareLaunchArgument("address", default_value="0.0.0.0"),
            DeclareLaunchArgument("port", default_value="21274"),
            DeclareLaunchArgument("use_sim_time", default_value="True"),
            DeclareLaunchArgument("camera_streams", default_value="True"),
            OpaqueFunction(function=_bridge_node),
            Node(
                package="tools_demo",
                executable="scene_markers",
                name="competition_scene_markers",
                parameters=[{"use_sim_time": ParameterValue(
                    LaunchConfiguration("use_sim_time"), value_type=bool)}],
                output="screen",
            ),
        ]
    )
