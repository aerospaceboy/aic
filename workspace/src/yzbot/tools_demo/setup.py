from glob import glob

from setuptools import find_packages
from setuptools import setup


package_name = "tools_demo"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/config", glob("config/*.json")),
        ("share/" + package_name + "/config", glob("config/*.xml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/web", glob("web/*.html")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="sunrise",
    maintainer_email="sunrise@todo.todo",
    description="Competition task, perception, and navigation helpers",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "simulation_probe = tools_demo.simulation_probe:main",
            "scene_markers = tools_demo.scene_markers:main",
            "chat = tools_demo.chat_interactive:main",
            "chat_ros = tools_demo.chat_interactive_ros:main",
            "task_question = tools_demo.task_question_node:main",
            "task_parser = tools_demo.task_parser_node:main",
            "legacy_costudio_status = tools_demo.legacy_costudio_status:main",
            "color_target_detector = "
            "tools_demo.color_target_detector:main",
            "task_navigator = tools_demo.task_navigator:main",
            "moving_obstacle = tools_demo.moving_obstacle:main",
            "dynamic_obstacles = tools_demo.dynamic_obstacle_node:main",
            "payload_scan_mask = tools_demo.payload_scan_mask:main",
            "sweep_band_keepout = tools_demo.sweep_band_keepout:main",
            "avoidance_monitor = tools_demo.avoidance_monitor:main",
            "safety_limiter = tools_demo.safety_limiter:main",
            "velocity_selector = tools_demo.velocity_selector:main",
            "mission_coordinator = tools_demo.mission_coordinator:main",
            "competition_dashboard = "
            "tools_demo.competition_dashboard:main",
            "competition_acceptance = "
            "tools_demo.competition_acceptance:main",
            "navigation_matrix_probe = "
            "tools_demo.navigation_matrix_probe:main",
            "competition_case_runner = "
            "tools_demo.competition_case_runner:main",
            "speed_profile_recorder = "
            "tools_demo.speed_profile_recorder:main",
        ],
    },
)
