from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch
from launch_ros.actions import SetParameter


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "six_arm",
        package_name="mybot",
    ).to_moveit_configs()
    launch_description = generate_move_group_launch(moveit_config)
    launch_description.entities.insert(
        0,
        SetParameter(name="use_sim_time", value=True),
    )
    launch_description.entities.insert(
        1,
        SetParameter(
            name="trajectory_execution.allowed_start_tolerance",
            value=3.2,
        ),
    )
    return launch_description
