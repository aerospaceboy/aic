import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from ament_index_python.packages import get_package_prefix
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
 
from launch.event_handlers import OnProcessExit
 
import xacro
 
import re
def remove_comments(text):
    pattern = r'<!--(.*?)-->'
    return re.sub(pattern, '', text, flags=re.DOTALL)


def client_after_spawn(event, context, *, clients):
    """Open the GUI only after the running world accepted the robot."""
    if event.returncode != 0:
        return []
    return list(clients)
 
def generate_launch_description():
    robot_name_in_model = 'six_arm'
    package_name = 'mybot_description'
    urdf_name = "originbot_with_rgbd_gazebo_arm.xacro"
    world_file_path = 'offic_room.world'
    #world_file_path = 'cloister.world'
 
    pkg_share = FindPackageShare(package=package_name).find(package_name) 
    urdf_model_path = os.path.join(pkg_share, f'urdf/{urdf_name}')
    world_file_path = os.path.join(pkg_share, f'worlds/{world_file_path}')
    print(f'[competition_world] loading Gazebo world: {world_file_path}')
 
    headless_argument = DeclareLaunchArgument(
        'headless',
        default_value='False',
        description='Start gzserver without the Gazebo graphical client',
    )
    debug_gui_argument = DeclareLaunchArgument(
        'debug_gui',
        default_value='True',
        description='Use the low-refresh, client-only Gazebo debug GUI',
    )
    robot_x_argument = DeclareLaunchArgument(
        'robot_x',
        default_value='15.0',
        description='Robot start X in the 2026 office competition world',
    )
    robot_y_argument = DeclareLaunchArgument(
        'robot_y',
        default_value='-2.0',
        description='Robot start Y in the 2026 office competition world',
    )
    robot_yaw_argument = DeclareLaunchArgument(
        'robot_yaw',
        default_value='3.14159',
        description='Robot start yaw in radians',
    )

    debug_gui_plugin = os.path.join(
        get_package_prefix('gazebo_debug_gui'),
        'lib',
        'libgazebo_debug_gui_plugin.so',
    )
    standard_gui_condition = IfCondition(PythonExpression([
        'not ', LaunchConfiguration('headless'),
        ' and not ', LaunchConfiguration('debug_gui'),
    ]))
    debug_gui_condition = IfCondition(PythonExpression([
        'not ', LaunchConfiguration('headless'),
        ' and ', LaunchConfiguration('debug_gui'),
    ]))

    # A simultaneous `gazebo` server/client start can leave gzclient at the
    # splash screen while ROS services are healthy. Give launch ownership of
    # both processes and gate the client on successful world/entity creation.
    start_gazebo_cmd = ExecuteProcess(
        cmd=['gzclient', '--verbose'],
        condition=standard_gui_condition,
        emulate_tty=True,
        output='screen')
    start_gazebo_debug_cmd = ExecuteProcess(
        cmd=['gzclient', '--verbose',
             '--gui-client-plugin', debug_gui_plugin],
        condition=debug_gui_condition,
        emulate_tty=True,
        output='screen')
    start_gzserver_cmd = ExecuteProcess(
        cmd=['gzserver', '--verbose',
             world_file_path,
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so'],
        output='screen')
 
 
    # 因为 urdf文件中有一句 $(find mybot) 需要用xacro进行编译一下才行
    xacro_file = urdf_model_path
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    # params = {'robot_description': doc.toxml()}
    params = {'robot_description': remove_comments(doc.toxml())} # type: ignore
 
    # 启动了robot_state_publisher节点后，该节点会发布 robot_description 话题，话题内容是模型文件urdf的内容
    # 并且会订阅 /joint_states 话题，获取关节的数据，然后发布tf和tf_static话题.
    # 这些节点、话题的名称可不可以自定义？
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': True}, params, {"publish_frequency":15.0}],
        output='screen'
    )
 
    # Launch the robot, 通过robot_description话题进行模型内容获取从而在gazebo中生成模型
    spawn_entity_cmd = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=[
            '-entity', robot_name_in_model,
            '-topic', 'robot_description',
            '-x', LaunchConfiguration('robot_x'),
            '-y', LaunchConfiguration('robot_y'),
            '-Y', LaunchConfiguration('robot_yaw'),
        ],
        output='screen')

    open_client_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_entity_cmd,
            on_exit=lambda event, context: client_after_spawn(
                event, context,
                clients=(start_gazebo_cmd, start_gazebo_debug_cmd)),
        )
    )
 
 
    # # Launch the robot, 这个是通过传递文件路径来在gazebo里生成模型.此时要求urdf文件里面没有xacro的语句
    # spawn_entity_cmd = Node(
    #     package='gazebo_ros', 
    #     executable='spawn_entity.py',
    #     arguments=['-entity', robot_name_in_model,  '-file', urdf_model_path ], output='screen')
    
    # node_robot_state_publisher = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     arguments=[urdf_model_path],
    #     parameters=[{'use_sim_time': True}],
    #     output='screen'
    # )
 
    # gazebo在加载urdf时，根据urdf的设定，会启动一个joint_states节点
    # 关节状态发布器
    load_joint_state_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--controller-manager', '/controller_manager'],
        output='screen'
    )
 
    # 路径执行控制器，也就是那个action
    load_joint_trajectory_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller',
                   '--controller-manager', '/controller_manager'],
        output='screen'
    )
    load_joint_state_controller_gripper = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller',
                   '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # 监听 spawn_entity_cmd，当其退出（完全启动）时，启动load_joint_state_controller
    close_evt1 =  RegisterEventHandler( 
            event_handler=OnProcessExit(
                target_action=spawn_entity_cmd,
                on_exit=[load_joint_state_controller],
            )
    )
    # 监听 load_joint_state_controller，当其退出（完全启动）时，启动load_joint_trajectory_controller
    close_evt2 = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_joint_trajectory_controller],
            )
    )
    # 监听 load_joint_trajectory_controller，当其退出（完全启动）时，启动load_joint_state_controller_gripper
    close_evt3 = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_trajectory_controller,
                on_exit=[load_joint_state_controller_gripper],
            )
    )
    
    ld = LaunchDescription()
 
    ld.add_action(headless_argument)
    ld.add_action(debug_gui_argument)
    ld.add_action(robot_x_argument)
    ld.add_action(robot_y_argument)
    ld.add_action(robot_yaw_argument)
    ld.add_action(close_evt1)
    ld.add_action(close_evt2)
    ld.add_action(close_evt3)
    ld.add_action(open_client_after_spawn)
 
    ld.add_action(start_gzserver_cmd)
    ld.add_action(node_robot_state_publisher)
    ld.add_action(spawn_entity_cmd)
 
    return ld
