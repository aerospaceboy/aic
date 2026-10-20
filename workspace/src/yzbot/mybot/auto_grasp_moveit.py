#!/usr/bin/env python3

"""
Safely verify one Gazebo attach, carry, and detach cycle.

This stage intentionally uses the already-tested joint trajectory controller.
The cube must already be inside the gripper's safe grasp window. Arbitrary
world-pose planning with MoveIt is a later stage.
"""

import math
import time
from typing import Dict, List, Optional, Tuple

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from gazebo_msgs.srv import GetEntityState
from linkattacher_msgs.srv import AttachLink, DetachLink
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty
from trajectory_msgs.msg import JointTrajectoryPoint


class SafeAutoGrasper(Node):
    JOINT_NAMES = [
        'joint1',
        'joint2',
        'joint3',
        'joint4',
        'joint5',
        'joint6',
    ]

    def __init__(self) -> None:
        super().__init__('safe_auto_grasper')

        self.robot_name = 'six_arm'
        self.hand_link_name = 'link6'
        self.declare_parameter(
            'cube_names',
            [
                'red_cube_1',
                'red_cube_2',
                'red_cube_3',
                'red_cube_4',
                'red_cube_5',
            ],
        )
        self.cube_names = list(
            self.get_parameter('cube_names').get_parameter_value().string_array_value
        )

        # The tested cube center is 45 mm along link6's positive Z axis.
        self.target_z = 0.045
        self.xy_tolerance = 0.020
        self.z_min = 0.025
        self.z_max = 0.065

        self.joint_positions: Dict[str, float] = {}

        self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_states_callback,
            10,
        )

        self.get_state_client = self.create_client(
            GetEntityState,
            '/gazebo/get_entity_state',
        )
        self.attach_client = self.create_client(AttachLink, '/ATTACHLINK')
        self.detach_client = self.create_client(DetachLink, '/DETACHLINK')
        self.unpause_client = self.create_client(Empty, '/unpause_physics')
        self.trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory',
        )

    def _joint_states_callback(self, msg: JointState) -> None:
        for name, position in zip(msg.name, msg.position):
            self.joint_positions[name] = position

    def wait_until_ready(self, timeout_sec: float = 30.0) -> bool:
        deadline = time.monotonic() + timeout_sec

        services = [
            (self.get_state_client, '/gazebo/get_entity_state'),
            (self.attach_client, '/ATTACHLINK'),
            (self.detach_client, '/DETACHLINK'),
            (self.unpause_client, '/unpause_physics'),
        ]
        for client, service_name in services:
            remaining = deadline - time.monotonic()
            if remaining <= 0.0 or not client.wait_for_service(
                timeout_sec=remaining
            ):
                self.get_logger().error(f'等待服务超时：{service_name}')
                return False

        remaining = deadline - time.monotonic()
        if remaining <= 0.0 or not self.trajectory_client.wait_for_server(
            timeout_sec=remaining
        ):
            self.get_logger().error('等待机械臂轨迹控制器超时')
            return False

        return True

    def wait_for_joint_states(self, timeout_sec: float = 5.0) -> bool:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if all(name in self.joint_positions for name in self.JOINT_NAMES):
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().error('等待 /joint_states 超时')
        return False

    def _wait_for_future(self, future, timeout_sec: float):
        rclpy.spin_until_future_complete(
            self,
            future,
            timeout_sec=timeout_sec,
        )
        if not future.done():
            return None
        try:
            return future.result()
        except Exception as exc:  # rclpy forwards service/action exceptions here
            self.get_logger().error(f'异步请求失败：{exc}')
            return None

    def get_relative_pose(self, model_name: str):
        request = GetEntityState.Request()
        request.name = model_name
        request.reference_frame = (
            f'{self.robot_name}::{self.hand_link_name}'
        )

        response = self._wait_for_future(
            self.get_state_client.call_async(request),
            timeout_sec=5.0,
        )
        if response is None or not response.success:
            self.get_logger().warn(f'无法查询模型位姿：{model_name}')
            return None
        return response.state.pose

    def find_cube_in_grasp_window(self) -> Optional[Tuple[str, object]]:
        valid_candidates = []

        for name in self.cube_names:
            pose = self.get_relative_pose(name)
            if pose is None:
                continue

            x = pose.position.x
            y = pose.position.y
            z = pose.position.z
            in_window = (
                abs(x) <= self.xy_tolerance
                and abs(y) <= self.xy_tolerance
                and self.z_min <= z <= self.z_max
            )
            score = math.sqrt(
                x * x + y * y + (z - self.target_z) ** 2
            )

            self.get_logger().info(
                f'{name} 相对 link6：x={x:.4f}, y={y:.4f}, '
                f'z={z:.4f}, 安全窗口={in_window}'
            )
            if in_window:
                valid_candidates.append((score, name, pose))

        if not valid_candidates:
            self.get_logger().error(
                '没有红色方块位于夹爪安全窗口。'
                '请先把方块放到 link6 坐标系 (0, 0, 0.045) 附近。'
            )
            return None

        _, name, pose = min(valid_candidates, key=lambda item: item[0])
        return name, pose

    def attach(self, cube_name: str) -> bool:
        request = AttachLink.Request()
        request.model1_name = self.robot_name
        request.link1_name = self.hand_link_name
        request.model2_name = cube_name
        request.link2_name = 'link'

        response = self._wait_for_future(
            self.attach_client.call_async(request),
            timeout_sec=5.0,
        )
        if response is None:
            return False

        if response.success:
            self.get_logger().info(response.message)
        else:
            self.get_logger().error(response.message)
        return response.success

    def detach(self, cube_name: str) -> bool:
        request = DetachLink.Request()
        request.model1_name = self.robot_name
        request.link1_name = self.hand_link_name
        request.model2_name = cube_name
        request.link2_name = 'link'

        response = self._wait_for_future(
            self.detach_client.call_async(request),
            timeout_sec=5.0,
        )
        if response is None:
            return False

        if response.success:
            self.get_logger().info(response.message)
        else:
            self.get_logger().error(response.message)
        return response.success

    def unpause_physics(self) -> bool:
        response = self._wait_for_future(
            self.unpause_client.call_async(Empty.Request()),
            timeout_sec=5.0,
        )
        if response is None:
            self.get_logger().error('解除 Gazebo 暂停失败')
            return False
        self.get_logger().info('Gazebo 物理仿真已运行')
        return True

    def current_arm_positions(self) -> Optional[List[float]]:
        if not all(name in self.joint_positions for name in self.JOINT_NAMES):
            self.get_logger().error('当前关节位置不完整')
            return None
        return [self.joint_positions[name] for name in self.JOINT_NAMES]

    def move_arm(self, positions: List[float], duration_sec: int) -> bool:
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(self.JOINT_NAMES)

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=duration_sec, nanosec=0)
        goal.trajectory.points = [point]

        goal_handle = self._wait_for_future(
            self.trajectory_client.send_goal_async(goal),
            timeout_sec=5.0,
        )
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('机械臂轨迹目标被拒绝')
            return False

        wrapped_result = self._wait_for_future(
            goal_handle.get_result_async(),
            timeout_sec=float(duration_sec + 10),
        )
        if wrapped_result is None:
            self.get_logger().error('等待机械臂轨迹结果超时')
            return False

        result = wrapped_result.result
        if result.error_code != 0:
            self.get_logger().error(
                f'机械臂轨迹执行失败：{result.error_string}'
            )
            return False
        return True

    def run_once(self) -> bool:
        if not self.wait_until_ready():
            return False

        candidate = self.find_cube_in_grasp_window()
        if candidate is None:
            return False
        cube_name, _ = candidate
        self.get_logger().info(f'选择安全窗口内的方块：{cube_name}')

        attached = False
        try:
            if not self.attach(cube_name):
                return False
            attached = True

            time.sleep(1.0)
            attached_pose = self.get_relative_pose(cube_name)
            if attached_pose is None:
                return False

            # The recommended test places the cube while Gazebo is paused.
            # Unpausing an already-running simulation is harmless.
            if not self.unpause_physics():
                return False
            time.sleep(1.0)

            if not self.wait_for_joint_states():
                return False
            start_positions = self.current_arm_positions()
            if start_positions is None:
                return False

            carry_positions = list(start_positions)
            # Apply a small relative change while staying inside joint2 limits.
            delta = -0.10 if carry_positions[1] > 2.20 else 0.10
            carry_positions[1] += delta

            self.get_logger().info(
                f'开始搬运：joint2 相对移动 {delta:+.2f} rad'
            )
            if not self.move_arm(carry_positions, duration_sec=10):
                return False

            carried_pose = self.get_relative_pose(cube_name)
            if carried_pose is None:
                return False

            drift = math.sqrt(
                (carried_pose.position.x - attached_pose.position.x) ** 2
                + (carried_pose.position.y - attached_pose.position.y) ** 2
                + (carried_pose.position.z - attached_pose.position.z) ** 2
            )
            self.get_logger().info(f'搬运后相对位置漂移：{drift:.9f} m')
            if drift > 0.001:
                self.get_logger().error('吸附相对位置漂移超过 1 mm')
                return False

            if not self.detach(cube_name):
                return False
            attached = False

            self.get_logger().info('方块已释放，机械臂开始回到初始位置')
            if not self.move_arm(start_positions, duration_sec=10):
                return False

            self.get_logger().info('单次安全抓取测试成功')
            return True
        finally:
            if attached:
                self.get_logger().warn('流程异常结束，尝试释放当前方块')
                self.detach(cube_name)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SafeAutoGrasper()
    try:
        success = node.run_once()
        if not success:
            node.get_logger().error('单次安全抓取测试失败')
    except KeyboardInterrupt:
        node.get_logger().warn('用户终止测试')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
