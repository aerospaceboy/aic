"""Execute one real Nav2 goal and fail quickly on sustained no progress."""

import argparse
from collections import Counter
import json
import math
import time

from action_msgs.msg import GoalStatus
from dwb_msgs.msg import LocalPlanEvaluation
from lifecycle_msgs.srv import GetState
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node


class NavigationMatrixProbe(Node):
    """Observe odometry while one NavigateToPose action is active."""

    def __init__(self, x: float, y: float, yaw: float, timeout: float):
        super().__init__("navigation_matrix_probe")
        self.target = (x, y, yaw)
        self.timeout = timeout
        self.started = time.monotonic()
        self.last_progress = self.started
        self.last_pose = None
        self.translation_pose = None
        self.start_pose = None
        self.result = None
        self.goal_handle = None
        self.last_commands = {}
        self.command_history = []
        self.last_command_sample = {}
        self.dwb_evaluation = None
        self.obstacle_history = []
        self.last_obstacle_sample = {}
        self.closest_scan = None
        self.plan_points = []
        self.client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.lifecycle_clients = [
            self.create_client(GetState, "/controller_server/get_state"),
            self.create_client(GetState, "/planner_server/get_state"),
            self.create_client(GetState, "/bt_navigator/get_state"),
        ]
        self.create_subscription(Odometry, "/odom", self._odom, 20)
        self.create_subscription(LaserScan, "/scan", self._scan, 20)
        self.create_subscription(Path, "/plan", self._plan, 10)
        self.create_subscription(
            LocalPlanEvaluation, "/evaluation", self._evaluation, 10
        )
        for topic in (
            "/competition/obstacles/obstacle_1",
            "/competition/obstacles/obstacle_2",
            "/competition/obstacles/preview_moving_partition_1",
            "/competition/obstacles/preview_moving_partition_2",
        ):
            self.create_subscription(
                String,
                topic,
                lambda message, name=topic: self._obstacle(name, message),
                20,
            )
        for topic in (
            "/cmd_vel_nav",
            "/cmd_vel",
            "/cmd_vel_collision_safe",
            "/cmd_vel_safe",
        ):
            self.create_subscription(
                Twist,
                topic,
                lambda message, name=topic: self._command(name, message),
                20,
            )
        self.create_timer(0.25, self._watchdog)

    def _odom(self, message: Odometry) -> None:
        pose = message.pose.pose
        yaw = 2.0 * math.atan2(pose.orientation.z, pose.orientation.w)
        current = (pose.position.x, pose.position.y, yaw)
        if self.start_pose is None:
            self.start_pose = current
        self.last_pose = current
        if self.translation_pose is None:
            self.translation_pose = current
            self.last_progress = time.monotonic()
            return
        moved = math.hypot(
            current[0] - self.translation_pose[0],
            current[1] - self.translation_pose[1],
        )
        if moved >= 0.04:
            self.translation_pose = current
            self.last_progress = time.monotonic()

    def _scan(self, message: LaserScan) -> None:
        candidates = [
            (value, index)
            for index, value in enumerate(message.ranges)
            if math.isfinite(value) and message.range_min <= value <= message.range_max
        ]
        if candidates:
            distance, index = min(candidates)
            self.closest_scan = {
                "range": round(distance, 3),
                "angle": round(message.angle_min + index * message.angle_increment, 3),
            }

    def _plan(self, message: Path) -> None:
        self.plan_points = [
            (pose.pose.position.x, pose.pose.position.y) for pose in message.poses
        ]
        # A recovery replan starts a fresh bounded FollowPath attempt.
        self.last_progress = time.monotonic()
        if self.last_pose is not None:
            self.translation_pose = self.last_pose

    def _command(self, topic: str, message: Twist) -> None:
        command = [
            round(message.linear.x, 4),
            round(message.angular.z, 4),
        ]
        self.last_commands[topic] = command
        now = time.monotonic()
        if now - self.last_command_sample.get(topic, 0.0) >= 1.0:
            self.last_command_sample[topic] = now
            self.command_history.append(
                {
                    "elapsed": round(now - self.started, 2),
                    "topic": topic,
                    "command": command,
                    "pose": self.last_pose,
                }
            )
            self.command_history = self.command_history[-120:]

    def _evaluation(self, message: LocalPlanEvaluation) -> None:
        if not message.twists or message.best_index >= len(message.twists):
            return
        best = message.twists[message.best_index]
        legal = [score for score in message.twists if score.total >= 0.0]
        moving = [
            score
            for score in legal
            if abs(score.traj.velocity.x) > 0.001
            or abs(score.traj.velocity.theta) > 0.001
        ]
        best_moving = min(moving, key=lambda score: score.total) if moving else None
        illegal_counts = Counter(
            critic.name
            for candidate in message.twists
            if candidate.total < 0.0
            for critic in candidate.scores
            if critic.raw_score < 0.0
        )

        def summarize(score):
            if score is None:
                return None
            return {
                "velocity": [
                    round(score.traj.velocity.x, 4),
                    round(score.traj.velocity.theta, 4),
                ],
                "total": round(score.total, 4),
                "scores": [
                    {
                        "name": critic.name,
                        "raw": round(critic.raw_score, 4),
                        "scale": round(critic.scale, 4),
                        "weighted": round(critic.raw_score * critic.scale, 4),
                    }
                    for critic in score.scores
                ],
            }

        self.dwb_evaluation = {
            "best_index": message.best_index,
            "candidate_count": len(message.twists),
            "legal_count": len(legal),
            "illegal_counts": dict(sorted(illegal_counts.items())),
            "best": summarize(best),
            "best_moving": summarize(best_moving),
        }

    def _obstacle(self, topic: str, message: String) -> None:
        try:
            state = json.loads(message.data)
            x_value = round(float(state["x"]), 3)
            y_value = round(float(state["y"]), 3)
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            return
        now = time.monotonic()
        if now - self.last_obstacle_sample.get(topic, 0.0) < 1.0:
            return
        self.last_obstacle_sample[topic] = now
        self.obstacle_history.append(
            {
                "elapsed": round(now - self.started, 2),
                "topic": topic,
                "position": [x_value, y_value],
                "robot": self.last_pose,
            }
        )
        self.obstacle_history = self.obstacle_history[-120:]

    def send(self) -> None:
        ready_deadline = time.monotonic() + 60.0
        while rclpy.ok() and self.start_pose is None:
            if time.monotonic() >= ready_deadline:
                self.result = {"status": "odom_unavailable"}
                return
            rclpy.spin_once(self, timeout_sec=0.25)
        for client in self.lifecycle_clients:
            if not self._wait_for_active(client, ready_deadline):
                self.result = {"status": "nav2_inactive"}
                return
        remaining = max(0.0, ready_deadline - time.monotonic())
        if not self.client.wait_for_server(timeout_sec=remaining):
            self.result = {"status": "server_unavailable"}
            return
        # Readiness time belongs to the harness, not to navigation timing.
        self.started = time.monotonic()
        self.last_progress = self.started
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = self.target[0]
        goal.pose.pose.position.y = self.target[1]
        goal.pose.pose.orientation.z = math.sin(self.target[2] / 2.0)
        goal.pose.pose.orientation.w = math.cos(self.target[2] / 2.0)
        future = self.client.send_goal_async(goal)
        future.add_done_callback(self._accepted)

    def _wait_for_active(self, client, deadline: float) -> bool:
        while rclpy.ok() and time.monotonic() < deadline:
            if not client.wait_for_service(timeout_sec=0.5):
                continue
            future = client.call_async(GetState.Request())
            remaining = max(0.0, deadline - time.monotonic())
            rclpy.spin_until_future_complete(
                self, future, timeout_sec=min(2.0, remaining)
            )
            if future.done() and future.result() is not None:
                if future.result().current_state.label == "active":
                    return True
            time.sleep(0.25)
        return False

    def _accepted(self, future) -> None:
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.result = {"status": "rejected"}
            return
        result = self.goal_handle.get_result_async()
        result.add_done_callback(self._finished)

    def _finished(self, future) -> None:
        status = future.result().status
        names = {
            GoalStatus.STATUS_SUCCEEDED: "succeeded",
            GoalStatus.STATUS_ABORTED: "aborted",
            GoalStatus.STATUS_CANCELED: "canceled",
        }
        self.result = {"status": names.get(status, f"status_{status}")}

    def _watchdog(self) -> None:
        now = time.monotonic()
        if self.result is not None:
            return
        if now - self.started >= self.timeout:
            self.result = {"status": "timeout"}
        elif self.goal_handle is not None and now - self.last_progress >= 18.0:
            self.result = {"status": "stalled"}
        if self.result is not None and self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()

    def report(self) -> dict:
        payload = dict(self.result or {"status": "unknown"})
        path_near = None
        if self.last_pose is not None and self.plan_points:
            index = min(
                range(len(self.plan_points)),
                key=lambda item: math.hypot(
                    self.plan_points[item][0] - self.last_pose[0],
                    self.plan_points[item][1] - self.last_pose[1],
                ),
            )
            ahead = min(index + 5, len(self.plan_points) - 1)
            path_near = {
                "index": index,
                "point": self.plan_points[index],
                "ahead": self.plan_points[ahead],
                "total": len(self.plan_points),
            }
        payload.update(
            {
                "elapsed_sec": round(time.monotonic() - self.started, 2),
                "start": self.start_pose,
                "end": self.last_pose,
                "target": self.target,
                "closest_scan": self.closest_scan,
                "commands": self.last_commands,
                "command_history": self.command_history,
                "dwb_evaluation": self.dwb_evaluation,
                "obstacle_history": self.obstacle_history,
                "path_near": path_near,
            }
        )
        return payload


def main(args=None) -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--x", type=float, required=True)
    parser.add_argument("--y", type=float, required=True)
    parser.add_argument("--yaw", type=float, default=0.0)
    parser.add_argument("--timeout", type=float, default=90.0)
    parsed, ros_args = parser.parse_known_args(args)
    rclpy.init(args=ros_args)
    node = NavigationMatrixProbe(parsed.x, parsed.y, parsed.yaw, parsed.timeout)
    node.send()
    while rclpy.ok() and node.result is None:
        rclpy.spin_once(node, timeout_sec=0.25)
    report = node.report()
    print("MATRIX_RESULT=" + json.dumps(report, sort_keys=True))
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(0 if report["status"] == "succeeded" else 1)


if __name__ == "__main__":
    main()
