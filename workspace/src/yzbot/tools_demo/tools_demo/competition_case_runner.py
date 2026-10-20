"""Run one competition task with direct, low-bandwidth safety checks."""

from __future__ import annotations

import json
import sys
import time

from geometry_msgs.msg import Twist
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from std_msgs.msg import String


def retained_qos() -> QoSProfile:
    """Return reliable transient-local QoS used by mission state topics."""
    return QoSProfile(
        depth=10,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        reliability=ReliabilityPolicy.RELIABLE,
    )


class CompetitionCaseRunner(Node):
    """Use one DDS participant for preflight, execution, and evidence."""

    def __init__(self) -> None:
        super().__init__("competition_case_runner")
        self.declare_parameter(
            "question",
            "Move 3 red cubes to zone A and 2 blue cubes to zone C.",
        )
        self.declare_parameter("timeout_sec", 700.0)
        self.declare_parameter("collision_check_period_sec", 5.0)
        self.declare_parameter("robot_radius", 0.28)
        self.declare_parameter("obstacle_radius", 0.18)
        self.declare_parameter("minimum_center_distance", 0.56)

        self.question_publisher = self.create_publisher(
            String,
            "/competition/task/question",
            retained_qos(),
        )
        self.safe_velocity_publisher = self.create_publisher(
            Twist,
            "/cmd_vel",
            10,
        )
        self.create_subscription(
            Twist,
            "/cmd_vel_safe",
            self.safe_velocity_callback,
            10,
        )
        self.create_subscription(
            String,
            "/competition/mission/state",
            self.mission_callback,
            retained_qos(),
        )
        self.create_subscription(
            String,
            "/competition/avoidance/state",
            self.avoidance_callback,
            retained_qos(),
        )
        self.navigation_client = ActionClient(
            self,
            NavigateToPose,
            "/navigate_to_pose",
        )
        self.collision_client = self.create_client(
            GetState,
            "/collision_monitor/get_state",
        )
        self.safe_velocity_seen = False
        self.mission = None
        self.avoidance = None
        self.task_seen = False
        self.collision_faults = []

    def safe_velocity_callback(self, _message: Twist) -> None:
        self.safe_velocity_seen = True

    def mission_callback(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            return
        if int(payload.get("total", 0)) > 0:
            self.task_seen = True
        self.mission = payload

    def avoidance_callback(self, message: String) -> None:
        try:
            self.avoidance = json.loads(message.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            return

    def spin_until(self, predicate, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            if predicate():
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        return bool(predicate())

    def collision_state(self, timeout_sec: float = 5.0) -> str:
        if not self.collision_client.wait_for_service(
            timeout_sec=timeout_sec
        ):
            return "service_unavailable"
        future = self.collision_client.call_async(GetState.Request())
        if not self.spin_until(future.done, timeout_sec):
            return "service_timeout"
        result = future.result()
        if result is None:
            return "service_failed"
        return str(result.current_state.label)

    def preflight(self) -> bool:
        if not self.navigation_client.wait_for_server(timeout_sec=20.0):
            self.get_logger().error("PREFLIGHT_NAV_ACTION=false")
            return False
        self.get_logger().info("PREFLIGHT_NAV_ACTION=true")
        state = self.collision_state()
        self.get_logger().info(f"PREFLIGHT_COLLISION_STATE={state}")
        if state != "active":
            return False
        deadline = time.monotonic() + 10.0
        # Collision Monitor suppresses an all-zero command, so a zero probe
        # cannot prove that the complete output chain is alive. Use a tiny
        # rotation-only command: it produces an observable safe output with
        # no translation and less than 0.1 rad motion over this preflight.
        probe = Twist()
        probe.angular.z = 0.01
        while time.monotonic() < deadline and not self.safe_velocity_seen:
            self.safe_velocity_publisher.publish(probe)
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info(
            f"PREFLIGHT_SAFETY_CHAIN={str(self.safe_velocity_seen).lower()}"
        )
        if not self.safe_velocity_seen:
            return False
        state = self.collision_state()
        self.get_logger().info(
            f"PREFLIGHT_COLLISION_STATE_AFTER_CHAIN={state}"
        )
        return state == "active"

    def publish_task(self) -> None:
        """Publish the question exactly once.

        The old five-shot loop made the parser answer the same question five
        times with fresh LLM samples. The coordinator latched the first plan,
        so a wrong first sample (2 red / 3 blue for a 1 red / 4 blue question)
        drove the whole round to a silent zero (20261014T072436Z-fvs9Wr).
        """
        message = String()
        message.data = str(self.get_parameter("question").value)
        self.question_publisher.publish(message)
        rclpy.spin_once(self, timeout_sec=0.2)
        self.get_logger().info(f"TASK_PUBLISHED={message.data}")

    def run_to_terminal(self) -> bool:
        timeout = float(self.get_parameter("timeout_sec").value)
        check_period = float(
            self.get_parameter("collision_check_period_sec").value
        )
        deadline = time.monotonic() + timeout
        next_check = time.monotonic() + check_period
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if (
                self.task_seen
                and self.mission is not None
                and self.mission.get("state")
                in ("completed", "failed", "canceled", "timed_out")
            ):
                return True
            if time.monotonic() >= next_check:
                state = self.collision_state()
                if state != "active":
                    self.collision_faults.append(state)
                    self.get_logger().error(
                        f"RUNTIME_COLLISION_STATE={state}"
                    )
                next_check = time.monotonic() + check_period
        return False

    def evidence_is_safe(self) -> bool:
        if not self.avoidance:
            self.get_logger().error("AVOIDANCE_EVIDENCE=missing")
            return False
        clearance = self.avoidance.get("minimum_clearance")
        if clearance is None:
            self.get_logger().error("AVOIDANCE_EVIDENCE=no_samples")
            return False
        robot_radius = float(self.get_parameter("robot_radius").value)
        obstacle_radius = float(
            self.get_parameter("obstacle_radius").value
        )
        required_surface = float(
            self.get_parameter("minimum_center_distance").value
        ) - robot_radius - obstacle_radius
        required = float(
            self.get_parameter("minimum_center_distance").value
        )
        self.get_logger().info(
            "AVOIDANCE_EVIDENCE="
            + json.dumps(self.avoidance, separators=(",", ":"))
        )
        self.get_logger().info(
            f"MINIMUM_SURFACE_CLEARANCE={float(clearance):.4f}; "
            f"required_surface_clearance={required - robot_radius - obstacle_radius:.4f}"
        )
        return (
            not self.avoidance.get("collision_detected", False)
            and float(clearance) > required_surface
            and self.avoidance.get("safe") is True
            and self.avoidance.get("avoidance_demonstrated") is True
            and len(set(self.avoidance.get("avoidance_events", []))) >= 2
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    runner = CompetitionCaseRunner()
    success = runner.preflight()
    terminal = False
    if success:
        runner.publish_task()
        terminal = runner.run_to_terminal()
    mission = runner.mission or {}
    runner.get_logger().info(
        "MISSION_TERMINAL=" + json.dumps(mission, separators=(",", ":"))
    )
    # Keep diagnostic safety evidence even when the runner times out.
    # Acceptance still separately requires a completed terminal mission.
    evidence_safe = runner.evidence_is_safe()
    success = (
        success
        and terminal
        and mission.get("state") == "completed"
        and int(mission.get("completed", -1)) == 5
        and int(mission.get("skipped", -1)) == 0
        and not runner.collision_faults
        and evidence_safe
    )
    runner.get_logger().info(
        f"CASE_ACCEPTED={str(success).lower()} "
        f"collision_faults={runner.collision_faults}"
    )
    runner.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
