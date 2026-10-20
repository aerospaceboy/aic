"""Publish judge-visible clearance evidence for moving-obstacle avoidance."""

from __future__ import annotations

import json
import math
from typing import Dict, Optional, Tuple

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from std_msgs.msg import String

from tools_demo.dynamic_obstacle_filter import distance_to_box


def retained_qos() -> QoSProfile:
    """Return a reliable retained profile for avoidance evidence."""
    return QoSProfile(
        depth=10,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        reliability=ReliabilityPolicy.RELIABLE,
    )


class AvoidanceMonitor(Node):
    """Measure robot clearance from both moving Gazebo obstacles."""

    def __init__(self) -> None:
        super().__init__("competition_avoidance_monitor")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter(
            "obstacle_topics",
            [
                "/competition/obstacles/obstacle_1",
                "/competition/obstacles/obstacle_2",
                "/competition/obstacles/crossing_cart_1",
                "/competition/obstacles/crossing_cart_2",
            ],
        )
        self.declare_parameter(
            "state_topic",
            "/competition/avoidance/state",
        )
        self.declare_parameter("box_obstacles", [""])
        self.declare_parameter("additional_obstacles", [""])
        self.additional_obstacles = set(
            name for name in self.get_parameter("additional_obstacles").value if name)
        self.declare_parameter("circular_obstacles", [""])
        self.circle_radii = {}
        for specification in self.get_parameter("circular_obstacles").value:
            if not specification:
                continue
            name, radius = specification.split(":")
            radius = float(radius)
            if not math.isfinite(radius) or radius <= 0:
                raise ValueError("Circular obstacle radius must be finite and positive")
            self.circle_radii[name] = radius
        self.obstacle_boxes = {}
        for specification in self.get_parameter("box_obstacles").value:
            if not specification:
                continue
            name, sx, sy = specification.split(":")
            dimensions = (float(sx), float(sy))
            if min(dimensions) <= 0:
                raise ValueError("Box dimensions must be positive")
            self.obstacle_boxes[name] = dimensions
        self.declare_parameter("robot_radius", 0.28)
        self.declare_parameter("obstacle_radius", 0.18)
        # 0.28 m chassis + 0.18 m actor + 0.10 m clear air enforces the
        # competition's strict centre-distance requirement (>0.56 m).
        self.declare_parameter("safety_margin", 0.10)
        self.declare_parameter("expected_obstacle_count", 2)
        self.declare_parameter("encounter_distance", 1.50)
        self.declare_parameter("publish_rate_hz", 5.0)

        self.robot_radius = self._positive("robot_radius")
        self.obstacle_radius = self._positive("obstacle_radius")
        self.safety_margin = self._nonnegative("safety_margin")
        self.encounter_distance = self._positive("encounter_distance")
        self.expected_obstacle_count = int(
            self.get_parameter("expected_obstacle_count").value
        )
        if self.expected_obstacle_count < 2:
            raise ValueError("expected_obstacle_count must be at least two")
        publish_rate = self._positive("publish_rate_hz")
        self.required_distance = (
            self.robot_radius
            + self.obstacle_radius
            + self.safety_margin
        )
        self.robot: Optional[Tuple[float, float]] = None
        self.robot_speed = 0.0
        self.obstacles: Dict[str, Tuple[float, float]] = {}
        self.obstacle_updates: Dict[str, int] = {}
        self.minimum_clearance: Optional[float] = None
        self.minimum_event = None
        self.collision_detected = False
        self.encountered_obstacles = set()
        self.response_obstacles = set()
        self.response_kinds = {}
        self.sample_count = 0

        self.publisher = self.create_publisher(
            String,
            str(self.get_parameter("state_topic").value),
            retained_qos(),
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            self.odom_callback,
            20,
        )
        for topic in self.get_parameter("obstacle_topics").value:
            self.create_subscription(
                String,
                str(topic),
                self.obstacle_callback,
                20,
            )
        self.timer = self.create_timer(
            1.0 / publish_rate,
            self.publish_state,
        )

    def _positive(self, parameter: str) -> float:
        value = float(self.get_parameter(parameter).value)
        if value <= 0.0:
            raise ValueError(f"{parameter} must be positive")
        return value

    def _nonnegative(self, parameter: str) -> float:
        value = float(self.get_parameter(parameter).value)
        if value < 0.0:
            raise ValueError(f"{parameter} cannot be negative")
        return value

    def odom_callback(self, message: Odometry) -> None:
        """Store the latest planar robot pose and sample clearances."""
        position = message.pose.pose.position
        velocity = message.twist.twist
        self.robot = (float(position.x), float(position.y))
        self.robot_speed = math.hypot(
            float(velocity.linear.x),
            float(velocity.linear.y),
        )
        self.evaluate()

    def obstacle_callback(self, message: String) -> None:
        """Store one successful obstacle truth update."""
        try:
            payload = json.loads(message.data)
            model = str(payload["model"])
            x_value = float(payload["x"])
            y_value = float(payload["y"])
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            return
        if payload.get("success") is not True:
            return
        self.obstacles[model] = (x_value, y_value)
        if payload.get("response_observed") is True:
            self.response_obstacles.add(model)
            response_kind = payload.get("response_kind")
            if response_kind in {"yield", "detour"}:
                self.response_kinds[model] = response_kind
        self.obstacle_updates[model] = (
            self.obstacle_updates.get(model, 0) + 1
        )
        self.evaluate()

    def evaluate(self) -> None:
        """Accumulate minimum physical clearance for every known obstacle."""
        if self.robot is None or not self.obstacles:
            return
        robot_x, robot_y = self.robot
        for model, (obstacle_x, obstacle_y) in self.obstacles.items():
            center_distance = math.hypot(
                robot_x - obstacle_x,
                robot_y - obstacle_y,
            )
            dimensions = getattr(self, "obstacle_boxes", {}).get(model)
            clearance = (
                distance_to_box(robot_x, robot_y, obstacle_x, obstacle_y,
                                *dimensions) - self.robot_radius
                if dimensions else center_distance - (
                    self.robot_radius + getattr(self,"circle_radii",{}).get(
                        model,self.obstacle_radius))
            )
            if not hasattr(self, "minimum_clearance_by_model"):
                self.minimum_clearance_by_model = {}
            self.minimum_clearance_by_model[model] = min(
                self.minimum_clearance_by_model.get(model, math.inf), clearance)
            if (
                self.minimum_clearance is None
                or clearance < self.minimum_clearance
            ):
                self.minimum_clearance = clearance
                self.minimum_event = {
                    "model": model,
                    "robot": [round(robot_x, 3), round(robot_y, 3)],
                    "obstacle": [
                        round(obstacle_x, 3),
                        round(obstacle_y, 3),
                    ],
                }
            if clearance <= 0.0:
                self.collision_detected = True
            if center_distance <= self.encounter_distance:
                self.encountered_obstacles.add(model)
            self.sample_count += 1

    def publish_state(self) -> None:
        """Publish two independently auditable spawn-gate events."""
        seen = len(self.obstacles)
        gate_members = {
            "spawn_avoidance_1": {"competition_moving_obstacle_1"},
            "spawn_avoidance_2": {"competition_moving_obstacle_2"},
        }
        avoidance_events = sorted(
            name
            for name, members in gate_members.items()
            if (
                members.issubset(self.encountered_obstacles)
                and members.issubset(self.response_obstacles)
                and all(self.response_kinds.get(m) in {"yield", "detour"}
                        for m in members)
            )
        )
        original_gate_events = list(avoidance_events)
        additional = sorted(
            name for name in (set(getattr(self, "obstacle_boxes", {})) |
                              getattr(self, "additional_obstacles", set()))
            if name in self.encountered_obstacles
            and name in self.response_obstacles
            and self.response_kinds.get(name) in {"yield", "detour"})
        avoidance_events.extend("dynamic_avoidance_" + name
                                for name in additional)
        safe = (
            seen >= self.expected_obstacle_count
            and self.sample_count > 0
            and not self.collision_detected
            and self.minimum_clearance is not None
            and self.minimum_clearance >= self.safety_margin
        )
        message = String()
        message.data = json.dumps(
            {
                "schema_version": "1.0",
                "obstacles_seen": seen,
                "obstacle_updates": dict(self.obstacle_updates),
                "sample_count": self.sample_count,
                "minimum_clearance": (
                    round(self.minimum_clearance, 4)
                    if self.minimum_clearance is not None
                    else None
                ),
                "minimum_event": self.minimum_event,
                "minimum_clearance_by_model": getattr(
                    self, "minimum_clearance_by_model", {}),
                "box_obstacles": getattr(self, "obstacle_boxes", {}),
                "circular_obstacles": getattr(self, "circle_radii", {}),
                "additional_avoidance_events": additional,
                "original_gate_events": original_gate_events,
                "original_pair_demonstrated": len(original_gate_events) == 2,
                "avoidance_policy": "two_distinct_effective_movers",
                "obstacles": {
                    model: [round(position[0], 3), round(position[1], 3)]
                    for model, position in self.obstacles.items()
                },
                "required_clearance": round(
                    self.safety_margin,
                    4,
                ),
                "collision_detected": self.collision_detected,
                "encountered_obstacles": sorted(
                    self.encountered_obstacles
                ),
                "response_obstacles": sorted(self.response_obstacles),
                "response_kinds": dict(self.response_kinds),
                "robot_speed": round(self.robot_speed, 3),
                "avoidance_events": avoidance_events,
                "avoidance_event_count": len(avoidance_events),
                "avoidance_demonstrated": len(avoidance_events) >= 2,
                "safe": safe,
            },
            separators=(",", ":"),
            sort_keys=True,
        )
        self.publisher.publish(message)


def main(args=None) -> None:
    """Run the moving-obstacle clearance monitor."""
    rclpy.init(args=args)
    node = AvoidanceMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
