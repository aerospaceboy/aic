"""Record low-overhead, phase-aware competition motion statistics."""

from __future__ import annotations

from collections import defaultdict
import json
import math
from pathlib import Path
import statistics
import time
from typing import Dict, Optional

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


SPEED_BINS = (0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.8, 1.0, 1.2, 1.5, 1.8, 2.1, 2.4)
DOCK_PHASES = {
    "direct_docking",
    "zone_egress",
    "departure_aisle",
    "pre_navigation_alignment",
    "hazard_escape",
    "bounded_recovery",
    "obstacle_reverse_resume",
}
TERMINAL_MISSION_STATES = {"completed", "failed", "canceled", "timed_out"}


def retained_qos() -> QoSProfile:
    """Return the competition state-topic QoS."""
    return QoSProfile(
        depth=10,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        reliability=ReliabilityPolicy.RELIABLE,
    )

def live_qos() -> QoSProfile:
    """Return live-only QoS for per-process mission timing."""
    return QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
    )


def classify_motion(
    mission_state: str,
    phase: str,
    linear_speed: float,
    angular_speed: float,
    front_clearance: Optional[float],
    smoother_speed: float,
    collision_speed: float,
    final_speed: float,
) -> str:
    """Classify one motion sample without changing any control behavior."""
    if not mission_state.startswith("navigating"):
        return "manipulation_or_wait"
    if phase in DOCK_PHASES:
        return "docking_or_direct"
    if smoother_speed > 0.08 and collision_speed < 0.88 * smoother_speed:
        return "collision_slow"
    if collision_speed > 0.08 and final_speed < 0.88 * collision_speed:
        return "truth_safety_slow"
    if angular_speed >= 0.75:
        return "sharp_turn"
    if angular_speed >= 0.20:
        return "ordinary_turn"
    if front_clearance is not None and front_clearance >= 1.50:
        return "open_straight"
    if linear_speed >= 0.03:
        return "constrained_straight"
    return "stationary"


def percentile(values, fraction: float) -> float:
    """Return a deterministic nearest-rank percentile."""
    if not values:
        return 0.0
    ordered = sorted(values)
    index = min(len(ordered) - 1, max(0, math.ceil(fraction * len(ordered)) - 1))
    return float(ordered[index])


class SpeedProfileRecorder(Node):
    """Sample the existing command chain and odometry at five hertz."""

    def __init__(self) -> None:
        super().__init__("competition_speed_profile")
        self.declare_parameter("output_path", "/tmp/competition_speed_profile.json")
        self.declare_parameter("sample_period_sec", 0.2)
        self.output_path = Path(str(self.get_parameter("output_path").value))
        self.sample_period = float(self.get_parameter("sample_period_sec").value)
        if not 0.1 <= self.sample_period <= 1.0:
            raise ValueError("sample_period_sec must be in [0.1, 1.0]")

        self.actual_linear = 0.0
        self.actual_angular = 0.0
        self.front_clearance: Optional[float] = None
        self.commands: Dict[str, float] = defaultdict(float)
        self.navigation = {}
        self.mission = {}
        self.task_id = None
        self.started_monotonic: Optional[float] = None
        self.last_sample_monotonic: Optional[float] = None
        self.maximum_sample_gap_sec = 0.0
        self.delayed_sample_count = 0
        self.finished = False
        self.samples = defaultdict(list)
        self.duration = defaultdict(float)
        self.distance = defaultdict(float)
        self.histogram = defaultdict(lambda: [0.0] * (len(SPEED_BINS) - 1))

        self.create_subscription(Odometry, "/odom", self.odom_callback, 20)
        self.create_subscription(LaserScan, "/scan", self.scan_callback, 10)
        for topic in (
            "/cmd_vel_nav",
            "/cmd_vel",
            "/cmd_vel_collision_safe",
            "/cmd_vel_safe",
        ):
            self.create_subscription(
                Twist,
                topic,
                lambda message, name=topic: self.command_callback(name, message),
                10,
            )
        self.create_subscription(
            String,
            "/competition/navigation/state",
            self.navigation_callback,
            retained_qos(),
        )
        self.create_subscription(
            String,
            "/competition/mission/state",
            self.mission_callback,
            live_qos(),
        )
        self.create_timer(self.sample_period, self.sample)

    def odom_callback(self, message: Odometry) -> None:
        twist = message.twist.twist
        self.actual_linear = math.hypot(twist.linear.x, twist.linear.y)
        self.actual_angular = abs(twist.angular.z)

    def scan_callback(self, message: LaserScan) -> None:
        half_angle = math.radians(55.0)
        candidates = []
        for index, value in enumerate(message.ranges):
            angle = message.angle_min + index * message.angle_increment
            if (
                abs(angle) <= half_angle
                and math.isfinite(value)
                and message.range_min <= value <= message.range_max
            ):
                candidates.append(float(value))
        self.front_clearance = min(candidates) if candidates else None

    def command_callback(self, topic: str, message: Twist) -> None:
        self.commands[topic] = abs(float(message.linear.x))

    def navigation_callback(self, message: String) -> None:
        try:
            self.navigation = json.loads(message.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            return

    def mission_callback(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            return
        self.mission = payload
        total = int(payload.get("total", 0) or 0)
        incoming_task = payload.get("task_id")
        state = str(payload.get("state", ""))
        if (
            total > 0
            and self.started_monotonic is None
            and state not in TERMINAL_MISSION_STATES
        ):
            self.task_id = incoming_task
            self.started_monotonic = time.monotonic()
            self.last_sample_monotonic = self.started_monotonic
            self.finished = False
            self.get_logger().info(f"speed_profile_start task_id={self.task_id}")
        if (
            self.started_monotonic is not None
            and not self.finished
            and incoming_task == self.task_id
            and state in TERMINAL_MISSION_STATES
        ):
            self.finished = True
            self.write_report(state)

    def sample(self) -> None:
        if self.started_monotonic is None or self.finished:
            return
        now = time.monotonic()
        previous = (
            self.last_sample_monotonic
            if self.last_sample_monotonic is not None else now
        )
        gap = max(0.0, now - previous)
        self.maximum_sample_gap_sec = max(self.maximum_sample_gap_sec, gap)
        if gap > 0.5:
            self.delayed_sample_count += 1
        # Do not infer several seconds of motion from one stale odometry
        # sample. Report the unobserved wall time separately instead.
        dt = min(0.5, gap)
        self.last_sample_monotonic = now
        phase = str(self.navigation.get("phase", ""))
        category = classify_motion(
            str(self.mission.get("state", "")),
            phase,
            self.actual_linear,
            self.actual_angular,
            self.front_clearance,
            self.commands["/cmd_vel"],
            self.commands["/cmd_vel_collision_safe"],
            self.commands["/cmd_vel_safe"],
        )
        self.samples[category].append(self.actual_linear)
        self.duration[category] += dt
        self.distance[category] += self.actual_linear * dt
        for index, (lower, upper) in enumerate(zip(SPEED_BINS, SPEED_BINS[1:])):
            if lower <= self.actual_linear < upper or (
                index == len(SPEED_BINS) - 2 and self.actual_linear >= upper
            ):
                self.histogram[category][index] += dt
                break

    def write_report(self, terminal_state: str) -> None:
        now = time.monotonic()
        started = self.started_monotonic
        elapsed = max(0.0, now - started) if started is not None else 0.0
        categories = {}
        for category in sorted(self.samples):
            values = self.samples[category]
            moving = [value for value in values if value >= 0.03]
            categories[category] = {
                "duration_sec": round(self.duration[category], 2),
                "distance_m": round(self.distance[category], 2),
                "moving_mean_mps": round(statistics.fmean(moving), 3) if moving else 0.0,
                "p50_mps": round(percentile(moving, 0.50), 3),
                "p90_mps": round(percentile(moving, 0.90), 3),
                "sample_count": len(values),
                "histogram_seconds": {
                    f"{lower:.1f}-{upper:.1f}": round(seconds, 2)
                    for lower, upper, seconds in zip(
                        SPEED_BINS,
                        SPEED_BINS[1:],
                        self.histogram[category],
                    )
                },
            }
        report = {
            "schema_version": "1.0",
            "task_id": self.task_id,
            "terminal_state": terminal_state,
            "mission": self.mission,
            "elapsed_sec": round(elapsed, 2),
            "timing_coverage": {
                "sampled_wall_sec": round(sum(self.duration.values()), 3),
                "unobserved_wall_sec": round(
                    max(0.0, elapsed - sum(self.duration.values())), 3,
                ),
                "maximum_sample_gap_sec": round(self.maximum_sample_gap_sec, 3),
                "delayed_sample_count": self.delayed_sample_count,
            },
            "speed_bins_mps": list(SPEED_BINS),
            "categories": categories,
        }
        self.output_path.write_text(
            json.dumps(report, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        summary = ", ".join(
            f"{name}={data['duration_sec']:.1f}s/{data['moving_mean_mps']:.2f}mps"
            for name, data in categories.items()
        )
        self.get_logger().info(f"speed_profile_complete {summary}")
        self.get_logger().info(f"speed_profile_path={self.output_path}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SpeedProfileRecorder()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
