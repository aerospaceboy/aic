"""Move a Gazebo obstacle across the competition navigation corridor."""

from functools import partial
import json
import math
import time

from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import GetEntityState
from gazebo_msgs.srv import SetEntityState
from nav_msgs.msg import Odometry
import rclpy
from rclpy.client import Client
from rclpy.node import Node
from std_msgs.msg import String

from tools_demo.navigation_math import triangle_wave
from tools_demo.continuous_response import ContinuousResponse
from tools_demo.dynamic_motion import (
    PolylinePatrol,
    admit_proximity_guarded_step,
)


class MovingObstacle(Node):
    """Drive one Gazebo actor through one sensor-triggered crossing."""

    def __init__(self) -> None:
        super().__init__("competition_moving_obstacle")
        self.declare_parameter(
            "model_name", "competition_moving_obstacle"
        )
        self.declare_parameter("reference_frame", "world")
        self.declare_parameter("center_x", 1.0)
        self.declare_parameter("center_y", 0.0)
        self.declare_parameter("center_z", 0.30)
        self.declare_parameter("amplitude_x", 0.0)
        self.declare_parameter("amplitude_y", 0.75)
        self.declare_parameter("period_sec", 8.0)
        self.declare_parameter("phase_offset_sec", 0.0)
        self.declare_parameter("motion_mode", "continuous")
        # [0.0] disables the optional audited path and preserves the original
        # two actors' exact triangle-X/Y motion and parameters.
        self.declare_parameter("patrol_points", [0.0])
        self.declare_parameter("crossing_x", 1.0)
        self.declare_parameter("crossing_y", 0.0)
        self.declare_parameter("trigger_distance", 2.60)
        self.declare_parameter("trigger_minimum_speed", 0.05)
        self.declare_parameter("enter_duration_sec", 0.80)
        self.declare_parameter("minimum_block_duration_sec", 0.40)
        self.declare_parameter("maximum_block_duration_sec", 2.50)
        self.declare_parameter("response_distance", 1.30)
        self.declare_parameter("response_speed", 0.75)
        self.declare_parameter("response_lateral_distance", 0.20)
        self.declare_parameter("exit_duration_sec", 0.80)
        self.declare_parameter("update_rate_hz", 5.0)
        self.declare_parameter("truth_read_rate_hz", 5.0)
        self.declare_parameter("service_call_timeout_sec", 1.0)
        # Disabled by default so the original spawn-corridor movers retain
        # their exact motion.  The central patrols enable this guard to stop a
        # kinematic actor from moving through the robot footprint.
        self.declare_parameter("robot_clearance_stop_distance", 0.0)
        self.declare_parameter("robot_clearance_release_distance", 0.0)
        # After this long frozen, the guard admits steps that only increase the
        # distance to the base, so a base that cannot clear the hysteresis
        # cannot deadlock the corridor.
        self.declare_parameter("robot_clearance_freeze_timeout_sec", 5.0)
        # Competition runs are submitted after a variable startup delay.
        # Re-anchor the two spawn actors to a known patrol phase when a new
        # task plan arrives, so the first-two-response test is repeatable.
        self.declare_parameter("reset_phase_on_plan", False)
        # Separate from phase_offset_sec so the startup/world pose contract
        # is unchanged; this offset applies only after a competition plan.
        self.declare_parameter("mission_phase_offset_sec", 0.0)
        self.declare_parameter(
            "plan_topic", "/competition/task/plan"
        )
        self.declare_parameter(
            "state_topic", "/competition/obstacle/state"
        )

        self.model_name = str(self.get_parameter("model_name").value)
        self.reference_frame = str(
            self.get_parameter("reference_frame").value
        )
        self.center_x = float(self.get_parameter("center_x").value)
        self.center_y = float(self.get_parameter("center_y").value)
        self.center_z = float(self.get_parameter("center_z").value)
        self.amplitude_x = float(
            self.get_parameter("amplitude_x").value
        )
        self.amplitude_y = float(
            self.get_parameter("amplitude_y").value
        )
        self.period_sec = float(self.get_parameter("period_sec").value)
        self.phase_offset_sec = float(
            self.get_parameter("phase_offset_sec").value
        )
        self.mission_phase_offset_sec = float(
            self.get_parameter("mission_phase_offset_sec").value
        )
        self.motion_phase_offset_sec = self.phase_offset_sec
        self.motion_mode = str(
            self.get_parameter("motion_mode").value
        ).strip().lower()
        coordinates = list(self.get_parameter("patrol_points").value)
        self.patrol = (None if coordinates == [0.0] else
                       PolylinePatrol(coordinates,self.period_sec))
        self.crossing_x = float(self.get_parameter("crossing_x").value)
        self.crossing_y = float(self.get_parameter("crossing_y").value)
        self.trigger_distance = self._positive_parameter(
            "trigger_distance"
        )
        self.trigger_minimum_speed = self._nonnegative_parameter(
            "trigger_minimum_speed"
        )
        self.enter_duration_sec = self._positive_parameter(
            "enter_duration_sec"
        )
        self.minimum_block_duration_sec = self._nonnegative_parameter(
            "minimum_block_duration_sec"
        )
        self.maximum_block_duration_sec = self._positive_parameter(
            "maximum_block_duration_sec"
        )
        self.response_distance = self._positive_parameter(
            "response_distance"
        )
        self.response_speed = self._nonnegative_parameter("response_speed")
        self.response_lateral_distance = self._nonnegative_parameter(
            "response_lateral_distance"
        )
        self.exit_duration_sec = self._positive_parameter(
            "exit_duration_sec"
        )
        if self.motion_mode not in {"continuous", "triggered_crossing"}:
            raise ValueError(
                "motion_mode must be continuous or triggered_crossing"
            )
        if self.maximum_block_duration_sec < self.minimum_block_duration_sec:
            raise ValueError(
                "maximum_block_duration_sec cannot be shorter than minimum"
            )
        update_rate = float(
            self.get_parameter("update_rate_hz").value
        )
        truth_read_rate = float(
            self.get_parameter("truth_read_rate_hz").value
        )
        if update_rate <= 0.0:
            raise ValueError("update_rate_hz must be positive")
        if truth_read_rate <= 0.0:
            raise ValueError("truth_read_rate_hz must be positive")
        service_timeout = float(
            self.get_parameter("service_call_timeout_sec").value
        )
        if service_timeout <= 0.0:
            raise ValueError("service_call_timeout_sec must be positive")
        self.service_call_timeout_ns = int(service_timeout * 1_000_000_000)
        self.robot_clearance_stop_distance = self._nonnegative_parameter(
            "robot_clearance_stop_distance"
        )
        self.robot_clearance_release_distance = self._nonnegative_parameter(
            "robot_clearance_release_distance"
        )
        self.robot_clearance_freeze_timeout_sec = self._nonnegative_parameter(
            "robot_clearance_freeze_timeout_sec"
        )
        self.proximity_guard_paused_since = None
        if (
            self.robot_clearance_stop_distance > 0.0
            and self.robot_clearance_release_distance
            < self.robot_clearance_stop_distance
        ):
            raise ValueError(
                "robot_clearance_release_distance must be at least the "
                "stop distance"
            )
        self.update_period_sec = 1.0 / update_rate
        triangle_wave(0.0, self.period_sec)

        self.set_client: Client = self.create_client(
            SetEntityState,
            "/gazebo/set_entity_state",
        )
        self.get_client: Client = self.create_client(
            GetEntityState,
            "/gazebo/get_entity_state",
        )
        self.publisher = self.create_publisher(
            String,
            str(self.get_parameter("state_topic").value),
            10,
        )
        self.create_subscription(Odometry, "/odom", self.odom_callback, 20)
        self.reset_phase_on_plan = bool(
            self.get_parameter("reset_phase_on_plan").value
        )
        self.mission_task_id = None
        if self.reset_phase_on_plan:
            self.create_subscription(
                String,
                str(self.get_parameter("plan_topic").value),
                self.plan_callback,
                10,
            )
        self.robot_stamp_ns = None
        self.continuous_response = ContinuousResponse(
            self.trigger_distance, self.response_distance, self.response_speed)
        self.robot_x = None
        self.robot_y = None
        self.robot_speed = 0.0
        self.crossing_state = "waiting"
        self.crossing_elapsed_sec = 0.0
        self.response_observed = False
        self.response_kind = None
        self.motion_elapsed_sec = 0.0
        self.motion_started = False
        self.last_motion_position = None
        self.proximity_guard_paused = False
        self.pending_set = None
        self.pending_get = None
        self.pending_set_started_ns = 0
        self.pending_get_started_ns = 0
        self.last_service_warning_ns = 0
        self.timer = self.create_timer(1.0 / update_rate, self.tick)
        self.truth_timer = self.create_timer(
            1.0 / truth_read_rate,
            self.truth_tick,
        )

    def _positive_parameter(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"{name} must be positive and finite")
        return value

    def _nonnegative_parameter(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value < 0.0:
            raise ValueError(f"{name} must be nonnegative and finite")
        return value

    def odom_callback(self, message: Odometry) -> None:
        """Track whether the robot visibly reacts to the crossing actor."""
        self.robot_stamp_ns = self.get_clock().now().nanoseconds
        position = message.pose.pose.position
        velocity = message.twist.twist
        self.robot_x = float(position.x)
        self.robot_y = float(position.y)
        self.robot_speed = math.hypot(
            float(velocity.linear.x),
            float(velocity.linear.y),
        )

    def _robot_distance_to_crossing(self) -> float:
        if self.robot_x is None or self.robot_y is None:
            return float("inf")
        return math.hypot(
            self.robot_x - self.crossing_x,
            self.robot_y - self.crossing_y,
        )

    def _triggered_crossing_position(self) -> tuple:
        """Return a bounded park-cross-wait-retreat state-machine pose."""
        distance = self._robot_distance_to_crossing()
        if self.crossing_state == "waiting":
            if (
                distance <= self.trigger_distance
                and self.robot_speed >= self.trigger_minimum_speed
            ):
                self.crossing_state = "entering"
                self.crossing_elapsed_sec = 0.0
                self.get_logger().info(
                    f"crossing_triggered:{self.model_name} distance={distance:.2f}"
                )
            return self.center_x, self.center_y

        self.crossing_elapsed_sec += self.update_period_sec
        if self.crossing_state == "entering":
            fraction = min(
                self.crossing_elapsed_sec / self.enter_duration_sec,
                1.0,
            )
            x_value = self.center_x + fraction * (
                self.crossing_x - self.center_x
            )
            y_value = self.center_y + fraction * (
                self.crossing_y - self.center_y
            )
            if fraction >= 1.0:
                self.crossing_state = "blocking"
                self.crossing_elapsed_sec = 0.0
            return x_value, y_value

        if self.crossing_state == "blocking":
            lateral_response = (
                self.robot_y is not None
                and abs(self.robot_y - self.crossing_y)
                >= self.response_lateral_distance
            )
            speed_response = self.robot_speed <= self.response_speed
            reacted = distance <= self.response_distance and (
                lateral_response or speed_response
            )
            if reacted:
                self.response_observed = True
                self.response_kind = (
                    "detour" if lateral_response else "yield"
                )
            ready_to_leave = (
                self.crossing_elapsed_sec
                >= self.minimum_block_duration_sec
                and self.response_observed
            )
            timed_out = (
                self.crossing_elapsed_sec
                >= self.maximum_block_duration_sec
            )
            if ready_to_leave or timed_out:
                self.crossing_state = "leaving"
                self.crossing_elapsed_sec = 0.0
                self.get_logger().info(
                    f"crossing_retreat:{self.model_name} "
                    f"response={self.response_kind or 'bounded_timeout'}"
                )
            return self.crossing_x, self.crossing_y

        if self.crossing_state == "leaving":
            fraction = min(
                self.crossing_elapsed_sec / self.exit_duration_sec,
                1.0,
            )
            x_value = self.crossing_x + fraction * (
                self.center_x - self.crossing_x
            )
            y_value = self.crossing_y + fraction * (
                self.center_y - self.crossing_y
            )
            if fraction >= 1.0:
                self.crossing_state = "completed"
                self.crossing_elapsed_sec = 0.0
            return x_value, y_value

        return self.center_x, self.center_y

    def tick(self) -> None:
        """Send the next model pose if the prior request completed."""
        now_ns = self.get_clock().now().nanoseconds
        if self.pending_set is not None and not self.pending_set.done():
            if (
                now_ns - self.pending_set_started_ns
                <= self.service_call_timeout_ns
            ):
                return
            self.pending_set.cancel()
            self.pending_set = None
            self.get_logger().warn(
                "Gazebo set-entity-state service timed out; retrying"
            )
            return
        if not self.set_client.service_is_ready():
            if now_ns - self.last_service_warning_ns > 5_000_000_000:
                self.get_logger().warn(
                    "Waiting for Gazebo entity state services"
                )
                self.last_service_warning_ns = now_ns
            return

        if self.motion_mode == "triggered_crossing":
            candidate_x, candidate_y = self._triggered_crossing_position()
        else:
            candidate_x, candidate_y = self._continuous_position()
        request = SetEntityState.Request()
        request.state = EntityState()
        request.state.name = self.model_name
        request.state.reference_frame = self.reference_frame
        request.state.pose.position.x = candidate_x
        request.state.pose.position.y = candidate_y
        request.state.pose.position.z = self.center_z
        request.state.pose.orientation.w = 1.0
        self.pending_set = self.set_client.call_async(request)
        self.pending_set_started_ns = now_ns
        self.pending_set.add_done_callback(
            partial(
                self.set_response_callback,
                x=request.state.pose.position.x,
                y=request.state.pose.position.y,
            )
        )

    def plan_callback(self, message: String) -> None:
        """Re-anchor continuous patrol phase for one new competition plan."""
        if not self.reset_phase_on_plan:
            return
        try:
            payload = json.loads(message.data)
            task_id = str(payload.get("task_id", ""))
        except (TypeError, ValueError, json.JSONDecodeError):
            return
        if not task_id or task_id == self.mission_task_id:
            return
        self.mission_task_id = task_id
        self.motion_elapsed_sec = 0.0
        self.motion_phase_offset_sec = self.mission_phase_offset_sec
        self.motion_started = False
        self.last_motion_position = None
        self.proximity_guard_paused = False
        self.proximity_guard_paused_since = None
        self.response_observed = False
        self.response_kind = None
        self.continuous_response = ContinuousResponse(
            self.trigger_distance, self.response_distance,
            self.response_speed,
        )
        self.get_logger().info(
            "patrol_phase_reset task_id=" + task_id
        )

    def _continuous_position(self) -> tuple:
        """Retain the original triangle motion for non-competition use."""
        # Advance at most one nominal controller step per accepted update.
        # Absolute simulation time can jump ahead when Nav2/Gazebo starves
        # this Python timer; using it directly would teleport the model.
        candidate_elapsed = self.motion_elapsed_sec
        if self.motion_started:
            candidate_elapsed += self.update_period_sec
        patrol = getattr(self,"patrol",None)
        if patrol is not None:
            candidate = patrol.position(
                candidate_elapsed + self.motion_phase_offset_sec
            )
        else:
            wave = triangle_wave(
                candidate_elapsed + self.motion_phase_offset_sec,
                self.period_sec,
            )
            candidate = (
                self.center_x + self.amplitude_x * wave,
                self.center_y + self.amplitude_y * wave,
            )
        robot = (
            None
            if self.robot_x is None or self.robot_y is None
            else (self.robot_x, self.robot_y)
        )
        paused_sec = (
            0.0
            if self.proximity_guard_paused_since is None
            else time.monotonic() - self.proximity_guard_paused_since
        )
        admitted, paused = admit_proximity_guarded_step(
            self.last_motion_position,
            candidate,
            robot,
            self.robot_clearance_stop_distance,
            self.robot_clearance_release_distance,
            self.proximity_guard_paused,
            paused_sec=paused_sec,
            freeze_timeout_sec=self.robot_clearance_freeze_timeout_sec,
        )
        if paused:
            if self.proximity_guard_paused_since is None:
                self.proximity_guard_paused_since = time.monotonic()
        else:
            self.proximity_guard_paused_since = None
        self.proximity_guard_paused = paused
        if not admitted:
            return self.last_motion_position
        self.motion_started = True
        self.motion_elapsed_sec = candidate_elapsed
        self.last_motion_position = candidate
        return candidate

    def set_response_callback(
        self,
        future,
        x: float,
        y: float,
    ) -> None:
        """Record whether Gazebo accepted one motion command."""
        success = False
        try:
            response = future.result()
            success = bool(response.success)
        except Exception as exception:  # noqa: B902
            self.get_logger().error(
                f"Moving obstacle service call failed: {exception}"
            )
        self.pending_set = None
        if not success:
            self.publish_state(False, x, y, source="command")

    def truth_tick(self) -> None:
        """Sample Gazebo truth independently from the motion command loop."""
        now_ns = self.get_clock().now().nanoseconds
        if self.pending_get is not None and not self.pending_get.done():
            if (
                now_ns - self.pending_get_started_ns
                <= self.service_call_timeout_ns
            ):
                return
            self.pending_get.cancel()
            self.pending_get = None
            self.get_logger().warn(
                "Gazebo get-entity-state service timed out; retrying"
            )
            return
        if not self.get_client.service_is_ready():
            return
        request = GetEntityState.Request()
        request.name = self.model_name
        request.reference_frame = self.reference_frame
        self.pending_get = self.get_client.call_async(request)
        self.pending_get_started_ns = now_ns
        self.pending_get.add_done_callback(self.truth_response_callback)

    def truth_response_callback(self, future) -> None:
        """Publish the latest pose returned by Gazebo, not a cached command."""
        success = False
        x_value = self.center_x
        y_value = self.center_y
        try:
            response = future.result()
            success = bool(response.success)
            if success:
                position = response.state.pose.position
                x_value = float(position.x)
                y_value = float(position.y)
                header = getattr(response,"header",None)
                self.truth_stamp_sec = (None if header is None else
                                        header.stamp.sec+header.stamp.nanosec*1e-9)
        except Exception as exception:  # noqa: B902
            self.get_logger().error(
                f"Moving obstacle truth query failed: {exception}"
            )
        self.pending_get = None
        now_ns = self.get_clock().now().nanoseconds
        if (success and self.motion_mode == "continuous"
                and self.robot_stamp_ns is not None
                and 0 <= now_ns - self.robot_stamp_ns <= 750_000_000):
            distance = math.hypot(self.robot_x - x_value, self.robot_y - y_value)
            if self.continuous_response.observe(distance, self.robot_speed, now_ns / 1e9):
                self.response_observed = True
                self.response_kind = "yield"
        self.publish_state(success, x_value, y_value, source="gazebo")

    def publish_state(
        self,
        success: bool,
        x_value: float,
        y_value: float,
        source: str,
    ) -> None:
        """Publish one bounded latest-pose snapshot for monitors and GUI."""
        message = String()
        message.data = json.dumps(
            {
                "model": self.model_name,
                "success": success,
                "source": source,
                "stamp_sec": getattr(self,"truth_stamp_sec",None),
                "x": round(x_value, 3),
                "y": round(y_value, 3),
                "motion_state": (
                    "continuous_guarded_hold"
                    if self.motion_mode == "continuous"
                    and self.proximity_guard_paused
                    else "continuous"
                    if self.motion_mode == "continuous"
                    else self.crossing_state
                ),
                "continuous_response": self.continuous_response.evidence,
                "response_observed": self.response_observed,
                "response_kind": self.response_kind,
                "robot_speed": round(self.robot_speed, 3),
            },
            separators=(",", ":"),
            sort_keys=True,
        )
        self.publisher.publish(message)


def main(args=None) -> None:
    """Run the Gazebo moving obstacle node."""
    rclpy.init(args=args)
    node = MovingObstacle()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
