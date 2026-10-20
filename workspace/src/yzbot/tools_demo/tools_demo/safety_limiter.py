"""
Velocity safety limiter using Gazebo-truth moving-obstacle positions.

The local costmap relies on laser scans that are occasionally dropped when
the VM is CPU-bound, so the robot can drive into a moving obstacle before
the costmap sees it. This node is the deterministic backstop: it watches
the final /cmd_vel stream and the obstacle truth topics, and when a moving
obstacle is close and roughly in front of the robot it scales the command
down (stopping entirely inside the danger radius). The planar_move plugin
subscribes to /cmd_vel_safe instead of /cmd_vel.
"""

from __future__ import annotations

import json
import math
from typing import Dict, Optional, Tuple

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from tools_demo.navigation_math import corridor_lane_release_clear
from tools_demo.navigation_math import EAST_LANE_RELEASE_DISTANCE
from tools_demo.navigation_math import CORRIDOR_TRANSIT_SPEED

from tools_demo.safety_math import compute_scale


class SafetyLimiter(Node):
    """Gate /cmd_vel against the nearest forward moving obstacle."""

    def __init__(self) -> None:
        super().__init__("competition_safety_limiter")
        self.declare_parameter("cmd_vel_in", "/cmd_vel")
        self.declare_parameter("cmd_vel_out", "/cmd_vel_safe")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("danger_radius", 0.75)
        self.declare_parameter("caution_radius", 1.60)
        self.declare_parameter("cone_half_angle_deg", 65.0)
        self.declare_parameter("obstacle_timeout_sec", 0.75)
        # A moving obstacle sweeps along y on a fixed x lane. A base that
        # stops inside that lane band waits for the returning mover to hit
        # it. Campaign policy_20261001T233650 (GUI 48 rounds) collided 4
        # times plus 2 unsafe clearances, always with obstacle_1 on x=8.5
        # while the robot sat at x 8.13..8.48; every one of those rounds
        # had the truth guard zero the commanded speed first.
        self.declare_parameter("swept_band_half_width", 0.60)
        self.declare_parameter("swept_band_transit_speed", CORRIDOR_TRANSIT_SPEED)
        self.declare_parameter("swept_band_commit_sec", 6.0)
        self.declare_parameter(
            "obstacle_topics",
            [
                "/competition/obstacles/obstacle_1",
                "/competition/obstacles/obstacle_2",
                "/competition/obstacles/crossing_cart_1",
                "/competition/obstacles/crossing_cart_2",
            ],
        )
        self.base_hold = False

        self.danger = float(self.get_parameter("danger_radius").value)
        self.caution = float(self.get_parameter("caution_radius").value)
        self.cone_half = math.radians(
            float(self.get_parameter("cone_half_angle_deg").value)
        )
        self.obstacle_timeout = float(
            self.get_parameter("obstacle_timeout_sec").value
        )
        self.swept_band_half = float(
            self.get_parameter("swept_band_half_width").value
        )
        self.swept_band_speed = float(
            self.get_parameter("swept_band_transit_speed").value
        )
        self.swept_band_commit = float(
            self.get_parameter("swept_band_commit_sec").value
        )
        if not 0.0 < self.danger < self.caution:
            raise ValueError("danger_radius must be < caution_radius")
        self.robot: Optional[Tuple[float, float, float]] = None
        self.obstacles: Dict[str, Tuple[float, float, int]] = {}
        self.obstacle_previous_y: Dict[str, float] = {}
        self.east_gate_open = False
        self.west_lane_open = False
        self.swept_band_since_ns = None

        self.publisher = self.create_publisher(
            Twist,
            str(self.get_parameter("cmd_vel_out").value),
            10,
        )
        self.create_subscription(
            Twist,
            str(self.get_parameter("cmd_vel_in").value),
            self.cmd_vel_callback,
            10,
        )
        self.create_subscription(
            Bool,
            "/competition/base_hold",
            self.base_hold_callback,
            10,
        )
        self.create_timer(0.05, self.publish_base_hold)
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

    def odom_callback(self, message: Odometry) -> None:
        """Cache the latest base pose."""
        pose = message.pose.pose
        q = pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self.robot = (pose.position.x, pose.position.y, yaw)

    def obstacle_callback(self, message: String) -> None:
        """Store one successful Gazebo-truth obstacle position."""
        try:
            payload = json.loads(message.data)
            model = str(payload["model"])
            x_value = float(payload["x"])
            y_value = float(payload["y"])
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            return
        if payload.get("success") is not True:
            return
        previous = self.obstacles.get(model)
        if previous is not None:
            self.obstacle_previous_y[model] = previous[1]
        self.obstacles[model] = (
            x_value,
            y_value,
            self.get_clock().now().nanoseconds,
        )

    def scale_for_obstacles(self, motion_sign: float) -> float:
        """Return the minimum scale for fresh obstacles in travel direction."""
        now = self.get_clock().now().nanoseconds
        timeout_ns = int(self.obstacle_timeout * 1e9)
        fresh = {
            model: (x_value, y_value)
            for model, (x_value, y_value, stamp) in self.obstacles.items()
            if 0 <= now - stamp <= timeout_ns
        }
        return compute_scale(
            self.robot,
            fresh,
            self.danger,
            self.caution,
            self.cone_half,
            motion_sign,
        )

    def west_lane_blocked(self, motion_sign: float) -> bool:
        """Yield before the second corridor lane until its phase is seen."""
        # The first corridor actor has had a lane gate since the spawn
        # corridor work; the second one had none, so a re-planned route could
        # drive straight through its patrol band. On 2026-10-06 matrix case
        # 3rB_2bA the base collided there (robot (5.357,-0.592) against actor
        # (5.0,-0.575), clearance -0.1025 m) while the mission still reported
        # 5/5. Mirror the same rule: release only while the actor is observed
        # retreating south below -0.65, and stay open while it stays south of
        # -0.55.
        if self.robot is None or motion_sign <= 0.0:
            return False
        x_value, y_value, yaw = self.robot
        actor_name = "competition_moving_obstacle_2"
        lane_x = 5.0
        if x_value < 3.0:
            self.west_lane_open = False
        if not -2.00 <= y_value <= 1.30:
            # Outside the lateral reach of the patrol (centre y=-0.35).
            return False
        approach_east = (
            lane_x + 0.80 <= x_value <= lane_x + 1.55
            and math.cos(yaw) < -0.4
        )
        approach_west = (
            lane_x - 1.55 <= x_value <= lane_x - 0.80
            and math.cos(yaw) > 0.4
        )
        if not (approach_east or approach_west):
            return False
        actor = self.obstacles.get(actor_name)
        now_ns = self.get_clock().now().nanoseconds
        actor_fresh = actor is not None and 0 <= now_ns - actor[2] <= int(
            self.obstacle_timeout * 1e9
        )
        previous_y = self.obstacle_previous_y.get(actor_name)
        # This corridor's route passes ~0.4 m south of the actor's southern
        # extreme, so the phase that releases the eastern gate - the actor
        # retreating south - is the worst one here: the actor turns around
        # 0.3 m from the base's path. On 2026-10-06 case 3rB_2bA that
        # release met the base at (5.0,-0.80) against (5.20,-1.10) for
        # -0.1039 m clearance, and waiting for the far extreme instead costs
        # 2-6 s per crossing (case 4bB_1rA 312.8 s, over the 300 s bound).
        # Judge the lane by separation from the base's own travel line:
        # cross while the actor is at least 0.60 m clear in y AND moving
        # further away, and keep the latch while it stays 0.40 m clear.
        # That window is ~5 s of every 8 s patrol instead of ~1 s.
        separation = 0.0
        moving_away = False
        if actor is not None and previous_y is not None:
            separation = abs(actor[1] - y_value)
            delta = actor[1] - previous_y
            moving_away = (
                abs(delta) >= 0.01
                and (actor[1] - y_value) * delta > 0.0
            )
        if self.west_lane_open:
            if actor_fresh and separation >= 0.40:
                return False
            self.west_lane_open = False
        if actor_fresh and separation >= 0.60 and moving_away:
            self.west_lane_open = True
            self.get_logger().info(
                "west mover gate released clear of the base lane "
                f"separation={separation:.2f}"
            )
            return False
        return True

    def east_gate_blocked(self, motion_sign: float) -> bool:
        """Yield east of mover 1 until its southbound phase is observed."""
        if self.robot is None or motion_sign <= 0.0:
            return False
        x_value, y_value, yaw = self.robot
        if x_value < 8.0:
            self.east_gate_open = False
        # Wait only east of the mover-1 sweep (x~8.5). Stopping inside
        # x=9.0 is only 0.50 m from the patrol: after the 0.28 m base and
        # 0.18 m actor radii it leaves 0.04 m clearance. Keep the entire
        # wait/re-lock band at least 0.80 m east, including phase expiry.
        # A wider y band catches more approach paths before commitment.
        if not (
            9.3 <= x_value <= 9.75
            and -1.2 <= y_value <= 1.2
            and math.cos(yaw) < -0.4
        ):
            return False
        actor = self.obstacles.get("competition_moving_obstacle_1")
        now_ns = self.get_clock().now().nanoseconds
        actor_fresh = actor is not None and 0 <= now_ns - actor[2] <= int(
            self.obstacle_timeout * 1e9
        )
        previous_y = self.obstacle_previous_y.get(
            "competition_moving_obstacle_1")
        # Release on separation from the base's own travel line, not on the
        # actor's phase. The phase rule opened only once mover 1 reached its
        # southern extreme, leaving ~3.3 s before the actor swept back over
        # the crossing, and the pursuit controller keeps slowing the base for
        # the actor ahead (26 "detected collision ahead" in
        # distributed48-r4 case 4bB_1rA). That base was still at
        # (8.931,0.446) when the actor returned to +0.10 - clearance
        # 0.0929 m, below the scored 0.10 m. Separation is the quantity that
        # protects the crossing and it adapts to the route's own y: releasing
        # at 0.80 m hands the base ~5.7 s instead of ~3.3 s of clear lane.
        separation = 0.0
        moving_away = False
        if actor is not None and previous_y is not None:
            separation = abs(actor[1] - y_value)
            delta = actor[1] - previous_y
            moving_away = (
                abs(delta) >= 0.01
                and (actor[1] - y_value) * delta > 0.0
            )
        # Never demand more separation than this patrol can offer, or a route
        # that crosses straight over the lane would wait forever: mover 1
        # sweeps y in [-0.80, 0.10].
        # Floor 0.75: at 0.60 a base that had drifted INTO the patrol band
        # (2026-10-06 case 4bA_1rC left the corridor at y=0.45, waited at
        # y=-0.09) was released with only 0.71 m of separation and the actor
        # swept back through it (-0.1461 m). Holding instead of releasing is
        # the safe answer: the navigator's own stall bound fails the leg and
        # the coordinator retries from a different approach.
        # Floor 1.00 even when the patrol cannot offer it: with 0.75 a
        # base that had drifted to y=0.10 was released at 0.81 m and the
        # actor passed at 0.0892 m surface clearance (2026-10-06 case
        # 2rB_3bA, below the 0.10 m requirement). Holding instead is the
        # safe answer: the navigator's stall bound fails that leg and the
        # coordinator retries from another approach.
        release = EAST_LANE_RELEASE_DISTANCE
        # 1.00 m reproduces the validated phase rule for the usual route, which
        # crosses lane 1 at y ~ 0.35-0.70 north of the patrol: the actor is
        # then south of -0.65 (0.90 m of the latch) exactly when the old rule
        # opened. A 0.80 m release was tried on 2026-10-06 and let the base
        # start while the actor was still inside the sweeper's reach: the
        # guard then stopped it inside the lane and the actor passed at
        # 0.0049 m surface clearance (case 4bB_1rA, 0 of 5 cubes delivered).
        # The same rule now also covers routes that pass south of the patrol,
        # which the phase rule opened at the worst possible moment.
        if self.east_gate_open:
            # Relock only while the base is still well east of the actor.
            # Stopping after entry would park it in the swept lane.
            if actor_fresh and separation >= release - 0.10:
                return False
            self.east_gate_open = False
        if (actor_fresh and corridor_lane_release_clear(
                y_value, actor[1], moving_away, release)):
            self.east_gate_open = True
            self.get_logger().info(
                "east mover gate released clear of the base lane "
                f"separation={separation:.2f}"
            )
            return False
        return True

    def base_hold_callback(self, message: Bool) -> None:
        """Enable or release the manipulation-time final velocity gate."""
        self.base_hold = bool(message.data)
        if self.base_hold:
            self.publisher.publish(Twist())

    def publish_base_hold(self) -> None:
        """Continuously clear velocity retained by the planar plugin."""
        if self.base_hold:
            self.publisher.publish(Twist())

    def _swept_band_transit(self, message: Twist) -> Optional[Twist]:
        """Keep the base moving when it is stuck inside a swept band.
        The guard answers "is an obstacle near and ahead", which is the
        right question for a static obstacle because stopping is safe.
        A mover sweeping a lane is different: a base that stops inside
        the band simply waits for the mover to return and hit it. When
        the guard would zero the command while the base is inside a live
        lane band, transit instead - keep the upstream direction at a
        bounded speed so the band is cleared - and give up after
        swept_band_commit_sec so a wedged base cannot drive forever.
        """
        if self.robot is None:
            return None
        x_value, y_value, _ = self.robot
        if abs(message.linear.x) < 0.03:
            return None
        now_ns = self.get_clock().now().nanoseconds
        timeout_ns = int(self.obstacle_timeout * 1e9)
        inside = False
        for model, entry in self.obstacles.items():
            if "moving_obstacle" not in model:
                continue
            obstacle_x, obstacle_y, stamp = entry
            if abs(y_value - obstacle_y) > self.caution:
                continue
            if not 0 <= now_ns - stamp <= timeout_ns:
                continue
            if abs(x_value - obstacle_x) <= self.swept_band_half:
                inside = True
                break
        if not inside:
            self.swept_band_since_ns = None
            return None
        if self.swept_band_since_ns is None:
            self.swept_band_since_ns = now_ns
            self.get_logger().info(
                "swept band entered; transit committed "
                f"x={x_value:.3f} limit={self.swept_band_speed:.2f}m/s"
            )
        elif now_ns - self.swept_band_since_ns > int(
            self.swept_band_commit * 1e9
        ):
            # Commit expired: fall back to the guard so a wedged base
            # cannot drive indefinitely inside the lane.
            self.swept_band_since_ns = None
            return None
        transit = Twist()
        limit = self.swept_band_speed
        transit.linear.x = max(-limit, min(limit, message.linear.x))
        transit.linear.y = 0.0
        transit.angular.z = message.angular.z
        return transit

    def cmd_vel_callback(self, message: Twist) -> None:
        """Scale one velocity command and republish it safely."""
        if self.base_hold:
            self.publisher.publish(Twist())
            return
        motion_sign = 0.0
        if abs(message.linear.x) >= 0.03:
            motion_sign = math.copysign(1.0, message.linear.x)
        gate_message = None
        if self.east_gate_blocked(motion_sign):
            gate_message = "east mover gate waiting clear of swept lane"
        elif self.west_lane_blocked(motion_sign):
            gate_message = "west mover gate waiting clear of swept lane"
        if gate_message is not None:
            safe = Twist()
            # Wait clear of the sweep while retaining steering already
            # checked upstream; do not delay heading until release.
            safe.angular.z = message.angular.z
            self.publisher.publish(safe)
            self.get_logger().info(gate_message, throttle_duration_sec=1.0)
            return
        scale = self.scale_for_obstacles(motion_sign)
        if scale <= 0.01:
            transit = self._swept_band_transit(message)
            if transit is not None:
                self.publisher.publish(transit)
                return
        if scale < 1.0:
            safe = Twist()
            safe.linear.x = message.linear.x * scale
            safe.linear.y = message.linear.y * scale
            # Nav2 keeps steering authority while this final guard limits
            # translation; the laser monitor still protects rotation.
            safe.angular.z = message.angular.z
            if scale <= 0.01:
                self.get_logger().info(
                    "safety stop: fresh obstacle in travel direction",
                    throttle_duration_sec=1.0,
                )
            message = safe
        self.publisher.publish(message)


def main(args=None) -> None:
    """Run the safety limiter node."""
    rclpy.init(args=args)
    node = SafetyLimiter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
