"""Serve a local browser dashboard backed by competition ROS topics."""

from __future__ import annotations

from functools import partial
from http.server import BaseHTTPRequestHandler
from http.server import ThreadingHTTPServer
import json
from pathlib import Path
import threading
import time
from typing import Optional
import xml.etree.ElementTree as ElementTree

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Path as NavigationPath
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from std_msgs.msg import String
from tf2_ros import Buffer
from tf2_ros import TransformException
from tf2_ros import TransformListener

from tools_demo.dashboard_state import DashboardState


def load_world_layout() -> dict:
    """Read scored entity poses from the same world Gazebo launches."""
    world_path = (
        Path(get_package_share_directory("mybot_description"))
        / "worlds"
        / "offic_room.world"
    )
    root = ElementTree.parse(world_path).getroot()
    world = root.find("world")
    if world is None:
        raise ValueError(f"Gazebo world missing <world>: {world_path}")

    pickups = []
    zones = []
    cubes = []
    points = []
    for model in world.findall("model"):
        name = str(model.get("name", ""))
        values = [float(value) for value in model.findtext(
            "pose", "0 0 0 0 0 0"
        ).split()]
        x_value, y_value = values[:2]
        item = {"name": name, "x": x_value, "y": y_value}
        if name.startswith((
            "competition_red_pickup_tray",
            "competition_blue_pickup_tray",
        )):
            size = model.findtext("link/collision/geometry/box/size")
            if size:
                width, height, _ = (float(value) for value in size.split())
                item.update({"width": width, "height": height})
            pickups.append(item)
        elif name in {"zone_a", "zone_b", "zone_c"}:
            size = model.findtext("link/visual/geometry/box/size")
            if size:
                width, height, _ = (float(value) for value in size.split())
                item.update({"width": width, "height": height})
            item["label"] = name[-1].upper()
            zones.append(item)
        elif (
            name.startswith(("red_cube_", "blue_cube_"))
            and not name.endswith("_anchor")
        ):
            cubes.append(item)
        else:
            continue
        points.append((x_value, y_value))
    if len(pickups) != 10 or len(zones) != 3 or len(cubes) != 10:
        raise ValueError("Active world has an incomplete competition layout")
    margin = 1.2
    bounds = {
        "minX": min(point[0] for point in points) - margin,
        "maxX": max(point[0] for point in points) + margin,
        "minY": min(point[1] for point in points) - margin,
        "maxY": max(point[1] for point in points) + margin,
    }
    return {
        "source": str(world_path),
        "pickups": pickups,
        "zones": zones,
        "cubes": cubes,
        "bounds": bounds,
    }


def transient_qos() -> QoSProfile:
    """Return reliable retained QoS for dashboard snapshot topics."""
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


class DashboardRequestHandler(BaseHTTPRequestHandler):
    """Expose the dashboard page and one read-only state endpoint."""

    server_version = "CompetitionDashboard/1.0"

    def __init__(
        self,
        *args,
        state,
        page,
        problem_publisher=None,
        **kwargs,
    ) -> None:
        self.dashboard_state = state
        self.dashboard_page = page
        self.problem_publisher = problem_publisher
        super().__init__(*args, **kwargs)

    def _respond(
        self,
        status: int,
        content_type: str,
        body: bytes,
    ) -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self) -> None:  # noqa: N802 - BaseHTTPRequestHandler API
        """Serve only known read-only routes."""
        path = self.path.split("?", 1)[0]
        if path == "/":
            self._respond(
                200,
                "text/html; charset=utf-8",
                self.dashboard_page,
            )
            return
        if path == "/api/state":
            body = json.dumps(
                self.dashboard_state.snapshot(),
                ensure_ascii=False,
                separators=(",", ":"),
            ).encode("utf-8")
            self._respond(200, "application/json; charset=utf-8", body)
            return
        if path == "/healthz":
            self._respond(200, "text/plain; charset=utf-8", b"ok\n")
            return
        self._respond(404, "text/plain; charset=utf-8", b"not found\n")

    def do_POST(self) -> None:  # noqa: N802 - BaseHTTPRequestHandler API
        """Accept one generator problem and its mapping rules."""
        if self.path.split("?", 1)[0] != "/api/task":
            self._respond(404, "text/plain; charset=utf-8", b"not found\n")
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
            body = self.rfile.read(length)
            payload = json.loads(body.decode("utf-8"))
        except (ValueError, json.JSONDecodeError, UnicodeDecodeError):
            self._respond(400, "text/plain; charset=utf-8", b"invalid JSON\n")
            return
        if not isinstance(payload, dict):
            self._respond(
                400,
                "text/plain; charset=utf-8",
                b"object required\n",
            )
            return
        question = str(payload.get("question", "")).strip()
        mapping_rules = str(payload.get("mapping_rules", "")).strip()
        if not question:
            self._respond(
                400,
                "text/plain; charset=utf-8",
                b"question required\n",
            )
            return
        self.dashboard_state.update_text("question", question)
        if self.problem_publisher is not None:
            problem_message = String()
            problem_message.data = json.dumps(
                {
                    "question": question,
                    "mapping_rules": mapping_rules,
                    "task_id": str(payload.get("task_id", "")).strip()
                    or None,
                },
                ensure_ascii=False,
                separators=(",", ":"),
                sort_keys=True,
            )
            self.problem_publisher.publish(problem_message)
        response = json.dumps(
            {"status": "accepted", "question": question},
            ensure_ascii=False,
        ).encode("utf-8")
        self._respond(200, "application/json; charset=utf-8", response)

    def log_message(self, format_string: str, *args) -> None:
        """Keep polling requests out of the ROS console."""


class CompetitionDashboard(Node):
    """Aggregate competition topics and run a local HTTP server."""

    def __init__(self) -> None:
        super().__init__("competition_dashboard")
        self.declare_parameter("bind_address", "0.0.0.0")
        self.declare_parameter("port", 8080)
        self.declare_parameter("maximum_path_points", 240)
        self.declare_parameter(
            "question_topic",
            "/competition/task/question",
        )
        self.declare_parameter("plan_topic", "/competition/task/plan")
        self.declare_parameter(
            "parse_result_topic", "/competition/task/parse_result"
        )
        self.declare_parameter(
            "llm_status_topic",
            "/competition/task/llm_status",
        )
        self.declare_parameter(
            "mission_topic",
            "/competition/mission/state",
        )
        self.declare_parameter(
            "navigation_topic",
            "/competition/navigation/state",
        )
        self.declare_parameter(
            "perception_topic",
            "/competition/perception/targets",
        )
        self.declare_parameter(
            "avoidance_topic",
            "/competition/avoidance/state",
        )
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
            "gripper_topic",
            "/competition/gripper/state",
        )
        self.declare_parameter(
            "problem_topic",
            "/competition/task/problem",
        )
        self.declare_parameter("path_topic", "/plan")
        self.declare_parameter("local_path_topic", "/local_plan")
        self.declare_parameter("robot_pose_topic", "/amcl_pose")

        port = int(self.get_parameter("port").value)
        if not 1 <= port <= 65535:
            raise ValueError("port must be between 1 and 65535")
        self.state = DashboardState(
            int(self.get_parameter("maximum_path_points").value),
        )
        self.state.update_layout(load_world_layout())
        # Prefer the authoritative TF map->base_footprint pose for the GUI
        # robot marker; /amcl_pose can lag or report a stale initial pose.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.last_tf_pose_monotonic = 0.0
        self.create_timer(0.05, self.refresh_robot_from_tf)
        retained = transient_qos()
        question_topic = str(
            self.get_parameter("question_topic").value
        )
        self.create_subscription(
            String,
            question_topic,
            self.question_callback,
            retained,
        )
        self.create_subscription(
            String,
            question_topic,
            self.question_callback,
            10,
        )
        for parameter, key in (
            ("plan_topic", "plan"),
            ("parse_result_topic", "parse_result"),
            ("llm_status_topic", "llm"),
            ("mission_topic", "mission"),
            ("navigation_topic", "navigation"),
        ):
            self.create_subscription(
                String,
                str(self.get_parameter(parameter).value),
                lambda message, state_key=key: self.state.update_json(
                    state_key,
                    message.data,
                ),
                retained,
            )
        self.create_subscription(
            String,
            str(self.get_parameter("perception_topic").value),
            lambda message: self.state.update_json(
                "perception",
                message.data,
            ),
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("avoidance_topic").value),
            lambda message: self.state.update_json(
                "avoidance",
                message.data,
            ),
            10,
        )
        for topic in self.get_parameter("obstacle_topics").value:
            self.create_subscription(
                String,
                str(topic),
                lambda message: self.state.update_obstacle(message.data),
                20,
            )
        self.create_subscription(
            String,
            str(self.get_parameter("gripper_topic").value),
            lambda message: self.state.update_text(
                "gripper", message.data
            ),
            10,
        )
        self.create_subscription(
            NavigationPath,
            str(self.get_parameter("path_topic").value),
            self.path_callback,
            10,
        )
        self.create_subscription(
            NavigationPath,
            str(self.get_parameter("local_path_topic").value),
            self.local_path_callback,
            10,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            str(self.get_parameter("robot_pose_topic").value),
            self.pose_callback,
            10,
        )

        page_path = (
            Path(get_package_share_directory("tools_demo"))
            / "web"
            / "dashboard.html"
        )
        page = page_path.read_bytes()
        self.problem_publisher = self.create_publisher(
            String,
            str(self.get_parameter("problem_topic").value),
            transient_qos(),
        )
        handler = partial(
            DashboardRequestHandler,
            state=self.state,
            page=page,
            problem_publisher=self.problem_publisher,
        )
        address = str(self.get_parameter("bind_address").value)
        self.server = ThreadingHTTPServer((address, port), handler)
        self.server.daemon_threads = True
        self.server_thread = threading.Thread(
            target=self.server.serve_forever,
            name="competition-dashboard-http",
            daemon=True,
        )
        self.server_thread.start()
        self.get_logger().info(
            f"Dashboard ready at http://{address}:{port}/"
        )

    def path_callback(self, message: NavigationPath) -> None:
        """Store x-y points from the current Nav2 global plan."""
        self.state.update_path(
            (
                pose.pose.position.x,
                pose.pose.position.y,
            )
            for pose in message.poses
        )

    def local_path_callback(self, message: NavigationPath) -> None:
        """Store the current controller trajectory used for avoidance."""
        points = (
            (
                pose.pose.position.x,
                pose.pose.position.y,
            )
            for pose in message.poses
        )
        self.state.update_path(
            points,
            key="local_path",
        )

    def question_callback(self, message: String) -> None:
        """Accept both volatile and retained task-question publishers."""
        self.state.update_text("question", message.data)

    def refresh_robot_from_tf(self) -> None:
        """Update the GUI robot pose from the TF tree when available."""
        for frame in ("base_footprint", "base_link"):
            try:
                transform = self.tf_buffer.lookup_transform(
                    "map",
                    frame,
                    rclpy.time.Time(),
                )
            except TransformException:
                continue
            translation = transform.transform.translation
            orientation = transform.transform.rotation
            self.state.update_robot(
                translation.x,
                translation.y,
                (
                    orientation.x,
                    orientation.y,
                    orientation.z,
                    orientation.w,
                ),
            )
            self.last_tf_pose_monotonic = time.monotonic()
            return

    def pose_callback(self, message: PoseWithCovarianceStamped) -> None:
        """Store the localized mobile-base pose (fallback when TF is stale)."""
        if time.monotonic() - self.last_tf_pose_monotonic < 1.0:
            return
        pose = message.pose.pose
        orientation = pose.orientation
        self.state.update_robot(
            pose.position.x,
            pose.position.y,
            (
                orientation.x,
                orientation.y,
                orientation.z,
                orientation.w,
            ),
        )

    def destroy_node(self) -> bool:
        """Stop accepting HTTP requests before destroying ROS resources."""
        self.server.shutdown()
        self.server.server_close()
        self.server_thread.join(timeout=2.0)
        return super().destroy_node()


def main(args=None) -> None:
    """Run the competition dashboard node."""
    rclpy.init(args=args)
    node: Optional[CompetitionDashboard] = None
    try:
        node = CompetitionDashboard()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
