"""Validate that the integrated competition runtime is ready."""

from __future__ import annotations

from datetime import datetime, timezone
import json
from pathlib import Path
import time
from typing import Any, Dict, Optional
from urllib.error import URLError
from urllib.request import urlopen

from rclpy.action import get_action_names_and_types
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.time import Time
import rclpy
from std_msgs.msg import String
from tf2_ros import Buffer
from tf2_ros import TransformListener

from tools_demo.acceptance_contract import fully_qualified_node_names
from tools_demo.acceptance_contract import missing_typed_interfaces
from tools_demo.acceptance_contract import parse_transform_specification
from tools_demo.acceptance_contract import successful_full_mission
from tools_demo.acceptance_contract import typed_interface_map
from tools_demo.acceptance_contract import validated_llm_plan


TERMINAL_MISSION_STATES = {"completed", "failed", "canceled"}


def retained_qos() -> QoSProfile:
    """Return the retained QoS used by mission state."""
    return QoSProfile(
        depth=10,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        reliability=ReliabilityPolicy.RELIABLE,
    )


class CompetitionAcceptance(Node):
    """Poll the ROS graph, TF tree, dashboard, and optional mission."""

    def __init__(self) -> None:
        super().__init__("competition_acceptance")
        self.declare_parameter("startup_timeout_sec", 120.0)
        self.declare_parameter("poll_period_sec", 1.0)
        self.declare_parameter("monitor_mission", False)
        self.declare_parameter("mission_timeout_sec", 300.0)
        self.declare_parameter("require_llm_plan", True)
        self.declare_parameter("require_obstacle_avoidance", True)
        self.declare_parameter(
            "dashboard_url",
            "http://127.0.0.1:8080",
        )
        self.declare_parameter(
            "evidence_path",
            "/tmp/competition_acceptance.json",
        )
        self.declare_parameter(
            "mission_topic",
            "/competition/mission/state",
        )
        self.declare_parameter(
            "llm_status_topic",
            "/competition/task/llm_status",
        )
        self.declare_parameter(
            "avoidance_topic",
            "/competition/avoidance/state",
        )
        self.declare_parameter("required_nodes", [""])
        self.declare_parameter("required_topics", [""])
        self.declare_parameter("required_publishers", [""])
        self.declare_parameter("required_services", [""])
        self.declare_parameter("required_actions", [""])
        self.declare_parameter("required_transforms", [""])

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.mission_state: Dict[str, Any] = {}
        self.llm_status: Dict[str, Any] = {}
        self.avoidance_state: Dict[str, Any] = {}
        self.create_subscription(
            String,
            str(self.get_parameter("mission_topic").value),
            self.mission_callback,
            retained_qos(),
        )
        self.create_subscription(
            String,
            str(self.get_parameter("llm_status_topic").value),
            self.llm_status_callback,
            retained_qos(),
        )
        self.create_subscription(
            String,
            str(self.get_parameter("avoidance_topic").value),
            self.avoidance_callback,
            retained_qos(),
        )

    def _string_list(self, parameter: str) -> list:
        return [
            str(value)
            for value in self.get_parameter(parameter).value
            if str(value)
        ]

    def mission_callback(self, message: String) -> None:
        """Keep the latest valid mission snapshot."""
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError:
            return
        if isinstance(payload, dict):
            self.mission_state = payload

    def llm_status_callback(self, message: String) -> None:
        """Keep model-origin evidence for startup and mission checks."""
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError:
            return
        if isinstance(payload, dict):
            self.llm_status = payload

    def avoidance_callback(self, message: String) -> None:
        """Keep the latest two-obstacle clearance evidence."""
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError:
            return
        if isinstance(payload, dict):
            self.avoidance_state = payload

    def _dashboard_state(self) -> Optional[Dict[str, Any]]:
        base_url = str(self.get_parameter("dashboard_url").value).rstrip("/")
        if not base_url:
            return {}
        try:
            with urlopen(f"{base_url}/healthz", timeout=0.5) as response:
                if response.status != 200 or response.read().strip() != b"ok":
                    return None
            with urlopen(f"{base_url}/api/state", timeout=0.5) as response:
                payload = json.loads(response.read().decode("utf-8"))
        except (OSError, URLError, ValueError, json.JSONDecodeError):
            return None
        return payload if isinstance(payload, dict) else None

    def missing_contracts(self) -> tuple:
        """Return missing graph, TF, and dashboard requirements."""
        nodes = fully_qualified_node_names(
            self.get_node_names_and_namespaces()
        )
        topics = typed_interface_map(
            self.get_topic_names_and_types()
        )
        services = typed_interface_map(
            self.get_service_names_and_types()
        )
        actions = typed_interface_map(
            get_action_names_and_types(self)
        )

        missing: Dict[str, list] = {}
        missing_nodes = sorted(
            set(self._string_list("required_nodes")) - nodes
        )
        if missing_nodes:
            missing["nodes"] = missing_nodes
        for parameter, available in (
            ("required_topics", topics),
            ("required_services", services),
            ("required_actions", actions),
        ):
            absent = missing_typed_interfaces(
                self._string_list(parameter),
                available,
            )
            if absent:
                missing[parameter.removeprefix("required_")] = absent
        missing_publishers = [
            topic
            for topic in self._string_list("required_publishers")
            if self.count_publishers(topic) == 0
        ]
        if missing_publishers:
            missing["publishers"] = missing_publishers

        missing_transforms = []
        for specification in self._string_list("required_transforms"):
            try:
                target, source = parse_transform_specification(
                    specification
                )
            except ValueError:
                missing_transforms.append(f"invalid:{specification}")
                continue
            if not self.tf_buffer.can_transform(
                target,
                source,
                Time(),
                timeout=Duration(seconds=0.05),
            ):
                missing_transforms.append(specification)
        if missing_transforms:
            missing["transforms"] = missing_transforms

        dashboard_state = self._dashboard_state()
        if dashboard_state is None:
            missing["dashboard"] = [
                str(self.get_parameter("dashboard_url").value)
            ]
        if bool(self.get_parameter("require_llm_plan").value):
            if not validated_llm_plan(self.llm_status):
                missing["llm"] = ["validated_model_plan"]
            if isinstance(dashboard_state, dict):
                plan = dashboard_state.get("plan", {})
                if not (
                    isinstance(plan, dict)
                    and plan.get("source") == "llm"
                ):
                    missing["dashboard_plan"] = ["source=llm"]
        if bool(
            self.get_parameter("require_obstacle_avoidance").value
        ):
            if not (
                int(self.avoidance_state.get("obstacles_seen", 0)) >= 2
                and self.avoidance_state.get("collision_detected") is False
            ):
                missing["avoidance"] = [
                    "two_obstacles_without_collision"
                ]
        return missing, dashboard_state

    def wait_for_startup(self) -> tuple:
        """Wait until every required runtime contract is present."""
        timeout = float(self.get_parameter("startup_timeout_sec").value)
        poll = float(self.get_parameter("poll_period_sec").value)
        if timeout <= 0.0 or poll <= 0.0:
            raise ValueError("acceptance timeouts must be positive")
        deadline = time.monotonic() + timeout
        last_missing = None
        dashboard_state = None
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=min(poll, 0.25))
            missing, dashboard_state = self.missing_contracts()
            if not missing:
                return True, {}, dashboard_state
            if missing != last_missing:
                self.get_logger().info(
                    "Waiting for contracts: "
                    + json.dumps(missing, ensure_ascii=False)
                )
                last_missing = missing
            time.sleep(max(0.0, poll - 0.25))
        missing, dashboard_state = self.missing_contracts()
        return False, missing, dashboard_state

    def wait_for_mission(self) -> tuple:
        """Wait for a successful mission terminal state within five minutes."""
        timeout = float(self.get_parameter("mission_timeout_sec").value)
        if timeout <= 0.0 or timeout > 300.0:
            raise ValueError(
                "mission_timeout_sec must be within (0, 300]"
            )
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.25)
            state = str(self.mission_state.get("state", ""))
            if state in TERMINAL_MISSION_STATES:
                success = successful_full_mission(
                    self.mission_state,
                    require_llm=bool(
                        self.get_parameter("require_llm_plan").value
                    ),
                    llm_status=self.llm_status,
                    require_avoidance=bool(
                        self.get_parameter(
                            "require_obstacle_avoidance"
                        ).value
                    ),
                    avoidance_status=self.avoidance_state,
                )
                return success, dict(self.mission_state)
        return False, {
            "state": "timed_out",
            "detail": "No successful terminal mission within 300 seconds",
        }

    def write_evidence(self, report: Dict[str, Any]) -> None:
        """Persist a machine-readable acceptance snapshot."""
        path = Path(str(self.get_parameter("evidence_path").value))
        try:
            path.write_text(
                json.dumps(report, ensure_ascii=False, indent=2) + "\n",
                encoding="utf-8",
            )
        except OSError as error:
            self.get_logger().error(
                f"Could not write acceptance evidence: {error}"
            )


def main(args=None) -> None:
    """Run startup checks and, when requested, one timed mission check."""
    rclpy.init(args=args)
    node = CompetitionAcceptance()
    started_at = time.monotonic()
    report: Dict[str, Any] = {
        "schema_version": "1.0",
        "checked_at": datetime.now(timezone.utc).isoformat(),
    }
    exit_code = 1
    try:
        ready, missing, dashboard = node.wait_for_startup()
        report["startup"] = {
            "result": "pass" if ready else "fail",
            "missing": missing,
        }
        report["dashboard"] = dashboard
        report["llm"] = dict(node.llm_status)
        report["avoidance"] = dict(node.avoidance_state)
        success = ready
        if ready and bool(node.get_parameter("monitor_mission").value):
            mission_success, mission = node.wait_for_mission()
            report["mission"] = mission
            success = mission_success
        report["result"] = "pass" if success else "fail"
        report["elapsed_sec"] = round(time.monotonic() - started_at, 3)
        node.write_evidence(report)
        label = "PASS" if success else "FAIL"
        node.get_logger().info(
            f"COMPETITION_ACCEPTANCE={label} "
            f"elapsed_sec={report['elapsed_sec']} "
            f"evidence={node.get_parameter('evidence_path').value}"
        )
        exit_code = 0 if success else 1
    except (ValueError, RuntimeError) as error:
        report["result"] = "error"
        report["error"] = str(error)
        report["elapsed_sec"] = round(time.monotonic() - started_at, 3)
        node.write_evidence(report)
        node.get_logger().error(f"COMPETITION_ACCEPTANCE=ERROR {error}")
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
