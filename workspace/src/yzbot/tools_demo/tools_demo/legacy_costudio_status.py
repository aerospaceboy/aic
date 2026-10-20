"""Expose the current split telemetry as the legacy coStudio status schema."""

import json
import time
from typing import Any, Dict

import rclpy
from rclpy.clock import Clock
from rclpy.clock import ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from std_msgs.msg import String


def retained_qos() -> QoSProfile:
    return QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def decode(value: str) -> Dict[str, Any]:
    try:
        payload = json.loads(value)
    except (TypeError, json.JSONDecodeError):
        return {}
    return payload if isinstance(payload, dict) else {}


class LegacyCoStudioStatus(Node):
    """Bridge current competition topics to `/competition/status` only."""

    def __init__(self) -> None:
        super().__init__("legacy_costudio_status")
        self.question = ""
        self.mapping_rules = ""
        self.plan: Dict[str, Any] = {}
        self.mission: Dict[str, Any] = {}
        self.navigation: Dict[str, Any] = {}
        self.llm: Dict[str, Any] = {}
        self.parse_result: Dict[str, Any] = {}
        self.perception: Dict[str, Any] = {}
        self.perception_at = 0.0
        self.gripper = "idle"
        self.started_at = time.monotonic()
        # Per-round completion tracking so the panel can show which cube
        # finished in each round instead of only a running count.
        self.round_task_id = ""
        self.pending_cubes: Dict[int, Any] = {}
        self.completed_steps: list = []
        self.completed_seen = 0
        qos = retained_qos()
        self.publisher = self.create_publisher(
            String, "/competition/status", qos
        )
        self.create_subscription(
            String, "/competition/task/question", self.question_callback, qos
        )
        self.create_subscription(
            String, "/competition/task/problem", self.problem_callback, qos
        )
        self.create_subscription(
            String, "/competition/task/plan", self.json_callback("plan"), qos
        )
        self.create_subscription(
            String, "/competition/mission/state", self.json_callback("mission"), qos
        )
        self.create_subscription(
            String, "/competition/navigation/state", self.json_callback("navigation"), qos
        )
        self.create_subscription(
            String, "/competition/task/llm_status", self.json_callback("llm"), qos
        )
        self.create_subscription(
            String, "/competition/task/parse_result", self.parse_callback, qos
        )
        self.create_subscription(
            String, "/competition/gripper/state", self.gripper_callback, 10
        )
        # Live perception results: the detector publishes continuously, so a
        # volatile depth-10 subscription matches its publisher profile.
        self.create_subscription(
            String, "/competition/perception/targets", self.targets_callback, 10
        )
        # This is operator telemetry, so it must remain live while Gazebo is
        # paused or has not yet started advancing `/clock`.
        self.create_timer(
            0.5,
            self.publish_status,
            clock=Clock(clock_type=ClockType.SYSTEM_TIME),
        )

    def json_callback(self, name: str):
        def callback(message: String) -> None:
            setattr(self, name, decode(message.data))
        return callback

    def question_callback(self, message: String) -> None:
        self.question = message.data.strip()

    def problem_callback(self, message: String) -> None:
        payload = decode(message.data)
        self.question = str(payload.get("question", self.question)).strip()
        self.mapping_rules = str(
            payload.get("mapping_rules", self.mapping_rules)
        ).strip()

    def gripper_callback(self, message: String) -> None:
        self.gripper = message.data.strip() or "idle"

    def targets_callback(self, message: String) -> None:
        """Keep the newest perception snapshot for the panel's 识别 field."""
        self.perception = decode(message.data)
        self.perception_at = time.monotonic()

    def _track_round_progress(self, total: int) -> list:
        """Record one completed cube per round and detect rule deviations."""
        task_id = str(self.mission.get("task_id") or "")
        if task_id != self.round_task_id:
            self.round_task_id = task_id
            self.pending_cubes = {}
            self.completed_steps = []
            self.completed_seen = 0
        index = int(self.mission.get("current_index", 0) or 0)
        cube = str(self.mission.get("cube_name") or "")
        zone = str(self.mission.get("target_zone") or "")
        # Terminal snapshots retain the last selected cube but have no active
        # target zone. Do not erase its earlier valid delivery evidence.
        if cube and zone in {"A", "B", "C"}:
            self.pending_cubes[index] = (cube, zone)
        completed = int(self.mission.get("completed", 0) or 0)
        while self.completed_seen < completed:
            self.completed_seen += 1
            pending = self.pending_cubes.get(self.completed_seen)
            if not pending:
                continue
            name, target = pending
            self.completed_steps.append({
                "step": self.completed_seen,
                "category": name.split("_")[0],
                "cube_name": name,
                "target_region": target or "",
            })
        return self.completed_steps

    def parse_callback(self, message: String) -> None:
        """Replace old instructions with a complete current-request snapshot."""
        self.parse_result = decode(message.data)
        self.question = str(self.parse_result.get("question", ""))
        self.mapping_rules = str(self.parse_result.get("mapping_rules", ""))

    def publish_status(self) -> None:
        evidence = self.parse_result
        current_plan = evidence.get("plan") or {} if evidence else {}
        items = current_plan.get("items")
        items = items if isinstance(items, list) else []
        resources = {"red": 0, "blue": 0}
        targets = {"red": "", "blue": ""}
        objects = []
        order = []
        for raw in items:
            if not isinstance(raw, dict):
                continue
            color = str(raw.get("color", "")).lower()
            count = raw.get("count")
            zone = str(raw.get("target_zone", "")).upper()
            if color not in resources or not isinstance(count, int) or count < 0:
                continue
            if zone not in {"A", "B", "C"}:
                continue
            resources[color] += count
            targets[color] = zone
            objects.append(
                {"category": color, "count": count, "target_region": zone}
            )
            for _ in range(count):
                order.append({"step": len(order) + 1,
                              "category": color, "target_region": zone})
        total = int(self.mission.get("total", sum(resources.values())) or 0)
        completed = int(self.mission.get("completed", 0) or 0)
        state = str(self.mission.get("state", "idle"))
        detail = str(self.mission.get("detail", state))
        actual_steps = self._track_round_progress(total)
        current_step = min(completed + 1, total) if total else 0
        compliance = "IN_PROGRESS"
        if total and completed >= total:
            compliance = "RULE_MATCH"
        if any(
            step["category"] not in targets
            or step["target_region"] != targets[step["category"]]
            for step in actual_steps
        ) or any(
            sum(step["category"] == color for step in actual_steps) > count
            for color, count in resources.items()
        ):
            compliance = "RULE_VIOLATION"
        if total and completed >= total and len(actual_steps) != completed:
            compliance = "INCOMPLETE_EVIDENCE"
        if self.mission.get("enforce_color_order", True) and actual_steps:
            if [step["category"] for step in actual_steps] != [
                step["category"] for step in order[: len(actual_steps)]
            ]:
                compliance = "RULE_VIOLATION"
        visible: list = []
        if time.monotonic() - self.perception_at <= 2.0:
            for item in self.perception.get("targets", []) or []:
                color = str(item.get("color", "")).lower()
                if color and color not in visible:
                    visible.append(color)
        current_rule = order[current_step - 1] if 0 < current_step <= len(order) else None
        if not self.mission.get("enforce_color_order", True):
            color = str(self.mission.get("color") or "")
            zone = str(self.mission.get("target_zone") or "")
            if color in resources and zone in {"A", "B", "C"}:
                current_rule = {"step": current_step, "category": color,
                                "target_region": zone}
        payload = {
            "schema_version": "1.0",
            "parse_result": evidence,
            "task_id": evidence.get("task_id", ""),
            "mission_task_id": self.mission.get("task_id", ""),
            "mission_finished_at_unix_ms": self.mission.get("finished_at_unix_ms"),
            "planned_grasp_order": order,
            "actual_grasp_order": actual_steps,
            "current_grasp_step": current_step,
            "current_rule": current_rule,
            "grasp_rule_compliance": compliance,
            "enforce_color_order": self.mission.get("enforce_color_order", True),
            "question": self.question,
            "resources": resources,
            "target_zones": targets,
            "objects": objects,
            "rules": (
                [{"condition": "映射规则", "mapping": self.mapping_rules}]
                if self.mapping_rules else []
            ),
            "current_index": int(self.mission.get("current_index", 0) or 0),
            "total": total,
            "completed": completed,
            "remaining": max(0, total - completed),
            "skipped": int(self.mission.get("skipped", 0) or 0),
            "current_state": state,
            "task_manager_state": state,
            "current_action": state,
            "mission_detail": detail,
            "navigation": {"state": self.navigation.get("state", "idle")},
            "gripper": {"state": self.gripper},
            "grasp_state": self.gripper,
            "elapsed_sec": self.mission.get(
                "elapsed_sec", round(time.monotonic() - self.started_at, 1)
            ),
            "current_target_block": self.mission.get("cube_name") or "",
            "current_target_zone": self.mission.get("target_zone") or "",
            "visible_targets": visible,
            "llm_state": evidence.get("state", "waiting"),
            "error": evidence.get("error", ""),
            "task_running": state not in {"idle", "completed", "failed", "canceled"},
        }
        self.publisher.publish(String(data=json.dumps(
            payload, ensure_ascii=False, separators=(",", ":")
        )))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LegacyCoStudioStatus()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
