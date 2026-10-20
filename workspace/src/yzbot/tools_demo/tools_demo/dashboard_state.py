"""Thread-safe state aggregation for the competition dashboard."""

from __future__ import annotations

import copy
import json
import math
import threading
import time
from typing import Any, Dict, Iterable, Mapping, Sequence


class DashboardState:
    """Collect ROS snapshots without exposing mutable shared state."""

    def __init__(
        self,
        maximum_path_points: int = 240,
    ) -> None:
        if maximum_path_points < 2:
            raise ValueError("maximum_path_points must be at least 2")
        self.maximum_path_points = maximum_path_points
        self._active_task_id = None
        self._lock = threading.Lock()
        self._state: Dict[str, Any] = {
            "question": "",
            "plan": {},
            "parse_result": {},
            "llm": {},
            "mission": {},
            "navigation": {},
            "perception": {},
            "avoidance": {},
            "obstacles": {},
            "layout": {},
            "gripper": "unknown",
            "path": [],
            "local_path": [],
            "path_updated_at": 0.0,
            "local_path_updated_at": 0.0,
            "robot": {},
            "robot_updated_at": 0.0,
            "obstacles_updated_at": 0.0,
            # One entry per mission round: the 2026 rules require the panel to
            # show each round's completion result, not just running totals.
            "rounds": [],
            "updated_at": 0.0,
        }
        self._round_index = None
        self._round_completed = 0
        self._round_skipped = 0

    def _store(self, key: str, value: Any) -> None:
        with self._lock:
            self._state[key] = value
            self._state["updated_at"] = time.time()

    def update_text(self, key: str, value: str) -> None:
        """Store one plain-text topic value."""
        self._store(key, str(value))

    def update_layout(self, layout: Mapping[str, Any]) -> None:
        """Store the immutable layout parsed from the active Gazebo world."""
        self._store("layout", copy.deepcopy(dict(layout)))

    def update_json(self, key: str, value: str) -> bool:
        """Store a JSON object, leaving prior valid data on parse errors."""
        try:
            payload = json.loads(value)
        except (json.JSONDecodeError, TypeError):
            return False
        if not isinstance(payload, Mapping):
            return False
        with self._lock:
            if key == "mission":
                task_id = payload.get("task_id")
                if task_id and task_id != self._active_task_id:
                    self._state["path"] = []
                    self._state["local_path"] = []
                    self._state["rounds"] = []
                    self._active_task_id = task_id
                    self._round_index = None
                    self._round_completed = 0
                    self._round_skipped = 0
                self._record_round_locked(payload)
            if key == "navigation" and payload.get("state") in {
                "ready", "sending", "handoff", "succeeded", "failed",
                "canceled", "aborted", "timed_out",
            }:
                self._state["path"] = []
                self._state["local_path"] = []
            self._state[key] = dict(payload)
            self._state["updated_at"] = time.time()
        return True

    def _record_round_locked(self, payload: Mapping[str, Any]) -> None:
        """
        Append or update this round's entry from one mission payload.

        Caller must hold the lock. A round is identified by current_index. Its
        outcome is inferred from the completed/skipped counters moving, so the
        coordinator needs no extra publication: whichever counter advances
        names the result of the round that just ended.
        """
        index = payload.get("current_index")
        if not isinstance(index, int) or index < 1:
            return
        completed = payload.get("completed")
        skipped = payload.get("skipped")
        completed = completed if isinstance(completed, int) else 0
        skipped = skipped if isinstance(skipped, int) else 0

        rounds = self._state["rounds"]
        if index != self._round_index:
            # A new round started, so the previous one has finished. Whichever
            # counter advanced tells us how it ended.
            if rounds:
                previous = rounds[-1]
                if previous.get("result") == "active":
                    if completed > self._round_completed:
                        previous["result"] = "completed"
                    elif skipped > self._round_skipped:
                        previous["result"] = "skipped"
                    else:
                        previous["result"] = "unfinished"
            self._round_completed = completed
            self._round_skipped = skipped
            self._round_index = index
            rounds.append(
                {
                    "index": index,
                    "color": payload.get("color"),
                    "cube_name": payload.get("cube_name"),
                    "target_zone": payload.get("target_zone"),
                    "result": "active",
                    "detail": payload.get("detail"),
                    "elapsed_sec": payload.get("elapsed_sec"),
                }
            )
            return

        entry = rounds[-1] if rounds else None
        if entry is None:
            return
        # Same round: keep the live fields fresh.
        for field in ("color", "cube_name", "target_zone", "detail"):
            value = payload.get(field)
            if value is not None:
                entry[field] = value
        elapsed = payload.get("elapsed_sec")
        if elapsed is not None:
            entry["elapsed_sec"] = elapsed
        state = payload.get("state")
        if state in {"completed", "failed", "canceled"}:
            # Terminal mission state: close the round that was in flight.
            if completed > self._round_completed:
                entry["result"] = "completed"
            elif skipped > self._round_skipped:
                entry["result"] = "skipped"
            elif state == "completed":
                entry["result"] = "completed"
            else:
                entry["result"] = state
            self._round_completed = completed
            self._round_skipped = skipped

    def update_obstacle(self, value: str) -> bool:
        """Store one latest Gazebo obstacle pose without any history."""
        try:
            payload = json.loads(value)
            model = str(payload["model"])
            x_value = round(float(payload["x"]), 4)
            y_value = round(float(payload["y"]), 4)
        except (json.JSONDecodeError, KeyError, TypeError, ValueError):
            return False
        if payload.get("success") is not True:
            return False
        now = time.time()
        with self._lock:
            self._state["obstacles"][model] = [x_value, y_value]
            self._state["obstacles_updated_at"] = now
            self._state["updated_at"] = now
        return True

    def update_path(
        self,
        points: Iterable[Sequence[float]],
        key: str = "path",
    ) -> None:
        """Store a compact global or local path with both endpoints."""
        if key not in ("path", "local_path"):
            raise ValueError("path key must be path or local_path")
        normalized = [
            [round(float(point[0]), 4), round(float(point[1]), 4)]
            for point in points
        ]
        if len(normalized) > self.maximum_path_points:
            last_index = len(normalized) - 1
            indexes = {
                round(
                    offset * last_index
                    / (self.maximum_path_points - 1)
                )
                for offset in range(self.maximum_path_points)
            }
            normalized = [normalized[index] for index in sorted(indexes)]
        now = time.time()
        with self._lock:
            navigation_state = self._state.get("navigation", {}).get("state")
            if navigation_state and navigation_state != "active":
                self._state[key] = []
                self._state[f"{key}_updated_at"] = now
                return
            self._state[key] = normalized
            self._state[f"{key}_updated_at"] = now
            self._state["updated_at"] = now

    def update_robot(
        self,
        x_value: float,
        y_value: float,
        quaternion: Sequence[float],
    ) -> None:
        """Store a planar robot pose derived from x-y-z-w orientation."""
        qx, qy, qz, qw = (float(value) for value in quaternion)
        yaw = math.atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz),
        )
        x_value = round(float(x_value), 4)
        y_value = round(float(y_value), 4)
        robot = {"x": x_value, "y": y_value, "yaw": round(yaw, 4)}
        now = time.time()
        with self._lock:
            for key in ("path", "local_path"):
                path = self._state[key]
                if not path:
                    continue
                nearest = min(
                    range(len(path)),
                    key=lambda index: math.hypot(
                        x_value - path[index][0],
                        y_value - path[index][1],
                    ),
                )
                remaining = path[nearest:]
                current = [x_value, y_value]
                if math.hypot(
                    x_value - remaining[0][0],
                    y_value - remaining[0][1],
                ) <= 1e-4:
                    remaining[0] = current
                else:
                    remaining.insert(0, current)
                self._state[key] = remaining
            self._state["robot"] = robot
            self._state["robot_updated_at"] = now
            self._state["updated_at"] = now

    def snapshot(self) -> Dict[str, Any]:
        """Return an isolated JSON-compatible copy."""
        with self._lock:
            return copy.deepcopy(self._state)
