"""Pure ordered mission state machine for competition pick-and-place."""

from dataclasses import dataclass
import json
from typing import Any, Dict, Mapping, Optional, Sequence, Tuple

from tools_demo.task_protocol import SCHEMA_VERSION
from tools_demo.task_protocol import TaskItem
from tools_demo.task_protocol import TaskPlan
from tools_demo.task_protocol import TaskProtocolError


MISSION_STATES = (
    "idle",
    "navigating_pickup",
    "waiting_perception",
    "picking",
    "navigating_destination",
    "navigating_zone_exit",
    "placing",
    "verifying",
    "releasing",
    "completed",
    "failed",
    "canceled",
)


class MissionValueError(ValueError):
    """Raised when mission input or an event violates the contract."""


def selected_cube_snapshot(
    snapshot: Mapping[str, Any], cube_name: str
) -> Dict[str, Any]:
    """Copy public state while replacing only the runtime cube choice."""
    if not isinstance(snapshot, Mapping) or not isinstance(cube_name, str):
        raise MissionValueError("Snapshot and cube name are invalid")
    result = dict(snapshot)
    result["cube_name"] = cube_name
    return result


@dataclass(frozen=True)
class MissionJob:
    """One expanded single-cube job in exact task order."""

    index: int
    total: int
    color: str
    cube_name: str
    target_zone: str


@dataclass(frozen=True)
class MissionAction:
    """One side effect requested from the ROS coordinator."""

    kind: str
    location: Optional[str] = None
    job: Optional[MissionJob] = None
    target: Optional[Mapping[str, Any]] = None
    reason: Optional[str] = None


def parse_task_plan_json(content: str) -> TaskPlan:
    """Rebuild and validate a public task-plan JSON message."""
    try:
        value = json.loads(content)
    except (TypeError, json.JSONDecodeError) as exception:
        raise MissionValueError("Task plan is not valid JSON") from exception
    if not isinstance(value, Mapping):
        raise MissionValueError("Task plan root must be an object")
    required = {"schema_version", "task_id", "source", "items"}
    if not required.issubset(value):
        missing = sorted(required - set(value))
        raise MissionValueError(
            f"Task plan omitted required fields: {missing}"
        )
    raw_items = value["items"]
    if not isinstance(raw_items, Sequence) or isinstance(raw_items, str):
        raise MissionValueError("Task plan items must be an array")
    try:
        items = tuple(
            TaskItem(
                color=item["color"],
                count=item["count"],
                target_zone=item["target_zone"],
            )
            for item in raw_items
        )
        plan = TaskPlan(
            task_id=value["task_id"],
            source=value["source"],
            items=items,
            schema_version=value["schema_version"],
        )
    except (
        KeyError,
        TypeError,
        TaskProtocolError,
    ) as exception:
        raise MissionValueError(f"Invalid task plan: {exception}") from exception
    if plan.schema_version != SCHEMA_VERSION:
        raise MissionValueError(
            f"Unsupported task schema: {plan.schema_version}"
        )
    if "counts" in value and value["counts"] != plan.counts:
        raise MissionValueError("Task plan counts disagree with items")
    if (
        "execution_order" in value
        and tuple(value["execution_order"]) != plan.execution_order
    ):
        raise MissionValueError(
            "Task execution_order disagrees with items"
        )
    return plan


def expand_jobs(
    plan: TaskPlan,
    maximum_cubes_per_color: int = 5,
) -> Tuple[MissionJob, ...]:
    """Expand item quantities to stable Gazebo cube names."""
    if maximum_cubes_per_color <= 0:
        raise MissionValueError("maximum_cubes_per_color must be positive")
    counters = {"red": 0, "blue": 0}
    expanded = []
    for item in plan.items:
        for _ in range(item.count):
            counters[item.color] += 1
            number = counters[item.color]
            if number > maximum_cubes_per_color:
                raise MissionValueError(
                    f"Task requests more than {maximum_cubes_per_color} "
                    f"{item.color} cubes"
                )
            expanded.append(
                (
                    item.color,
                    f"{item.color}_cube_{number}",
                    item.target_zone,
                )
            )
    total = len(expanded)
    return tuple(
        MissionJob(
            index=index,
            total=total,
            color=color,
            cube_name=cube_name,
            target_zone=target_zone,
        )
        for index, (color, cube_name, target_zone) in enumerate(
            expanded,
            start=1,
        )
    )


def select_reachable_target(
    targets: Sequence[Mapping[str, Any]],
    color: str,
    minimum_x: float,
    maximum_x: float,
    maximum_abs_y: float,
    minimum_z: float = 0.005,
    preferred_y: float = 0.0,
    preferred_lateral_sign: Optional[int] = None,
    expected_position: Optional[Sequence[float]] = None,
    expected_x_tolerance: float = 0.10,
    expected_y_tolerance: float = 0.08,
) -> Optional[Mapping[str, Any]]:
    """
    Choose the reachable target nearest the expected inventory slot.

    When ``expected_position`` is provided (the known world slot of the
    current cube expressed in the base frame), only targets close to that
    slot are considered so the mission never selects a different same-colour
    cube. Detections whose z is below the platform surface (reflections or
    the tray lip) are rejected as invalid.
    """
    matches = []
    for target in targets:
        if target.get("color") != color:
            continue
        position = target.get("position")
        if not isinstance(position, Mapping):
            continue
        try:
            x_value = float(position["x"])
            y_value = float(position["y"])
            z_value = float(position["z"])
        except (KeyError, TypeError, ValueError):
            continue
        if (
            minimum_x <= x_value <= maximum_x
            and abs(y_value) <= maximum_abs_y
            and z_value >= minimum_z
        ):
            if expected_position is not None:
                expected_x = float(expected_position[0])
                expected_y = float(expected_position[1])
                if (
                    abs(x_value - expected_x) > expected_x_tolerance
                    or abs(y_value - expected_y) > expected_y_tolerance
                ):
                    continue
            matches.append(target)
    if not matches:
        return None
    if expected_position is not None:
        expected_x = float(expected_position[0])
        expected_y = float(expected_position[1])
        return min(
            matches,
            key=lambda target: (
                abs(float(target["position"]["y"]) - expected_y),
                abs(float(target["position"]["x"]) - expected_x),
                float(target["position"]["y"]),
            ),
        )
    return min(
        matches,
        key=lambda target: (
            (
                0
                if preferred_lateral_sign is None
                or (
                    preferred_lateral_sign < 0
                    and float(target["position"]["y"]) < preferred_y
                )
                or (
                    preferred_lateral_sign > 0
                    and float(target["position"]["y"]) > preferred_y
                )
                else 1
            ),
            abs(float(target["position"]["y"]) - preferred_y),
            float(target["position"]["y"]),
            float(target["position"]["x"]),
        ),
    )


class MissionStateMachine:
    """Advance one ordered mission through explicit verified events."""

    def __init__(
        self,
        maximum_retries: int = 1,
        maximum_cubes_per_color: int = 5,
        maximum_perception_retries: Optional[int] = None,
        allow_skips: bool = True,
    ) -> None:
        if maximum_retries < 0:
            raise MissionValueError("maximum_retries cannot be negative")
        if maximum_perception_retries is None:
            maximum_perception_retries = maximum_retries
        if maximum_perception_retries < 0:
            raise MissionValueError(
                "maximum_perception_retries cannot be negative"
            )
        self.maximum_retries = maximum_retries
        self.allow_skips = allow_skips
        self.maximum_perception_retries = maximum_perception_retries
        self.maximum_cubes_per_color = maximum_cubes_per_color
        self.state = "idle"
        self.plan: Optional[TaskPlan] = None
        self.jobs: Tuple[MissionJob, ...] = ()
        self.job_offset = 0
        self.completed_count = 0
        self.skipped_count = 0
        self.attempts: Dict[str, int] = {}
        self.selected_target: Optional[Mapping[str, Any]] = None
        self.pending_skip_reason: Optional[str] = None
        self.pending_zone_exit_skipped = False
        self.pending_zone_exit_reason: Optional[str] = None
        # A pickup skipped without carrying leaves the base beside the tray;
        # drive back to the tray-clear entry before advancing to the next job.
        self.skip_after_departure = False
        # A failed physical grasp excludes only that concrete cube. Leave its
        # dock first, then retry the same ordered task item with another cube
        # of the same color.
        self.switch_after_departure = False
        self.detail = "waiting_for_task"

    @property
    def current_job(self) -> Optional[MissionJob]:
        """Return the active job, if any."""
        if 0 <= self.job_offset < len(self.jobs):
            return self.jobs[self.job_offset]
        return None

    @property
    def terminal(self) -> bool:
        """Return whether no further mission events are accepted."""
        return self.state in {"completed", "failed", "canceled"}

    def start(self, plan: TaskPlan) -> MissionAction:
        """Start a new validated task plan."""
        if self.state not in {"idle", "completed", "failed", "canceled"}:
            raise MissionValueError("A mission is already active")
        self.plan = plan
        self.jobs = expand_jobs(plan, self.maximum_cubes_per_color)
        self.job_offset = 0
        self.completed_count = 0
        self.skipped_count = 0
        self.attempts = {}
        self.selected_target = None
        self.pending_skip_reason = None
        self.pending_zone_exit_skipped = False
        self.pending_zone_exit_reason = None
        self.skip_after_departure = False
        self.switch_after_departure = False
        return self._navigate_pickup()

    def _navigate_pickup(self) -> MissionAction:
        job = self._require_job()
        self.state = "navigating_pickup"
        self.detail = f"going_to_{job.color}_pickup"
        return MissionAction(
            "navigate",
            location=job.color.upper(),
            job=job,
        )

    def navigation_finished(self, success: bool) -> MissionAction:
        """Handle the terminal result of the active navigation."""
        job = self._require_job()
        if self.state == "navigating_pickup":
            if not success:
                # The base never reached the dock, so the tray-departure
                # assumption below does not hold: skip advances directly
                # and the next Nav2 leg plans from the current position.
                return self._retry_or_skip(
                    "pickup_navigation",
                    "pickup_navigation_failed",
                    self._navigate_pickup,
                    carrying=False,
                    docked=False,
                )
            self.attempts.pop("pickup_navigation", None)
            self.state = "waiting_perception"
            self.detail = f"recognizing_{job.color}"
            return MissionAction("perceive", job=job)
        if self.state == "navigating_departure":
            if not success:
                return self._retry_or_skip(
                    "departure_navigation",
                    "departure_navigation_failed",
                    self._navigate_departure,
                    carrying=not (
                        self.skip_after_departure
                        or self.switch_after_departure
                    ),
                )
            self.attempts.pop("departure_navigation", None)
            if self.switch_after_departure:
                self.switch_after_departure = False
                return self._navigate_pickup()
            if self.skip_after_departure:
                self.skip_after_departure = False
                reason = self.pending_skip_reason or "pickup_skipped"
                self.pending_skip_reason = None
                return self._advance(skipped=True, reason=reason)
            return self._navigate_destination()
        if self.state == "navigating_destination":
            if not success:
                return self._retry_or_skip(
                    "destination_navigation",
                    "destination_navigation_failed",
                    self._navigate_destination,
                    carrying=True,
                )
            self.attempts.pop("destination_navigation", None)
            self.state = "placing"
            self.detail = f"placing_in_zone_{job.target_zone}"
            return MissionAction("place", job=job)
        if self.state == "navigating_zone_exit":
            if not success:
                attempts = self.attempts.get("zone_exit_navigation", 0) + 1
                self.attempts["zone_exit_navigation"] = attempts
                if attempts <= self.maximum_retries:
                    return self._navigate_zone_exit()
                self.state = "failed"
                self.detail = "zone_exit_navigation_failed"
                return MissionAction("done", reason=self.detail)
            self.attempts.pop("zone_exit_navigation", None)
            skipped = self.pending_zone_exit_skipped
            reason = self.pending_zone_exit_reason or "zone_exit_complete"
            self.pending_zone_exit_skipped = False
            self.pending_zone_exit_reason = None
            return self._advance(skipped=skipped, reason=reason)
        raise MissionValueError(
            f"Navigation result is invalid while state={self.state}"
        )

    def perception_finished(
        self,
        target: Optional[Mapping[str, Any]],
    ) -> MissionAction:
        """Use a fresh reachable detection or retry/skip the current cube."""
        job = self._require_job()
        if self.state != "waiting_perception":
            raise MissionValueError(
                f"Perception result is invalid while state={self.state}"
            )
        if target is None:
            return self._retry_or_skip(
                "perception",
                "target_unavailable",
                self._retry_perception,
                carrying=False,
            )
        self.attempts.pop("perception", None)
        self.selected_target = target
        self.state = "picking"
        self.detail = f"picking_{job.cube_name}"
        return MissionAction("pick", job=job, target=target)

    def manipulation_finished(self, success: bool) -> MissionAction:
        """Handle pick, place, or safety-release completion."""
        if self.state == "picking":
            if not success:
                return self._retry_or_skip(
                    "pick",
                    "pick_failed",
                    self._retry_pick,
                    carrying=False,
                )
            self.attempts.pop("pick", None)
            return self._navigate_departure()
        if self.state == "placing":
            if not success:
                return self._retry_or_skip(
                    "place",
                    "place_failed",
                    self._retry_place,
                    carrying=True,
                )
            self.attempts.pop("place", None)
            self.state = "verifying"
            self.detail = "verifying_delivery"
            return MissionAction("verify", job=self._require_job())
        if self.state == "releasing":
            reason = self.pending_skip_reason or "safety_release"
            self.pending_skip_reason = None
            if reason == "mission_canceled":
                self.state = "canceled"
                self.detail = reason
                return MissionAction("done", reason=reason)
            return self._advance(skipped=True, reason=reason)
        raise MissionValueError(
            f"Manipulation result is invalid while state={self.state}"
        )

    def verification_finished(self, success: bool) -> MissionAction:
        """Count only a cube proven to be inside its requested zone."""
        if self.state != "verifying":
            raise MissionValueError(
                f"Verification is invalid while state={self.state}"
            )
        if success and self.job_offset + 1 >= len(self.jobs):
            # The final cube is already verified and no route follows it.
            # Keep intermediate zone exits to clear placed cubes, but do not
            # spend a navigation leg leaving the terminal scoring zone.
            return self._advance(
                skipped=False, reason="delivery_verified"
            )
        self.pending_zone_exit_skipped = not success
        self.pending_zone_exit_reason = (
            "delivery_verified"
            if success
            else "delivery_verification_failed"
        )
        return self._navigate_zone_exit()

    def cancel(self) -> MissionAction:
        """
        Cancel safely, releasing a cube only when the arm is mid-motion.

        Releasing protects the arm when it is inside a pick, place or
        verification motion and may be in a pose that is unsafe to hold.

        On a navigation leg the cube is already secured in the same transport
        pose used for every delivery, so the robot stops and keeps holding it.
        Dropping it there would put a cube at an arbitrary floor position and
        discard the evidence of a grasp that already succeeded -- and under
        2026 scoring grasping is worth 4 points per item independently of
        whether the placement completes.
        """
        arm_in_motion = self.state in {"picking", "placing", "verifying"}
        if arm_in_motion:
            self.state = "releasing"
            self.pending_skip_reason = "mission_canceled"
            self.detail = "releasing_after_cancel"
            return MissionAction(
                "release",
                job=self.current_job,
                reason="mission_canceled",
            )
        carrying_in_transit = self.state in {
            "navigating_departure",
            "navigating_destination",
        }
        self.state = "canceled"
        if carrying_in_transit:
            self.detail = "mission_canceled_holding_cube"
            return MissionAction(
                "done", reason="mission_canceled_holding_cube"
            )
        self.detail = "mission_canceled"
        return MissionAction("done", reason="mission_canceled")

    def _navigate_destination(self) -> MissionAction:
        job = self._require_job()
        self.state = "navigating_destination"
        self.detail = f"going_to_zone_{job.target_zone}"
        return MissionAction(
            "navigate",
            location=job.target_zone,
            job=job,
        )

    def _navigate_zone_exit(self) -> MissionAction:
        """Back away from the delivered cube before any next route."""
        job = self._require_job()
        self.state = "navigating_zone_exit"
        self.detail = f"leaving_zone_{job.target_zone}"
        return MissionAction(
            "navigate",
            location=f"{job.target_zone}_EXIT",
            job=job,
        )

    def _navigate_departure(self) -> MissionAction:
        job = self._require_job()
        self.state = "navigating_departure"
        self.detail = f"leaving_{job.color}_pickup"
        return MissionAction(
            "navigate",
            location=f"{job.color.upper()}_EXIT",
            job=job,
        )

    def _retry_perception(self) -> MissionAction:
        self.state = "waiting_perception"
        self.detail = "retrying_perception"
        return MissionAction("perceive", job=self._require_job())

    def _retry_pick(self) -> MissionAction:
        # A failed grasp often means that the RGB-D pose was stale or a
        # similarly coloured shelf feature was selected. Never repeat the
        # identical Cartesian pose: return to the scan pose and require a
        # fresh reachable target before the single retry.
        self.selected_target = None
        self.state = "waiting_perception"
        self.detail = "rescanning_after_pick_failure"
        return MissionAction("perceive", job=self._require_job())

    def pick_failed_switch_cube(self) -> MissionAction:
        """Leave the failed dock, then retry the same ordered item."""
        job = self._require_job()
        self.selected_target = None
        self.attempts.pop("pick", None)
        self.attempts.pop("perception", None)
        self.switch_after_departure = True
        self.skip_after_departure = False
        self.pending_skip_reason = None
        self.detail = f"switching_{job.color}_cube_after_pick_failure"
        return self._navigate_departure()

    def pickup_inventory_exhausted(self) -> MissionAction:
        """Skip an item when no unhandled cube of its colour remains."""
        if self.state != "navigating_pickup":
            raise MissionValueError(
                "Pickup inventory exhaustion is invalid while "
                f"state={self.state}"
            )
        self.attempts.pop("pickup_navigation", None)
        return self._advance(
            skipped=True,
            reason="pickup_inventory_exhausted",
        )

    def _retry_place(self) -> MissionAction:
        self.state = "placing"
        self.detail = "retrying_place"
        return MissionAction("place", job=self._require_job())

    def _retry_or_skip(
        self,
        key: str,
        reason: str,
        retry,
        carrying: bool,
        docked: bool = True,
    ) -> MissionAction:
        count = self.attempts.get(key, 0) + 1
        self.attempts[key] = count
        retry_limit = (
            self.maximum_perception_retries
            if key == "perception"
            else self.maximum_retries
        )
        if count <= retry_limit:
            return retry()
        self.attempts.pop(key, None)
        if not self.allow_skips:
            self.state = "failed"
            self.detail = reason + (":holding_cube" if carrying else "")
            return MissionAction("done", reason=self.detail)
        if carrying:
            self.state = "releasing"
            self.pending_skip_reason = reason
            self.detail = f"safety_release:{reason}"
            return MissionAction(
                "release",
                job=self._require_job(),
                reason=reason,
            )
        if docked and key in {"perception", "pick", "pickup_navigation"}:
            # The base is parked at the per-cube dock inside the tray's
            # costmap inflation. Drive back to the tray-clear entry first so
            # the next Nav2 leg can plan from un-inflated space.
            self.skip_after_departure = True
            self.pending_skip_reason = reason
            return self._navigate_departure()
        return self._advance(skipped=True, reason=reason)

    def _advance(self, skipped: bool, reason: str) -> MissionAction:
        if skipped and not self.allow_skips:
            self.state = "failed"
            self.detail = reason
            return MissionAction("done", reason=reason)
        if skipped:
            self.skipped_count += 1
        else:
            self.completed_count += 1
        self.job_offset += 1
        self.attempts = {}
        self.selected_target = None
        self.switch_after_departure = False
        self.detail = reason
        if self.job_offset >= len(self.jobs):
            if (
                self.completed_count == len(self.jobs)
                and self.skipped_count == 0
            ):
                self.state = "completed"
                self.detail = (
                    f"completed={self.completed_count},"
                    f"skipped={self.skipped_count}"
                )
            else:
                self.state = "failed"
                self.detail = (
                    f"incomplete:completed={self.completed_count},"
                    f"skipped={self.skipped_count},"
                    f"total={len(self.jobs)}"
                )
            return MissionAction("done", reason=self.detail)
        return self._navigate_pickup()

    def _require_job(self) -> MissionJob:
        job = self.current_job
        if job is None:
            raise MissionValueError("Mission has no active job")
        return job

    def snapshot(self) -> Dict[str, Any]:
        """Return deterministic public state for dashboard and announcer."""
        job = self.current_job
        return {
            "schema_version": "1.0",
            "task_id": self.plan.task_id if self.plan else None,
            "state": self.state,
            "detail": self.detail,
            "current_index": job.index if job else len(self.jobs),
            "total": len(self.jobs),
            "completed": self.completed_count,
            "skipped": self.skipped_count,
            "color": job.color if job else None,
            "cube_name": job.cube_name if job else None,
            "target_zone": job.target_zone if job else None,
        }
