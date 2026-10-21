"""Coordinate ordered navigation, perception, and manipulation."""

import json
import math
import struct
import time
from dataclasses import replace
from pathlib import Path
from typing import Optional, Sequence

import yaml

from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import GetEntityState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from std_msgs.msg import Bool
from std_msgs.msg import String

from tools_demo.mission_state_machine import MissionAction
from tools_demo.mission_state_machine import MissionJob
from tools_demo.mission_state_machine import MissionStateMachine
from tools_demo.mission_state_machine import MissionValueError
from tools_demo.mission_state_machine import selected_cube_snapshot
from tools_demo.navigation_math import APPROACH_EAST
from tools_demo.navigation_math import APPROACH_WEST
from tools_demo.navigation_math import UNIVERSAL_SPAWN_MAX_Y
from tools_demo.navigation_math import UNIVERSAL_SPAWN_MIN_X
from tools_demo.navigation_math import UNIVERSAL_SPAWN_MIN_Y
from tools_demo.navigation_math import ApproachCandidate
from tools_demo.navigation_math import approach_dock
from tools_demo.navigation_math import approach_entry
from tools_demo.navigation_math import bounded_dock_realign
from tools_demo.navigation_math import DOCK_REALIGN_TARGET_X
from tools_demo.navigation_math import choose_approach
from tools_demo.navigation_math import cube_delivery_cost
from tools_demo.navigation_math import staged_cube_delivery_cost
from tools_demo.navigation_math import delivery_leg_estimate
from tools_demo.navigation_math import CUBE_APPROACH_SIDES
from tools_demo.navigation_math import VALID_ZONES
from tools_demo.navigation_math import angle_delta
from tools_demo.navigation_math import prefer_same_side_approaches
from tools_demo.navigation_math import ZonePose
from tools_demo.mission_state_machine import parse_task_plan_json
from tools_demo.mission_state_machine import select_reachable_target
from tools_demo.pick_order_policy import ordered_task_key
from tools_demo.pick_order_policy import validate_pick_entries
from tools_demo.unordered_schedule import optimize_schedule
from tools_demo.transport_schedule import (
    TransportTimeEstimator, centered_slot_offset, load_timing_profile, route_estimate,
)


TERMINAL_NAVIGATION_STATES = {
    "succeeded",
    "canceled",
    "aborted",
    "failed",
    "timed_out",
}


# Known static cube positions in offic_room.world. The mission must select the
# detection that matches the CURRENT job slot; several same-colour cubes are
# in view at once and picking by colour alone can select the wrong cube.
CUBE_WORLD_SLOTS = {
    "red_cube_1": (-7.5000, -2.8000),
    "red_cube_2": (-0.8000, 0.8000),
    "red_cube_3": (0.0000, 3.8000),
    "red_cube_4": (11.4000, -9.8000),
    "red_cube_5": (-4.7000, -12.4000),
    "blue_cube_1": (7.4000, 3.6000),
    "blue_cube_2": (-11.0000, -5.6000),
    "blue_cube_3": (-2.0000, -6.0000),
    "blue_cube_4": (-0.5000, -8.8000),
    "blue_cube_5": (11.8000, -13.0000),
}


# Distance kept between base_link and the cube centre at a per-cube dock.
# 0.36 m centres the cube in the arm reach window (0.285 - 0.430 m) and
# leaves 0.08 m between the 0.32 m chassis front and the tray face.
CUBE_DOCK_STANDOFF = 0.36

UNIVERSAL_ORIGIN_STAGING_POSE = (0.0, 0.0, 0.0)
# Spawn-box bounds live in navigation_math and are imported below with the
# other shared geometry helpers, so both consumers cannot drift apart.


def _default_cube_order_policy_file() -> str:
    """Locate the shipped offline policy table, if installed."""
    try:
        return str(
            Path(get_package_share_directory("tools_demo"))
            / "config"
            / "cube_order_policy.json"
        )
    except Exception:
        return ""


# Cube centre height in base_link (tray-supported world z=0.13 minus the
# 0.1034 m base_link offset).
CUBE_SLOT_BASE_Z = 0.0266
# A refused placement whose slot and measured base pose have not changed is
# deterministic: the same request reaches the same rejection. v15 proved it
# (two identical `Slot outside checked placement workspace` refusals 0.065 s
# apart). The retry is therefore only allowed once the geometry actually
# moved; otherwise the mission fails closed with the cube retained.
PLACE_REPEAT_POSE_TOLERANCE = 0.010
PLACE_REPEAT_YAW_TOLERANCE = 0.020
# Keep a delivery slot at least this far inside the pad edge after the
# approach bias, so the cube never straddles the painted boundary.
DELIVERY_SLOT_EDGE_MARGIN = 0.10
# Aim the delivery extension at the middle of the arm's 0.25-0.38 m
# checked window, leaving margin on both bounds for docking error.
DELIVERY_REACH_TARGET = 0.33
# Arm-reported reasons that are deterministic in the delivered geometry.
DETERMINISTIC_PLACE_FAILURES = ("slot_unreachable", "dock_too_far")
CENTERED_PICK_FORWARD_COMPENSATION = 0.045
# A lateral offset above this triggers a base centring move, which in turn
# applies the forward compensation above. Two constraints set the value:
#
#   - It must stay well inside GRASP_XY_LIMIT (0.035 in pick_place_moveit), or
#     the arm would be asked to grasp outside its safe window. Measured
#     successful grasps sit at y = 0.009-0.014 with no centring at all.
#   - It must not be so tight that a small offset triggers a centring move,
#     because the +0.045 m compensation is calibrated for a much larger
#     correction. At 0.012, offsets of 12.4-16.0 mm drove 34-37 mm centring
#     moves and every one of them overshot to tool x = 0.0587-0.0596 against
#     the 0.035 limit, failing the grasp and forcing a rescan.
#
# The cached world-slot estimate also carries roughly 15 mm of odometry error,
# so it must not be trusted to decide this either (see the cached-path gate).
CENTERING_LATERAL_THRESHOLD = 0.033

# Lateral band allowed for a slot-fallback grasp after scan retries are
# exhausted. The fallback pick still runs lateral centring, so a few cm of
# dock odometry error (observed 2-3 cm on red_cube_1 at y=3 shelf) must not
# turn a reachable cube into a skip. 0.10 m is well inside the arm window
# while keeping the grasp valid (0.19 m previously failed validation).
SLOT_FALLBACK_MAX_ABS_Y = 0.10

# A refused descend (approach_rejected) means the target data did not match the
# cube's live pose, not that the grasp failed. Baseline policy_20260918T190835
# descended onto slot-fallback targets 0.78 m and 2.01 m away from the cube;
# each wasted descend counted as a grasp failure, and a second failure excluded
# the cube, which is how 1bA_4rC reached pickup_inventory_exhausted with only
# three cubes delivered. Refusals are therefore bounded separately and do not
# consume the grasp-failure budget that controls exclusion.
APPROACH_REJECTION_LIMIT = 2



def selection_pose_after_universal_staging(
    robot_pose: Optional[Sequence[float]],
) -> tuple:
    """Score the first cube from the mandatory origin staging pose."""
    if robot_pose is None:
        return UNIVERSAL_ORIGIN_STAGING_POSE
    pose = tuple(float(value) for value in robot_pose)
    if len(pose) != 3 or not all(math.isfinite(value) for value in pose):
        raise ValueError("robot_pose must contain three finite values")
    if (
        pose[0] >= UNIVERSAL_SPAWN_MIN_X
        and UNIVERSAL_SPAWN_MIN_Y <= pose[1] <= UNIVERSAL_SPAWN_MAX_Y
    ):
        return UNIVERSAL_ORIGIN_STAGING_POSE
    return pose

def world_target_in_base(
    robot_pose: Sequence[float],
    target_xy: Sequence[float],
) -> tuple:
    """Transform one world-frame delivery target into the robot frame."""
    robot_x, robot_y, robot_yaw = (float(value) for value in robot_pose)
    delta_x = float(target_xy[0]) - robot_x
    delta_y = float(target_xy[1]) - robot_y
    cosine = math.cos(robot_yaw)
    sine = math.sin(robot_yaw)
    return (
        cosine * delta_x + sine * delta_y,
        -sine * delta_x + cosine * delta_y,
    )


def centering_time_budget(
    target_y: float,
    commanded_speed: float = 0.07,
) -> float:
    """
    Return the wall-clock budget for one lateral-centring move.

    The loaded v2 base may reach only ~50% of commanded lateral speed, so
    a target/commanded-speed budget expires short and the outer shelf
    slots (|y| up to 0.24 m) keep a residual. Budget against the observed
    speed; the closed loop exits as soon as the odometry travel reaches
    the target.
    """
    # Gazebo measurements with a loaded arm show that the planar base reaches
    # about 40% of the requested lateral speed.  Budget against the measured
    # rate so cached detections are centred on the gripper before committing
    # to a grasp, including the outermost cube slots.
    observed_speed = 0.40 * abs(float(commanded_speed))
    if observed_speed <= 0.0:
        raise ValueError("commanded_speed must be non-zero")
    return min(abs(float(target_y)) / observed_speed, 4.0)


def two_leg_navigation_cost(
    start_xy: Sequence[float],
    pickup_xy: Sequence[float],
    delivery_xy: Sequence[float],
) -> float:
    """Return start-to-pickup plus pickup-to-delivery path distance."""
    values = tuple(
        float(value)
        for point in (start_xy, pickup_xy, delivery_xy)
        for value in point[:2]
    )
    if len(values) != 6 or not all(math.isfinite(value) for value in values):
        raise ValueError("Two-leg navigation points must be finite XY pairs")
    start_x, start_y, pickup_x, pickup_y, delivery_x, delivery_y = values
    return math.hypot(pickup_x - start_x, pickup_y - start_y) + (
        delivery_leg_estimate(
            (pickup_x, pickup_y),
            (delivery_x, delivery_y),
        )
    )


BLUE_C_ROUTE_ORIGINS = {
    "A_EXIT": (-2.86, 3.20),
    # Zone C moved to (-9, -7): the exit origin follows the new bay.
    # The C_EXIT cost column below is inherited measured data from the
    # former bay and still needs re-calibration with a real C round.
    "C_EXIT": (-9.85, -5.80),
}
BLUE_C_STATIC_PATH_COSTS = {
    "A_EXIT": {
        "blue_cube_1": 20.35,
        "blue_cube_2": 31.71,
        "blue_cube_3": 36.83,
        "blue_cube_4": 38.20,
        "blue_cube_5": 46.65,
    },
    "C_EXIT": {
        "blue_cube_1": 28.45,
        "blue_cube_2": 35.00,
        "blue_cube_3": 43.48,
        "blue_cube_4": 44.88,
        "blue_cube_5": 53.40,
    },
}
BLUE_C_ROUTE_ORIGIN_TOLERANCE = 2.50

# With multiple blue-to-B rounds left immediately after zone A, a greedy
# first choice can leave a lower tray for the next, longer round from zone B.
# Break only a near tie at the measured A exit and take the best lower-row
# candidate first. Requested colour and destination order remain unchanged.
BLUE_B_LOOKAHEAD_ORIGIN = (-2.86, 3.54)
BLUE_B_LOOKAHEAD_ORIGIN_TOLERANCE = 2.50
BLUE_B_LOOKAHEAD_MAX_COST_GAP = 1.00



BLUE_B_SPAWN_ORIGIN = (15.0, -2.0)
BLUE_B_SPAWN_ORIGIN_TOLERANCE = 2.50
BLUE_B_SPAWN_FIRST_CUBE = "blue_cube_5"


def blue_b_spawn_first_choice(
    robot_pose,
    remaining_same_color_jobs: int,
    scored: Sequence[dict],
):
    """Choose blue 5 only at the opening of an all-blue-to-B mission."""
    if robot_pose is None or int(remaining_same_color_jobs) != 5 or not scored:
        return None
    robot_x, robot_y = (float(value) for value in robot_pose[:2])
    if not all(math.isfinite(value) for value in (robot_x, robot_y)):
        return None
    if math.hypot(
        robot_x - BLUE_B_SPAWN_ORIGIN[0],
        robot_y - BLUE_B_SPAWN_ORIGIN[1],
    ) > BLUE_B_SPAWN_ORIGIN_TOLERANCE:
        return None
    available = {item.get("cube") for item in scored}
    if BLUE_B_SPAWN_FIRST_CUBE not in available:
        return None
    return BLUE_B_SPAWN_FIRST_CUBE

def blue_b_first_round_lookahead(
    robot_pose,
    remaining_same_color_jobs: int,
    scored: Sequence[dict],
):
    """Return a near-tied lower blue candidate for the first A-to-B round."""
    if robot_pose is None or int(remaining_same_color_jobs) < 2 or not scored:
        return None
    robot_x, robot_y = (float(value) for value in robot_pose[:2])
    if not all(math.isfinite(value) for value in (robot_x, robot_y)):
        return None
    if math.hypot(
        robot_x - BLUE_B_LOOKAHEAD_ORIGIN[0],
        robot_y - BLUE_B_LOOKAHEAD_ORIGIN[1],
    ) > BLUE_B_LOOKAHEAD_ORIGIN_TOLERANCE:
        return None
    ordered = sorted(
        scored,
        key=lambda item: (item["total"], item["lateral"], item["cube"]),
    )
    lower = [item for item in ordered if item.get("group") == "lower"]
    if not lower:
        return None
    if lower[0]["total"] > ordered[0]["total"] + BLUE_B_LOOKAHEAD_MAX_COST_GAP:
        return None
    return lower[0]["cube"]

# The red-3 entry is geometrically free but its real A-exit route grazes the
# rock/tray inflation corner. A successful physical run spent 32 seconds in
# RPP collision prediction there (49.7 seconds for the pickup leg), while
# other available red slots completed the same round without that stall.
# Blue cubes 4/5 sit on the far east/south trays; delivering them to zone C
# needs a long diagonal bypass of the moving obstacle-2 sweep (observed
# ~42 s versus ~21 s for the west/north trays), so ordering prefers the
# closer west/north blue cubes when the task allows a free choice.
# Red cube 5 is the far south-west tray (-4.36,-11): with four reds needed
# out of five available, preferring the closer four keeps the round under
# 300 s even under VM load (observed 276.2 s without it vs 312.4 s when the
# far tray was chosen as the fourth red).
# Map-calibrated red-to-C costs from the same A/C exits used for C.
# Euclidean scoring undervalues the south-west C bay: a (0,3) tray looks
# only ~13 m from C while the real Nav2 corridor is ~35-45 s. Prefer the
# trays that already sit south/west of the central wall.
RED_C_STATIC_PATH_COSTS = {
    "A_EXIT": {
        "red_cube_1": 28.0,
        "red_cube_2": 36.0,
        "red_cube_3": 22.0,
        "red_cube_4": 30.0,
        "red_cube_5": 32.0,
    },
    "C_EXIT": {
        "red_cube_1": 18.0,
        "red_cube_2": 28.0,
        "red_cube_3": 12.0,
        "red_cube_4": 22.0,
        "red_cube_5": 16.0,
    },
}

CUBE_ROUTE_RISK_PENALTIES = {
    ("A", "red_cube_3"): 40.0,
    ("C", "blue_cube_4"): 30.0,
    ("C", "blue_cube_5"): 30.0,
    # B-to-blue5 must use the obstacle_2 west bypass and then traverse the
    # full lower-east corridor. One measured safe round consumed 93.4 s for
    # this final job while blue2 was incorrectly scored as more expensive.
    # A task needs only four of five blue cubes, so leave this longest spare
    # unused instead of paying the unsafe shortcut or its full safe detour.
    ("B", "blue_cube_5"): 30.0,
    ("A", "red_cube_5"): 25.0,
    ("B", "red_cube_5"): 25.0,
    ("C", "red_cube_5"): 25.0,
}


def red_c_static_path_cost(robot_pose, cube_name: str):
    """Return map-calibrated red-to-C route cost near a known A/C exit."""
    values = tuple(float(value) for value in robot_pose[:2])
    if len(values) != 2 or not all(math.isfinite(value) for value in values):
        raise ValueError("Red-C route scoring needs a finite robot XY pose")
    robot_x, robot_y = values
    nearest = min(
        BLUE_C_ROUTE_ORIGINS,
        key=lambda origin: math.hypot(
            robot_x - BLUE_C_ROUTE_ORIGINS[origin][0],
            robot_y - BLUE_C_ROUTE_ORIGINS[origin][1],
        ),
    )
    distance = math.hypot(
        robot_x - BLUE_C_ROUTE_ORIGINS[nearest][0],
        robot_y - BLUE_C_ROUTE_ORIGINS[nearest][1],
    )
    if distance > BLUE_C_ROUTE_ORIGIN_TOLERANCE:
        return None
    return RED_C_STATIC_PATH_COSTS[nearest].get(cube_name.lower())


def cube_route_risk_penalty(target_zone: str, cube_name: str) -> float:
    """Return a measured route-risk penalty used only for cube ordering."""
    return CUBE_ROUTE_RISK_PENALTIES.get(
        (str(target_zone).upper(), str(cube_name).lower()),
        0.0,
    )


def blue_c_static_path_cost(robot_pose, cube_name: str):
    """Return map-calibrated route cost near a known A/C exit."""
    values = tuple(float(value) for value in robot_pose[:2])
    if len(values) != 2 or not all(math.isfinite(value) for value in values):
        raise ValueError("Blue-C route scoring needs a finite robot XY pose")
    robot_x, robot_y = values
    nearest = min(
        BLUE_C_ROUTE_ORIGINS,
        key=lambda name: math.hypot(
            robot_x - BLUE_C_ROUTE_ORIGINS[name][0],
            robot_y - BLUE_C_ROUTE_ORIGINS[name][1],
        ),
    )
    distance = math.hypot(
        robot_x - BLUE_C_ROUTE_ORIGINS[nearest][0],
        robot_y - BLUE_C_ROUTE_ORIGINS[nearest][1],
    )
    if distance > BLUE_C_ROUTE_ORIGIN_TOLERANCE:
        return None
    return BLUE_C_STATIC_PATH_COSTS[nearest].get(cube_name.lower())


def cube_selection_cost(
    base_cost: float,
    lateral_offset: float,
    failure_count: int,
    hardest_first: bool = False,
    group_preference: float = 0.0,
) -> float:
    """
    Score one pickup candidate: navigation cost plus grasp-side costs.

    Large missions take the outer, high-offset cubes first while Gazebo and
    the repeated arm constraints are freshest. Small missions retain the
    shortest-centering-first behaviour. Each prior failure remains costly.
    ``group_preference`` is a soft bonus (in metres) for a delivery-zone
    group preference (C -> upper blocks, A/B -> lower blocks); it must never
    be large enough to override an obviously better or unreachable candidate.
    """
    lateral_cost = 20.0 * abs(float(lateral_offset))
    if hardest_first:
        lateral_cost = -lateral_cost
    return (
        float(base_cost)
        + lateral_cost
        + 8.0 * int(failure_count)
        - float(group_preference)
    )


def reliable_transient_qos() -> QoSProfile:
    """Return QoS used by public task and mission state topics."""
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


class MissionCoordinator(Node):
    """Execute each requested cube in strict task-plan order."""

    def __init__(self) -> None:
        super().__init__("competition_mission_coordinator")
        self.declare_parameter("plan_topic", "/competition/task/plan")
        # Experiment/policy hook: forced pick order, e.g.
        # ["blue_cube_1:east", ...]. Declared with an explicit STRING_ARRAY
        # type: an empty-list default would leave the runtime type unset
        # and ros2 param set would silently fail (exit 0) on type mismatch.
        self.declare_parameter(
            "cube_order_override",
            descriptor=ParameterDescriptor(
                name="cube_order_override",
                type=Parameter.Type.STRING_ARRAY.value,
            ),
        )
        # Offline policy table (pure function of the parsed task). Empty
        # string disables the lookup and keeps the shipped selector.
        self.declare_parameter(
            "cube_order_policy_file",
            _default_cube_order_policy_file(),
        )
        self.declare_parameter("transport_schedule_enabled", False)
        self.declare_parameter("transport_replan_minimum_gain_sec", 8.0)
        self.declare_parameter("transport_uncertainty_weight", 0.15)
        self.declare_parameter("transport_timing_profile_file", "")
        self.declare_parameter(
            "perception_topic",
            "/competition/perception/targets",
        )
        self.declare_parameter(
            "navigation_goal_topic",
            "/competition/navigation/goal",
        )
        self.declare_parameter(
            "navigation_cancel_topic",
            "/competition/navigation/cancel",
        )
        self.declare_parameter(
            "navigation_state_topic",
            "/competition/navigation/state",
        )
        self.declare_parameter(
            "mission_state_topic",
            "/competition/mission/state",
        )
        self.declare_parameter(
            "mission_error_topic",
            "/competition/mission/error",
        )
        self.declare_parameter(
            "mission_cancel_topic",
            "/competition/mission/cancel",
        )
        self.declare_parameter("maximum_retries", 1)
        self.declare_parameter("maximum_perception_retries", 1)
        self.declare_parameter("maximum_cubes_per_color", 5)
        self.declare_parameter("enforce_color_order", False)
        self.declare_parameter("use_legacy_selection_costs", True)
        self.declare_parameter("cube_approach_allowlist", [""])
        self.cube_approach_allowlist = set(
            self.get_parameter("cube_approach_allowlist").value)
        self.use_legacy_selection_costs = bool(
            self.get_parameter("use_legacy_selection_costs").value)
        # Soft group preference (metres of path-cost equivalent) for
        # delivery zones: C prefers upper blocks, A/B prefer lower blocks.
        self.declare_parameter("cube_group_preference_bonus", 0.0)
        self.declare_parameter("perception_timeout_sec", 2.0)
        # Bounded recovery scans run only after ordinary fresh-frame retries
        # are exhausted; they never turn an unobserved slot into a blind
        # grasp. Several cycles are needed because one bounded realign moves
        # the base ~0.023 m while real dock errors reach 0.19 m (234 samples:
        # median 0.038, p90 0.073, max 0.192), and the fallback grasp band is
        # 0.10 m. Each cycle reuses the already validated realign caps.
        self.declare_parameter("perception_recovery_rescans", 5)
        self.declare_parameter("manipulation_timeout_sec", 60.0)
        self.declare_parameter("mission_timeout_sec", 300.0)
        # A cancel that waits on a manipulation release must not gate the
        # mission watchdog forever. Observed 20261013T180611: cancel_pending
        # stayed set after a MoveIt 'Cannot push a new trajectory' pick
        # failure, timer_callback skipped the five-minute deadline, and the
        # round hung at navigating_departure with no message.
        self.declare_parameter("cancel_timeout_sec", 15.0)
        self.declare_parameter("manipulation_velocity_scaling", 0.50)
        self.declare_parameter("manipulation_acceleration_scaling", 0.50)
        self.declare_parameter("manipulation_cartesian_duration", 1.5)
        self.declare_parameter("manipulation_gripper_duration", 0.75)
        self.declare_parameter("manipulation_pick_transit_height", 0.180)
        self.declare_parameter("manipulation_pick_lift_duration", 1.1)
        self.declare_parameter("manipulation_transport_x", 0.280)
        self.declare_parameter("manipulation_transport_duration", 1.2)
        self.declare_parameter("reachable_minimum_x", 0.285)
        self.declare_parameter("reachable_maximum_x", 0.430)
        self.declare_parameter("reachable_maximum_abs_y", 0.30)
        self.declare_parameter("reachable_minimum_z", 0.005)
        self.declare_parameter("pickup_preferred_y", 0.045)
        self.declare_parameter(
            "pick_tool_offset",
            [0.010, 0.001, 0.025],
        )
        self.declare_parameter("pick_safe_lift", 0.070)
        self.declare_parameter("scan_pose", [0.300, -0.030, 0.390])
        self.declare_parameter(
            "scan_tool_orientation",
            [-0.2369, 0.9667, 0.0225, 0.0946],
        )
        self.declare_parameter(
            "pick_tool_orientation",
            [0.0, 1.0, 0.0, 0.0],
        )
        self.declare_parameter("place_pose", [0.350, 0.000, 0.160])
        self.declare_parameter(
            "place_safe_pose",
            [0.350, 0.000, 0.200],
        )
        self.declare_parameter("place_slot_spacing", 0.080)
        self.declare_parameter("place_minimum_x", 0.290)
        self.declare_parameter("place_maximum_x", 0.350)
        self.declare_parameter("place_maximum_abs_y", 0.160)
        self.declare_parameter("zone_a_center", [-5.4, 3.8])
        self.declare_parameter("zone_b_center", [11.7, -5.0])
        self.declare_parameter("zone_c_center", [3.3, -12.1])
        self.declare_parameter("pickup_red_dock", [-2.40, -1.00, 0.0])
        self.declare_parameter("pickup_blue_dock", [-0.40, -1.00, 0.0])
        # Per-cube docks: the 2026 layout distributes cubes over y in
        # [-7, 3], so a single shared per-colour dock cannot reach them.
        # Each dock approaches its cube from the west with the cube
        # 0.36 m ahead of base_link.
        for cube_name, (slot_x, slot_y) in CUBE_WORLD_SLOTS.items():
            self.declare_parameter(
                f"{cube_name}_dock",
                [round(slot_x - CUBE_DOCK_STANDOFF, 2), slot_y, 0.0],
            )
        self.declare_parameter(
            "pickup_red_exit_clear", [-2.80, 0.40, -1.5708]
        )
        self.declare_parameter(
            "pickup_blue_exit_clear", [-2.30, 0.40, -1.5708]
        )
        self.declare_parameter("zone_a_nav", [-5.0, 3.8, 3.141592653589793])
        self.declare_parameter("zone_b_nav", [11.299999999999999, -5.0, 0.0])
        self.declare_parameter("zone_c_nav", [2.9, -12.1, 0.0])
        self.declare_parameter("zone_half_extent", [0.485, 0.235])
        self.declare_parameter("delivery_surface_z", 0.020)
        self.declare_parameter("delivery_cube_half_height", 0.015)
        self.declare_parameter("delivery_height_tolerance", 0.10)
        self.declare_parameter("delivery_precision_tolerance", 0.06)
        # Pull the slot toward the approach side so the arm's checked 0.38 m
        # extension actually reaches it. The zone dock stops 0.40 m from the
        # pad centre and the goal tolerance adds up to 0.13 m, so without this
        # shift the cube is released short and set_entity_state finishes the
        # delivery. The shift stays well inside the 1.0 x 0.5 m pad.
        self.declare_parameter("delivery_slot_approach_bias", 0.13)
        self.declare_parameter("ros2_executable", "ros2")
        self.declare_parameter("manipulation_package", "mybot")
        self.declare_parameter(
            "manipulation_executable",
            "pick_place_moveit",
        )

        self.machine = MissionStateMachine(
            allow_skips=False,
            maximum_retries=int(
                self.get_parameter("maximum_retries").value
            ),
            maximum_perception_retries=int(
                self.get_parameter("maximum_perception_retries").value
            ),
            maximum_cubes_per_color=int(
                self.get_parameter("maximum_cubes_per_color").value
            ),
        )
        self.cube_group_preference_bonus = max(
            0.0,
            float(
                self.get_parameter("cube_group_preference_bonus").value
            ),
        )
        self.perception_timeout = self._positive_parameter(
            "perception_timeout_sec"
        )
        self.perception_recovery_rescans = int(
            self.get_parameter("perception_recovery_rescans").value
        )
        if self.perception_recovery_rescans < 0:
            raise ValueError("perception_recovery_rescans cannot be negative")
        self._perception_recovery_used = {}
        # Which path produced the current pick target, and when that data
        # was obtained; both travel with the manipulation request so a
        # refused descend can name the source and the age.
        self.target_source = "unknown"
        self.target_observed_at = None
        self.manipulation_timeout = self._positive_parameter(
            "manipulation_timeout_sec"
        )
        self.mission_timeout = self._positive_parameter(
            "mission_timeout_sec"
        )
        self.cancel_timeout = self._positive_parameter("cancel_timeout_sec")
        self.manipulation_velocity_scaling = self._unit_parameter(
            "manipulation_velocity_scaling"
        )
        self.manipulation_acceleration_scaling = self._unit_parameter(
            "manipulation_acceleration_scaling"
        )
        self.manipulation_cartesian_duration = self._positive_parameter(
            "manipulation_cartesian_duration"
        )
        self.manipulation_gripper_duration = self._positive_parameter(
            "manipulation_gripper_duration"
        )
        self.manipulation_pick_transit_height = self._positive_parameter(
            "manipulation_pick_transit_height"
        )
        self.manipulation_pick_lift_duration = self._positive_parameter(
            "manipulation_pick_lift_duration"
        )
        self.manipulation_transport_x = self._positive_parameter(
            "manipulation_transport_x"
        )
        self.manipulation_transport_duration = self._positive_parameter(
            "manipulation_transport_duration"
        )
        self.reachable_minimum_x = float(
            self.get_parameter("reachable_minimum_x").value
        )
        self.reachable_maximum_x = float(
            self.get_parameter("reachable_maximum_x").value
        )
        self.reachable_maximum_abs_y = float(
            self.get_parameter("reachable_maximum_abs_y").value
        )
        self.reachable_minimum_z = float(
            self.get_parameter("reachable_minimum_z").value
        )
        self.pickup_preferred_y = float(
            self.get_parameter("pickup_preferred_y").value
        )
        self.pick_tool_offset = self._vector_parameter(
            "pick_tool_offset", 3
        )
        self.pick_safe_lift = self._positive_parameter("pick_safe_lift")
        self.scan_pose = self._vector_parameter("scan_pose", 3)
        self.scan_tool_orientation = self._vector_parameter(
            "scan_tool_orientation",
            4,
        )
        self.pick_tool_orientation = self._vector_parameter(
            "pick_tool_orientation",
            4,
        )
        self.place_pose = self._vector_parameter("place_pose", 3)
        self.place_safe_pose = self._vector_parameter(
            "place_safe_pose", 3
        )
        self.place_slot_spacing = self._positive_parameter(
            "place_slot_spacing"
        )
        self.place_minimum_x = self._positive_parameter(
            "place_minimum_x"
        )
        self.place_maximum_x = self._positive_parameter(
            "place_maximum_x"
        )
        self.place_maximum_abs_y = self._positive_parameter(
            "place_maximum_abs_y"
        )
        self.zone_centers = {
            zone: self._vector_parameter(
                f"zone_{zone.lower()}_center", 2
            )
            for zone in ("A", "B", "C")
        }
        self.zone_half_extent = self._vector_parameter(
            "zone_half_extent", 2
        )
        self.delivery_surface_z = float(
            self.get_parameter("delivery_surface_z").value
        )
        self.delivery_cube_half_height = self._positive_parameter(
            "delivery_cube_half_height"
        )
        self.delivery_height_tolerance = self._positive_parameter(
            "delivery_height_tolerance"
        )
        self.delivery_precision_tolerance = self._positive_parameter(
            "delivery_precision_tolerance"
        )
        self.delivery_slot_approach_bias = float(
            self.get_parameter("delivery_slot_approach_bias").value
        )
        if not 0.0 <= self.delivery_slot_approach_bias < min(
            self.zone_half_extent[0], self.zone_half_extent[1]
        ) - DELIVERY_SLOT_EDGE_MARGIN:
            raise ValueError(
                "delivery_slot_approach_bias must keep the slot inside the pad"
            )
        self.zone_nav_poses = {
            zone: self._vector_parameter(
                f"zone_{zone.lower()}_nav", 3
            )
            for zone in ("A", "B", "C")
        }

        qos = reliable_transient_qos()
        self.state_publisher = self.create_publisher(
            String,
            str(self.get_parameter("mission_state_topic").value),
            qos,
        )
        self.error_publisher = self.create_publisher(
            String,
            str(self.get_parameter("mission_error_topic").value),
            qos,
        )
        self.navigation_goal_publisher = self.create_publisher(
            String,
            str(self.get_parameter("navigation_goal_topic").value),
            10,
        )
        self.navigation_cancel_publisher = self.create_publisher(
            Bool,
            str(self.get_parameter("navigation_cancel_topic").value),
            10,
        )
        self.cmd_vel_publisher = self.create_publisher(
            Twist, "/cmd_vel", 10
        )
        self.manipulation_request_publisher = self.create_publisher(
            String,
            "/competition/manipulation/request",
            10,
        )
        self.placed_cubes_publisher = self.create_publisher(
            PointCloud2,
            "/competition/placed_cubes",
            QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.BEST_EFFORT,
            ),
        )
        # Nav2 obstacle_layer subscribes with BEST_EFFORT sensor QoS, so a
        # one-shot RELIABLE publish was never received (the crossing-zone bug
        # remained). Publish BEST_EFFORT and re-publish at 1 Hz so the
        # already-placed cubes stay visible in the costmap all mission long.
        self._placed_cubes_timer = self.create_timer(
            1.0,
            self._publish_placed_cubes,
        )
        self.manipulation_result_subscription = self.create_subscription(
            String,
            "/competition/manipulation/result",
            self.manipulation_result_callback,
            10,
        )
        self.odom_pose: Optional[tuple] = None
        self.placed_cubes = {}
        self._odom_callback_group = MutuallyExclusiveCallbackGroup()
        self.create_subscription(
            Odometry,
            "/odom",
            self._odom_callback,
            QoSProfile(
                depth=5,
                reliability=ReliabilityPolicy.BEST_EFFORT,
            ),
            callback_group=self._odom_callback_group,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("plan_topic").value),
            self.plan_callback,
            qos,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("perception_topic").value),
            self.perception_callback,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("navigation_state_topic").value),
            self.navigation_state_callback,
            qos,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("mission_cancel_topic").value),
            self.cancel_callback,
            10,
        )
        self.entity_client = self.create_client(
            GetEntityState,
            "/gazebo/get_entity_state",
        )
        self.timer = self.create_timer(0.2, self.timer_callback)

        self.active_task_id: Optional[str] = None
        self.mission_started_at: Optional[float] = None
        self.mission_finished_at_unix_ms: Optional[int] = None
        self.action_started_at: Optional[float] = None
        self.perception_deadline: Optional[float] = None
        self.navigation_location: Optional[str] = None
        self.navigation_seen_active = False
        # Dynamic per-round cube selection state.
        self._round_cube_name: Optional[str] = None
        self._round_zone: Optional[str] = None
        self._cube_done = set()
        self._cube_excluded = set()
        self._cube_failures = {}
        self._approach_rejections = {}
        # Experiment hook: forced pick order (see _select_best_cube).
        # Empty by default; when set at runtime it consumes one entry per
        # pick and falls back to normal scoring for anything unusable.
        self._override_index = 0
        self._override_pending_side: Optional[str] = None
        # Offline policy table: task key -> ordered pick entries.
        self._cube_order_policy: dict = {}
        self._task_order_entries = None
        self.inventory_scan_complete = False
        self._perception_recovery_used = {}
        # Arm-reported residual between the checked extension endpoint and the
        # commanded slot; recorded in delivery_truth so the score can separate
        # a real placement from a pose written after release.
        self.last_place_arm_residual = None
        self.last_place_envelope_drift = None
        # Geometry of the last deterministic placement refusal, so a retry
        # that has not moved cannot repeat the identical request.
        self._refused_place_geometry = None
        self.manipulation_request_id = 0
        self.manipulation_pending_id: Optional[int] = None
        self.manipulation_active = False
        self.manipulation_operation: Optional[str] = None
        self.manipulation_job = None
        self.manipulation_deadline: Optional[float] = None
        self.manipulation_timed_out = False
        self.manipulation_result: Optional[dict] = None
        self.verification_future = None
        self.verification_cube_name: Optional[str] = None
        self.cancel_pending = False
        self.cancel_deadline: Optional[float] = None
        self.odom_pose = None
        self.pending_plan_message: Optional[String] = None
        self.publish_state()

    def _positive_parameter(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if value <= 0.0:
            raise ValueError(f"{name} must be positive")
        return value

    def _unit_parameter(self, name: str) -> float:
        value = self._positive_parameter(name)
        if value > 1.0:
            raise ValueError(f"{name} must not exceed 1.0")
        return value

    def _vector_parameter(
        self,
        name: str,
        length: int,
    ) -> tuple:
        values = tuple(float(item) for item in self.get_parameter(name).value)
        if len(values) != length:
            raise ValueError(f"{name} must contain {length} numbers")
        return values

    def publish_state(self) -> None:
        """Publish the complete dashboard-ready mission snapshot."""
        payload = self.machine.snapshot()
        if self._round_cube_name:
            payload = selected_cube_snapshot(payload, self._round_cube_name)
        elapsed = (
            time.monotonic() - self.mission_started_at
            if self.mission_started_at is not None
            else 0.0
        )
        payload["elapsed_sec"] = round(elapsed, 1)
        # Operator wall-clock endpoint, captured once at the actual transition.
        # ROS /clock, dashboard polling and later reconnects must not extend it.
        if self.active_task_id and self.machine.terminal:
            if self.mission_finished_at_unix_ms is None:
                self.mission_finished_at_unix_ms = time.time_ns() // 1_000_000
        payload["finished_at_unix_ms"] = self.mission_finished_at_unix_ms
        payload["enforce_color_order"] = bool(
            self.get_parameter("enforce_color_order").value
        )
        message = String()
        message.data = json.dumps(
            payload,
            ensure_ascii=False,
            separators=(",", ":"),
            sort_keys=True,
        )
        self.state_publisher.publish(message)
        self.get_logger().info(message.data)

    def publish_error(self, text: str) -> None:
        """Publish a public error without hiding it in process logs."""
        message = String()
        message.data = text
        self.error_publisher.publish(message)
        self.get_logger().error(text)

    def plan_callback(self, message: String) -> None:
        """Start one new task plan while rejecting unsafe replacement."""
        try:
            plan = parse_task_plan_json(message.data)
        except MissionValueError as exception:
            self.publish_error(str(exception))
            return
        if plan.task_id == self.active_task_id:
            return
        if self._odom_pose() is None:
            # A transient-local plan can arrive before the first /odom frame.
            # Defer the whole mission so cube selection uses the real start
            # pose instead of fixing a costly default order for all five jobs.
            if (
                self.pending_plan_message is None
                or self.pending_plan_message.data != message.data
            ):
                self.get_logger().info(
                    f"mission_start_waiting_for_odom task_id={plan.task_id}"
                )
            self.pending_plan_message = message
            return
        if not self.machine.terminal and self.machine.state != "idle":
            self.publish_error(
                f"Mission {self.active_task_id} is already active"
            )
            return
        try:
            action = self.machine.start(plan)
        except MissionValueError as exception:
            self.publish_error(str(exception))
            return
        self.active_task_id = plan.task_id
        self.mission_started_at = time.monotonic()
        self.mission_finished_at_unix_ms = None
        self.cancel_pending = False
        # Selection state is mission-scoped. Keeping it across task IDs can
        # make a second run treat every shelf slot as already consumed, while
        # stale perception still points at cubes delivered by the first run.
        self._round_cube_name = None
        self._round_zone = None
        self._cube_done.clear()
        self._cube_excluded.clear()
        self._cube_failures.clear()
        self._perception_recovery_used.clear()
        self.placed_cubes = {}
        self._publish_placed_cubes()
        self.inventory_scan_complete = False
        self._override_index = 0
        self._override_pending_side = None
        self._seed_cube_order_override()
        self._transport_automatic = bool(self.get_parameter("transport_schedule_enabled").value)
        self._transport_automatic = self._transport_automatic and not bool(self._cube_order_override_entries())
        self._transport_automatic = self._transport_automatic and not bool(self._task_order_entries)
        self._transport_responses_complete = False
        self._transport_epoch_sec = self.get_clock().now().nanoseconds * 1e-9
        if self._schedule_unordered_jobs():
            action = self.machine._navigate_pickup()
        self.handle_action(action)

    def navigation_state_callback(self, message: String) -> None:
        """Consume only states belonging to the goal just published."""
        if self.machine.state not in {
            "navigating_pickup",
            "navigating_departure",
            "navigating_destination",
            "navigating_zone_exit",
        }:
            return
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError:
            return
        if payload.get("zone") != self.navigation_location:
            return
        state = payload.get("state")
        if getattr(self, "_transport_automatic", False):
            epoch = payload.get("encounter_epoch_sec")
            responses = payload.get("verified_central_responses")
            if (payload.get("encounter_task_id") == self.active_task_id
                    and isinstance(epoch, (int, float)) and math.isfinite(epoch)
                    and epoch >= self._transport_epoch_sec
                    and isinstance(responses, list) and all(isinstance(r, str) for r in responses)
                    and set(responses)
                    >= {"crossing_cart_1", "crossing_cart_2"}):
                self._transport_responses_complete = True
        if state in {"sending", "active", "canceling", "timing_out"}:
            self.navigation_seen_active = True
            return
        if (
            state not in TERMINAL_NAVIGATION_STATES
            or not self.navigation_seen_active
        ):
            return
        self.navigation_location = None
        self.navigation_seen_active = False
        self.get_logger().info(
            f"nav_result state={state} elapsed="
            f"{time.monotonic() - self.action_started_at:.2f}s"
        )
        previous_state = self.machine.state
        failed_pick_departure = (previous_state == "navigating_departure"
                                 and self.machine.switch_after_departure)
        try:
            action = self.machine.navigation_finished(
                state == "succeeded"
            )
        except MissionValueError as exception:
            self.publish_error(str(exception))
            return
        if (state == "succeeded" and action.kind == "navigate"
                and action.location in {"RED", "BLUE"}
                and (previous_state == "navigating_zone_exit" or failed_pick_departure)
                and getattr(self, "_transport_automatic", False)):
            if previous_state == "navigating_zone_exit" and self._round_cube_name:
                self._cube_done.add(self._round_cube_name)
                self._round_cube_name = None
            if self._schedule_transport_jobs(replan=True):
                action = self.machine._navigate_pickup()
        if (
            state == "succeeded"
            and action.kind == "perceive"
            and self._round_cube_name is not None
        ):
            self._realign_dock_to_cached_slot(self._round_cube_name)
        self.handle_action(action)

    def perception_callback(self, message: String) -> None:
        """Select the fresh target corresponding to the current cube."""
        job = self._active_job()
        if (
            self.machine.state != "waiting_perception"
            or job is None
            or self.perception_deadline is None
        ):
            return
        try:
            payload = json.loads(message.data)
            targets = payload["targets"]
        except (json.JSONDecodeError, KeyError, TypeError):
            return
        if not isinstance(targets, Sequence) or isinstance(targets, str):
            return
        expected = self._expected_cube_base_position(job.cube_name)
        target = select_reachable_target(
            targets,
            color=job.color,
            minimum_x=self.reachable_minimum_x,
            maximum_x=self.reachable_maximum_x,
            maximum_abs_y=self.reachable_maximum_abs_y,
            minimum_z=self.reachable_minimum_z,
            preferred_y=self.pickup_preferred_y,
            expected_position=expected,
        )
        if target is None:
            # Do not turn one transient no-match frame into a blind slot
            # grasp. Later cubes may have been nudged; keep consuming fresh
            # RGB-D frames until the short deadline triggers a rescan.
            return
        self.get_logger().info(
            "selected_target:"
            f"{job.cube_name}:"
            f"{json.dumps(target['position'], sort_keys=True)}"
        )
        self.target_source = "perception"
        self.target_observed_at = time.monotonic()
        self.inventory_scan_complete = True
        self.perception_deadline = None
        self._perception_recovery_used.pop(job.cube_name, None)
        try:
            action = self.machine.perception_finished(target)
        except MissionValueError as exception:
            self.publish_error(str(exception))
            return
        self.handle_action(action)

    def cancel_callback(self, message: Bool) -> None:
        """Cancel navigation/process work and request a safe release."""
        if not message.data or self.machine.terminal:
            return
        # A repeated cancel must not extend the hard deadline or replace the
        # release request that is already queued behind an active operation.
        # The old code reset cancel_deadline to None on every callback, so a
        # noisy publisher could keep cancellation pending forever.
        if self.cancel_pending:
            return
        self.cancel_pending = True
        self.cancel_deadline = time.monotonic() + self.cancel_timeout
        if self.navigation_location is not None:
            cancel = Bool()
            cancel.data = True
            self.navigation_cancel_publisher.publish(cancel)
        if self.manipulation_active:
            self._send_manipulation_request(
                "release", self.manipulation_job, None
            )
            return
        self.finish_cancel()

    def finish_cancel(self) -> None:
        """Advance cancellation after any child process has stopped."""
        self.cancel_pending = False
        self.cancel_deadline = None
        try:
            action = self.machine.cancel()
        except MissionValueError as exception:
            self.publish_error(str(exception))
            return
        self.handle_action(action)

    def _expire_stuck_cancel(self, now: float) -> bool:
        """Finish a cancel whose release never reported back.

        cancel_pending gates the mission watchdog, so a release that never
        returns hangs the round with no deadline message at all.
        """
        if not self.cancel_pending:
            self.cancel_deadline = None
            return False
        if self.cancel_deadline is None:
            self.cancel_deadline = now + self.cancel_timeout
            return False
        if now < self.cancel_deadline:
            return False
        self.get_logger().warn(
            "cancel_release_timeout: finishing the cancel without the "
            "child result"
        )
        self.cancel_deadline = None
        self.finish_cancel()
        return True

    def timer_callback(self) -> None:
        """Enforce timeouts and poll the isolated manipulation process."""
        if (
            self.pending_plan_message is not None
            and self._odom_pose() is not None
        ):
            pending = self.pending_plan_message
            self.pending_plan_message = None
            self.get_logger().info("mission_start_odom_ready")
            self.plan_callback(pending)
            return
        now = time.monotonic()
        if self._expire_stuck_cancel(now):
            return
        if (
            self.mission_started_at is not None
            and not self.machine.terminal
            and now - self.mission_started_at >= self.mission_timeout
            and not self.cancel_pending
        ):
            self.publish_error("Mission exceeded the five-minute deadline")
            cancel = Bool()
            cancel.data = True
            self.cancel_callback(cancel)
            return

        if (
            self.machine.state == "waiting_perception"
            and self.perception_deadline is not None
            and now >= self.perception_deadline
        ):
            self.perception_deadline = None
            if self._attempt_no_target_recovery():
                return
            if self._attempt_slot_fallback():
                return
            action = self.machine.perception_finished(None)
            self.handle_action(action)
            return

        if not self.manipulation_active:
            return
        if self.manipulation_result is not None:
            result = self.manipulation_result
            operation = self.manipulation_operation
            success = bool(result.get("success"))
            success = success and not self.manipulation_timed_out
            self._reset_manipulation()
            if self._reject_unsafe_manipulation_departure(result, operation):
                return
            if self.cancel_pending:
                self.finish_cancel()
                return
            if operation == "scan":
                if success:
                    self.perception_deadline = (
                        time.monotonic() + self.perception_timeout
                    )
                    self.publish_state()
                else:
                    action = self.machine.perception_finished(None)
                    self.handle_action(action)
                return
            if operation not in {"pick", "place", "release"}:
                self.publish_error(
                    f"Unknown completed operation: {operation}"
                )
                return
            if operation == "place" and success:
                # A verified delivery does not disturb the remaining shelf,
                # and the arm now returns home in the background. Reuse the
                # known world slots on the next visit; any failed grasp still
                # invalidates this cache and forces a fresh RGB-D scan.
                self.inventory_scan_complete = True
            self._note_place_outcome(result, operation, success)
            if operation == "pick" and not success:
                # A disturbed or stale slot must be observed again before a
                # retry or any later cached pick.
                self.inventory_scan_complete = False
                cube = self._round_cube_name
                rejection_detail = str(result.get("detail", ""))
                if (
                    cube is not None
                    and rejection_detail.startswith("approach_rejected")
                ):
                    rejections = (
                        self._approach_rejections.get(cube, 0) + 1
                    )
                    self._approach_rejections[cube] = rejections
                    if "target_out_of_reach" in rejection_detail:
                        self._realign_dock_to_cached_slot(cube)
                    if rejections <= APPROACH_REJECTION_LIMIT:
                        # The manipulation node refused the descend because
                        # the requested target did not match the cube's live
                        # pose. That is bad target data, not a failed grasp,
                        # so it must not consume the failure budget that
                        # excludes a cube and can exhaust the inventory.
                        self.get_logger().warn(
                            f"cube_selection: approach rejected for "
                            f"{cube} ({rejection_detail}); rescanning "
                            f"same colour rejection={rejections}/"
                            f"{APPROACH_REJECTION_LIMIT}"
                        )
                        self._round_cube_name = None
                        action = self.machine.pick_failed_switch_cube()
                        self.handle_action(action)
                        return
                if cube is not None:
                    failures = self._cube_failures.get(cube, 0) + 1
                    self._cube_failures[cube] = failures
                    # Keep the cube eligible after its first failure: the
                    # failure costs 25-50 s because the mission has to fly to
                    # another shelf, while a second approach to the same cube
                    # costs a few seconds and succeeds about three times in
                    # four (measured 2026-09-12: roughly one pick in four ends
                    # with the cube slipping out of the gripper onto the tray,
                    # the grasp, close and attach all having been clean).
                    if failures >= 2:
                        self._cube_excluded.add(cube)
                        detail = f"exclude {cube}"
                    else:
                        detail = f"retry {cube}"
                    self._round_cube_name = None
                    self.get_logger().warn(
                        f"cube_selection: {detail} after pick "
                        f"failure={failures}; switching same-color cube"
                    )
                    action = self.machine.pick_failed_switch_cube()
                    self.handle_action(action)
                    return
            action = self.machine.manipulation_finished(success)
            self.handle_action(action)
            return
        if (
            self.manipulation_deadline is not None
            and now >= self.manipulation_deadline
            and not self.manipulation_timed_out
        ):
            self.manipulation_timed_out = True
            operation = self.manipulation_operation
            self.publish_error(
                f"Manipulation {operation} timed out"
            )
            job = self.manipulation_job
            self._reset_manipulation()
            self._send_manipulation_request("release", job, None)
            if operation == "scan":
                action = self.machine.perception_finished(None)
            else:
                action = self.machine.manipulation_finished(False)
            self.handle_action(action)



        # Hard stall escape: even if the timer body is re-entered oddly, do not
        # freeze in picking/placing after the deadline by more than 5 s.
        if (
            self.manipulation_active
            and self.manipulation_deadline is not None
            and now >= self.manipulation_deadline + 5.0
        ):
            self.get_logger().error(
                f"manip stall force-finish op={self.manipulation_operation}"
            )
            self.manipulation_timed_out = True
            operation = self.manipulation_operation
            job = self.manipulation_job
            self._reset_manipulation()
            self._send_manipulation_request("release", job, None)
            if operation == "scan":
                action = self.machine.perception_finished(None)
            else:
                action = self.machine.manipulation_finished(False)
            self.handle_action(action)

    def _reject_unsafe_manipulation_departure(self, result, operation) -> bool:
        """Fail closed when the worker has not confirmed cleanup."""
        unsafe_base = result.get("base_released") is False
        unsafe_cube = (
            operation == "pick"
            and not result.get("success")
            and result.get("cube_attached") is True
        )
        if not (unsafe_base or unsafe_cube):
            return False
        reason = (
            "manipulation_base_release_failed" if unsafe_base
            else "failed_pick_cube_still_attached"
        )
        self.publish_error(reason)
        self.machine.state = "failed"
        self.machine.detail = reason
        self.handle_action(MissionAction("done", reason=reason))
        return True

    def _consume_manipulation_result(self) -> None:
        """Handle a finished manipulation request (result path or timeout)."""
        if self.manipulation_result is None:
            return
        result = self.manipulation_result
        operation = self.manipulation_operation
        success = bool(result.get("success"))
        success = success and not self.manipulation_timed_out
        self._reset_manipulation()
        if self._reject_unsafe_manipulation_departure(result, operation):
            return
        if self.cancel_pending:
            self.finish_cancel()
            return
        if operation == "scan":
            if success:
                self.perception_deadline = (
                    time.monotonic() + self.perception_timeout
                )
                self.publish_state()
            else:
                action = self.machine.perception_finished(None)
                self.handle_action(action)
            return
        if operation not in {"pick", "place", "release"}:
            self.publish_error(
                f"Unknown completed operation: {operation}"
            )
            return
        if operation == "place" and success:
            # A verified delivery does not disturb the remaining shelf,
            # and the arm now returns home in the background. Reuse the
            # known world slots on the next visit; any failed grasp still
            # invalidates this cache and forces a fresh RGB-D scan.
            self.inventory_scan_complete = True
        self._note_place_outcome(result, operation, success)
        if operation == "pick" and not success:
            # A disturbed or stale slot must be observed again before a
            # retry or any later cached pick.
            self.inventory_scan_complete = False
            cube = self._round_cube_name
            rejection_detail = str(result.get("detail", ""))
            if (
                cube is not None
                and rejection_detail.startswith("approach_rejected")
            ):
                rejections = (
                    self._approach_rejections.get(cube, 0) + 1
                )
                self._approach_rejections[cube] = rejections
                if rejections <= APPROACH_REJECTION_LIMIT:
                    # The manipulation node refused the descend because
                    # the requested target did not match the cube's live
                    # pose. That is bad target data, not a failed grasp,
                    # so it must not consume the failure budget that
                    # excludes a cube and can exhaust the inventory.
                    self.get_logger().warn(
                        f"cube_selection: approach rejected for "
                        f"{cube} ({rejection_detail}); rescanning "
                        f"same colour rejection={rejections}/"
                        f"{APPROACH_REJECTION_LIMIT}"
                    )
                    self._round_cube_name = None
                    action = self.machine.pick_failed_switch_cube()
                    self.handle_action(action)
                    return
            if cube is not None:
                failures = self._cube_failures.get(cube, 0) + 1
                self._cube_failures[cube] = failures
                # Keep the cube eligible after its first failure: the
                # failure costs 25-50 s because the mission has to fly to
                # another shelf, while a second approach to the same cube
                # costs a few seconds and succeeds about three times in
                # four (measured 2026-09-12: roughly one pick in four ends
                # with the cube slipping out of the gripper onto the tray,
                # the grasp, close and attach all having been clean).
                if failures >= 2:
                    self._cube_excluded.add(cube)
                    detail = f"exclude {cube}"
                else:
                    detail = f"retry {cube}"
                self._round_cube_name = None
                self.get_logger().warn(
                    f"cube_selection: {detail} after pick "
                    f"failure={failures}; switching same-color cube"
                )
                action = self.machine.pick_failed_switch_cube()
                self.handle_action(action)
                return
        action = self.machine.manipulation_finished(success)
        self.handle_action(action)
        return

    def handle_action(self, action: MissionAction) -> None:
        """Perform one side effect requested by the pure state machine."""
        self.action_started_at = time.monotonic()
        self.publish_state()
        if action.kind == "navigate":
            location = action.location
            if action.location in {"RED", "BLUE"} and action.job is not None:
                same_job = getattr(self, "_selection_job_index", None) == action.job.index
                if self._round_cube_name is not None and not same_job:
                    self._cube_done.add(self._round_cube_name)
                self._round_zone = action.job.target_zone
                if not (same_job and self._round_cube_name is not None):
                    self._round_cube_name = self._select_best_cube(action.job)
                self._selection_job_index = action.job.index
                if self._round_cube_name is None:
                    self.get_logger().warn(
                        "cube_selection: inventory exhausted; skipping "
                        f"{action.job.color} item without revisiting a "
                        "consumed shelf slot"
                    )
                    self.handle_action(
                        self.machine.pickup_inventory_exhausted()
                    )
                    return
                # Navigate to the selected cube's own dock instead of the
                # shared per-colour pickup point. The approach side is chosen
                # with the delivery leg in mind so the base ends up on the
                # side that shortens the trip to the target zone. An active
                # pick-order override pins the side for its entry only.
                side = self._override_pending_side
                self._override_pending_side = None
                if not side:
                    side = self._cube_approach_side(
                        self._round_cube_name,
                        action.job.target_zone,
                    )
                self.start_navigation(
                    action,
                    location=self._round_cube_name.upper(),
                    side_hint=side,
                )
                return
            self.start_navigation(action, location=location)
        elif action.kind == "perceive":
            job = self._active_job() or action.job
            expected = (
                self._expected_cube_base_position(job.cube_name)
                if job is not None
                else None
            )
            # Only skip the scan when the cached estimate is good enough to
            # grasp directly. If it claims a lateral offset large enough to
            # need a centring move, its own error is the same size as the
            # thing it is measuring, and acting on it produces an invalid
            # grasp window plus a forced rescan.
            cached_is_graspable = (
                expected is not None
                and abs(expected[1]) <= 0.025
            )
            if expected is not None and not cached_is_graspable:
                self.get_logger().info(
                    "perception_cached_slot_rejected:"
                    f"y={expected[1]:.4f} exceeds "
                    f"{CENTERING_LATERAL_THRESHOLD:.3f}; scanning instead"
                )
            if (
                self.inventory_scan_complete
                and job is not None
                and expected
                and cached_is_graspable
            ):
                target = {
                    "color": job.color,
                    "position": {
                        "x": float(expected[0]),
                        "y": float(expected[1]),
                        "z": CUBE_SLOT_BASE_Z,
                    },
                }
                self.get_logger().info(
                    f"perception_cached_slot:{job.cube_name}:"
                    f"x={expected[0]:.3f},y={expected[1]:.3f}"
                )
                self.target_source = "cached_slot"
                self.target_observed_at = time.monotonic()
                next_action = self.machine.perception_finished(target)
                self.handle_action(next_action)
            else:
                self.start_manipulation(
                    MissionAction(
                        "scan",
                        job=action.job,
                        reason="prepare_wrist_camera",
                    )
                )
        elif action.kind in {"pick", "place", "release"}:
            self.start_manipulation(action)
        elif action.kind == "verify":
            self.start_verification(action)
        elif action.kind == "done":
            if self._round_cube_name is not None and self.machine.state == "completed":
                self._cube_done.add(self._round_cube_name)
            self.get_logger().info(f"mission_terminal:{action.reason}")
        else:
            self.publish_error(f"Unsupported mission action: {action.kind}")

    def _attempt_no_target_recovery(self) -> bool:
        """Re-align once and take a fresh scan before rejecting a cube."""
        job = self._active_job()
        if job is None:
            return False
        attempts = self.machine.attempts.get("perception", 0)
        used = self._perception_recovery_used.get(job.cube_name, 0)
        if (
            attempts < self.machine.maximum_perception_retries
            or used >= self.perception_recovery_rescans
        ):
            return False
        expected = self._expected_cube_base_position(job.cube_name)
        if expected is None:
            return False
        cube_x, cube_y = expected
        if not (
            self.reachable_minimum_x <= cube_x <= self.reachable_maximum_x
            and abs(cube_y) <= self.reachable_maximum_abs_y
        ):
            return False
        # After one dock realignment, hand a still safely reachable cached
        # slot to the guarded slot fallback instead of backing the base away.
        # The old second recovery moved an observed 0.366 m target to 0.469 m,
        # outside reachable_maximum_x, so the subsequent fallback was rejected.
        fallback_reachable = (
            self.reachable_minimum_x <= cube_x <= self.reachable_maximum_x
            and abs(cube_y) <= SLOT_FALLBACK_MAX_ABS_Y
        )
        if used >= 1 and fallback_reachable:
            self.get_logger().info(
                f"perception_recovery_handoff_slot_fallback:{job.cube_name}:"
                f"slot=({cube_x:.3f},{cube_y:.3f})"
            )
            return False
        self._perception_recovery_used[job.cube_name] = used + 1
        self.get_logger().warn(
            f"perception_recovery_rescan:{job.cube_name}:"
            f"attempt={used + 1}/{self.perception_recovery_rescans}:"
            f"slot=({cube_x:.3f},{cube_y:.3f})"
        )
        # Two orthogonal causes need different motions. The old branch keyed the
        # backup on the lateral error rather than the cube's depth, so a large
        # dock error was "recovered" by backing away from the cube.
        too_close_for_camera = cube_x < DOCK_REALIGN_TARGET_X
        lateral_out_of_band = abs(cube_y) > SLOT_FALLBACK_MAX_ABS_Y
        if used >= 1 and too_close_for_camera:
            # A cube nudged toward the robot can sit at 0.22 m, behind the
            # wrist camera's scan standoff, so even a fresh scan sees nothing.
            # Back up 0.10 m so the cube returns inside the camera depth range
            # and the reachable window, then scan again.
            self._backup_dock_for_rescan(job.cube_name)
        elif used >= 1 and lateral_out_of_band:
            # Still outside the band a fallback grasp accepts, so keep closing
            # the lateral dock error with the same bounded realign instead of
            # backing away. Real dock errors reach 0.19 m (234 samples: median
            # 0.038, p90 0.073, max 0.192) while one realign moves the base
            # ~0.023 m, so a single cycle could not reach the 0.10 m band and
            # the round ended as target_unavailable. Each step stays capped by
            # DOCK_REALIGN_MAX_LATERAL; only their number changes.
            self.get_logger().info(
                f"perception_recovery_lateral_realign:{job.cube_name}:"
                f"slot=({cube_x:.3f},{cube_y:.3f})"
            )
            self._realign_dock_to_cached_slot(job.cube_name)
        else:
            self._realign_dock_to_cached_slot(job.cube_name)
        self.start_manipulation(
            MissionAction(
                "scan",
                job=job,
                reason="no_target_recovery_rescan",
            )
        )
        return True

    def _backup_dock_for_rescan(self, cube_name: str) -> None:
        """
        Back the base up a bounded 0.10 m so a nudged cube is visible.

        A cube pushed toward the dock can sit closer than the wrist camera's
        scan standoff and is invisible to RGB-D; backing up brings it back
        into the depth range without leaving the reachable window.
        """
        start = self._odom_pose()
        if start is None:
            self.get_logger().warn(
                f"backup_rescan_skipped:{cube_name}:odom_unavailable"
            )
            return
        backup_distance = 0.10

        def travelled() -> Optional[float]:
            current = self._odom_pose()
            if current is None:
                return None
            yaw = start[2]
            dx = current[0] - start[0]
            dy = current[1] - start[1]
            forward = dx * math.cos(yaw) + dy * math.sin(yaw)
            return -forward  # negative of forward travel = backward travel

        speed = 0.20
        observed_speed = 0.40 * speed
        deadline = time.monotonic() + min(
            backup_distance / observed_speed, 6.0
        )
        self.get_logger().info(
            f"backup_rescan_start:{cube_name}:"
            f"distance={backup_distance:.2f}m"
        )
        while time.monotonic() < deadline:
            moved = travelled()
            if moved is not None and moved >= backup_distance - 0.005:
                break
            twist = Twist()
            twist.linear.x = -speed
            self.cmd_vel_publisher.publish(twist)
            time.sleep(0.05)
        stop = Twist()
        for _ in range(5):
            self.cmd_vel_publisher.publish(stop)
            time.sleep(0.02)
        moved = travelled()
        self.get_logger().info(
            f"backup_rescan_result:{cube_name}:moved={moved}"
        )

    def _attempt_slot_fallback(self) -> bool:
        """
        Try one direct slot grasp when perception keeps failing.

        The wrist camera can miss a cube that sits exactly under its scan
        axis (e.g. after a small disturbance), while the per-cube world slot
        is known. Once the scan retries are exhausted, attempt a direct pick
        at the expected slot position instead of skipping outright; the pick
        still validates the actual grasp window, so a genuinely moved cube
        falls through to the normal skip path.
        """
        job = self._active_job()
        if job is None:
            return False
        attempts = self.machine.attempts.get("perception", 0)
        if attempts < self.machine.maximum_perception_retries:
            return False
        expected = self._expected_cube_base_position(job.cube_name)
        if expected is None:
            return False
        cube_x, cube_y = expected
        if not (
            self.reachable_minimum_x <= cube_x <= self.reachable_maximum_x
        ):
            return False
        # Realign first: a guessed slot pick without a dock fix produced
        # link6 offsets of 0.45-0.48 m (target_out_of_reach cascades).
        self._realign_dock_to_cached_slot(job.cube_name)
        # A fallback must be inside a graspable lateral band. The pick
        # request below still runs lateral centring, so a small odometry
        # error (observed 2-3 cm on red_cube_1 after dock realign) must not
        # turn a perfectly reachable cube into a skip. 0.19 m previously
        # failed validate_grasp_window after moving the cube; 0.10 m keeps
        # the grasp window valid while absorbing the dock-odometry offset.
        if abs(cube_y) > SLOT_FALLBACK_MAX_ABS_Y:
            return False
        target = {
            "color": job.color,
            "position": {
                "x": float(cube_x),
                "y": float(cube_y),
                "z": CUBE_SLOT_BASE_Z,
            },
        }
        self.get_logger().warn(
            f"perception_slot_fallback:{job.cube_name}:"
            f"x={cube_x:.3f},y={cube_y:.3f}"
        )
        self.target_source = "slot_fallback"
        self.target_observed_at = time.monotonic()
        action = self.machine.perception_finished(target)
        self.handle_action(action)
        return True

    def _active_job(self):
        """Return the round job whose cube is dynamically selected."""
        job = self.machine.current_job
        if job is None:
            return None
        if (
            self._round_cube_name
            and self._round_cube_name != job.cube_name
        ):
            return MissionJob(
                index=job.index,
                total=job.total,
                color=job.color,
                cube_name=self._round_cube_name,
                target_zone=job.target_zone,
            )
        return job

    def _pickup_dock_pose(self, color: str):
        """Return the configured pickup dock pose for one color."""
        key = "pickup_red_dock" if color == "red" else "pickup_blue_dock"
        values = self.get_parameter(key).value
        return (float(values[0]), float(values[1]), float(values[2]))

    def _cube_approach_candidates(self, cube_key: str, cube_xy):
        """
        Build the same approach candidates TaskNavigator uses.

        West docks come from the deployed config; blue east-side (inner)
        docks also come from the config. The remaining cardinal sides are
        derived geometrically so scoring agrees with the navigator even when
        the robot approaches a cube from the inside or from the side.
        """
        cube_x, cube_y = cube_xy
        upper = cube_key.upper()
        candidates = []
        for side in CUBE_APPROACH_SIDES.get(cube_key, (APPROACH_WEST,)):
            if side == APPROACH_WEST:
                dock_values = self.get_parameter(f"{cube_key}_dock").value
                dock = (
                    float(dock_values[0]),
                    float(dock_values[1]),
                    float(dock_values[2]),
                )
                entry = approach_entry(cube_x, cube_y, side)
                candidates.append(
                    ApproachCandidate(
                        side,
                        cube_key,
                        entry,
                        ZonePose(cube_key, dock[0], dock[1], dock[2]),
                    )
                )
            elif (
                side == APPROACH_EAST
                and upper.startswith("BLUE")
                and self.has_parameter(f"{cube_key}_dock_inner")
                and self.has_parameter(f"{cube_key}_entry_inner")
            ):
                entry_values = self.get_parameter(
                    f"{cube_key}_entry_inner"
                ).value
                dock_values = self.get_parameter(
                    f"{cube_key}_dock_inner"
                ).value
                candidates.append(
                    ApproachCandidate(
                        side,
                        cube_key,
                        ZonePose(cube_key, *entry_values[:3]),
                        ZonePose(cube_key, *dock_values[:3]),
                    )
                )
            else:
                candidates.append(
                    ApproachCandidate(
                        side,
                        cube_key,
                        approach_entry(cube_x, cube_y, side),
                        approach_dock(cube_x, cube_y, side),
                    )
                )
        allowed = getattr(self, "cube_approach_allowlist", set())
        if allowed and allowed != {""}:
            candidates = [c for c in candidates
                          if f"{cube_key}:{c.side}" in allowed]
        return candidates

    def _choose_delivery_candidate(
        self,
        candidates,
        robot_pose,
        target_zone,
        failure_count=0,
    ):
        """Choose one approach using the full per-zone delivery cost."""
        if target_zone is None:
            return choose_approach(candidates, robot_pose, ())
        cube_xy = (
            CUBE_WORLD_SLOTS.get(candidates[0].cube)
            if candidates
            else None
        )
        if cube_xy is not None:
            candidates = prefer_same_side_approaches(
                candidates, robot_pose[0], cube_xy[0]
            )
        place_xy = self.zone_centers[target_zone]
        return min(
            candidates,
            key=lambda candidate: cube_delivery_cost(
                robot_pose,
                candidate,
                target_zone,
                place_xy,
                failure_count=failure_count,
                use_legacy_route_estimate=getattr(
                    self, "use_legacy_selection_costs", True
                ),
            ),
        )

    def _cube_approach_side(
        self,
        cube_name: str,
        target_zone: str,
    ) -> str:
        """
        Return the cheapest approach side including the delivery leg.

        The delivery-zone term makes the approach leave the base on the side
        that shortens the subsequent trip to the target zone (e.g. west for
        red cubes delivered to A/C, east for red cubes delivered to B).
        """
        cube_key = cube_name.lower()
        cube_xy = CUBE_WORLD_SLOTS.get(cube_key)
        if cube_xy is None:
            return APPROACH_WEST
        pose = self._odom_pose()
        if pose is None:
            pose = (0.0, 0.0, 0.0)
        candidates = self._cube_approach_candidates(cube_key, cube_xy)
        chosen = self._choose_delivery_candidate(
            candidates, pose, target_zone,
        )
        return chosen.side

    def _cube_dock_pose(self, cube_name: str, target_zone: str = None):
        """
        Return the side-aware per-cube dock used for selection scoring.

        Uses the same dynamic approach planner as TaskNavigator so the cube
        selected for a round is the one whose actual approach dock is
        cheapest, including the subsequent delivery leg.
        """
        cube_key = cube_name.lower()
        cube_xy = CUBE_WORLD_SLOTS.get(cube_key)
        if cube_xy is None:
            values = self.get_parameter(f"{cube_name}_dock").value
            return (float(values[0]), float(values[1]), float(values[2]))
        pose = self._odom_pose()
        if pose is None:
            pose = (0.0, 0.0, 0.0)
        candidates = self._cube_approach_candidates(cube_key, cube_xy)
        chosen = self._choose_delivery_candidate(
            candidates, pose, target_zone,
        )
        return (chosen.dock.x, chosen.dock.y, chosen.dock.yaw)

    def _zone_nav_pose(self, zone: str):
        """Return the configured zone navigation pose for one zone."""
        key = f"zone_{zone.lower()}_nav"
        values = self.get_parameter(key).value
        return (float(values[0]), float(values[1]), float(values[2]))

    @staticmethod
    def _cube_group(colors_y: Sequence[float], cube_y: float) -> str:
        """
        Return 'upper' or 'lower' from the colour's actual y values.

        The split is the median of the remaining same-colour cube y
        coordinates, so the grouping adapts to the current layout instead of
        being hard-coded to array indices.
        """
        ordered = sorted(colors_y)
        if not ordered:
            return "upper"
        median = ordered[len(ordered) // 2]
        return "upper" if cube_y >= median else "lower"

    def _consume_cube_order_override(self, job) -> Optional[str]:
        """Return the experiment-forced cube for one pick, if any.

        `cube_order_override` entries ("cube" or "cube:side") are consumed
        in pick order from a snapshot taken when the task arrives.
        A future-color entry is preserved across failed-pick retries;
        unavailable entities fall through to normal same-color scoring.
        """
        entries = getattr(self, "_task_order_entries", None)
        if entries is None:
            entries = self._cube_order_override_entries()
        # Each pick resolves its side afresh: a skipped entry must not
        # leak the previous entry's side into normal scoring.
        self._override_pending_side = None
        while self._override_index < len(entries):
            entry = str(entries[self._override_index]).strip().lower()
            name, _, side = entry.partition(":")
            side = side.strip() or None
            if not name:
                self._override_index += 1
                continue
            if name.split("_", 1)[0] != job.color:
                self.get_logger().warn(
                    "cube_selection_override colour_mismatch "
                    f"entry={entry} job_color={job.color}"
                )
                # A failed pick can exhaust this color before the next
                # instruction starts. Preserve that future-color entry.
                return None
            self._override_index += 1
            if name not in CUBE_WORLD_SLOTS or (
                side and side not in CUBE_APPROACH_SIDES.get(name, ())
            ):
                self.get_logger().warn(
                    f"cube_selection_override invalid entry={entry}"
                )
                continue
            if name in self._cube_done or name in self._cube_excluded:
                self.get_logger().warn(
                    f"cube_selection_override unavailable entry={entry}"
                )
                continue
            self._override_pending_side = side
            self.get_logger().info(
                f"cube_selection_override name={name} side={side or 'auto'}"
            )
            return name
        return None

    def _load_cube_order_policy(self) -> None:
        """Load the offline policy table once; failures disable it only."""
        if self._cube_order_policy:
            return
        try:
            file_name = str(
                self.get_parameter("cube_order_policy_file").value or ""
            )
        except Exception:
            return
        if not file_name or not Path(file_name).exists():
            return
        try:
            with open(file_name, encoding="utf-8") as handle:
                payload = json.load(handle)
            tasks = payload.get("tasks", {}) if isinstance(payload, dict) else {}
            if isinstance(tasks, dict):
                self._cube_order_policy = tasks
                self.get_logger().info(
                    f"cube_order_policy_loaded tasks={len(tasks)}"
                )
        except (OSError, ValueError) as exception:
            self.get_logger().warn(
                f"cube_order_policy_load_failed: {exception}"
            )

    def _policy_entries_for_jobs(self, jobs) -> list:
        """Policy pick entries for this task, or [] when unlisted."""
        if not self._cube_order_policy:
            return []
        key = ordered_task_key(jobs)
        task = self._cube_order_policy.get(key)
        if not isinstance(task, dict):
            return []
        try:
            entries = [
                "%s:%s" % (item["cube"], item["side"])
                for item in task.get("order", [])
            ]
        except (KeyError, TypeError):
            return []
        return validate_pick_entries(entries, jobs)

    def _cube_order_override_entries(self) -> list:
        """Current forced-order entries; [] when unset or uninitialized.

        A STRING_ARRAY parameter declared without a default raises
        ParameterUninitializedException on read until it is set; that
        state simply means "no override".
        """
        try:
            return list(
                self.get_parameter("cube_order_override").value or []
            )
        except Exception:
            return []

    def _seed_cube_order_override(self) -> None:
        """Seed the pick-order override from the offline policy table.

        The policy is a pure function of the parsed task. An explicit
        runtime override (set before the task arrived, e.g. by an
        experiment runner) wins and is never overwritten.
        """
        # Snapshot for this task only. Never write a derived order into
        # the operator's parameter: it would override every later task.
        self._task_order_entries = []
        self._override_index = 0
        self._override_pending_side = None
        self._load_cube_order_policy()
        current = self._cube_order_override_entries()
        if current:
            self._task_order_entries = validate_pick_entries(
                current, self.machine.jobs
            )
            self.get_logger().info(
                "cube_order_policy_runtime_override_present "
                f"accepted={bool(self._task_order_entries)} n={len(current)}"
            )
            return
        entries = self._policy_entries_for_jobs(self.machine.jobs)
        if not entries:
            key = ordered_task_key(self.machine.jobs)
            self.get_logger().info(
                f"cube_order_policy_no_entry jobs={len(self.machine.jobs)} "
                f"key={key}"
            )
            return
        self._task_order_entries = entries
        self._override_index = 0
        self.get_logger().info(
            f"cube_order_policy_seeded n={len(entries)}"
        )

    def _schedule_transport_jobs(self, replan=False) -> bool:
        """Replace only unexecuted jobs at an empty, safe planning boundary."""
        if (not getattr(self, "_transport_automatic", False)
                or self.get_parameter("enforce_color_order").value
                or self.machine.state != "navigating_pickup"):
            return False
        offset = self.machine.job_offset
        remaining = self.machine.jobs[offset:]
        if not remaining:
            return False
        try:
            share = Path(get_package_share_directory("bot_navigation"))
            nav = yaml.safe_load((share / "config/competition_zones.yaml").read_text())[
                "competition_task_navigator"]["ros__parameters"]
            if nav.get("use_legacy_layout_routes", True):
                return False
            strategies = yaml.safe_load(
                (share / "config/navigation_risk_zones.yaml").read_text())["zone_strategies"]
            options = {}
            unavailable = self._cube_done | self._cube_excluded
            for name, xy in CUBE_WORLD_SLOTS.items():
                if name not in unavailable:
                    options.setdefault(
                        name.split("_")[0],
                        []).extend(
                        (name,
                         c.side,
                         c) for c in self._cube_approach_candidates(
                            name,
                            xy))
            totals = {zone: sum(j.target_zone == zone for j in self.machine.jobs)
                      for zone in self.zone_centers}
            counts = {zone: sum(j.target_zone == zone for j in self.machine.jobs[:offset])
                      for zone in self.zone_centers}
            timing_samples = []
            profile_name = str(self.get_parameter(
                "transport_timing_profile_file").value or "")
            if profile_name:
                tools_share = Path(get_package_share_directory("tools_demo"))
                profile_path = Path(profile_name)
                if not profile_path.is_absolute():
                    profile_path = tools_share / "config" / profile_name
                try:
                    timing_samples = load_timing_profile(profile_path, {
                        "navigation_math.py": Path(__file__).with_name("navigation_math.py"),
                        "transport_routes.py": Path(__file__).with_name("transport_routes.py"),
                        "task_navigator.py": Path(__file__).with_name("task_navigator.py"),
                        "competition_zones.yaml": share / "config/competition_zones.yaml",
                        "originbot_nav2.yaml": share / "param/originbot_nav2.yaml",
                        "navigation_risk_zones.yaml": share / "config/navigation_risk_zones.yaml",
                        "mission.yaml": tools_share / "config/mission.yaml"})
                except (OSError, ValueError) as exception:
                    self.get_logger().warn(
                        f"transport_timing_profile_rejected: {exception}")
            estimator = TransportTimeEstimator(
                self.zone_nav_poses,
                strategies,
                CUBE_WORLD_SLOTS,
                totals,
                spacing=self.place_slot_spacing,
                maximum_abs_y=self.place_maximum_abs_y,
                uncertainty_weight=float(
                    self.get_parameter("transport_uncertainty_weight").value),
                failures=dict(
                    self._cube_failures),
                responses_complete=self._transport_responses_complete,
                navigation_parameters=nav,
                timing_samples=timing_samples)
            # A geometry-only replacement must not silently remove the only
            # planned central encounter. Real proof still comes from Nav2's
            # timestamped, current-epoch response verification, not this DP.
            required = ()
            if (not self._transport_responses_complete
                    and "blue_cube_3" not in self._cube_done
                    and any(j.color == "blue" for j in remaining)):
                required = ("blue_cube_3",)
            start = self._odom_pose()
            jobs, entries, estimate = optimize_schedule(
                remaining, options, start, self.zone_nav_poses, None,
                transition=estimator.transition, required_cubes=required,
                zone_counts=counts)
            if not validate_pick_entries(entries, jobs):
                return False
            incumbent_entries = self._task_order_entries[self._override_index:]
            incumbent = route_estimate(
                remaining,
                incumbent_entries,
                options,
                start,
                estimator.transition,
                counts)
            if not set(required) <= {
                    e.split(":")[0] for e in incumbent_entries}:
                incumbent = math.inf
            threshold = float(self.get_parameter(
                "transport_replan_minimum_gain_sec").value)
            if not math.isfinite(threshold) or threshold < 0:
                raise ValueError(
                    "Replan minimum gain must be finite and nonnegative")
            if (replan or incumbent_entries) and incumbent - \
                    estimate < threshold:
                self.get_logger().info(
                    f"transport_replan_hold remaining={len(remaining)} "
                    f"incumbent_sec={incumbent:.2f} best_sec={estimate:.2f} "
                    f"threshold_sec={threshold:.2f}")
                return False
            prefix = tuple(self.machine.jobs[:offset])
            jobs = tuple(
                replace(
                    j,
                    index=offset+i+1,
                    total=len(
                        self.machine.jobs)) for i,
                j in enumerate(jobs))
            self.machine.jobs = prefix + jobs
            self._task_order_entries = entries
            self._override_index = 0
            self._override_pending_side = None
            self.get_logger().info(
                f"transport_schedule replan={replan} offset={offset} "
                f"risk_adjusted_estimate_sec={estimate:.2f} "
                f"incumbent_sec={incumbent:.2f} required={required} "
                f"responses_complete={self._transport_responses_complete} "
                f"timing_samples={len(timing_samples)} "
                f"entries={entries} requirements={[(j.color, j.target_zone) for j in jobs]}")
            lookup = {f"{n}:{s}": c for items in options.values()
                      for n, s, c in items}
            trace_counts, trace_pose = dict(counts), start
            for i, (job, entry) in enumerate(zip(jobs, entries)):
                slot = trace_counts.get(job.target_zone, 0)
                candidate = lookup[entry]
                parts = estimator.components(
                    trace_pose, candidate, job.target_zone, slot, i == len(jobs)-1)
                self.get_logger().info(
                    f"transport_prediction job={job.index} entry={entry} "
                    f"zone={job.target_zone} components_sec="
                    f"{json.dumps({k: round(v, 2) for k, v in parts.items()}, sort_keys=True)}")
                _, trace_pose = estimator.transition(
                    trace_pose, candidate, job.target_zone, slot, i == len(jobs)-1)
                trace_counts[job.target_zone] = slot + 1
            return True
        except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError) as exception:
            self.get_logger().warn(f"transport_schedule_fallback: {exception}")
            return False

    def _schedule_unordered_jobs(self) -> bool:
        """Schedule legal goods freely; explicit operator overrides win."""
        if (self.get_parameter("enforce_color_order").value
                or self._cube_order_override_entries()
                or getattr(self, "_task_order_entries", ())):
            return False
        options = {}
        for name, xy in CUBE_WORLD_SLOTS.items():
            color = name.split("_", 1)[0]
            options.setdefault(color, []).extend(
                (name, candidate.side, candidate)
                for candidate in self._cube_approach_candidates(name, xy))
        zone_poses = {}
        for zone in self.zone_centers:
            zone_poses[zone] = self._zone_nav_pose(zone)
        try:
            jobs, entries, estimate = optimize_schedule(
                self.machine.jobs, options, self._odom_pose(), zone_poses,
                lambda pose, candidate, zone: staged_cube_delivery_cost(
                    pose, candidate, zone, self.zone_centers[zone],
                    use_legacy_route_estimate=self.use_legacy_selection_costs))
        except (TypeError, ValueError) as exception:
            self.get_logger().warn(f"unordered_schedule_fallback: {exception}")
            return False
        if not validate_pick_entries(entries, jobs):
            return False
        self.machine.jobs = jobs
        self._task_order_entries = entries
        self._override_index = 0
        self._override_pending_side = None
        self.get_logger().info(
            f"unordered_schedule estimate={estimate:.2f} entries={entries} "
            f"requirements={[(j.color, j.target_zone) for j in jobs]}")
        # Keep a complete legal incumbent before evaluating the richer model.
        # The same gain floor protects both initial and rolling replacements;
        # unmeasured alternatives must not win merely by a few predicted
        # seconds.
        if self.get_parameter("transport_schedule_enabled").value:
            self._schedule_transport_jobs()
        return True

    def _select_best_cube(self, job) -> Optional[str]:
        """Choose the same-colour cube with the lowest feasible total cost."""
        forced = self._consume_cube_order_override(job)
        if forced is not None:
            return forced
        color = job.color
        candidates = []
        for name, (wx, wy) in CUBE_WORLD_SLOTS.items():
            if not name.startswith(color):
                continue
            if name in self._cube_done or name in self._cube_excluded:
                continue
            candidates.append((name, wx, wy))
        if not candidates:
            self.get_logger().warn(
                f"cube_selection: no remaining {color} candidates"
            )
            return None
        # Each cube now has its own dock, so the differentiator is the
        # straight-line distance from the current robot pose to that cube's
        # dock plus the prior failure count. Keep selection deterministic
        # and avoid blocking the coordinator executor.
        robot_pose = self._odom_pose()
        selection_pose = selection_pose_after_universal_staging(robot_pose)
        if robot_pose is None or selection_pose != tuple(robot_pose):
            self.get_logger().info(
                "cube_selection: scoring_from_origin_staging"
            )
        scored = []
        # A five-job mission does not require emptying either five-slot
        # shelf. Prefer the closest cube; the bounded per-cube dock keeps
        # the arm centred on every cube without lateral base corrections.
        hardest_first = False
        colors_y = [wy for n, _, wy in candidates
                    if n.startswith(color)]
        for name, wx, wy in candidates:
            failure_count = self._cube_failures.get(name, 0)
            pose = selection_pose or (0.0, 0.0, 0.0)
            approach_candidates = self._cube_approach_candidates(
                name, (wx, wy)
            )
            chosen = self._choose_delivery_candidate(
                approach_candidates,
                pose,
                job.target_zone,
                failure_count=failure_count,
            )
            dock = (chosen.dock.x, chosen.dock.y, chosen.dock.yaw)
            # Lateral offset is a grasp difficulty term kept visible in the
            # log even though the selected cardinal dock is centred.
            cos_yaw = math.cos(dock[2])
            sin_yaw = math.sin(dock[2])
            dx = wx - dock[0]
            dy = wy - dock[1]
            lateral = abs(-dx * sin_yaw + dy * cos_yaw)
            base_cost = cube_delivery_cost(
                pose,
                chosen,
                job.target_zone,
                self.zone_centers[job.target_zone],
                failure_count=failure_count,
                use_legacy_route_estimate=getattr(
                    self, "use_legacy_selection_costs", True
                ),
            )
            # Euclidean scoring reverses the true order around the y=1.5 wall:
            # NavFn measures A-exit -> B1 -> C at 20.35 m versus 31.71 m for
            # B2. The former DWB-specific B2 penalty is no longer applicable:
            # current interpolated RPP completed B2 safely, and B2 is the
            # shorter second round from C exit than the distant B3 staging.
            if getattr(self, "use_legacy_selection_costs", True):
                if job.target_zone == "C" and color in ("blue", "red"):
                    map_cost = (
                        blue_c_static_path_cost(pose, name)
                        if color == "blue"
                        else red_c_static_path_cost(pose, name)
                    )
                    if map_cost is not None:
                        base_cost = map_cost + 8.0 * failure_count
                    # Keep the named route-risk penalty for ordering outliers
                    # whether or not the static table was used.
                    base_cost += cube_route_risk_penalty(job.target_zone, name)
                else:
                    base_cost += cube_route_risk_penalty(job.target_zone, name)
            # Soft delivery-zone group preference: C favours upper blocks,
            # A/B favour lower blocks. The bonus is capped so it can never
            # override an obviously better or unreachable candidate.
            group = self._cube_group(colors_y, wy)
            preference = 0.0
            if (
                self.cube_group_preference_bonus > 0.0
                and (
                    (job.target_zone == "C" and group == "upper")
                    or (job.target_zone in ("A", "B") and group == "lower")
                )
            ):
                preference = min(
                    self.cube_group_preference_bonus,
                    1.5,
                )
            total = cube_selection_cost(
                base_cost,
                lateral,
                0,
                hardest_first=hardest_first,
                group_preference=preference,
            )
            scored.append(
                {
                    "cube": name,
                    "dock": [round(dock[0], 2), round(dock[1], 2)],
                    "base": round(base_cost, 3),
                    "lateral": round(lateral, 3),
                    "failures": failure_count,
                    "group": group,
                    "pref": round(preference, 2),
                    "total": round(total, 2),
                }
            )
        scored.sort(
            key=lambda item: (
                item["total"],
                item["lateral"],
                item["cube"],
            )
        )
        for item in scored:
            self.get_logger().info(
                f"cube_selection: {item['cube']} total={item['total']} "
                f"dock={item['dock']} base={item['base']} "
                f"lateral={item['lateral']} failures={item['failures']} "
                f"group={item.get('group')} pref={item.get('pref')}"
            )
        selected = scored[0]["cube"]
        if (job.target_zone == "B" and color == "blue"
                and getattr(self, "use_legacy_selection_costs", True)):
            remaining_same_color_jobs = sum(
                1
                for candidate in self.machine.jobs[self.machine.job_offset:]
                if (
                    candidate.color == color
                    and candidate.target_zone == job.target_zone
                )
            )
            spawn_choice = blue_b_spawn_first_choice(
                selection_pose, remaining_same_color_jobs, scored
            )
            lookahead = spawn_choice or blue_b_first_round_lookahead(
                selection_pose, remaining_same_color_jobs, scored
            )
            if lookahead is not None and lookahead != selected:
                self.get_logger().info(
                    "cube_selection: blue_B_bounded_lookahead "
                    f"choose={lookahead} greedy={selected} "
                    f"remaining={remaining_same_color_jobs}"
                )
                selected = lookahead
        self.get_logger().info(
            f"cube_selection: choose {selected} for {color} -> "
            f"{job.target_zone} hardest_first={hardest_first}"
        )
        return selected

    def start_navigation(
        self,
        action: MissionAction,
        location: Optional[str] = None,
        side_hint: Optional[str] = None,
    ) -> None:
        """Publish one semantic location command to TaskNavigator."""
        if action.location is None:
            self.publish_error("Navigation action omitted its location")
            return
        if location is None:
            location = action.location
        # The tracked location stays the canonical zone name because the
        # navigator reports state with the normalized zone; the optional
        # approach-side hint is appended only to the wire command.
        self.navigation_location = location
        self.navigation_seen_active = False
        pose = self._odom_pose()
        pose_text = (
            f"robot=({pose[0]:.3f},{pose[1]:.3f},yaw={pose[2]:.3f})"
            if pose is not None
            else "robot=unknown"
        )
        self.get_logger().info(
            f"nav_start location={location} "
            f"side={side_hint or 'auto'} {pose_text}"
        )
        # A zone delivery carries the job's fixed world slot so the navigator
        # can finish its dock on a heading that faces that slot. The arm's
        # checked placement window is only +/-0.12 m lateral while a zone's
        # slots spread +/-0.16 m, so a fixed zone heading leaves every end
        # slot unreachable (v15: A slot 4/4, place_failed:holding_cube).
        slot_world = None
        if action.job is not None and location in VALID_ZONES:
            slot_world = self._delivery_slot_world(action.job)
        message = String()
        if slot_world is not None:
            message.data = (
                f"{location}:slot={slot_world[0]:.6f},{slot_world[1]:.6f}"
            )
            self.get_logger().info(
                f"nav_slot location={location} "
                f"slot=({slot_world[0]:.3f},{slot_world[1]:.3f})"
            )
        elif side_hint is not None:
            message.data = f"{location}:{side_hint}"
        else:
            message.data = location
        self.navigation_goal_publisher.publish(message)

    def start_manipulation(self, action: MissionAction) -> None:
        """Send one operation to the persistent pick-place node."""
        if self.manipulation_active:
            self.publish_error("A manipulation operation is already active")
            return
        job = self._active_job() or action.job
        if job is None:
            self.publish_error("Manipulation action omitted its job")
            return
        if action.kind == "place":
            repeat = self._place_repeat_refusal(job)
            if repeat is not None:
                # The slot and the measured base pose are unchanged, so the
                # arm would reject the identical request again. Keep the cube,
                # report a recoverable reason and stop instead of looping.
                self.publish_error(repeat)
                self.machine.state = "failed"
                self.machine.detail = (
                    "place_failed:slot_unreachable:holding_cube"
                )
                self.handle_action(
                    MissionAction("done", reason=self.machine.detail)
                )
                return
        pose = self._odom_pose()
        pose_text = (
            f"robot=({pose[0]:.3f},{pose[1]:.3f},yaw={pose[2]:.3f})"
            if pose is not None
            else "robot=unknown"
        )
        self.get_logger().info(
            f"manip_start operation={action.kind} "
            f"cube={job.cube_name} {pose_text}"
        )
        target = None
        if action.kind == "pick":
            if action.target is None:
                self.publish_error("Pick action omitted its detected target")
                return
            position = action.target["position"]
            raw_y = float(position.get("y", 0.0))
            expected_before = self._expected_cube_base_position(
                job.cube_name
            )
            # Perception matches the known slots to ~1-3 cm, so a large
            # mismatch means the cube was nudged and the camera is
            # authoritative; otherwise the static slot is the better
            # estimate for the outer slots where the camera adds parallax.
            slot_consistent = (
                expected_before is not None
                and abs(raw_y - expected_before[1]) <= 0.06
            )
            if abs(raw_y) > CENTERING_LATERAL_THRESHOLD:
                self._center_target_laterally(raw_y)
            if slot_consistent:
                expected = self._expected_cube_base_position(job.cube_name)
                if expected is not None:
                    target = {
                        "x": float(expected[0])
                        + self.pick_tool_offset[0],
                        "y": float(expected[1])
                        + self.pick_tool_offset[1],
                        "z": CUBE_SLOT_BASE_Z
                        + self.pick_tool_offset[2],
                    }
                else:
                    centred_y = (
                        0.0
                        if abs(raw_y) > CENTERING_LATERAL_THRESHOLD
                        else raw_y
                    )
                    target = {
                        "x": float(position["x"])
                        + self.pick_tool_offset[0],
                        "y": centred_y + self.pick_tool_offset[1],
                        "z": float(position["z"])
                        + self.pick_tool_offset[2],
                    }
            else:
                centred_y = (
                    0.0
                    if abs(raw_y) > CENTERING_LATERAL_THRESHOLD
                    else raw_y
                )
                target = {
                    "x": float(position["x"]) + self.pick_tool_offset[0],
                    "y": centred_y + self.pick_tool_offset[1],
                    "z": float(position["z"]) + self.pick_tool_offset[2],
                }
            # Lateral centring only moves the base in Y, so the cube's X in
            # the robot frame is unchanged. The old +0.045 m forward
            # compensation was calibrated for a stale 61.7 mm shortfall and
            # now overshoots to tool x ~= 0.059 m (grasp limit 0.035 m),
            # which is exactly the first-descent failure the user observed.
            # The un-compensated re-scan grasp lands at x ~= 0.0145 m, so no
            # forward compensation is needed at all.
        self._send_manipulation_request(action.kind, job, target)

    def _send_manipulation_request(
        self,
        operation: str,
        job,
        target: Optional[dict],
    ) -> None:
        """Publish one manipulation request and arm its deadline."""
        job = self._manipulation_job if job is None else job
        if job is None:
            self.publish_error("Manipulation request omitted its job")
            return
        self.manipulation_request_id += 1
        request_id = self.manipulation_request_id
        self.manipulation_pending_id = request_id
        self.manipulation_operation = operation
        self.manipulation_job = job
        self.manipulation_active = True
        self.manipulation_timed_out = False
        self.manipulation_result = None
        self.manipulation_deadline = (
            time.monotonic() + self.manipulation_timeout
        )
        cube_name = job.cube_name
        release_world_pose = None
        if operation == "scan":
            pick_pose = self.scan_pose
            pick_safe_pose = self.scan_pose
            tool_orientation = self.scan_tool_orientation
            place_pose = self.place_pose
            place_safe_pose = self.place_safe_pose
        elif operation == "pick":
            if target is None:
                self.publish_error("Pick request omitted its target")
                return
            pick_pose = [
                float(target["x"]),
                float(target["y"]),
                float(target["z"]),
            ]
            pick_safe_pose = [
                pick_pose[0],
                pick_pose[1],
                pick_pose[2] + self.pick_safe_lift,
            ]
            tool_orientation = list(self.pick_tool_orientation)
            place_pose = self.place_pose
            place_safe_pose = self.place_safe_pose
        elif operation == "place":
            place_pose, place_safe_pose = self._place_slot_poses(job)
            slot_x, slot_y = self._delivery_slot_world(job)
            release_world_pose = [
                float(slot_x),
                float(slot_y),
                float(
                    self.delivery_surface_z
                    + self.delivery_cube_half_height
                ),
            ]
            pick_pose = self.place_pose
            pick_safe_pose = self.place_safe_pose
            tool_orientation = list(self.pick_tool_orientation)
        elif operation == "release":
            pick_pose = self.place_pose
            pick_safe_pose = self.place_safe_pose
            tool_orientation = list(self.pick_tool_orientation)
            place_pose = self.place_pose
            place_safe_pose = self.place_safe_pose
        else:
            self.publish_error(f"Unsupported manipulation: {operation}")
            return
        request = {
            "request_id": request_id,
            "operation": operation,
            "cube_name": cube_name,
            "target_source": self.target_source,
            "target_age_sec": (
                None
                if self.target_observed_at is None
                else round(
                    time.monotonic() - self.target_observed_at, 3
                )
            ),
            "pick_pose": [float(v) for v in pick_pose],
            "pick_safe_pose": [float(v) for v in pick_safe_pose],
            "place_pose": [float(v) for v in place_pose],
            "place_safe_pose": [float(v) for v in place_safe_pose],
            "tool_orientation": [float(v) for v in tool_orientation],
            "pick_transit_height": float(
                self.manipulation_pick_transit_height
            ),
            "pick_lift_duration": float(
                self.manipulation_pick_lift_duration
            ),
            "transport_x": float(self.manipulation_transport_x),
            "transport_duration": float(
                self.manipulation_transport_duration
            ),
            "velocity_scaling": float(
                self.manipulation_velocity_scaling
            ),
            "acceleration_scaling": float(
                self.manipulation_acceleration_scaling
            ),
            "cartesian_duration": float(
                self.manipulation_cartesian_duration
            ),
            "gripper_duration": float(
                self.manipulation_gripper_duration
            ),
        }
        if release_world_pose is not None:
            request["release_world_pose"] = release_world_pose
        message = String()
        message.data = json.dumps(request, separators=(",", ":"))
        self.manipulation_request_publisher.publish(message)
        self.get_logger().info(
            f"manip_request id={request_id} op={operation} "
            f"cube={cube_name}"
        )

    def manipulation_result_callback(self, message: String) -> None:
        """Store and immediately consume a matching manipulation result.

        Waiting for the next timer tick was enough when the executor was
        idle, but a laterally-centring sleep or a slow callback could stall
        the tick past the watchdog and freeze the placing state even after
        pick_place already published success.
        """
        try:
            result = json.loads(message.data)
        except (TypeError, json.JSONDecodeError):
            return
        if result.get("request_id") != self.manipulation_pending_id:
            self.get_logger().warn(
                "manip_result ignored request_id="
                f"{result.get('request_id')} pending="
                f"{self.manipulation_pending_id}"
            )
            return
        self.manipulation_result = result
        self._consume_manipulation_result()

    def _reset_manipulation(self) -> None:
        """Clear the in-flight manipulation state."""
        self.manipulation_active = False
        self.manipulation_operation = None
        self.manipulation_job = None
        self.manipulation_deadline = None
        self.manipulation_timed_out = False
        self.manipulation_result = None

    def _place_slot_poses(self, job) -> tuple:
        """Assign a world-fixed slot despite Nav2 docking pose variation."""
        y_offset, slot_index, slot_count = self._delivery_slot_offset(job)
        robot_pose = self._odom_pose()
        if robot_pose is None:
            local_x = self.place_pose[0]
            local_y = self.place_pose[1] + y_offset
            frame = "fixed_fallback"
        else:
            center_x, center_y = self.zone_centers[job.target_zone]
            local_x, local_y = world_target_in_base(
                robot_pose,
                (center_x, center_y + y_offset),
            )
            frame = "world_compensated"
        requested_x, requested_y = local_x, local_y
        local_x = min(
            max(local_x, self.place_minimum_x),
            self.place_maximum_x,
        )
        local_y = min(
            max(local_y, -self.place_maximum_abs_y),
            self.place_maximum_abs_y,
        )
        clamped = not (
            math.isclose(local_x, requested_x)
            and math.isclose(local_y, requested_y)
        )
        place_pose = (local_x, local_y, self.place_pose[2])
        place_safe_pose = (local_x, local_y, self.place_safe_pose[2])
        self.get_logger().info(
            "delivery_slot "
            f"zone={job.target_zone} slot={slot_index + 1}/"
            f"{slot_count} y_offset={y_offset:.3f} frame={frame} "
            f"local=({local_x:.3f},{local_y:.3f}) clamped={clamped}"
        )
        return place_pose, place_safe_pose

    def _note_place_outcome(self, result, operation: str,
                            success: bool) -> None:
        """Remember or clear the deterministic place-refusal geometry.

        A refusal the arm reports as a workspace/dock geometry rejection is
        reproducible for the same slot and the same measured base pose, so
        the requested geometry is stored and the next request is refused
        unless the base actually moved.
        """
        if operation != "place":
            return
        self.last_place_envelope_drift = result.get("place_envelope_drift")
        correction = result.get("place_slot_correction")
        if isinstance(correction, (list, tuple)) and len(correction) == 2:
            self.last_place_arm_residual = (
                float(correction[0]),
                float(correction[1]),
            )
        else:
            self.last_place_arm_residual = None
        if success:
            self._refused_place_geometry = None
            return
        reason = str(result.get("failure_reason") or "")
        if reason not in DETERMINISTIC_PLACE_FAILURES:
            self._refused_place_geometry = None
            return
        job = self.manipulation_job or self._active_job()
        pose = self._odom_pose()
        if job is None or pose is None:
            self._refused_place_geometry = None
            return
        self._refused_place_geometry = {
            "slot": self._delivery_slot_world(job),
            "pose": (float(pose[0]), float(pose[1]), float(pose[2])),
            "reason": reason,
        }
        self.get_logger().warn(
            "place_geometry_refused "
            f"reason={reason} cube={job.cube_name} "
            f"slot=({self._refused_place_geometry['slot'][0]:.3f},"
            f"{self._refused_place_geometry['slot'][1]:.3f}) "
            f"base=({pose[0]:.3f},{pose[1]:.3f},{pose[2]:.3f})"
        )

    def _place_repeat_refusal(self, job) -> Optional[str]:
        """Return a reason when a place would repeat a refused geometry."""
        previous = self._refused_place_geometry
        if previous is None or job is None:
            return None
        pose = self._odom_pose()
        if pose is None:
            return None
        slot = self._delivery_slot_world(job)
        slot_delta = math.hypot(
            slot[0] - previous["slot"][0],
            slot[1] - previous["slot"][1],
        )
        pose_delta = math.hypot(
            pose[0] - previous["pose"][0],
            pose[1] - previous["pose"][1],
        )
        yaw_delta = abs(angle_delta(pose[2], previous["pose"][2]))
        if (
            slot_delta <= PLACE_REPEAT_POSE_TOLERANCE
            and pose_delta <= PLACE_REPEAT_POSE_TOLERANCE
            and yaw_delta <= PLACE_REPEAT_YAW_TOLERANCE
        ):
            return (
                f"slot_unreachable_repeat slot_delta={slot_delta:.4f} "
                f"pose_delta={pose_delta:.4f} yaw_delta={yaw_delta:.4f}"
            )
        self.get_logger().info(
            "place_retry_geometry_changed "
            f"slot_delta={slot_delta:.4f} pose_delta={pose_delta:.4f} "
            f"yaw_delta={yaw_delta:.4f}"
        )
        return None

    def _delivery_slot_world(self, job) -> tuple:
        """Return the job's commanded world slot centre as (x, y).

        Single source of the world slot: the arm's ``release_world_pose``,
        the delivery truth check and the navigator's final dock heading all
        read this, so the delivery point and the dock cannot drift apart.

        The slot is pulled toward the approach side so the arm's checked
        extension lands in the middle of its reach band instead of at the
        0.38 m clamp. The zone dock keeps a 0.40 m standoff and Nav2 may stop
        from 0.20 m short to slightly past the goal, so the shift is measured
        against the current base pose rather than assumed: the cube is then
        released where the arm reaches, still inside the pad.
        """
        y_offset, _, _ = self._delivery_slot_offset(job)
        center_x, center_y = self.zone_centers[job.target_zone]
        slot_x = float(center_x)
        slot_y = float(center_y + y_offset)
        bias = self.delivery_slot_approach_bias
        if bias <= 0.0:
            return (slot_x, slot_y)
        nav = self.zone_nav_poses[job.target_zone]
        dx = float(nav[0]) - center_x
        dy = float(nav[1]) - center_y
        norm = math.hypot(dx, dy)
        if norm <= 1e-9:
            return (slot_x, slot_y)
        shift = min(bias, self.zone_half_extent[0] - DELIVERY_SLOT_EDGE_MARGIN)
        pose = self._odom_pose()
        if pose is not None:
            # Distance from the pad centre to the base along the approach
            # axis; keep the commanded extension at the band centre.
            forward = (
                (float(pose[0]) - slot_x) * dx / norm
                + (float(pose[1]) - slot_y) * dy / norm
            )
            shift = min(max(forward - DELIVERY_REACH_TARGET, 0.0), shift)
        return (
            slot_x + shift * dx / norm,
            slot_y + shift * dy / norm,
        )

    def _delivery_slot_offset(self, job) -> tuple:
        """Return the job's centered zone-slot offset and stable index."""
        jobs_in_zone = [
            candidate
            for candidate in self.machine.jobs
            if candidate.target_zone == job.target_zone
        ]
        # Dynamic inventory selection may substitute another physical cube
        # for this logical job. Delivery slots belong to the job order, not
        # to the selected cube name.
        slot_index = next(
            index
            for index, candidate in enumerate(jobs_in_zone)
            if candidate.index == job.index
        )
        count = len(jobs_in_zone)
        # Spread the slots across the arm's lateral reach window instead of
        # a fixed 0.08 m pitch. The chassis clears the 3 cm cube (base plate
        # bottom sits ~5.3 cm above ground), so the observed collision is the
        # gripper brushing an already-placed cube while descending to place an
        # adjacent one. Few-cube deliveries now keep much larger separation
        # (2 cubes -> 0.32 m apart) while 5-cube fills stay at the arm limit.
        y_offset = centered_slot_offset(slot_index, count, self.place_slot_spacing,
                                        self.place_maximum_abs_y)
        return y_offset, slot_index, count

    def _expected_cube_base_position(
        self,
        cube_name: str,
    ) -> Optional[tuple]:
        """
        Return the expected base-frame cube position from the world slot.

        The wrist camera reports in base_link, so the expected position is
        the world slot expressed in the robot base frame using the current
        odometry yaw (a pure x/y difference is only valid when the robot
        faces exactly south, which the rotated office no longer allows).
        Returns None when the cube is unknown or localization is
        unavailable, falling back to colour-only pick.
        """
        world = CUBE_WORLD_SLOTS.get(cube_name)
        if world is None:
            return None
        pose = self._odom_pose()
        if pose is None:
            return None
        robot_x, robot_y, robot_yaw = pose[0], pose[1], pose[2]
        delta_x = world[0] - robot_x
        delta_y = world[1] - robot_y
        cos_yaw = math.cos(robot_yaw)
        sin_yaw = math.sin(robot_yaw)
        return (
            delta_x * cos_yaw + delta_y * sin_yaw,
            -delta_x * sin_yaw + delta_y * cos_yaw,
        )

    def _align_dock_yaw(self, cube_name: str) -> None:
        """
        Rotate the base back to the dock heading before scanning.

        The robot can arrive at a cube dock with a large yaw error (observed
        -0.8 rad on red_cube_3): the wrist camera then points away from the
        cube, the cached slot reads a large lateral offset and the scan
        finds nothing, which ends in a skip. Rotate in place (bounded, slow)
        to the dock's intended yaw whenever the error exceeds 0.08 rad.
        """
        dock = self._cube_dock_pose(cube_name)
        if dock is None:
            return
        target_yaw = float(dock[2])
        start = self._odom_pose()
        if start is None:
            return

        def yaw_error() -> Optional[float]:
            current = self._odom_pose()
            if current is None:
                return None
            return math.atan2(
                math.sin(target_yaw - current[2]),
                math.cos(target_yaw - current[2]),
            )

        error = yaw_error()
        if error is None or abs(error) <= 0.08:
            return
        self.get_logger().warn(
            f"dock_yaw_align:{cube_name}:"
            f"yaw={start[2]:.3f} target={target_yaw:.3f} "
            f"error={error:.3f}"
        )
        speed = 0.5
        deadline = time.monotonic() + min(
            abs(error) / 0.35, 4.0
        )
        while time.monotonic() < deadline:
            error = yaw_error()
            if error is None:
                break
            if abs(error) <= 0.06:
                break
            twist = Twist()
            twist.angular.z = math.copysign(
                max(0.15, min(speed, 1.8 * abs(error))),
                error,
            )
            self.cmd_vel_publisher.publish(twist)
            time.sleep(0.05)
        stop = Twist()
        for _ in range(5):
            self.cmd_vel_publisher.publish(stop)
            time.sleep(0.02)
        final = yaw_error()
        self.get_logger().info(
            f"dock_yaw_align_result:{cube_name}:error={final}"
        )

    def _realign_dock_to_cached_slot(self, cube_name: str) -> None:
        """Correct a small dock error before the normal camera scan."""
        self._align_dock_yaw(cube_name)
        expected = self._expected_cube_base_position(cube_name)
        if expected is None:
            self.get_logger().warn(
                f"dock_realign_skipped:{cube_name}:odom_unavailable"
            )
            return
        forward, lateral = bounded_dock_realign(*expected)
        if forward == 0.0 and lateral == 0.0:
            self.get_logger().info(
                f"dock_realign_within_tolerance:{cube_name}:"
                f"slot=({expected[0]:.3f},{expected[1]:.3f})"
            )
            return

        start = self._odom_pose()
        if start is None:
            return

        def displacement():
            current = self._odom_pose()
            if current is None:
                return None
            dx = current[0] - start[0]
            dy = current[1] - start[1]
            yaw = start[2]
            return (
                dx * math.cos(yaw) + dy * math.sin(yaw),
                -dx * math.sin(yaw) + dy * math.cos(yaw),
            )

        speed = 0.20
        observed_speed = 0.40 * speed
        deadline = time.monotonic() + min(
            max(abs(forward), abs(lateral)) / observed_speed,
            3.0,
        )
        self.get_logger().info(
            f"dock_realign_start:{cube_name}:"
            f"slot=({expected[0]:.3f},{expected[1]:.3f}) "
            f"correction=({forward:.3f},{lateral:.3f})"
        )
        while time.monotonic() < deadline:
            moved = displacement()
            if moved is None:
                break
            remaining_forward = forward - moved[0]
            remaining_lateral = lateral - moved[1]
            if (
                abs(remaining_forward) <= 0.005
                and abs(remaining_lateral) <= 0.005
            ):
                break
            twist = Twist()
            # Constant-speed bang-bang overshot the target: measured moves
            # were 1.5-1.7x the request (2026-10-06, e.g. asked
            # (-0.018,-0.034), moved (-0.052,-0.058)) because the base coasts
            # through the 0.05 s command latency at 0.20 m/s. That left
            # 2.1 cm of dock error, where the arm then failed
            # (pick_descend truncated, no IK fallback) and the same-colour
            # retry cost the case its ordered-pick audit. Approach the target
            # proportionally so the last centimetres are slow and land on it.
            if abs(remaining_forward) > 0.005:
                twist.linear.x = math.copysign(
                    min(speed, max(0.04, 2.0 * abs(remaining_forward))),
                    remaining_forward,
                )
            if abs(remaining_lateral) > 0.005:
                twist.linear.y = math.copysign(
                    min(speed, max(0.04, 2.0 * abs(remaining_lateral))),
                    remaining_lateral,
                )
            self.cmd_vel_publisher.publish(twist)
            time.sleep(0.05)
        stop = Twist()
        for _ in range(5):
            self.cmd_vel_publisher.publish(stop)
            time.sleep(0.02)
        self.get_logger().info(
            f"dock_realign_result:{cube_name}:moved={displacement()}"
        )

    def _center_target_laterally(self, target_y: float, timeout: float = 4.0):
        """
        Shift the base sideways so the detected cube is centred under link6.

        The wrist camera reports the cube offset in base_link (verified
        against all five known slots), so the base moves in the same
        direction as the detected y to centre the cube. Uses closed-loop
        odometry feedback so the manoeuvre is accurate even when the
        simulation runs slower or faster than wall-clock time.
        """
        if abs(target_y) < 0.012:
            return True
        sign = 1.0 if target_y > 0 else -1.0
        speed = 0.20
        # Outer shelf slots are up to 0.24 m off-centre; the wall-clock
        # budget must cover the observed ~85% of commanded base speed so the
        # closed loop can actually reach the target travel.
        duration = centering_time_budget(target_y, speed)
        start = self._odom_pose()
        twist = Twist()
        twist.linear.y = sign * speed
        deadline = time.monotonic() + duration
        self.get_logger().info(
            f"centering_lateral target_y={target_y:.4f} "
            f"sign={sign:.0f} speed={speed:.3f} duration={duration:.3f}"
        )
        rate = 20
        target_travel = abs(target_y)
        while time.monotonic() < deadline:
            self.cmd_vel_publisher.publish(twist)
            moved = self._lateral_displacement(start)
            if moved is not None and abs(moved) >= target_travel - 0.005:
                break
            time.sleep(1.0 / rate)
        twist.linear.y = 0.0
        for _ in range(5):
            self.cmd_vel_publisher.publish(twist)
            time.sleep(0.02)
        moved = self._lateral_displacement(start)
        self.get_logger().info(
            "centering_result "
            f"moved={moved} "
            f"residual_y={target_y - moved if moved is not None else 0.0:.4f}"
        )
        return True

    def _odom_callback(self, message: Odometry) -> None:
        """Cache the latest base pose for lateral-centring feedback."""
        pose = message.pose.pose
        quaternion = pose.orientation
        yaw = math.atan2(
            2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
            1.0 - 2.0
            * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
        )
        self.odom_pose = (pose.position.x, pose.position.y, yaw)

    def _odom_pose(self):
        """Return the latest (x, y, yaw) base pose from /odom or None."""
        return self.odom_pose

    def _lateral_displacement(self, start):
        """Return base-frame lateral travel since the start pose, or None."""
        current = self._odom_pose()
        if current is None or start is None:
            return None
        dx = current[0] - start[0]
        dy = current[1] - start[1]
        yaw = current[2]
        return -dx * math.sin(yaw) + dy * math.cos(yaw)

    @staticmethod
    def _vector_arguments(name: str, values: Sequence[float]) -> list:
        return [
            "-p",
            f"{name}:={json.dumps([round(value, 6) for value in values])}",
        ]

    def start_verification(self, action: MissionAction) -> None:
        """Query Gazebo truth before counting one delivered cube."""
        if action.job is None:
            self.publish_error("Verification action omitted its job")
            return
        if not self.entity_client.service_is_ready():
            self.publish_error("/gazebo/get_entity_state is unavailable")
            follow_up = self.machine.verification_finished(False)
            self.handle_action(follow_up)
            return
        request = GetEntityState.Request()
        active = self._active_job() or action.job
        self.verification_cube_name = active.cube_name
        request.name = active.cube_name
        request.reference_frame = "world"
        self.verification_future = self.entity_client.call_async(request)
        self.verification_future.add_done_callback(
            self.verification_callback
        )

    def verification_callback(self, future) -> None:
        """Check the queried cube against its requested zone bounds."""
        job = self.machine.current_job
        success = False
        try:
            response = future.result()
            if response.success and job is not None:
                center_x, center_y = self.zone_centers[job.target_zone]
                half_x, half_y = self.zone_half_extent
                expected_z = (
                    self.delivery_surface_z
                    + self.delivery_cube_half_height
                )
                position = response.state.pose.position
                slot_x, slot_y = self._delivery_slot_world(job)
                slot_error = math.hypot(
                    position.x - slot_x,
                    position.y - slot_y,
                )
                center_offset = math.hypot(
                    position.x - center_x,
                    position.y - center_y,
                )
                precise = slot_error <= self.delivery_precision_tolerance
                success = (
                    abs(position.x - center_x) <= half_x
                    and abs(position.y - center_y) <= half_y
                    and abs(position.z - expected_z)
                    <= self.delivery_height_tolerance
                )
                residual = self.last_place_arm_residual
                arm_residual = (
                    math.hypot(float(residual[0]), float(residual[1]))
                    if residual
                    else -1.0
                )
                drift = self.last_place_envelope_drift
                drift_text = "none" if drift is None else ",".join(
                    f"{float(value):.4f}" for value in drift
                )
                self.get_logger().info(
                    "delivery_truth "
                    f"cube={self.verification_cube_name} "
                    f"zone={job.target_zone} "
                    f"x={position.x:.4f} y={position.y:.4f} "
                    f"z={position.z:.4f} valid={success} "
                    f"slot_error={slot_error:.4f} precise={precise} "
                    f"center_offset={center_offset:.4f} "
                    f"arm_residual={arm_residual:.4f} "
                    f"envelope_drift={drift_text}"
                )
                if success:
                    self.placed_cubes[self.verification_cube_name] = (
                        float(position.x),
                        float(position.y),
                    )
                    self._publish_placed_cubes()
        except Exception as exception:  # noqa: B902
            self.publish_error(f"Delivery verification failed: {exception}")
        self.verification_future = None
        self.verification_cube_name = None
        try:
            action = self.machine.verification_finished(success)
        except MissionValueError as exception:
            self.publish_error(str(exception))
            return
        self.handle_action(action)

    def _publish_placed_cubes(self) -> None:
        """
        Publish placed-cube world positions as a PointCloud2.

        The global costmap obstacle_layer consumes this so the planner routes
        the chassis around already-placed cubes instead of driving over the
        delivery-slot line.
        """
        points = [
            (float(x), float(y), 0.035)
            for x, y in self.placed_cubes.values()
        ]
        cloud = PointCloud2()
        cloud.header.stamp = self.get_clock().now().to_msg()
        cloud.header.frame_id = "map"
        cloud.height = 1
        cloud.width = len(points)
        cloud.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = 12 * len(points)
        cloud.is_dense = True
        cloud.data = b"".join(struct.pack("<fff", *p) for p in points)
        self.placed_cubes_publisher.publish(cloud)


def main(args=None) -> None:
    """Run the competition mission coordinator."""
    rclpy.init(args=args)
    node = MissionCoordinator()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
