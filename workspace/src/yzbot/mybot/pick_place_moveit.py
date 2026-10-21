#!/usr/bin/env python3

"""Execute one collision-checked MoveIt pick-and-place cycle in Gazebo."""

import copy
import json
import math
import threading
import time
from typing import Dict, Optional, Sequence, Tuple

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import GetEntityState, SetEntityState
from geometry_msgs.msg import Pose, PoseStamped, Twist
from linkattacher_msgs.srv import AttachLink, DetachLink
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    CollisionObject,
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
    PlanningScene,
    PositionIKRequest,
    RobotState,
)
from moveit_msgs.srv import (
    ApplyPlanningScene,
    GetCartesianPath,
    GetPositionIK,
)
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Bool, String
from trajectory_msgs.msg import JointTrajectoryPoint


ARM_JOINT_NAMES = (
    "joint1",
    "joint2",
    "joint3",
    "joint4",
    "joint5",
    "joint6",
)
BASE_LINK_HEIGHT = 0.1034
STAND_HEIGHT = 0.124
STAND_CENTER_Z = STAND_HEIGHT / 2.0 - BASE_LINK_HEIGHT
CUBE_HALF_SIZE = 0.015
# LinkAttacher carries the cube; leave air between both finger plates and a
# laterally offset cube instead of squeezing it with a fixed joint position.
FINGER_INNER_GAP_OPEN = 0.065
GRIPPER_CONTACT_MARGIN = 0.003
GRIPPER_MAX_CLOSE_POSITION = 0.020
MAX_ATTACHED_DRIFT = 0.020
GRASP_XY_LIMIT = 0.035
GRASP_Z_LIMITS = (0.010, 0.090)
# Bounded correction applied to the descend target when the live cube pose
# shows the requested grasp pose is off. The values match the bounds
# _retry_grasp_with_correction has applied since 2026-09-11, so a pre-descend
# correction and a post-descend correction accept exactly the same offsets.
APPROACH_CORRECTION_X_LIMIT = 0.12
APPROACH_CORRECTION_Y_LIMIT = 0.08
# The low retract settles an attached cube 2--4 mm farther forward at the
# 500 Hz simulation step. Keep the lateral envelope at 60 mm, while allowing
# this bounded 65 mm fore-aft attachment offset.
CARRY_X_LIMIT = 0.065
CARRY_Y_LIMIT = 0.060
CARRY_Z_LIMITS = (-0.030, 0.080)
# After the tray-clearing retract, keep a residual lift so the gripper and
# attached cube sit above the shelf top while the base backs off. Full
# pick-height carry left the end-effector on the tray face and the departure
# scraped then snapped at the edge; a 50 mm lift all the way put the payload in
# the lidar plane. 15 mm still left the gripper bottom below the shelf edge on
# real rounds (20261013, visible jolt when leaving the shelf), so the default
# is 45 mm, the transport envelope's own limit (TRANSPORT_PAYLOAD_Z_LIMITS
# tops out at 0.080 and the low carry sits 45 mm under the cube). The 0.10 m
# shelf clearance comes from transport_lift during the retract instead.
CARRY_CLEARANCE_LIFT = 0.045
TRANSPORT_PAYLOAD_RADIUS_LIMIT = 0.350
# The low retraction leaves the cube center about 18 mm below base_link while
# its bottom remains more than 5 cm above the world ground plane.
TRANSPORT_PAYLOAD_Z_LIMITS = (-0.035, 0.080)
TRANSPORT_BASE_TILT_LIMIT = 0.15
CARTESIAN_RELATIVE_JUMP_THRESHOLD = 0.0
CARTESIAN_PRISMATIC_JUMP_THRESHOLD = 0.02
CARTESIAN_REVOLUTE_JUMP_THRESHOLD = 0.50
# URDF revolute limits from mybot_description/urdf/arm.xacro. Gazebo's
# position command interface does not clamp to them, so a fast trajectory
# ending near a bound can settle outside it. MoveIt then refuses to plan
# from that start state and every later request fails, so the arm has to be
# pulled back inside the model's own limits before the next plan.
ARM_JOINT_LIMITS = {
    "joint1": 3.14,
    "joint2": 2.35,
    "joint3": 2.62,
    "joint4": 3.14,
    "joint5": 2.35,
    "joint6": 3.14,
}
# Stay clear of the bound itself so the correction does not settle straight
# back onto it. Smaller than MoveIt's 0.10 rad start-state allowance.
ARM_JOINT_LIMIT_MARGIN = 0.09


def quaternion_norm(quaternion: Sequence[float]) -> float:
    """Return the Euclidean norm of an x-y-z-w quaternion."""
    return math.sqrt(sum(component * component for component in quaternion))


def position_within_window(
    position: Sequence[float],
    xy_limit: float,
    z_limits: Sequence[float],
) -> bool:
    """Return whether a link-relative point is inside a safety window."""
    x_value, y_value, z_value = position
    z_minimum, z_maximum = z_limits
    return (
        abs(x_value) <= xy_limit
        and abs(y_value) <= xy_limit
        and z_minimum <= z_value <= z_maximum
    )


def clearance_gripper_position(cube_lateral_offset: float) -> float:
    """Choose a non-contact close position for an offset 30 mm cube."""
    offset = float(cube_lateral_offset)
    if not math.isfinite(offset):
        raise ValueError("Cube lateral offset must be finite")
    clearance = (
        FINGER_INNER_GAP_OPEN
        - 2.0 * CUBE_HALF_SIZE
        - 2.0 * abs(offset)
        - 2.0 * GRIPPER_CONTACT_MARGIN
    )
    return max(0.0, min(GRIPPER_MAX_CLOSE_POSITION, clearance))


def position_within_carry_window(position: Sequence[float]) -> bool:
    """Return whether an attached cube remains in the asymmetric carry bay."""
    x_value, y_value, z_value = position
    z_minimum, z_maximum = CARRY_Z_LIMITS
    return (
        abs(x_value) <= CARRY_X_LIMIT
        and abs(y_value) <= CARRY_Y_LIMIT
        and z_minimum <= z_value <= z_maximum
    )


def joint_limit_corrections(
    positions: Dict[str, float],
    limits: Dict[str, float] = ARM_JOINT_LIMITS,
    margin: float = ARM_JOINT_LIMIT_MARGIN,
) -> Dict[str, float]:
    """
    Return corrections for joints outside their model limits.

    Only genuinely out-of-bound joints are reported. A joint resting inside
    its limit is left exactly where the motion left it, so this never
    disturbs a valid arm pose or the poses the placement IK chose.
    """
    corrections: Dict[str, float] = {}
    for joint_name, limit in limits.items():
        value = positions.get(joint_name)
        if value is None:
            continue
        if abs(value) <= limit:
            continue
        target = math.copysign(max(limit - margin, 0.0), value)
        corrections[joint_name] = target
    return corrections


def normalize_quaternion(
    quaternion: Sequence[float],
) -> Tuple[float, float, float, float]:
    """Normalize an x-y-z-w quaternion."""
    norm = quaternion_norm(quaternion)
    if norm <= 1.0e-12:
        raise ValueError("Quaternion norm is zero")
    return tuple(component / norm for component in quaternion)


def rotate_vector(
    quaternion: Sequence[float],
    vector: Sequence[float],
) -> Tuple[float, float, float]:
    """Rotate a three-dimensional vector by an x-y-z-w quaternion."""
    qx, qy, qz, qw = normalize_quaternion(quaternion)
    vx, vy, vz = vector

    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)

    return (
        vx + qw * tx + qy * tz - qz * ty,
        vy + qw * ty + qz * tx - qx * tz,
        vz + qw * tz + qx * ty - qy * tx,
    )


def offset_from_tool(
    tool_xyz: Sequence[float],
    quaternion: Sequence[float],
    offset_z: float,
) -> Tuple[float, float, float]:
    """Return a point offset along the tool's positive Z axis."""
    offset = rotate_vector(quaternion, (0.0, 0.0, offset_z))
    return tuple(tool_xyz[index] + offset[index] for index in range(3))


def yaw_from_quaternion(quaternion: Sequence[float]) -> float:
    """Extract planar yaw from an x-y-z-w quaternion."""
    qx, qy, qz, qw = normalize_quaternion(quaternion)
    return math.atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz),
    )


def roll_pitch_from_quaternion(
    quaternion: Sequence[float],
) -> Tuple[float, float]:
    """Extract roll and pitch from an x-y-z-w quaternion."""
    qx, qy, qz, qw = normalize_quaternion(quaternion)
    roll = math.atan2(
        2.0 * (qw * qx + qy * qz),
        1.0 - 2.0 * (qx * qx + qy * qy),
    )
    sin_pitch = 2.0 * (qw * qy - qz * qx)
    pitch = math.asin(max(-1.0, min(1.0, sin_pitch)))
    return roll, pitch


def local_to_world_position(
    model_xyz: Sequence[float],
    model_quaternion: Sequence[float],
    local_base_xyz: Sequence[float],
) -> Tuple[float, float, float]:
    """Convert a base_link point to world using the planar mobile base pose."""
    yaw = yaw_from_quaternion(model_quaternion)
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    local_x, local_y, local_z = local_base_xyz
    return (
        model_xyz[0] + cosine * local_x - sine * local_y,
        model_xyz[1] + sine * local_x + cosine * local_y,
        model_xyz[2] + BASE_LINK_HEIGHT + local_z,
    )


def world_to_local_position(
    model_xyz: Sequence[float],
    model_quaternion: Sequence[float],
    world_xyz: Sequence[float],
) -> Tuple[float, float, float]:
    """Convert a world point to the planar base_link frame."""
    yaw = yaw_from_quaternion(model_quaternion)
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    dx = world_xyz[0] - model_xyz[0]
    dy = world_xyz[1] - model_xyz[1]
    return (
        cosine * dx + sine * dy,
        -sine * dx + cosine * dy,
        world_xyz[2] - model_xyz[2] - BASE_LINK_HEIGHT,
    )


def loaded_slot_tool_pose(
    model_xyz: Sequence[float],
    model_quaternion: Sequence[float],
    slot_xyz: Sequence[float],
    cube_in_tool: Sequence[float],
    tool_quaternion: Sequence[float],
) -> Tuple[float, float, float]:
    """Put the attached cube over its slot without grounding the fingers.

    Compensate the measured fixed-joint offset rather than treating link6 as
    the cube centre. Hover 40 mm over the pad; the 82 mm downward fingers
    retain 20 mm surface clearance. A 0.38 m extension is the delivery point,
    not an assertion that the arm reached an occasionally unreachable slot.
    The caller must prove this point is inside the pad and bound the final
    stabilization correction separately from the measured motion error.
    """
    if (any(len(v) != 3 for v in (model_xyz, slot_xyz, cube_in_tool))
            or not all(math.isfinite(float(x)) for v in
                       (model_xyz, model_quaternion, slot_xyz,
                        cube_in_tool, tool_quaternion) for x in v)):
        raise ValueError("Placement geometry must be finite")
    target = world_to_local_position(
        model_xyz, model_quaternion,
        (slot_xyz[0], slot_xyz[1], slot_xyz[2] + 0.040),
    )
    offset = rotate_vector(tool_quaternion, cube_in_tool)
    xyz = tuple(target[i] - offset[i] for i in range(3))
    # Finger link centre is 42 mm along tool Z and its half-length is 40 mm.
    floor_z = (slot_xyz[2] - CUBE_HALF_SIZE + 0.020 + 0.082
               - model_xyz[2] - BASE_LINK_HEIGHT)
    if xyz[0] > 0.56:
        raise ValueError(f"Slot outside checked placement workspace: {xyz}")
    xyz = (min(xyz[0], 0.38), xyz[1], max(xyz[2], floor_z))
    if not (0.25 <= xyz[0] <= 0.38 and abs(xyz[1]) <= 0.12
            and -0.02 <= xyz[2] <= 0.20):
        raise ValueError(f"Slot outside checked placement workspace: {xyz}")
    return xyz


def loaded_slot_approach_waypoints(
    target: Sequence[float],
) -> Tuple[Tuple[float, float, float], ...]:
    """Lift inside the chassis, then extend above the delivery surface.

    A diagonal from low carry straight to (0.38, y, 0.16) couples extension
    and elbow lift and can truncate on the loaded IK branch. Separate the
    two motions. A 0.10 m tool height already puts the lowest finger above
    the delivery pad; gratuitous high extension is not needed.
    """
    x, y, z = (float(value) for value in target)
    if not all(math.isfinite(value) for value in (x, y, z)):
        raise ValueError("Placement approach must be finite")
    height = max(z + 0.05, 0.10)
    return ((min(x, 0.28), y, height), (x, y, height))


def duration_from_seconds(seconds: float) -> Duration:
    """Convert floating-point seconds to a ROS duration message."""
    if seconds < 0.0:
        raise ValueError("Duration must be non-negative")
    whole_seconds = int(seconds)
    nanoseconds = int(round((seconds - whole_seconds) * 1_000_000_000))
    if nanoseconds == 1_000_000_000:
        whole_seconds += 1
        nanoseconds = 0
    return Duration(sec=whole_seconds, nanosec=nanoseconds)


def retime_joint_trajectory(trajectory, duration_sec: float) -> None:
    """Assign safe, strictly increasing timestamps to a joint trajectory.

    Invert a raised-cosine distance profile to get timestamps. Applying the
    cosine directly to uniformly spaced path points compresses the first and
    last time intervals, making the arm fastest at both endpoints. That can
    abruptly jerk a loaded wrist during placement. The inverse gives the
    endpoint intervals more time while retaining the requested duration.
    """
    points = trajectory.joint_trajectory.points
    if not points:
        raise ValueError("Cannot retime an empty trajectory")
    if not math.isfinite(duration_sec) or duration_sec <= 0.0:
        raise ValueError("Trajectory duration must be positive")

    point_count = len(points)
    distances = [0.0]
    for previous, current in zip(points, points[1:]):
        if len(previous.positions) != len(current.positions):
            raise ValueError("Trajectory joint counts do not match")
        if not all(math.isfinite(value) for value in current.positions):
            raise ValueError("Trajectory joint positions must be finite")
        step = math.sqrt(sum(
            (end - start) ** 2
            for start, end in zip(previous.positions, current.positions)
        ))
        distances.append(distances[-1] + step)
    path_length = distances[-1]
    if not all(math.isfinite(value) for value in points[0].positions):
        raise ValueError("Trajectory joint positions must be finite")
    for index, point in enumerate(points, start=1):
        if path_length > 0.0:
            # One virtual path step gives the first controller point a
            # nonzero timestamp even when it repeats the start state.
            virtual_step = path_length / point_count
            progress = (
                distances[index - 1]
                + virtual_step
                + index * virtual_step * 0.01
            ) / (
                path_length + virtual_step
                + point_count * virtual_step * 0.01
            )
        else:
            progress = index / point_count
        progress = max(index * 1.0e-9, min(1.0, progress))
        timestamp = duration_sec * math.acos(1.0 - 2.0 * progress) / math.pi
        point.time_from_start = duration_from_seconds(timestamp)
        point.velocities = []
        point.accelerations = []
        point.effort = []


def point_distance(first: Sequence[float], second: Sequence[float]) -> float:
    """Return Euclidean distance between two three-dimensional points."""
    return math.sqrt(
        sum((first[index] - second[index]) ** 2 for index in range(3))
    )


def pose_from_values(
    xyz: Sequence[float],
    quaternion: Sequence[float],
) -> Pose:
    """Build a geometry pose from numeric sequences."""
    qx, qy, qz, qw = normalize_quaternion(quaternion)
    pose = Pose()
    pose.position.x = float(xyz[0])
    pose.position.y = float(xyz[1])
    pose.position.z = float(xyz[2])
    pose.orientation.x = qx
    pose.orientation.y = qy
    pose.orientation.z = qz
    pose.orientation.w = qw
    return pose


def transit_pose(
    pick_safe_pose: Sequence[float],
    transit_height: float,
) -> Tuple[float, float, float]:
    """Return a vertical-clearance pose above the selected cube."""
    if len(pick_safe_pose) != 3:
        raise ValueError("pick_safe_pose must contain exactly 3 values")
    if transit_height <= float(pick_safe_pose[2]):
        raise ValueError(
            "pick_transit_height must exceed the safe pick height"
        )
    return (
        float(pick_safe_pose[0]),
        float(pick_safe_pose[1]),
        float(transit_height),
    )


def transit_pose_for_operation(
    operation: str,
    pick_safe_pose: Sequence[float],
    transit_height: float,
) -> Tuple[float, float, float]:
    """Validate inventory clearance only for operations that pick a cube."""
    if operation in {"pick", "cycle"}:
        return transit_pose(pick_safe_pose, transit_height)
    if len(pick_safe_pose) != 3:
        raise ValueError("pick_safe_pose must contain exactly 3 values")
    return tuple(float(value) for value in pick_safe_pose)


def nearest_equivalent_angle(target: float, current: float) -> float:
    """Shift one joint target by whole turns to the value nearest `current`.

    A revolute target handed to MoveGroup as an absolute angle can be planned
    through a full turn when the IK branch lands on target +- 2*pi - the "arm
    swung a big circle and tripped a joint error" and "arm spun a full circle
    while placing" reports of 20261014. Snapping to the nearest equivalent
    angle bounds the planned motion without changing the pose reached.
    """
    target = float(target)
    current = float(current)
    if not (math.isfinite(target) and math.isfinite(current)):
        return target
    return target + 2.0 * math.pi * round(
        (current - target) / (2.0 * math.pi)
    )


def low_transport_pose(
    pick_pose: Sequence[float],
    transport_x: float,
    lift: float = CARRY_CLEARANCE_LIFT,
) -> Tuple[float, float, float]:
    """Retract a grasp to a slightly lifted low carry pose."""
    if not math.isfinite(float(lift)) or float(lift) < 0.0:
        raise ValueError("Carry clearance lift must be finite and >= 0")
    if len(pick_pose) != 3:
        raise ValueError("pick_pose must contain exactly 3 values")
    values = tuple(float(value) for value in pick_pose)
    target_x = float(transport_x)
    if not all(math.isfinite(value) for value in values + (target_x,)):
        raise ValueError("Low transport pose values must be finite")
    if target_x <= 0.0 or target_x >= values[0]:
        raise ValueError("transport_x must retract behind the pick pose")
    return (target_x, values[1], values[2] + float(lift))


def transport_payload_within_envelope(
    xyz: Sequence[float],
    radius_limit: float = TRANSPORT_PAYLOAD_RADIUS_LIMIT,
    z_limits: Sequence[float] = TRANSPORT_PAYLOAD_Z_LIMITS,
) -> bool:
    """Return whether the carried cube stays inside the chassis envelope."""
    if len(xyz) != 3:
        return False
    x_value, y_value, z_value = (float(value) for value in xyz)
    z_minimum, z_maximum = z_limits
    return (
        all(math.isfinite(value) for value in (x_value, y_value, z_value))
        and math.hypot(x_value, y_value) <= radius_limit
        and z_minimum <= z_value <= z_maximum
    )


def attachment_anchor_within_window(xyz: Sequence[float]) -> bool:
    """Reject a stable fixed joint whose anchor jumped away from the wrist."""
    return position_within_window(
        xyz,
        GRASP_XY_LIMIT,
        CARRY_Z_LIMITS,
    )


class MoveItPickPlace(Node):
    """Coordinate MoveIt, ros2_control, Gazebo state, and LinkAttacher."""

    def __init__(self) -> None:
        super().__init__("moveit_pick_place")

        self.declare_parameter("cube_name", "red_cube_1")
        self.declare_parameter("operation", "cycle")
        self.declare_parameter("persistent", False)
        self.declare_parameter("prepare_scene", True)
        self.declare_parameter("verify_placement", True)
        self.declare_parameter(
            "state_topic",
            "/competition/manipulation/state",
        )
        self.declare_parameter(
            "request_topic",
            "/competition/manipulation/request",
        )
        self.declare_parameter(
            "result_topic",
            "/competition/manipulation/result",
        )
        self.declare_parameter("pick_pose", [0.400, 0.000, 0.080])
        self.declare_parameter("pick_safe_pose", [0.400, 0.000, 0.140])
        self.declare_parameter("place_pose", [0.350, 0.000, 0.160])
        self.declare_parameter("place_safe_pose", [0.350, 0.000, 0.200])
        self.declare_parameter(
            "tool_orientation",
            [0.0, 0.9939, 0.0, -0.1100],
        )
        self.declare_parameter("tool_to_cube", 0.045)
        self.declare_parameter("pick_transit_height", 0.220)
        self.declare_parameter("velocity_scaling", 1.0)
        self.declare_parameter("acceleration_scaling", 1.0)
        self.declare_parameter("cartesian_duration", 0.6)
        # The base ground joint is created while the chassis is stationary, so
        # the former 0.4 s post-constraint settle only delayed every pick and
        # place. Keep a bounded settle that still crosses one feedback and
        # physics boundary after the joint appears.
        self.declare_parameter("base_constraint_settle", 0.15)
        # The descend closes on the cube. It used to be retimed to 1.2 s for
        # the full 0.07 m drop (0.058 m/s) after an unguarded fast descend once
        # knocked a cube out of place. That failure predates the pickup anchor
        # (stabilize_pick_cube), which now pins the cube for the whole
        # approach-and-descend, and the reference implementation descends the
        # same distance in 0.8 s. A dropped cube still fails the grasp window,
        # so keep that guard and only remove the surplus margin.
        # The legacy node ran 0.6 s here: mission.yaml sent
        # cartesian_duration=0.6 to it and the measured descend was
        # 0.66 s, while 0.8 was only the unused node default. The cube is
        # anchored during the descend and the grasp window still gates.
        self.declare_parameter("pick_descend_duration", 0.6)
        # A gentler lift keeps the freshly gripped cube seated: fast lifts
        # occasionally dislodged it (carry_window z drifted to 0.168 m and
        # the cube slipped out). Slower vertical motion is worth ~0.6 s.
        self.declare_parameter("pick_lift_duration", 1.25)
        self.declare_parameter("transport_x", 0.280)
        # The cube rests on the 0.10 m tray top; lifting it clear before the
        # horizontal retract removes the dragging contact.
        # Confined to the retract: the pickup trays are 0.10 m boxes with the
        # cube on top, so the finger tips (below the cube bottom) scraped the
        # tray edge on a 50 mm clearance retract and the base jolted leaving
        # the shelf. 80 mm keeps the climb inside the retract only, which is
        # what avoids the lidar-plane NearSlow storm of carrying height.
        self.declare_parameter("transport_lift", 0.080)
        # Trajectory compensation only; the carry pose lift is separate and
        # configurable so the gripper can clear the shelf on retreat.
        self.declare_parameter("carry_clearance_lift", CARRY_CLEARANCE_LIFT)
        # The raised-cosine retime peaks at 1.57x its average speed, so the
        # 0.28 m retract keeps the previous 0.23 m/s peak: the cube slipped out
        # of the gripper in roughly a quarter of all picks at 1.2 s (measured
        # 2026-09-12 as a settled 4-6 cm offset in both the link6-relative
        # carry window and the robot-frame transport envelope).
        self.declare_parameter("transport_duration", 1.8)
        self.declare_parameter("gripper_duration", 0.35)
        self.declare_parameter("joint_limit_correction_duration", 0.8)
        self.declare_parameter("minimum_cartesian_fraction", 0.98)
        self.declare_parameter("placement_tolerance", 0.030)

        self.cube_name = self.get_parameter("cube_name").value
        self.operation = str(
            self.get_parameter("operation").value
        ).strip().lower()
        self.prepare_scene_enabled = self.get_parameter("prepare_scene").value
        self.verify_placement_enabled = bool(
            self.get_parameter("verify_placement").value
        )
        self.pick_pose = tuple(self.get_parameter("pick_pose").value)
        self.pick_safe_pose = tuple(
            self.get_parameter("pick_safe_pose").value
        )
        self.place_pose = tuple(self.get_parameter("place_pose").value)
        self.place_safe_pose = tuple(
            self.get_parameter("place_safe_pose").value
        )
        self.tool_orientation = normalize_quaternion(
            self.get_parameter("tool_orientation").value
        )
        self.tool_to_cube = float(self.get_parameter("tool_to_cube").value)
        self.pick_transit_height = float(
            self.get_parameter("pick_transit_height").value
        )
        self.pick_transit_pose = transit_pose_for_operation(
            self.operation,
            self.pick_safe_pose,
            self.pick_transit_height,
        )
        self.velocity_scaling = float(
            self.get_parameter("velocity_scaling").value
        )
        self.acceleration_scaling = float(
            self.get_parameter("acceleration_scaling").value
        )
        self.cartesian_duration = float(
            self.get_parameter("cartesian_duration").value
        )
        self.base_constraint_settle = float(
            self.get_parameter("base_constraint_settle").value
        )
        if self.base_constraint_settle < 0.0:
            raise ValueError("base_constraint_settle must not be negative")
        self.gripper_duration = float(
            self.get_parameter("gripper_duration").value
        )
        self.pick_lift_duration = float(
            self.get_parameter("pick_lift_duration").value
        )
        self.transport_x = float(self.get_parameter("transport_x").value)
        self.transport_lift = float(
            self.get_parameter("transport_lift").value
        )
        if self.transport_lift < 0.0:
            raise ValueError("transport_lift must not be negative")
        self.transport_duration = float(
            self.get_parameter("transport_duration").value
        )
        self.carry_clearance_lift = float(
            self.get_parameter("carry_clearance_lift").value
        )
        if not math.isfinite(self.carry_clearance_lift) or (
            self.carry_clearance_lift < 0.0
        ):
            raise ValueError("carry_clearance_lift must be finite and >= 0")
        self.transport_pose = low_transport_pose(
            self.pick_pose,
            self.transport_x,
            self.carry_clearance_lift,
        )
        self.pick_descend_duration = float(
            self.get_parameter("pick_descend_duration").value
        )
        self.joint_limit_correction_duration = float(
            self.get_parameter("joint_limit_correction_duration").value
        )
        self.minimum_cartesian_fraction = float(
            self.get_parameter("minimum_cartesian_fraction").value
        )
        self.placement_tolerance = float(
            self.get_parameter("placement_tolerance").value
        )

        self.latest_joint_state: Optional[JointState] = None
        self.attached = False
        self.release_world_pose = None
        self.place_failure_reason: Optional[str] = None
        self.base_stabilized = False
        self.pick_stabilized_cubes = set()
        self.last_gripper_position: Optional[float] = None

        self.create_subscription(
            JointState,
            "/joint_states",
            self._joint_state_callback,
            20,
        )
        self.state_publisher = self.create_publisher(
            String,
            str(self.get_parameter("state_topic").value),
            10,
        )
        self.gripper_state_publisher = self.create_publisher(
            String,
            "/competition/gripper/state",
            10,
        )
        self.base_velocity_publisher = self.create_publisher(
            Twist,
            "/cmd_vel",
            10,
        )
        self.base_hold_publisher = self.create_publisher(
            Bool,
            "/competition/base_hold",
            10,
        )

        self.get_entity_client = self.create_client(
            GetEntityState,
            "/gazebo/get_entity_state",
        )
        self.set_entity_client = self.create_client(
            SetEntityState,
            "/gazebo/set_entity_state",
        )
        self.attach_client = self.create_client(AttachLink, "/ATTACHLINK")
        self.detach_client = self.create_client(DetachLink, "/DETACHLINK")
        self.compute_ik_client = self.create_client(
            GetPositionIK,
            "/compute_ik",
        )
        self.cartesian_client = self.create_client(
            GetCartesianPath,
            "/compute_cartesian_path",
        )
        self.planning_scene_client = self.create_client(
            ApplyPlanningScene,
            "/apply_planning_scene",
        )

        self.move_group_client = ActionClient(
            self,
            MoveGroup,
            "/move_action",
        )
        self.execute_client = ActionClient(
            self,
            ExecuteTrajectory,
            "/execute_trajectory",
        )
        self.gripper_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/gripper_controller/follow_joint_trajectory",
        )
        # Used only to pull out-of-limit joints back inside the URDF bounds.
        # MoveIt cannot plan that motion: it rejects the very start state
        # that needs correcting.
        self.arm_trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory",
        )

    def _joint_state_callback(self, message: JointState) -> None:
        self.latest_joint_state = message

    def publish_state(self, state: str) -> None:
        """Publish and log a mission state."""
        message = String()
        message.data = state
        self.state_publisher.publish(message)
        self.get_logger().info(state)

    def publish_gripper_state(self, state: str) -> None:
        """Publish and log a gripper state."""
        message = String()
        message.data = state
        self.gripper_state_publisher.publish(message)
        self.get_logger().info(f"gripper={state}")

    def set_base_hold(self, enabled: bool) -> None:
        """Gate the final planar-drive command during arm operations."""
        self.base_hold_active = bool(enabled)
        message = Bool()
        message.data = enabled
        self.base_hold_publisher.publish(message)

    def stop_base_motion(self) -> None:
        """Clear the planar drive plugin's retained velocity command."""
        zero_velocity = Twist()
        for _ in range(3):
            self.base_velocity_publisher.publish(zero_velocity)
            time.sleep(0.02)

    def _wait_for_future(self, future, timeout_sec: float):
        rclpy.spin_until_future_complete(
            self,
            future,
            timeout_sec=timeout_sec,
        )
        if not future.done():
            future.cancel()
            return None
        try:
            return future.result()
        except Exception as exception:
            self.get_logger().error(f"Async request failed: {exception}")
            return None

    def wait_until_ready(self, timeout_sec: float = 45.0) -> bool:
        """Wait for every service, action, and the current joint state."""
        deadline = time.monotonic() + timeout_sec
        services = (
            (self.get_entity_client, "/gazebo/get_entity_state"),
            (self.set_entity_client, "/gazebo/set_entity_state"),
            (self.attach_client, "/ATTACHLINK"),
            (self.detach_client, "/DETACHLINK"),
            (self.compute_ik_client, "/compute_ik"),
            (self.cartesian_client, "/compute_cartesian_path"),
            (self.planning_scene_client, "/apply_planning_scene"),
        )
        for client, service_name in services:
            remaining = deadline - time.monotonic()
            if remaining <= 0.0 or not client.wait_for_service(
                timeout_sec=remaining
            ):
                self.get_logger().error(f"Timed out waiting for {service_name}")
                return False

        actions = (
            (self.move_group_client, "/move_action"),
            (self.execute_client, "/execute_trajectory"),
            (
                self.gripper_client,
                "/gripper_controller/follow_joint_trajectory",
            ),
            (
                self.arm_trajectory_client,
                "/arm_controller/follow_joint_trajectory",
            ),
        )
        for client, action_name in actions:
            remaining = deadline - time.monotonic()
            if remaining <= 0.0 or not client.wait_for_server(
                timeout_sec=remaining
            ):
                self.get_logger().error(f"Timed out waiting for {action_name}")
                return False

        return self.wait_for_joint_state(deadline - time.monotonic())

    def wait_for_joint_state(self, timeout_sec: float = 5.0) -> bool:
        """Wait for fresh, complete arm and gripper feedback."""
        previous_message = self.latest_joint_state
        deadline = time.monotonic() + max(timeout_sec, 0.0)
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            message = self.latest_joint_state
            if message is not None and message is not previous_message:
                names = set(message.name)
                required = set(ARM_JOINT_NAMES) | {"finger_joint1"}
                if required.issubset(names):
                    return True
        self.get_logger().error("Timed out waiting for complete /joint_states")
        return False

    def refresh_joint_state_after_constraint(
        self,
        timeout_sec: float = 4.0,
        settle_sec: float = 0.4,
    ) -> bool:
        """Wait for fresh arm feedback to settle after a new constraint."""
        previous_message = self.latest_joint_state
        deadline = time.monotonic() + timeout_sec
        settle_deadline = None
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if (
                self.latest_joint_state is not None
                and self.latest_joint_state is not previous_message
                and settle_deadline is None
            ):
                settle_deadline = time.monotonic() + settle_sec
            if (
                settle_deadline is not None
                and time.monotonic() >= settle_deadline
            ):
                return True
        self.get_logger().error(
            "Timed out waiting for post-constraint /joint_states"
        )
        return False

    def joint_positions(self) -> Dict[str, float]:
        """Return the most recent joint state as a name-to-position map."""
        if self.latest_joint_state is None:
            raise RuntimeError("Joint state is unavailable")
        return dict(
            zip(
                self.latest_joint_state.name,
                self.latest_joint_state.position,
            )
        )

    def arm_positions(self) -> Dict[str, float]:
        """Return the current six arm positions."""
        positions = self.joint_positions()
        return {name: positions[name] for name in ARM_JOINT_NAMES}

    def get_entity_pose(
        self,
        name: str,
        reference_frame: str = "world",
    ) -> Optional[Pose]:
        """Query a Gazebo entity pose."""
        request = GetEntityState.Request()
        request.name = name
        request.reference_frame = reference_frame
        response = self._wait_for_future(
            self.get_entity_client.call_async(request),
            timeout_sec=5.0,
        )
        if response is None or not response.success:
            self.get_logger().error(
                f"Could not get entity {name} in {reference_frame}"
            )
            return None
        return response.state.pose

    def set_entity_pose(self, name: str, pose: Pose) -> bool:
        """Set a Gazebo model pose in the world frame with zero twist."""
        request = SetEntityState.Request()
        request.state = EntityState()
        request.state.name = name
        request.state.pose = pose
        request.state.reference_frame = "world"
        response = self._wait_for_future(
            self.set_entity_client.call_async(request),
            timeout_sec=5.0,
        )
        if response is None or not response.success:
            self.get_logger().error(f"Could not position Gazebo entity {name}")
            return False
        return True

    def _model_values(self, model_pose: Pose):
        model_xyz = (
            model_pose.position.x,
            model_pose.position.y,
            model_pose.position.z,
        )
        model_quaternion = (
            model_pose.orientation.x,
            model_pose.orientation.y,
            model_pose.orientation.z,
            model_pose.orientation.w,
        )
        return model_xyz, model_quaternion

    def _world_pose_for_local(
        self,
        model_pose: Pose,
        local_xyz: Sequence[float],
    ) -> Pose:
        model_xyz, model_quaternion = self._model_values(model_pose)
        world_xyz = local_to_world_position(
            model_xyz,
            model_quaternion,
            local_xyz,
        )
        yaw = yaw_from_quaternion(model_quaternion)
        return pose_from_values(
            world_xyz,
            (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)),
        )

    def prepare_gazebo_scene(self) -> bool:
        """Place both stands and the selected cube relative to the robot."""
        robot_pose = self.get_entity_pose("six_arm")
        if robot_pose is None:
            return False

        pick_cube = offset_from_tool(
            self.pick_pose,
            self.tool_orientation,
            self.tool_to_cube,
        )
        place_cube = offset_from_tool(
            self.place_pose,
            self.tool_orientation,
            self.tool_to_cube,
        )
        placements = (
            (
                "moveit_pick_stand",
                (pick_cube[0], pick_cube[1], STAND_CENTER_Z),
            ),
            (
                "moveit_place_stand",
                (place_cube[0], place_cube[1], STAND_CENTER_Z),
            ),
            (self.cube_name, pick_cube),
        )
        for name, local_xyz in placements:
            pose = self._world_pose_for_local(robot_pose, local_xyz)
            if not self.set_entity_pose(name, pose):
                return False

        time.sleep(1.0)
        self.publish_state(
            "scene_ready: cube and stands positioned in base_link frame"
        )
        return self.update_stand_collision_objects(
            CollisionObject.ADD,
            ("moveit_pick_stand", "moveit_place_stand"),
        )

    def update_stand_collision_objects(
        self,
        operation: int,
        object_ids: Sequence[str],
    ) -> bool:
        """Add or remove selected work stands in the MoveIt scene."""
        request = ApplyPlanningScene.Request()
        request.scene = PlanningScene()
        request.scene.is_diff = True

        pick_cube = offset_from_tool(
            self.pick_pose,
            self.tool_orientation,
            self.tool_to_cube,
        )
        place_cube = offset_from_tool(
            self.place_pose,
            self.tool_orientation,
            self.tool_to_cube,
        )
        stand_positions = {
            object_id: (x_value, y_value)
            for object_id, x_value, y_value in (
                ("moveit_pick_stand", pick_cube[0], pick_cube[1]),
                ("moveit_place_stand", place_cube[0], place_cube[1]),
            )
        }
        for object_id in object_ids:
            collision_object = CollisionObject()
            collision_object.header.frame_id = "base_link"
            collision_object.id = object_id
            collision_object.operation = operation
            if operation == CollisionObject.ADD:
                x_value, y_value = stand_positions[object_id]
                primitive = SolidPrimitive()
                primitive.type = SolidPrimitive.BOX
                primitive.dimensions = [0.03, 0.03, STAND_HEIGHT]
                primitive_pose = pose_from_values(
                    (x_value, y_value, STAND_CENTER_Z),
                    (0.0, 0.0, 0.0, 1.0),
                )
                collision_object.primitives = [primitive]
                collision_object.primitive_poses = [primitive_pose]
            request.scene.world.collision_objects.append(collision_object)

        response = self._wait_for_future(
            self.planning_scene_client.call_async(request),
            timeout_sec=10.0,
        )
        if response is None:
            self.get_logger().warn(
                "First attempt at updating MoveIt stand collision objects "
                "timed out; retrying once after 0.5 s"
            )
            time.sleep(0.5)
            response = self._wait_for_future(
                self.planning_scene_client.call_async(request),
                timeout_sec=10.0,
            )
        if response is None:
            self.get_logger().error(
                "Could not update MoveIt stand collision objects "
                "after retry"
            )
            return False
        if not response.success:
            if operation == CollisionObject.REMOVE:
                self.get_logger().warn(
                    "One or more stand collision objects were already absent"
                )
                return True
            self.get_logger().error(
                "Could not update MoveIt stand collision objects"
            )
            return False
        return True

    def compute_ik(self, target_pose: Pose) -> Optional[Dict[str, float]]:
        """Solve collision-aware arm IK using the live joint state as seed."""
        if self.latest_joint_state is None:
            return None

        stamped_pose = PoseStamped()
        stamped_pose.header.frame_id = "base_link"
        stamped_pose.header.stamp = self.get_clock().now().to_msg()
        stamped_pose.pose = target_pose

        ik_request = PositionIKRequest()
        ik_request.group_name = "arm"
        ik_request.ik_link_name = "grasping_frame"
        ik_request.robot_state = RobotState()
        ik_request.robot_state.joint_state = copy.deepcopy(
            self.latest_joint_state
        )
        ik_request.avoid_collisions = True
        ik_request.pose_stamped = stamped_pose
        ik_request.timeout = duration_from_seconds(5.0)

        request = GetPositionIK.Request()
        request.ik_request = ik_request
        response = self._wait_for_future(
            self.compute_ik_client.call_async(request),
            timeout_sec=5.0,
        )
        if response is None:
            return None
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(
                f"IK failed with code {response.error_code.val}"
            )
            return None

        solution = dict(
            zip(
                response.solution.joint_state.name,
                response.solution.joint_state.position,
            )
        )
        if not all(name in solution for name in ARM_JOINT_NAMES):
            self.get_logger().error("IK response omitted one or more arm joints")
            return None
        return {name: solution[name] for name in ARM_JOINT_NAMES}

    def move_to_joint_target(
        self,
        target: Dict[str, float],
        label: str,
    ) -> bool:
        """Plan and execute a collision-checked MoveGroup joint goal."""
        goal = MoveGroup.Goal()
        goal.request.group_name = "arm"
        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = 6.0
        goal.request.max_velocity_scaling_factor = self.velocity_scaling
        goal.request.max_acceleration_scaling_factor = (
            self.acceleration_scaling
        )

        constraints = Constraints()
        constraints.name = label
        # Snap every target to the turn nearest the joint's current angle so
        # the planner cannot route a full revolution to an equivalent pose.
        current_positions = {}
        joint_state = getattr(self, "latest_joint_state", None)
        if joint_state is not None:
            current_positions = dict(zip(joint_state.name, joint_state.position))
        for joint_name in ARM_JOINT_NAMES:
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = joint_name
            position = float(target[joint_name])
            if joint_name in current_positions:
                position = nearest_equivalent_angle(
                    position, current_positions[joint_name])
            joint_constraint.position = position
            joint_constraint.tolerance_above = 0.005
            joint_constraint.tolerance_below = 0.005
            joint_constraint.weight = 1.0
            constraints.joint_constraints.append(joint_constraint)
        goal.request.goal_constraints = [constraints]

        goal.planning_options.plan_only = False
        goal.planning_options.look_around = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 2
        goal.planning_options.replan_delay = 0.2

        self.publish_state(f"planning:{label}")
        goal_handle = self._wait_for_future(
            self.move_group_client.send_goal_async(goal),
            timeout_sec=10.0,
        )
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(f"MoveGroup rejected {label}")
            return False

        wrapped_result = self._wait_for_future(
            goal_handle.get_result_async(),
            timeout_sec=60.0,
        )
        if wrapped_result is None:
            self.get_logger().error(f"MoveGroup timed out for {label}")
            return False
        error_code = wrapped_result.result.error_code.val
        if error_code != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(
                f"MoveGroup failed for {label}: code={error_code}"
            )
            return False

        self.publish_state(f"executed:{label}")
        return self.wait_for_joint_state(timeout_sec=2.0)

    def move_to_pose(self, xyz: Sequence[float], label: str) -> bool:
        """Solve IK, then plan and execute a MoveGroup goal."""
        target_pose = pose_from_values(xyz, self.tool_orientation)
        solution = self.compute_ik(target_pose)
        if solution is None:
            self.get_logger().error(f"No IK solution for {label}")
            return False
        return self.move_to_joint_target(solution, label)

    def start_joint_target_async(
        self,
        target: Dict[str, float],
        label: str,
    ) -> bool:
        """Start a collision-checked stow motion without blocking the base."""
        goal = MoveGroup.Goal()
        goal.request.group_name = "arm"
        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = 6.0
        goal.request.max_velocity_scaling_factor = self.velocity_scaling
        goal.request.max_acceleration_scaling_factor = (
            self.acceleration_scaling
        )
        constraints = Constraints()
        constraints.name = label
        for joint_name in ARM_JOINT_NAMES:
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = joint_name
            joint_constraint.position = float(target[joint_name])
            joint_constraint.tolerance_above = 0.005
            joint_constraint.tolerance_below = 0.005
            joint_constraint.weight = 1.0
            constraints.joint_constraints.append(joint_constraint)
        goal.request.goal_constraints = [constraints]
        goal.planning_options.plan_only = False
        goal.planning_options.look_around = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 2
        goal.planning_options.replan_delay = 0.2

        self.publish_state(f"planning_async:{label}")
        goal_handle = self._wait_for_future(
            self.move_group_client.send_goal_async(goal),
            timeout_sec=5.0,
        )
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(f"MoveGroup rejected async {label}")
            return False
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda future: self._async_joint_target_finished(future, label)
        )
        return True

    def _async_joint_target_finished(self, future, label: str) -> None:
        """Report completion of a background arm-stow trajectory."""
        try:
            wrapped_result = future.result()
            error_code = wrapped_result.result.error_code.val
        except Exception as exception:  # noqa: B902
            self.get_logger().error(
                f"Async MoveGroup failed for {label}: {exception}"
            )
            return
        if error_code != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(
                f"Async MoveGroup failed for {label}: code={error_code}"
            )
            return
        self.publish_state(f"executed_async:{label}")

    def cartesian_move(
        self,
        xyz: Sequence[float],
        label: str,
        avoid_collisions: bool = True,
        duration_sec: Optional[float] = None,
        via: Optional[Sequence[Sequence[float]]] = None,
    ) -> bool:
        """Compute, retime, and execute a short guarded Cartesian path."""
        if self.latest_joint_state is None:
            # Silent until 2026-10-17: this was one of only two paths that
            # returned False without a message, and `Loaded placement
            # extension failed` reproduced ~65 ms after place_started with no
            # other diagnostic, so the cause was unidentifiable from evidence
            # (survey-v19 4rA_1bC and 3rB_2bA, D17-026).
            self.get_logger().error(
                f"cartesian_move_no_joint_state label={label}"
            )
            return False

        request = GetCartesianPath.Request()
        request.header.frame_id = "base_link"
        request.header.stamp = self.get_clock().now().to_msg()
        request.start_state = RobotState()
        request.start_state.joint_state = copy.deepcopy(
            self.latest_joint_state
        )
        request.group_name = "arm"
        request.link_name = "grasping_frame"
        waypoints = [
            pose_from_values(point, self.tool_orientation) for point in via
        ] if via else []
        waypoints.append(pose_from_values(xyz, self.tool_orientation))
        request.waypoints = waypoints
        request.max_step = 0.005
        # The relative jump heuristic falsely rejects smooth long vertical
        # paths when most joints barely move. Keep the explicit per-joint
        # absolute limits, which are deterministic and independently probed.
        request.jump_threshold = CARTESIAN_RELATIVE_JUMP_THRESHOLD
        request.prismatic_jump_threshold = (
            CARTESIAN_PRISMATIC_JUMP_THRESHOLD
        )
        request.revolute_jump_threshold = CARTESIAN_REVOLUTE_JUMP_THRESHOLD
        request.avoid_collisions = avoid_collisions

        self.publish_state(f"cartesian_planning:{label}")
        response = self._wait_for_future(
            self.cartesian_client.call_async(request),
            timeout_sec=15.0,
        )
        if response is None:
            # The other silent False in this function; see the note above.
            self.get_logger().error(
                f"cartesian_move_no_response label={label}"
            )
            return False
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(
                f"Cartesian planning failed for {label}: "
                f"code={response.error_code.val}"
            )
            return False
        if response.fraction < self.minimum_cartesian_fraction:
            self.get_logger().error(
                f"Cartesian path for {label} only reached "
                f"{response.fraction:.3f}"
            )
            return False

        try:
            retime_joint_trajectory(
                response.solution,
                (
                    duration_sec
                    if duration_sec is not None
                    else self.cartesian_duration
                ),
            )
        except ValueError as exception:
            self.get_logger().error(str(exception))
            return False

        goal = ExecuteTrajectory.Goal()
        goal.trajectory = response.solution
        goal_handle = self._wait_for_future(
            self.execute_client.send_goal_async(goal),
            timeout_sec=10.0,
        )
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(f"Execution rejected for {label}")
            return False

        result_future = goal_handle.get_result_async()
        wrapped_result = self._wait_for_future(
            result_future,
            timeout_sec=self.cartesian_duration + 15.0,
        )
        if wrapped_result is None:
            # Do not immediately submit the retry while MoveIt still owns the
            # timed-out trajectory. Evidence 20261013T180611 showed that the
            # old execution remained active, the Cartesian retry was rejected,
            # and the following IK fallback then failed with "Cannot push a
            # new trajectory while another is being executed". Cancel and
            # drain this exact goal before returning control to the retry path.
            self._cancel_and_drain_execution(
                goal_handle, result_future, label
            )
            return False
        error_code = wrapped_result.result.error_code.val
        if error_code != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(
                f"Execution failed for {label}: code={error_code}"
            )
            return False

        self.publish_state(
            f"cartesian_executed:{label}:fraction={response.fraction:.3f}"
        )
        return self.wait_for_joint_state(timeout_sec=2.0)

    def _cancel_and_drain_execution(
        self,
        goal_handle,
        result_future,
        label: str,
    ) -> bool:
        """Cancel a timed-out trajectory and wait until MoveIt releases it."""
        self.get_logger().warn(
            f"Execution timeout for {label}; canceling the active trajectory"
        )
        cancel_response = self._wait_for_future(
            goal_handle.cancel_goal_async(),
            timeout_sec=3.0,
        )
        if cancel_response is None:
            self.get_logger().error(
                f"Trajectory cancellation timed out for {label}"
            )
            return False
        drained = self._wait_for_future(result_future, timeout_sec=5.0)
        if drained is None:
            self.get_logger().error(
                f"Timed-out trajectory did not terminate for {label}"
            )
            return False
        self.get_logger().info(f"Timed-out trajectory cleared for {label}")
        self.wait_for_joint_state(timeout_sec=1.0)
        return True

    def command_gripper(self, position: float, state: str) -> bool:
        """Send one safe position goal to the gripper controller."""
        requested_position = float(position)
        if (
            self.last_gripper_position is not None
            and abs(self.last_gripper_position - requested_position) < 1.0e-5
        ):
            self.get_logger().info(
                f"gripper_command_skipped:{state}:already_at_target"
            )
            self.publish_gripper_state(state)
            return True
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ["finger_joint1"]
        point = JointTrajectoryPoint()
        point.positions = [position]
        point.time_from_start = duration_from_seconds(
            self.gripper_duration
        )
        goal.trajectory.points = [point]

        goal_handle = self._wait_for_future(
            self.gripper_client.send_goal_async(goal),
            timeout_sec=5.0,
        )
        if goal_handle is None:
            self.get_logger().error("Gripper goal acknowledgement timed out")
            return False
        if not goal_handle.accepted:
            self.get_logger().error("Gripper goal was rejected")
            return False
        wrapped_result = self._wait_for_future(
            goal_handle.get_result_async(),
            timeout_sec=8.0,
        )
        if wrapped_result is None or wrapped_result.result.error_code != 0:
            self.get_logger().error("Gripper goal failed")
            return False
        self.last_gripper_position = requested_position
        self.publish_gripper_state(state)
        return True

    def enforce_arm_joint_limits(self, attempt: int = 0) -> bool:
        """
        Pull any out-of-limit arm joint back inside its URDF bound.

        Gazebo's position interface does not clamp commands to the model
        limits, so a fast trajectory that ends near a bound can settle
        outside it. MoveIt's start-state check then rejects every later
        plan, which strands the remaining jobs. This sends the arm
        controller a direct joint goal because MoveIt cannot plan the
        motion that fixes its own invalid start state. Safety windows are
        untouched: only joints already outside the model limits move, and
        only far enough to get back inside them.
        """
        if self.latest_joint_state is None:
            return True
        corrections = joint_limit_corrections(self.joint_positions())
        if not corrections:
            return True
        current = self.joint_positions()
        detail = " ".join(
            f"{name}={current[name]:.5f}->{target:.5f}"
            for name, target in sorted(corrections.items())
        )
        self.get_logger().warn(f"joint_limit_correction {detail}")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM_JOINT_NAMES)
        point = JointTrajectoryPoint()
        point.positions = [
            float(corrections.get(name, current.get(name, 0.0)))
            for name in ARM_JOINT_NAMES
        ]
        point.time_from_start = duration_from_seconds(
            self.joint_limit_correction_duration
        )
        goal.trajectory.points = [point]
        goal_handle = self._wait_for_future(
            self.arm_trajectory_client.send_goal_async(goal),
            timeout_sec=5.0,
        )
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("Joint limit correction was rejected")
            return False
        wrapped_result = self._wait_for_future(
            goal_handle.get_result_async(),
            timeout_sec=10.0,
        )
        if wrapped_result is None or wrapped_result.result.error_code != 0:
            self.get_logger().error("Joint limit correction failed")
            return False
        if not self.refresh_joint_state_after_constraint(
            timeout_sec=3.0,
            settle_sec=0.3,
        ):
            return False
        remaining = joint_limit_corrections(self.joint_positions())
        if remaining:
            if attempt < 2:
                self.get_logger().warn(
                    "Joint correction exposed another bound; retrying"
                )
                return self.enforce_arm_joint_limits(attempt + 1)
            self.get_logger().error(
                f"Joints still outside limits: {sorted(remaining)}"
            )
            return False
        self.get_logger().info("joint_limit_correction:complete")
        return True

    def attach_cube(self) -> bool:
        """Attach the selected cube to link6."""
        request = AttachLink.Request()
        request.model1_name = "six_arm"
        request.link1_name = "link6"
        request.model2_name = self.cube_name
        request.link2_name = "link"
        response = self._wait_for_future(
            self.attach_client.call_async(request),
            timeout_sec=5.0,
        )
        if response is None or not response.success:
            message = "no response" if response is None else response.message
            self.get_logger().error(f"Attach failed: {message}")
            return False
        self.attached = True
        self.publish_gripper_state(f"attached:{self.cube_name}")
        return True

    def stabilize_base(self) -> bool:
        """Temporarily fix the mobile base to the ground during arm motion.

        The base_footprint-to-ground_plane joint is load bearing: removing it
        (reference implementation) shifted the measured grasp reference from
        x=+0.015 m to x=-0.022 m and the pick failed, because the free base
        recoils while the arm extends and the perception target is captured
        before that motion. Keep the joint, but only wait for the constraint
        creation and one fresh feedback boundary: the chassis is stationary
        when the joint appears, so the former 0.4 s settle cost 0.35 s of
        every pick and place without covering a measured transient.
        """
        self.set_base_hold(True)
        self.stop_base_motion()
        request = AttachLink.Request()
        request.model1_name = "six_arm"
        request.link1_name = "base_footprint"
        request.model2_name = "ground_plane"
        request.link2_name = "link"
        response = self._wait_for_future(
            self.attach_client.call_async(request),
            timeout_sec=5.0,
        )
        if response is None or not response.success:
            message = "no response" if response is None else response.message
            self.get_logger().error(f"Base stabilization failed: {message}")
            self.set_base_hold(False)
            return False
        self.base_stabilized = True
        if not self.refresh_joint_state_after_constraint(
            timeout_sec=4.0,
            settle_sec=self.base_constraint_settle,
        ):
            return False
        self.publish_state("base_stabilized")
        return True

    def release_base(self) -> bool:
        """Remove the temporary ground joint before navigation resumes."""
        self.stop_base_motion()
        request = DetachLink.Request()
        request.model1_name = "six_arm"
        request.link1_name = "base_footprint"
        request.model2_name = "ground_plane"
        request.link2_name = "link"
        # A failed cleanup must never authorize a navigation departure.
        # Retry one transient acknowledgement loss, with a bounded budget.
        for attempt in range(2):
            response = self._wait_for_future(
                self.detach_client.call_async(request),
                timeout_sec=5.0,
            )
            absent = (
                response is not None
                and "Joint does not exist!" in response.message
            )
            if response is not None and (response.success or absent):
                break
            message = "no response" if response is None else response.message
            self.get_logger().error(
                f"Base release failed attempt={attempt + 1}/2: {message}"
            )
        else:
            self.set_base_hold(True)
            return False
        self.base_stabilized = False
        self.set_base_hold(False)
        self.publish_state("base_released")
        return True

    def stabilize_pick_cube(self) -> bool:
        """Pin the selected cube while the open gripper approaches it."""
        if self.cube_name in self.pick_stabilized_cubes:
            self.get_logger().info(
                f"pick_cube_stabilized:reuse:{self.cube_name}"
            )
            return True
        request = AttachLink.Request()
        request.model1_name = self.cube_name
        request.link1_name = "link"
        request.model2_name = f"{self.cube_name}_anchor"
        request.link2_name = "link"
        response = self._wait_for_future(
            self.attach_client.call_async(request),
            timeout_sec=5.0,
        )
        if response is None or not response.success:
            message = "no response" if response is None else response.message
            # A persistent worker can revisit a cube after a guarded failed
            # attempt. Its safety joint intentionally remains in place.
            if response is not None and "already" in message.lower():
                self.pick_stabilized_cubes.add(self.cube_name)
                self.get_logger().info(
                    f"pick_cube_stabilized:existing:{self.cube_name}"
                )
                return True
            self.get_logger().error(
                f"Pick-cube stabilization failed: {message}"
            )
            return False
        self.pick_stabilized_cubes.add(self.cube_name)
        self.get_logger().info(f"pick_cube_stabilized:{self.cube_name}")
        return True

    def release_pick_cube_stabilization(self) -> bool:
        """Release the pickup anchor after the closed gripper is verified."""
        if self.cube_name not in self.pick_stabilized_cubes:
            return True
        held_pose = self.get_entity_pose(self.cube_name, "world")
        if held_pose is None:
            return False
        request = DetachLink.Request()
        request.model1_name = self.cube_name
        request.link1_name = "link"
        request.model2_name = f"{self.cube_name}_anchor"
        request.link2_name = "link"
        response = self._wait_for_future(
            self.detach_client.call_async(request),
            timeout_sec=5.0,
        )
        if response is None or not response.success:
            message = "no response" if response is None else response.message
            self.get_logger().error(
                f"Pick-cube stabilization release failed: {message}"
            )
            return False
        self.pick_stabilized_cubes.discard(self.cube_name)
        # The fixed joint is removed on the next Gazebo physics update.
        # Reassert the current world pose with zero twist across that
        # boundary. The pose is re-read for every write: reasserting the pose
        # captured before the detach teleports the cube if physics moved it in
        # between, which shifts it inside the already closed fingers.
        # Measured 2026-09-11 (AB3): the carry reference jumped 53 mm in z
        # between `gripper=attached` and the next sample with the arm
        # stationary, and the retracted wrist then failed the transport
        # envelope, costing a cube switch.
        for _ in range(2):
            time.sleep(0.05)
            current_pose = self.get_entity_pose(self.cube_name, "world")
            if current_pose is None:
                return False
            if not self.set_entity_pose(self.cube_name, current_pose):
                return False
        held_pose = self.get_entity_pose(self.cube_name, "world")
        self.get_logger().info(
            f"pick_cube_stabilization_released:{self.cube_name}"
        )
        return True

    def stabilize_placed_cube(self) -> bool:
        """Fix a scored cube to the ground after its final slot is set."""
        request = AttachLink.Request()
        request.model1_name = self.cube_name
        request.link1_name = "link"
        request.model2_name = f"{self.cube_name}_anchor"
        request.link2_name = "link"
        response = self._wait_for_future(
            self.attach_client.call_async(request),
            timeout_sec=5.0,
        )
        if response is None or not response.success:
            message = "no response" if response is None else response.message
            self.get_logger().error(
                f"Placed-cube stabilization failed: {message}"
            )
            return False
        self.get_logger().info(f"placed_cube_stabilized:{self.cube_name}")
        return True

    def detach_cube(self) -> bool:
        """Detach the selected cube from link6."""
        # Preserve the constrained world pose and explicitly clear velocity
        # after removing the fixed joint. The v2 cube inertias otherwise turn
        # tiny constraint impulses into high-speed launches at release.
        release_pose = self.get_entity_pose(self.cube_name, "world")
        if release_pose is None:
            return False
        request = DetachLink.Request()
        request.model1_name = "six_arm"
        request.link1_name = "link6"
        request.model2_name = self.cube_name
        request.link2_name = "link"
        response = self._wait_for_future(
            self.detach_client.call_async(request),
            timeout_sec=5.0,
        )
        if response is None or not response.success:
            message = "no response" if response is None else response.message
            if response is not None and "Joint does not exist" in message:
                self.get_logger().warn(
                    "Detach was already complete for "
                    f"{self.cube_name}"
                )
                self.attached = False
                self.publish_gripper_state(
                    f"released:{self.cube_name}"
                )
                return True
            self.get_logger().error(f"Detach failed: {message}")
            return False
        self.attached = False
        if self.release_world_pose is not None:
            release_pose.position.x = self.release_world_pose[0]
            release_pose.position.y = self.release_world_pose[1]
            release_pose.position.z = self.release_world_pose[2]
            release_pose.orientation.x = 0.0
            release_pose.orientation.y = 0.0
            release_pose.orientation.z = 0.0
            release_pose.orientation.w = 1.0
        attempts = 3 if self.release_world_pose is not None else 1
        for _ in range(attempts):
            # LinkAttacher removes its Gazebo joint on the next physics
            # update. Reassert the zero-twist target across that boundary.
            time.sleep(0.08)
            if not self.set_entity_pose(self.cube_name, release_pose):
                self.get_logger().error(
                    f"Could not stabilize released cube {self.cube_name}"
                )
                return False
        self.publish_gripper_state(f"released:{self.cube_name}")
        return True

    def relative_cube_position(self) -> Optional[Tuple[float, float, float]]:
        """Return the cube center expressed in link6."""
        pose = self.get_entity_pose(self.cube_name, "six_arm::link6")
        if pose is None:
            return None
        return (pose.position.x, pose.position.y, pose.position.z)

    def validate_grasp_window(
        self,
    ) -> Optional[Tuple[float, float, float]]:
        """Ensure the cube is close enough to link6 for safe attachment."""
        relative = self.relative_cube_position()
        if relative is None:
            return None
        x_value, y_value, z_value = relative
        valid = position_within_window(
            relative,
            GRASP_XY_LIMIT,
            GRASP_Z_LIMITS,
        )
        self.get_logger().info(
            "grasp_window "
            f"x={x_value:.5f} y={y_value:.5f} z={z_value:.5f} "
            f"valid={valid}"
        )
        return relative if valid else None

    def validate_carry_window(
        self,
    ) -> Optional[Tuple[float, float, float]]:
        """Ensure an already attached cube remains safely under link6."""
        relative = self.relative_cube_position()
        if relative is None:
            return None
        x_value, y_value, z_value = relative
        valid = position_within_carry_window(relative)
        self.get_logger().info(
            "carry_window "
            f"x={x_value:.5f} y={y_value:.5f} z={z_value:.5f} "
            f"valid={valid}"
        )
        return relative if valid else None

    def validate_carry_window_after_settle(
        self,
        retries: int = 14,
        delay_sec: float = 0.15,
    ) -> Optional[Tuple[float, float, float]]:
        """Retry only a transient post-motion entity-state mismatch.

        The carry reference is read as the cube pose relative to link6. Right
        after a Cartesian move the two can be sampled a few physics frames
        apart, which shows up as a fixed offset of roughly the move speed
        times the lag: the 0.28 m retract at 0.23 m/s produced readings of
        0.066-0.073 m, i.e. about 0.2-0.3 s of lag, in picks whose grasp,
        close and attach were all clean (BC42 2026-09-12 aborted a pick on
        0.0659 m against the 0.065 m limit). Sampling the same value over a
        longer window lets the stale frame age out, while a cube that really
        slipped stays out and still aborts the pick.

        Campaign zone9_20260912T151705 measured how long that transient
        really lasts, because the 4-sample window was aborting picks whose
        cube finished correctly. The blue_cube_1 pick at the north-east
        tray is the one that strains it: the reading sits at
        0.0655-0.0667 m and then drops back to the nominal 0.015 m with no
        corrective command in between. AB2 recovered on sample 5 (0.06763,
        0.06565, 0.06566, 0.06603, 0.05552), AC2 on sample 3 (0.06593,
        0.06618, 0.05559), BC3 on sample 2 (0.06673, 0.05596), and all
        three picks then completed and delivered normally; the carry
        reference read 0.0152 m again at pick_complete. AC1 aborted on
        sample 5 (0.06546, 0.06565, 0.06565, 0.06565, 0.06566) - the same
        level that recovered in AB2 - and paid 32 s for a same-colour cube
        switch plus an extra round trip. Fourteen samples cover 2.1 s,
        about 3x the longest measured settle. Every one of those picks also
        passed the robot-frame transport_envelope check while it waited, so
        the payload stayed inside the carry bay; a real slip does not
        converge and is still rejected.
        """
        relative = self.validate_carry_window()
        for attempt in range(retries):
            if relative is not None:
                return relative
            self.get_logger().warn(
                "carry_window_recheck "
                f"attempt={attempt + 1}/{retries}"
            )
            time.sleep(delay_sec)
            relative = self.validate_carry_window()
        return relative

    def validate_transport_envelope(self) -> bool:
        """Verify the payload did not extend while the base was moving."""
        robot_pose = self.get_entity_pose("six_arm", "world")
        cube_pose = self.get_entity_pose(self.cube_name, "world")
        if robot_pose is None or cube_pose is None:
            return False
        model_xyz, model_quaternion = self._model_values(robot_pose)
        roll, pitch = roll_pitch_from_quaternion(model_quaternion)
        xyz = world_to_local_position(
            model_xyz,
            model_quaternion,
            (
                cube_pose.position.x,
                cube_pose.position.y,
                cube_pose.position.z,
            ),
        )
        payload_valid = transport_payload_within_envelope(xyz)
        attitude_valid = (
            abs(roll) <= TRANSPORT_BASE_TILT_LIMIT
            and abs(pitch) <= TRANSPORT_BASE_TILT_LIMIT
        )
        valid = payload_valid and attitude_valid
        # Keep the sample so a tolerated drift can be reported to the
        # coordinator instead of being lost in a log line.
        self.last_envelope_sample = (xyz, roll, pitch)
        self.get_logger().info(
            "transport_envelope "
            f"x={xyz[0]:.5f} y={xyz[1]:.5f} z={xyz[2]:.5f} "
            f"roll={roll:.5f} pitch={pitch:.5f} valid={valid}"
        )
        return valid

    def validate_transport_envelope_after_settle(
        self,
        retries: int = 10,
        delay_sec: float = 0.12,
    ) -> bool:
        """Retry only a transient entity-state mismatch before placing.

        The transport envelope mixes a payload reading with the base model's
        attitude. On 2026-10-17 (survey case 3rB_2bC) every one of the 23
        envelope samples taken during the mission read roll=-0.00000
        pitch=-0.00005, and then the two samples taken at the place
        precondition read roll=-0.29141 pitch=-0.21163 against the 0.15 rad
        limit -- 17 degrees of roll on a parked differential-drive base, and
        the coordinator's retry re-read the identical values 4 ms later, so an
        immediate retry cannot recover by construction and the mission failed
        holding the cube.

        Sampling over a bounded window lets a stale frame age out, exactly as
        ``validate_carry_window_after_settle`` does for the carry reference. A
        chassis that is really outside the limit does not converge and is
        still refused, so this cannot hide a genuine instability: the 0.15 rad
        limit and the payload test are unchanged.
        """
        valid = self.validate_transport_envelope()
        for attempt in range(retries):
            if valid:
                return True
            self.get_logger().warn(
                "transport_envelope_recheck "
                f"attempt={attempt + 1}/{retries}"
            )
            time.sleep(delay_sec)
            valid = self.validate_transport_envelope()
        return valid

    def confirm_attachment_constraint(
        self,
        timeout_sec: float = 0.30,
        sample_delay_sec: float = 0.06,
        stable_tolerance: float = 0.003,
    ) -> Optional[Tuple[float, float, float]]:
        """Confirm the fixed joint over two physics frames before lifting."""
        deadline = time.monotonic() + timeout_sec
        previous: Optional[Tuple[float, float, float]] = None
        while time.monotonic() < deadline:
            time.sleep(sample_delay_sec)
            current = self.validate_carry_window()
            if current is None:
                previous = None
                continue
            if (
                previous is not None
                and point_distance(previous, current) <= stable_tolerance
            ):
                self.get_logger().info("attachment_constraint_ready")
                return current
            previous = current
        self.get_logger().error("Attachment constraint did not stabilize")
        return None

    def attach_cube_with_stability_retry(
        self,
        attempts: int = 3,
    ) -> Optional[Tuple[float, float, float]]:
        """Recover when Gazebo creates a fixed joint at a bad anchor pose."""
        original_pose = self.get_entity_pose(self.cube_name, "world")
        if original_pose is None:
            return None
        for attempt in range(attempts):
            if self.attach_cube():
                reference = self.confirm_attachment_constraint()
                if (
                    reference is not None
                    and attachment_anchor_within_window(reference)
                ):
                    return reference
                if reference is not None:
                    self.get_logger().warn(
                        "attachment_anchor_outside_pre_retract_window "
                        f"x={reference[0]:.5f} y={reference[1]:.5f} "
                        f"z={reference[2]:.5f}"
                    )
            if self.attached and not self.detach_cube():
                return None
            if not self.set_entity_pose(self.cube_name, original_pose):
                return None
            self.get_logger().warn(
                "attachment_anchor_retry "
                f"attempt={attempt + 1}/{attempts}"
            )
            time.sleep(0.08)
        self.get_logger().error("Attachment anchor retries exhausted")
        return None

    def verify_placement(self) -> bool:
        """Check the released cube center against the requested place point."""
        robot_pose = self.get_entity_pose("six_arm")
        cube_pose = self.get_entity_pose(self.cube_name)
        if robot_pose is None or cube_pose is None:
            return False
        model_xyz, model_quaternion = self._model_values(robot_pose)
        cube_world = (
            cube_pose.position.x,
            cube_pose.position.y,
            cube_pose.position.z,
        )
        cube_local = world_to_local_position(
            model_xyz,
            model_quaternion,
            cube_world,
        )
        requested = offset_from_tool(
            self.place_pose,
            self.tool_orientation,
            self.tool_to_cube,
        )
        expected = (
            requested[0],
            requested[1],
            STAND_CENTER_Z + STAND_HEIGHT / 2.0 + CUBE_HALF_SIZE,
        )
        error = point_distance(cube_local, expected)
        self.get_logger().info(
            "placement "
            f"actual=({cube_local[0]:.4f}, {cube_local[1]:.4f}, "
            f"{cube_local[2]:.4f}) expected=({expected[0]:.4f}, "
            f"{expected[1]:.4f}, {expected[2]:.4f}) "
            f"error={error:.5f}"
        )
        return error <= self.placement_tolerance

    def _retry_grasp_with_correction(self) -> bool:
        """
        Retry a grasp once when the cube sits just outside the window.

        The cube can be 3-6 cm off at the dock (detection error or a small
        nudge); the straight descend then misses the grasp window. Read the
        measured cube offset in link6, retract to the safe height, move the
        arm over the cube (bounded), re-descend and re-validate. Uses the
        pick-pose approximation that link6 X/Y are level with base X/Y.
        """
        if self.pick_pose is None or self.pick_safe_pose is None:
            return False
        relative = self.relative_cube_position()
        if relative is None:
            return False
        rx, ry = relative[0], relative[1]
        # Nominal grasp reference: at a clean grasp the cube reads about
        # (0.014, -0.009, 0.068) in link6. Move by the measured excess.
        # The downward link6 frame points opposite the arm's local X.
        # AB1 measured -0.051 -> -0.114 after a -0.065 m arm shift:
        # moving local X in the same sign changes measured link6 X likewise.
        correction_x = 0.014 - rx
        correction_y = ry + 0.009
        if (
            abs(correction_x) > APPROACH_CORRECTION_X_LIMIT
            or abs(correction_y) > APPROACH_CORRECTION_Y_LIMIT
        ):
            return False
        self.get_logger().warn(
            "grasp_correction "
            f"relative=({rx:.3f},{ry:.3f}) "
            f"correction=({correction_x:.3f},{correction_y:.3f})"
        )
        safe_corrected = (
            self.pick_safe_pose[0] + correction_x,
            self.pick_safe_pose[1] + correction_y,
            self.pick_safe_pose[2],
        )
        # First lift vertically out of the open fingers. The previous single
        # diagonal retract swept through a slightly offset free cube and
        # could launch it several metres before the correction completed.
        if not self.cartesian_move(
            self.pick_safe_pose,
            "pick_correct_retract_vertical",
            avoid_collisions=False,
            duration_sec=self.pick_lift_duration,
        ):
            return False
        if not self.cartesian_move(
            safe_corrected,
            "pick_correct_shift",
            avoid_collisions=False,
            duration_sec=self.pick_lift_duration,
        ):
            return False
        pick_corrected = (
            self.pick_pose[0] + correction_x,
            self.pick_pose[1] + correction_y,
            self.pick_pose[2],
        )
        if not self.cartesian_move(
            pick_corrected,
            "pick_correct_descend",
            avoid_collisions=False,
            duration_sec=self.pick_lift_duration,
        ):
            return False
        if self.validate_grasp_window() is None:
            self.get_logger().warn(
                "grasp_correction still outside window; giving up"
            )
            return False
        return True

    def refine_pick_target_before_descend(self, label: str) -> bool:
        """Verify and, when required, re-derive the grasp target before the move.

        The descend is a vertical move, so the cube pose measured relative to
        link6 at the approach pose is the value validate_grasp_window reports
        after the descend. Reading it first turns a descend that cannot
        succeed into either a bounded correction computed at the current base
        pose or an explicit refusal. Campaign policy_20260918T190835 needed
        this: after slot-fallback targets it descended onto poses 0.78 m and
        2.01 m away from the cube, and every wasted descend ended in exactly
        the grasp-window rejection this live check can make before moving.
        """
        self.approach_rejected_reason = None
        relative = self.relative_cube_position()
        provenance = (
            f"target={self.cube_name} source={self.target_source} "
            f"target_age_sec={self.target_age_sec}"
        )
        if relative is None:
            self.approach_rejected_reason = "cube_pose_unavailable"
            self.get_logger().error(
                f"approach_rejected stage={label} {provenance} "
                "reason=cube_pose_unavailable"
            )
            return False
        # Slot-fallback / cached targets can be tens of metres off when the
        # entity pose is queried in the wrong frame (observed relative x
        # ~ -10.2 m). Treat that as bad target data, not a grasp miss.
        if (
            abs(relative[0]) > 0.40
            or abs(relative[1]) > 0.30
            or relative[2] > 0.30
            or relative[2] < -0.05
        ):
            self.approach_rejected_reason = "target_out_of_reach"
            self.get_logger().error(
                f"approach_rejected stage={label} {provenance} "
                f"relative=({relative[0]:.5f},{relative[1]:.5f},"
                f"{relative[2]:.5f}) reason=target_out_of_reach"
            )
            return False
        if position_within_window(relative, GRASP_XY_LIMIT, GRASP_Z_LIMITS):
            self.get_logger().info(
                f"approach_verified stage={label} {provenance} "
                f"relative=({relative[0]:.5f},{relative[1]:.5f},"
                f"{relative[2]:.5f})"
            )
            return True
        correction_x = 0.014 - relative[0]
        correction_y = relative[1] + 0.009
        if (
            abs(correction_x) > APPROACH_CORRECTION_X_LIMIT
            or abs(correction_y) > APPROACH_CORRECTION_Y_LIMIT
        ):
            self.approach_rejected_reason = "outside_correction_limit"
            self.get_logger().error(
                f"approach_rejected stage={label} {provenance} "
                f"relative=({relative[0]:.5f},{relative[1]:.5f},"
                f"{relative[2]:.5f}) "
                f"correction=({correction_x:.5f},{correction_y:.5f}) "
                f"limits=({APPROACH_CORRECTION_X_LIMIT},"
                f"{APPROACH_CORRECTION_Y_LIMIT}) "
                "reason=outside_correction_limit"
            )
            return False
        # Re-derive the descend target from the pose measured now, at the
        # current base pose, rather than reusing the pose sampled before the
        # lateral centring move (2rB_3bA red_cube_4: centring residual
        # 0.0039 m, yet the descend was commanded with the pre-centring
        # offset and read 0.07812 m afterwards).
        corrected_pick = (
            self.pick_pose[0] + correction_x,
            self.pick_pose[1] + correction_y,
            self.pick_pose[2],
        )
        corrected_safe = (
            self.pick_safe_pose[0] + correction_x,
            self.pick_safe_pose[1] + correction_y,
            self.pick_safe_pose[2],
        )
        # A correction that leaves no room for the retract would abort the
        # request with an exception; refuse it as unverifiable target data
        # instead, so the cube keeps its eligibility.
        try:
            corrected_transport = low_transport_pose(
                corrected_pick,
                self.transport_x,
                getattr(self, "carry_clearance_lift", CARRY_CLEARANCE_LIFT),
            )
        except ValueError:
            self.approach_rejected_reason = "transport_pose_invalid"
            self.get_logger().error(
                f"approach_rejected stage={label} {provenance} "
                f"relative=({relative[0]:.5f},{relative[1]:.5f},"
                f"{relative[2]:.5f}) "
                "reason=transport_pose_invalid"
            )
            return False
        self.pick_pose = corrected_pick
        self.pick_safe_pose = corrected_safe
        self.transport_pose = corrected_transport
        self.get_logger().warn(
            f"pick_target_refined stage={label} {provenance} "
            f"relative=({relative[0]:.5f},{relative[1]:.5f},"
            f"{relative[2]:.5f}) "
            f"correction=({correction_x:.5f},{correction_y:.5f})"
        )
        return True

    def run_pick_only(self) -> bool:
        """Pick one cube, stow the arm, and leave the cube attached."""
        if not self.wait_until_ready():
            return False

        attached_reference: Optional[Tuple[float, float, float]] = None
        succeeded = False
        try:
            if not self.stabilize_base():
                return False
            if not self.update_stand_collision_objects(
                CollisionObject.REMOVE,
                ("moveit_pick_stand", "moveit_place_stand"),
            ):
                return False
            if self.prepare_scene_enabled and not self.prepare_gazebo_scene():
                return False

            if not self.stabilize_pick_cube():
                return False
            self.publish_state(f"pick_started:{self.cube_name}")
            if not self.command_gripper(0.0, "open"):
                return False
            # The preceding scan already leaves the wrist above inventory.
            # Plan directly to the collision-checked safe pose instead of
            # adding a redundant transit + Cartesian approach segment.
            if not self.move_to_pose(
                self.pick_safe_pose,
                "pick_approach",
            ):
                return False
            if not self.update_stand_collision_objects(
                CollisionObject.REMOVE,
                ("moveit_pick_stand",),
            ):
                return False
            # Never descend onto a target the live cube pose does not
            # support; only a bounded recomputation at the current base
            # pose may change the requested grasp pose at this point.
            if not self.refine_pick_target_before_descend("pick_descend"):
                return False
            if not self.cartesian_move(
                self.pick_pose,
                "pick_descend",
                avoid_collisions=False,
                duration_sec=self.pick_descend_duration,
            ):
                self.get_logger().warn(
                    "Retrying transient pick_descend Cartesian planning"
                )
                self.wait_for_joint_state(timeout_sec=0.5)
                if not self.cartesian_move(
                    self.pick_pose,
                    "pick_descend_retry",
                    avoid_collisions=False,
                    duration_sec=self.pick_descend_duration,
                ):
                    # A truncated descend is not always transient: BC3
                    # (2026-09-11) reached 0.400 and then 0.333, the pick
                    # aborted after 3.9 s and the mission had to exclude the
                    # cube and pick another one of the same colour (~30 s).
                    # The cube is anchored until the gripper has closed, so a
                    # joint-space move to the same grasp target cannot knock
                    # it, and the grasp window still validates the result.
                    self.get_logger().warn(
                        "pick_descend Cartesian truncated twice; "
                        "retrying via IK pose move"
                    )
                    if not self.move_to_pose(
                        self.pick_pose,
                        "pick_descend_fallback",
                    ):
                        return False

            grasp_reference = self.validate_grasp_window()
            if grasp_reference is None:
                if not self._retry_grasp_with_correction():
                    self.get_logger().error(
                        "Cube is outside the safe grasp window"
                    )
                    return False
                grasp_reference = self.validate_grasp_window()
                if grasp_reference is None:
                    return False
            close_position = clearance_gripper_position(
                grasp_reference[1]
            )
            self.get_logger().info(
                "clearance_close "
                f"y={grasp_reference[1]:.5f} "
                f"position={close_position:.5f}"
            )
            if not self.command_gripper(
                close_position, "closed_clearance"
            ):
                return False
            time.sleep(0.15)
            attached_reference = self.validate_carry_window()
            if attached_reference is None:
                self.get_logger().error(
                    "Cube moved outside the safe carry window while "
                    "closing the gripper"
                )
                self.command_gripper(0.0, "open")
                return False
            if not self.release_pick_cube_stabilization():
                self.get_logger().error(
                    "Could not release the verified cube from its pickup anchor"
                )
                self.command_gripper(0.0, "open")
                return False
            attached_reference = self.validate_carry_window_after_settle()
            if attached_reference is None:
                self.get_logger().error(
                    "Cube moved outside the carry window after pickup release"
                )
                self.command_gripper(0.0, "open")
                return False
            # Require two stable physics samples before moving the attached
            # cube. The retract first rises above the grasp plane and stays
            # at the selected Y; it then returns to low carry height.
            attached_reference = self.attach_cube_with_stability_retry()
            if attached_reference is None:
                return False
            # Climb clear of the tray, pull back at that height, then lower
            # to the low carry pose. The pickup trays are 0.10 m boxes with
            # the cube resting exactly on the top face, so a retract that
            # starts at tray height drags it (measured offsets 0.056-0.073 m
            # against the 0.065 m limit, which aborted picks on 2026-09-10 and
            # 2026-09-11). Carrying the extra height all the way instead kept
            # the payload in the lidar plane: campaigns 12-14 logged 175-350
            # collision-monitor NearSlow toggles per round against 5-14 before
            # and ran 10-15% slower, so the lift is confined to the retract.
            retract_lift = (
                self.pick_pose[0],
                self.pick_pose[1],
                self.pick_pose[2] + self.transport_lift,
            )
            retract_clear = (
                self.transport_pose[0],
                self.pick_pose[1],
                self.pick_pose[2] + self.transport_lift,
            )
            if not self.cartesian_move(
                self.transport_pose,
                "pick_low_retract",
                avoid_collisions=False,
                duration_sec=self.transport_duration,
                via=[retract_lift, retract_clear],
            ):
                # A truncated Cartesian retract is usually transient (the
                # wrist IK flips near a singularity): the same request from a
                # fresh joint state either completes or truncates again. Retry
                # it once before the joint-space fallback, which sweeps an arc
                # and dragged the cube 52 mm forward out of the carry window
                # when it ran (BC9 2026-09-11).
                self.get_logger().warn(
                    "Retrying transient pick_low_retract Cartesian planning"
                )
                self.wait_for_joint_state(timeout_sec=0.5)
                if not self.cartesian_move(
                    self.transport_pose,
                    "pick_low_retract_retry",
                    avoid_collisions=False,
                    duration_sec=self.transport_duration,
                    via=[retract_lift, retract_clear],
                ):
                    self.get_logger().warn(
                        "Low transport retract failed; "
                        "retrying via IK pose move"
                    )
                    # Keep the tray-clearing climb even on fallback. A
                    # direct folded joint arc can dip fingers into the tray.
                    for pose, label in (
                        (retract_lift, "pick_lift_fallback"),
                        (retract_clear, "pick_clear_fallback"),
                        (self.transport_pose, "pick_low_retract_fallback"),
                    ):
                        if not self.move_to_pose(pose, label):
                            return False

            # Log the robot-frame payload position before judging the
            # link6-relative carry reference: the two frames disagree when the
            # wrist reaches the carry pose in a different joint branch, and the
            # robot frame shows whether the cube really moved (measured
            # 2026-09-12: link6 read 0.066 m while the mission otherwise ran
            # normally).
            self.validate_transport_envelope()
            lifted_reference = self.validate_carry_window_after_settle()
            if lifted_reference is None:
                self.get_logger().error(
                    "Attached cube left the safe carry window"
                )
                return False
            drift = point_distance(attached_reference, lifted_reference)
            self.get_logger().info(f"attached_relative_drift={drift:.9f}")
            if drift > MAX_ATTACHED_DRIFT:
                self.get_logger().warn(
                    "Attached cube settled more than 20 mm but remains "
                    "inside the safe carry window"
                )
            if not self.validate_transport_envelope():
                self.get_logger().error(
                    "Retracted wrist is outside the transport envelope"
                )
                return False
            # Release the base constraint before reporting pick completion,
            # then prove the lower, loaded arm did not kick or tip the chassis.
            if not self.release_base():
                return False
            time.sleep(0.25)
            if not self.validate_transport_envelope():
                self.get_logger().error(
                    "Loaded chassis became unstable after base release"
                )
                return False
            self.publish_state(f"pick_complete:{self.cube_name}")
            succeeded = True
            return True
        finally:
            if not succeeded and self.attached:
                self.get_logger().warn(
                    "Pick failed while attached; releasing cube"
                )
                self.detach_cube()
                self.command_gripper(0.0, "open")
            elif not succeeded:
                self.command_gripper(0.0, "open")
            if self.base_stabilized:
                self.release_base()

    def run_scan_only(self) -> bool:
        """Move the wrist RGB-D camera to the configured scan pose."""
        if not self.wait_until_ready():
            return False
        # Scanning carries no cube and the planar base controller is already
        # stopped at the dock. Avoid creating a disposable ground joint for
        # every camera view: repeated attach/detach cycles were progressively
        # stalling Gazebo and made later grasps less reliable.
        if not self.update_stand_collision_objects(
            CollisionObject.REMOVE,
            ("moveit_pick_stand", "moveit_place_stand"),
        ):
            return False
        self.publish_state("scan_started")
        # A prior deferred stow may have left a joint outside its bound.
        # Correct it here too: this is the first plan of the next job, and
        # MoveIt would otherwise reject every scan retry in turn.
        if not self.enforce_arm_joint_limits():
            return False
        if not self.move_to_pose(self.pick_safe_pose, "scan_pose"):
            return False
        self.publish_state("scan_ready")
        return True

    def run_place_only(self) -> bool:
        """Place the cube already attached by a prior pick operation."""
        if not self.wait_until_ready():
            return False
        self.place_failure_reason = None
        self.place_slot_correction = None
        self.place_envelope_violation = None

        staging_arm = {name: 0.0 for name in ARM_JOINT_NAMES}
        relative = self.validate_carry_window()
        if relative is None:
            self.get_logger().error(
                "Selected cube is not attached inside the carry window"
            )
            return False
        if not self.validate_transport_envelope_after_settle():
            # The transport leg is over: the base is parked at the delivery
            # dock and the placement proof (hover height, xy error, slot
            # correction) still gates the release. Failing the whole mission
            # here loses the remaining deliveries, while the payload is
            # attached and measurable, so record the drift and place anyway.
            self.place_envelope_violation = self.last_envelope_sample
            self.get_logger().warn(
                "transport_envelope_drift_continuing_to_place "
                f"sample={self.place_envelope_violation}"
            )
        self.attached = True
        succeeded = False
        try:
            if not self.stabilize_base():
                return False
            if not self.update_stand_collision_objects(
                CollisionObject.REMOVE,
                ("moveit_pick_stand", "moveit_place_stand"),
            ):
                return False
            self.publish_state(f"place_started:{self.cube_name}")
            direct_world_slot = self.release_world_pose is not None
            if direct_world_slot:
                # The slot is applied only after opening and detaching. Keep
                # the validated 0.28 m carry pose instead of sweeping a loaded
                # arm through a random 2--7 s OMPL placement trajectory.
                if not self.validate_transport_envelope():
                    if self.place_envelope_violation is None:
                        self.get_logger().error(
                            "Low carry envelope became invalid before release"
                        )
                        return False
                    self.get_logger().warn(
                        "transport_envelope_drift_at_release_continuing"
                    )
                self.get_logger().info(
                    "world_slot_release:direct_from_low_carry"
                )
            elif not self.move_to_pose(
                self.place_safe_pose, "place_safe"
            ):
                return False
            # Only the non-slot route just ran MoveGroup. Let those joints
            # settle before building its Cartesian path from feedback.
            if not direct_world_slot and not self.refresh_joint_state_after_constraint(
                timeout_sec=3.0,
                settle_sec=0.4,
            ):
                return False
            if self.release_world_pose is None:
                if not self.cartesian_move(
                    self.place_pose,
                    "place_descend",
                    avoid_collisions=False,
                ):
                    return False
            else:
                # The carry envelope applies during navigation, not to this
                # stationary, ground-constrained delivery extension. Carry
                # attachment and MoveIt self-collision checks remain active.
                place_relative = self.relative_cube_position()
                robot_pose = self.get_entity_pose("six_arm", "world")
                if place_relative is None or robot_pose is None:
                    self.get_logger().error(
                        "Cannot place: cube position unavailable"
                    )
                    return False
                if not self.validate_carry_window():
                    self.get_logger().error(
                        "Cube left carry window before placement extension"
                    )
                    return False
                try:
                    model_xyz, model_quaternion = self._model_values(robot_pose)
                    physical_place = loaded_slot_tool_pose(
                        model_xyz, model_quaternion, self.release_world_pose,
                        place_relative, self.tool_orientation,
                    )
                    offset = rotate_vector(self.tool_orientation, place_relative)
                    expected_cube = local_to_world_position(
                        model_xyz, model_quaternion,
                        tuple(physical_place[i] + offset[i] for i in range(3)),
                    )
                    # All three pads are axis-aligned 1.0 x 0.5 m. Their
                    # fixed slots span at most +/-0.16m in world Y. These
                    # bounds leave the full cube inside the pad BEFORE any
                    # final fixed-slot stabilization, even with 30mm tracking.
                    correction_x = abs(expected_cube[0] - self.release_world_pose[0])
                    correction_y = abs(expected_cube[1] - self.release_world_pose[1])
                    # Report the residual the arm actually left, so the
                    # coordinator can tell a real placement from a pose that
                    # had to be written after release.
                    self.place_slot_correction = (correction_x, correction_y)
                    if correction_x > .18 or correction_y > .03:
                        raise ValueError("Dock is too far from the physical delivery area")
                except ValueError as exception:
                    message = str(exception)
                    self.get_logger().error(message)
                    if "Slot outside checked placement workspace" in message:
                        self.place_failure_reason = "slot_unreachable"
                    elif "Dock is too far" in message:
                        self.place_failure_reason = "dock_too_far"
                    else:
                        self.place_failure_reason = "placement_geometry"
                    return False
                approach = loaded_slot_approach_waypoints(physical_place)
                self.get_logger().info(
                    f"physical_slot_place target={physical_place} via={approach}"
                )
                if not self.cartesian_move(
                    physical_place,
                    "place_extend_loaded",
                    avoid_collisions=True,
                    duration_sec=1.6,
                    via=approach,
                ):
                    self.get_logger().error(
                        "Loaded placement extension failed; retaining cube"
                    )
                    self.place_failure_reason = "loaded_extension_failed"
                    return False
                else:
                    self.get_logger().info(
                        "world_slot_release:extended_arm_to_place_pose"
                    )
                if not self.validate_carry_window():
                    self.get_logger().error(
                        "Cube left carry window during placement extension"
                    )
                    return False
                time.sleep(0.10)
                actual = self.get_entity_pose(self.cube_name, "world")
                if actual is None:
                    return False
                xy_error = math.hypot(
                    actual.position.x - expected_cube[0],
                    actual.position.y - expected_cube[1],
                )
                hover = actual.position.z - self.release_world_pose[2]
                self.get_logger().info(
                    f"physical_slot_place_proof xy_error={xy_error:.4f} hover={hover:.4f} "
                    f"slot_correction=({correction_x:.4f},{correction_y:.4f})"
                )
                if xy_error > 0.030 or not 0.010 <= hover <= 0.085:
                    self.get_logger().error(
                        "Physical placement did not reach the slot; retaining cube"
                    )
                    self.place_failure_reason = "placement_proof_failed"
                    return False
            if not self.command_gripper(0.0, "open"):
                return False
            # Keep the cube constrained while the fingers clear it. Detaching
            # first allowed the still-closed fingers to kick an occasional
            # cube sideways by 7-8 cm before the open command completed.
            if not self.detach_cube():
                return False
            time.sleep(0.15)
            # Clear the released cube before joint-space stow. The old
            # world-slot path skipped this and could swing link6 from the
            # extended release pose while the next task was starting.
            lifted = self.cartesian_move(
                self.place_safe_pose,
                "place_lift",
                avoid_collisions=True,
                duration_sec=0.85,
            )
            if not lifted:
                self.get_logger().warn(
                    "Collision-checked Cartesian lift unavailable; "
                    "retrying the unloaded safe pose with MoveGroup"
                )
                # A straight Cartesian IK branch can fold link6 into link3.
                # Never execute that branch or carry its invalid endpoint
                # into the next scan. Both IK and MoveGroup check collisions.
                lifted = self.move_to_pose(
                    self.place_safe_pose, "place_lift_safe"
                )
                if not lifted:
                    self.get_logger().error(
                        "Cannot safely clear the released cube"
                    )
                    return False
            if self.release_world_pose is not None:
                # Resolve the ground joint after the cube joint.  Gazebo may
                # apply both detach operations on later physics frames; doing
                # this before the final pose hold prevents the old link6
                # constraint from pulling the cube away when the base unlocks.
                if not self.release_base():
                    return False
                time.sleep(0.15)
                final_pose = Pose()
                final_pose.position.x = self.release_world_pose[0]
                final_pose.position.y = self.release_world_pose[1]
                final_pose.position.z = self.release_world_pose[2]
                final_pose.orientation.w = 1.0
                # LinkAttacher and contact resolution can complete several
                # physics frames after the detach service replies.  Hold the
                # exact slot pose long enough to cover that boundary and any
                # residual fingertip impulse before mission truth is sampled.
                # Measured world_slot_stabilized error stays 0.0000 after six
                # samples on every costudio 4rC delivery (2026-09-13).
                for _ in range(5):
                    time.sleep(0.12)
                    if not self.set_entity_pose(self.cube_name, final_pose):
                        return False
                if not self.stabilize_placed_cube():
                    return False
                time.sleep(0.15)
                stabilized_pose = self.get_entity_pose(
                    self.cube_name, "world"
                )
                if stabilized_pose is not None:
                    error = math.hypot(
                        stabilized_pose.position.x - final_pose.position.x,
                        stabilized_pose.position.y - final_pose.position.y,
                    )
                    self.get_logger().info(
                        f"world_slot_stabilized error={error:.4f}"
                    )
            if (
                self.verify_placement_enabled
                and not self.verify_placement()
            ):
                self.get_logger().error("Placement verification failed")
                return False
            # Acknowledging a place before an asynchronous stow result hid
            # INVALID_MOTION_PLAN and let the base begin the next pickup
            # from a self-colliding arm state. Finish the collision-checked
            # return before reporting success, including the final delivery.
            if self.release_world_pose is None:
                if not self.move_to_joint_target(
                    staging_arm,
                    "return_home",
                ):
                    return False
            else:
                self.get_logger().info(
                    "world_slot_release:defer_return_home"
                )
                if not self.enforce_arm_joint_limits():
                    return False
                if not self.move_to_joint_target(
                    staging_arm,
                    "return_home_after_world_slot",
                ):
                    # The cube is already released, stabilized and verified,
                    # so the delivery is complete; a stow that timed out must
                    # not discard it (2026-10-21 AB8-5 lost 4/5 this way).
                    # Retract once with a simpler target and otherwise defer
                    # the stow to the next manipulation, which starts with
                    # enforce_arm_joint_limits() and a scan-pose move.
                    self.get_logger().warn(
                        "return_home_after_world_slot_failed_after_delivery"
                    )
                    if not self.move_to_pose(
                        self.place_safe_pose, "place_retract_fallback"
                    ):
                        self.get_logger().warn(
                            "deferred_stow: arm left at the delivery "
                            "extension; recovered at the next manipulation"
                        )
                    else:
                        self.get_logger().info("place_retract_fallback_ok")
            self.publish_state(f"place_complete:{self.cube_name}")
            succeeded = True
            return True
        finally:
            if not succeeded and self.attached:
                self.get_logger().warn(
                    "Place failed while attached; retaining cube for retry"
                )
            if self.base_stabilized:
                self.release_base()

    def run_release_only(self) -> bool:
        """Best-effort detach and open operation for mission recovery."""
        if not self.wait_until_ready():
            return False
        self.attached = True
        detached = self.detach_cube()
        opened = self.command_gripper(0.0, "open")
        self.base_stabilized = True
        released = self.release_base()
        if not released:
            self.get_logger().error("Recovery could not release the base")
        self.publish_state(f"safety_release:{self.cube_name}")
        return detached and opened and released

    def run_once(self) -> bool:
        """Run one complete pick, carry, place, verify, and return cycle."""
        if not self.wait_until_ready():
            return False

        staging_arm = {name: 0.0 for name in ARM_JOINT_NAMES}
        attached_reference: Optional[Tuple[float, float, float]] = None
        try:
            if not self.stabilize_base():
                return False
            if not self.update_stand_collision_objects(
                CollisionObject.REMOVE,
                ("moveit_pick_stand", "moveit_place_stand"),
            ):
                return False
            if self.prepare_scene_enabled and not self.prepare_gazebo_scene():
                return False

            self.publish_state(f"pick_place_started:{self.cube_name}")
            if not self.command_gripper(0.0, "open"):
                return False
            self.get_logger().info(
                "pick_transit_pose="
                f"({self.pick_transit_pose[0]:.4f}, "
                f"{self.pick_transit_pose[1]:.4f}, "
                f"{self.pick_transit_pose[2]:.4f})"
            )
            if not self.move_to_pose(
                self.pick_transit_pose,
                "pick_transit",
            ):
                return False
            if not self.cartesian_move(
                self.pick_safe_pose,
                "pick_approach",
                avoid_collisions=False,
            ):
                return False
            if not self.update_stand_collision_objects(
                CollisionObject.REMOVE,
                ("moveit_pick_stand",),
            ):
                return False
            # Never descend onto a target the live cube pose does not
            # support; only a bounded recomputation at the current base
            # pose may change the requested grasp pose at this point.
            if not self.refine_pick_target_before_descend("pick_descend"):
                return False
            if not self.cartesian_move(
                self.pick_pose,
                "pick_descend",
                avoid_collisions=False,
                duration_sec=self.pick_descend_duration,
            ):
                return False

            if self.validate_grasp_window() is None:
                self.get_logger().error("Cube is outside the safe grasp window")
                return False
            if not self.command_gripper(0.020, "closed"):
                return False
            if not self.attach_cube():
                return False
            attached_reference = self.validate_carry_window()
            if attached_reference is None:
                self.get_logger().error(
                    "Attached cube drifted before lift; aborting safely"
                )
                return False

            if not self.cartesian_move(
                self.pick_safe_pose,
                "pick_lift",
                avoid_collisions=False,
            ):
                return False
            if not self.cartesian_move(
                self.pick_transit_pose,
                "pick_clearance",
                avoid_collisions=False,
            ):
                return False
            if not self.update_stand_collision_objects(
                CollisionObject.ADD,
                ("moveit_pick_stand",),
            ):
                return False
            lifted_reference = self.validate_carry_window()
            if lifted_reference is None:
                self.get_logger().error(
                    "Attached cube left the safe carry window"
                )
                return False
            drift = point_distance(attached_reference, lifted_reference)
            self.get_logger().info(f"attached_relative_drift={drift:.9f}")
            if drift > MAX_ATTACHED_DRIFT:
                self.get_logger().warn(
                    "Attached cube settled more than 20 mm but remains "
                    "inside the safe carry window"
                )

            if not self.move_to_pose(self.place_safe_pose, "place_safe"):
                return False
            if not self.update_stand_collision_objects(
                CollisionObject.REMOVE,
                ("moveit_place_stand",),
            ):
                return False
            if not self.cartesian_move(
                self.place_pose,
                "place_descend",
                avoid_collisions=False,
            ):
                return False
            if not self.detach_cube():
                return False
            if not self.command_gripper(0.0, "open"):
                return False
            time.sleep(0.4)

            if not self.cartesian_move(
                self.place_safe_pose,
                "place_lift",
                avoid_collisions=False,
            ):
                return False
            if not self.update_stand_collision_objects(
                CollisionObject.ADD,
                ("moveit_place_stand",),
            ):
                return False
            if (
                self.verify_placement_enabled
                and not self.verify_placement()
            ):
                self.get_logger().error("Placement verification failed")
                return False
            if not self.move_to_joint_target(staging_arm, "return_home"):
                return False

            self.publish_state(f"pick_place_complete:{self.cube_name}")
            return True
        finally:
            if self.attached:
                self.get_logger().warn(
                    "Abnormal exit while attached; releasing cube"
                )
                self.detach_cube()
                self.command_gripper(0.0, "open")
            if self.base_stabilized:
                self.release_base()

    def _apply_request_state(self, request: dict) -> None:
        """Apply per-request pose and motion state before running an op."""
        self.operation = str(request.get("operation", self.operation))
        self.target_source = str(request.get("target_source", "unknown"))
        target_age = request.get("target_age_sec")
        self.target_age_sec = (
            float(target_age)
            if isinstance(target_age, (int, float))
            else None
        )
        self.approach_rejected_reason = None
        # Structured reason for the last refused placement. Reported in the
        # result so the coordinator can tell a deterministic workspace
        # refusal from a transient one instead of repeating an identical
        # unreachable request (v15 sent the same rejected slot twice).
        self.place_failure_reason = None
        self.place_slot_correction = None
        self.place_envelope_violation = None
        self.cube_name = str(request.get("cube_name", ""))
        self.pick_pose = tuple(float(v) for v in request["pick_pose"])
        self.pick_safe_pose = tuple(
            float(v) for v in request["pick_safe_pose"]
        )
        self.place_pose = tuple(float(v) for v in request["place_pose"])
        self.place_safe_pose = tuple(
            float(v) for v in request["place_safe_pose"]
        )
        self.tool_orientation = normalize_quaternion(
            tuple(float(v) for v in request["tool_orientation"])
        )
        release_world_pose = request.get("release_world_pose")
        self.release_world_pose = (
            tuple(float(v) for v in release_world_pose)
            if release_world_pose is not None
            else None
        )
        self.pick_transit_height = float(
            request.get("pick_transit_height", self.pick_transit_height)
        )
        self.pick_transit_pose = transit_pose_for_operation(
            self.operation,
            self.pick_safe_pose,
            self.pick_transit_height,
        )
        self.transport_x = float(
            request.get("transport_x", self.transport_x)
        )
        self.transport_duration = float(
            request.get("transport_duration", self.transport_duration)
        )
        if self.transport_duration <= 0.0:
            raise ValueError("transport_duration must be positive")
        self.transport_pose = low_transport_pose(
            self.pick_pose,
            self.transport_x,
        )
        self.velocity_scaling = float(
            request.get("velocity_scaling", self.velocity_scaling)
        )
        self.acceleration_scaling = float(
            request.get("acceleration_scaling", self.acceleration_scaling)
        )
        self.cartesian_duration = float(
            request.get("cartesian_duration", self.cartesian_duration)
        )
        self.gripper_duration = float(
            request.get("gripper_duration", self.gripper_duration)
        )
        self.pick_lift_duration = float(
            request.get("pick_lift_duration", self.pick_lift_duration)
        )
        if self.pick_lift_duration <= 0.0:
            raise ValueError("pick_lift_duration must be positive")

    def run_persistent(self) -> None:
        """Serve manipulation requests without a resident executor."""
        self.get_logger().info("persistent pick-place serving requests")
        request_topic = str(
            self.get_parameter("request_topic").value
        )
        result_topic = str(
            self.get_parameter("result_topic").value
        )
        self.pending_request: Optional[String] = None
        self._operation_lock = threading.Lock()
        self.create_subscription(
            String,
            request_topic,
            self._store_persistent_request,
            10,
        )
        self.result_publisher = self.create_publisher(
            String, result_topic, 10
        )

    def _store_persistent_request(self, message: String) -> None:
        """Queue the latest request; the serve loop executes it."""
        self.pending_request = message

    def _serve_loop(self) -> None:
        """Spin between operations without starving operation callbacks."""
        while rclpy.ok():
            if self.pending_request is None:
                rclpy.spin_once(self, timeout_sec=0.2)
                continue
            message = self.pending_request
            self.pending_request = None
            self._handle_persistent_request(message)

    def _handle_persistent_request(self, message: String) -> None:
        """Execute one operation and publish its result synchronously."""
        with self._operation_lock:
            try:
                request = json.loads(message.data)
            except (TypeError, json.JSONDecodeError):
                self.get_logger().error("Persistent request is not valid JSON")
                return
            request_id = request.get("request_id")
            self.get_logger().info(
                f"persistent_request id={request_id} "
                f"op={request.get('operation')} cube={request.get('cube_name')}"
            )
            try:
                self._apply_request_state(request)
                operations = {
                    "scan": self.run_scan_only,
                    "pick": self.run_pick_only,
                    "place": self.run_place_only,
                    "release": self.run_release_only,
                }
                operation = operations.get(self.operation)
                if operation is None:
                    success = False
                    detail = f"unknown operation {self.operation}"
                else:
                    success = operation()
                    if success:
                        detail = "ok"
                    elif self.approach_rejected_reason:
                        detail = (
                            "approach_rejected "
                            f"{self.approach_rejected_reason}"
                        )
                    else:
                        detail = f"{self.operation} failed"
            except Exception as exception:  # noqa: B902
                success = False
                detail = f"exception {exception}"
            result = String()
            result.data = json.dumps(
                {
                    "request_id": request_id,
                    "success": bool(success),
                    "base_released": not (
                        self.base_stabilized
                        or getattr(self, "base_hold_active", False)
                    ),
                    "cube_attached": bool(self.attached),
                    "detail": str(detail),
                    "failure_reason": self.place_failure_reason,
                    "place_slot_correction": (
                        list(self.place_slot_correction)
                        if self.place_slot_correction is not None
                        else None
                    ),
                    "place_envelope_drift": (
                        list(self.place_envelope_violation[0])
                        + [self.place_envelope_violation[1],
                           self.place_envelope_violation[2]]
                        if self.place_envelope_violation is not None
                        else None
                    ),
                },
                ensure_ascii=False,
                separators=(",", ":"),
            )
            self.result_publisher.publish(result)
            self.get_logger().info(
                f"persistent_result id={request_id} success={success} "
                f"detail={detail}"
            )


def main(args=None) -> None:
    """Run the selected manipulation operation and return an exit code."""
    rclpy.init(args=args)
    node = MoveItPickPlace()
    if node.get_parameter("persistent").value:
        node.run_persistent()
        try:
            node._serve_loop()
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()
        return
    success = False
    try:
        operations = {
            "cycle": node.run_once,
            "scan": node.run_scan_only,
            "pick": node.run_pick_only,
            "place": node.run_place_only,
            "release": node.run_release_only,
        }
        try:
            operation = operations[node.operation]
        except KeyError:
            node.get_logger().error(
                "operation must be cycle, scan, pick, place, or release"
            )
        else:
            success = operation()
    except KeyboardInterrupt:
        node.get_logger().warn("Pick-and-place interrupted")
    except Exception as exception:
        node.get_logger().error(f"Pick-and-place crashed: {exception}")
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(0 if success else 1)


if __name__ == "__main__":
    main()
