"""Filter lidar self returns using fresh TF and URDF collision primitives.

Only actual arm/finger volumes and the measured rigid attachment transform
are masked as unobserved (NaN). Close external walls remain visible. Missing
transforms preserve raw returns; no rectangular blind-zone fallback is used.
"""

import json
import math
import copy
import numpy as np
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException


# Collision primitives from arm.xacro, in the corresponding link frame.
SELF_SHAPES = (
    ('link1', 'cylinder', (0.03, 0.10, 0.0)),
    ('link2', 'cylinder', (0.03, 0.14, 0.07)),
    ('link3', 'cylinder', (0.03, 0.22, 0.11)),
    ('link4', 'cylinder', (0.025, 0.06, 0.03)),
    ('link5', 'cylinder', (0.03, 0.06, 0.03)),
    ('link6', 'cylinder', (0.05, 0.02, 0.01)),
    ('gripper_finger_link1', 'box', (.04, .005, .01)),
    ('gripper_finger_link2', 'box', (.04, .005, .01)),
    ('camera_rgbd_link', 'box', (.035, .15, .045)),
)


def geometric_masked_scan(scan, transforms, payload=None):
    """Remove only returns inside actual link geometry, preserving close walls.

    Transforms map the scan frame into each URDF link frame. Missing links
    remain unfiltered; no broad blind rectangle is used as a fallback.
    """
    result=copy.deepcopy(scan)
    ranges=np.asarray(scan.ranges,dtype=float)
    angles=scan.angle_min+np.arange(len(ranges))*scan.angle_increment
    finite=np.isfinite(ranges)
    points=np.zeros((len(ranges),3))
    points[:,0]=np.where(finite,ranges,0.)*np.cos(angles)
    points[:,1]=np.where(finite,ranges,0.)*np.sin(angles)
    removed=np.zeros(len(ranges),dtype=bool)
    for link,kind,shape in SELF_SHAPES:
        transform=transforms.get(link)
        if transform is None:continue
        t,q=transform
        x,y,z,w=q
        if (not all(math.isfinite(v) for v in (*t,*q))
                or abs(sum(v*v for v in q)-1.)>.01):continue
        rotation=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
            [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
            [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
        local=points@rotation.T+np.asarray(t)
        pad=.025  # Gazebo lidar has 0.01 m Gaussian range noise.
        if kind=='cylinder':
            radius,length,center_z=shape
            inside=(np.hypot(local[:,0],local[:,1])<=radius+pad)&(np.abs(local[:,2]-center_z)<=length/2+pad)
        else:
            inside=np.all(np.abs(local)<=np.asarray(shape)+pad,axis=1)
        if payload is not None and link=='link6':
            center,quaternion=payload
            a,b,c,d=quaternion
            if all(math.isfinite(v) for v in (*center,*quaternion)) and abs(sum(v*v for v in quaternion)-1.)<.01:
                rotation=np.array([[1-2*(b*b+c*c),2*(a*b-c*d),2*(a*c+b*d)],
                    [2*(a*b+c*d),1-2*(a*a+c*c),2*(b*c-a*d)],
                    [2*(a*c-b*d),2*(b*c+a*d),1-2*(a*a+b*b)]])
                cube=(local-np.asarray(center))@rotation
                inside |= np.all(np.abs(cube)<=.015+pad,axis=1)
        removed |= finite&inside
    result.ranges=[float('nan') if masked else value
                   for value,masked in zip(scan.ranges,removed)]
    return result


def in_payload_box(
    range_value: float,
    angle: float,
    forward_min: float,
    forward_max: float,
    lateral_half: float,
) -> bool:
    """Return whether one return falls inside the laser-frame payload box."""
    forward = range_value * math.cos(angle)
    lateral = range_value * math.sin(angle)
    return (
        forward_min <= forward <= forward_max
        and abs(lateral) <= lateral_half
    )


def should_mask(
    state: Optional[str],
    holding_states: frozenset,
) -> bool:
    """Return whether the payload box applies for the given mission state.

    The mission state is level information published on change with
    transient-local durability, not a heartbeat: a long docked picking
    phase must keep the mask active even when no message arrives for
    seconds. Fail-open applies only before the first state message
    arrives (the coordinator never started or died before publishing).
    """
    if state is None:
        return False
    return state in holding_states


def masked_scan(
    scan: LaserScan,
    forward_min: float,
    forward_max: float,
    lateral_half: float,
) -> LaserScan:
    """Copy the scan with payload-box beams marked unobserved (NaN)."""
    masked = LaserScan()
    masked.header = scan.header
    masked.angle_min = scan.angle_min
    masked.angle_max = scan.angle_max
    masked.angle_increment = scan.angle_increment
    masked.time_increment = scan.time_increment
    masked.scan_time = scan.scan_time
    masked.range_min = scan.range_min
    masked.range_max = scan.range_max
    masked.ranges = [
        float("nan")
        if in_payload_box(
            range_value, scan.angle_min + index * scan.angle_increment,
            forward_min, forward_max, lateral_half,
        )
        else range_value
        for index, range_value in enumerate(scan.ranges)
    ]
    masked.intensities = list(scan.intensities)
    return masked


class PayloadScanMask(Node):
    """Republish /scan without the carried payload's own returns."""

    def __init__(self) -> None:
        super().__init__("payload_scan_mask")
        self.declare_parameter("input_topic", "/scan")
        self.declare_parameter("output_topic", "/scan_payload_masked")
        self.declare_parameter("state_topic", "/competition/mission/state")
        self.declare_parameter("forward_min", 0.30)
        self.declare_parameter("forward_max", 0.48)
        self.declare_parameter("lateral_half", 0.24)
        self.declare_parameter(
            "holding_states",
            [
                "picking",
                "navigating_departure",
                "navigating_destination",
                "placing",
                "verifying",
                "releasing",
            ],
        )

        self.forward_min = float(self.get_parameter("forward_min").value)
        self.forward_max = float(self.get_parameter("forward_max").value)
        self.lateral_half = float(self.get_parameter("lateral_half").value)
        if not 0.0 <= self.forward_min < self.forward_max:
            raise ValueError(
                "payload box needs 0 <= forward_min < forward_max")
        if self.lateral_half < 0.0:
            raise ValueError("lateral_half must be nonnegative")
        self.holding_states = frozenset(
            str(value)
            for value in self.get_parameter("holding_states").value
        )

        self.mission_state: Optional[str] = None
        self.tf=Buffer()
        self.tf_listener=TransformListener(self.tf,self)
        self.payload_geometry=None
        self.create_subscription(PoseStamped,'/competition/payload/relative_pose',
                                 self.payload_callback,
                                 QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.masked_returns = 0
        self.published_scans = 0

        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        # Match the coordinator's reliable transient-local state publisher:
        # the current state is level information, and a mask node that joins
        # mid-carry must receive it without waiting for the next change.
        state_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.create_publisher(
            LaserScan, str(self.get_parameter("output_topic").value),
            sensor_qos
        )
        self.create_subscription(
            LaserScan,
            str(self.get_parameter("input_topic").value),
            self.scan_callback,
            sensor_qos,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("state_topic").value),
            self.state_callback,
            state_qos,
        )

    def state_callback(self, message: String) -> None:
        """Track the latest mission state."""
        try:
            payload = json.loads(message.data)
        except ValueError:
            return
        if not isinstance(payload, dict):
            return
        state = payload.get("state")
        if not isinstance(state, str):
            return
        self.mission_state = state

    def payload_callback(self,message):
        if message.header.frame_id!='link6':
            self.payload_geometry=None
            return
        p=message.pose.position;q=message.pose.orientation
        self.payload_geometry=((p.x,p.y,p.z),(q.x,q.y,q.z,q.w))

    def scan_callback(self, scan: LaserScan) -> None:
        """Republish the scan, filtering physical self geometry while holding."""
        if should_mask(self.mission_state, self.holding_states):
            transforms={}
            for link,_,_ in SELF_SHAPES:
                try:
                    transform=self.tf.lookup_transform(link,scan.header.frame_id,
                                                       Time.from_msg(scan.header.stamp))
                    t=transform.transform.translation;q=transform.transform.rotation
                    transforms[link]=((t.x,t.y,t.z),(q.x,q.y,q.z,q.w))
                except TransformException:
                    try:
                        transform=self.tf.lookup_transform(link,scan.header.frame_id,Time())
                        age=(Time.from_msg(scan.header.stamp).nanoseconds-
                             Time.from_msg(transform.header.stamp).nanoseconds)/1e9
                        if not -.02<=age<=.10:continue
                        t=transform.transform.translation;q=transform.transform.rotation
                        transforms[link]=((t.x,t.y,t.z),(q.x,q.y,q.z,q.w))
                    except TransformException:
                        continue
            output=geometric_masked_scan(scan,transforms,self.payload_geometry)
        else:
            output = scan
        self.publisher.publish(output)
        self.published_scans += 1


def main(args=None) -> None:
    """Run the payload scan mask node."""
    rclpy.init(args=args)
    node = PayloadScanMask()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
