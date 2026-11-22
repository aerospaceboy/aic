"""Independent final guard for the added experimental actors only."""
import json,math,time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Polygon, Point32
from rclpy.qos import QoSProfile, DurabilityPolicy
from tools_demo.transport_motion import dense_velocity_scale, dense_rotation_clear, dense_snapshot_valid
from tools_demo.transport_scene import load_dense_scene
from tools_demo.payload_envelope import navigation_footprint


class DenseSafetyGuard(Node):
    def __init__(self):
        super().__init__('competition_dense_safety_guard')
        self.robot=None;self.actors=[];self.stamp=None;self.payload=False
        self.command=None;self.command_time=None;self.received=None;self.last_diag=0.
        self.attached=None;self.planar_valid=False
        self.names={a['name'] for a in load_dense_scene()['actors']}
        self.patrols={a['name']:a for a in load_dense_scene()['actors']}
        self.publisher=self.create_publisher(Twist,'/cmd_vel_safe',1)
        self.status=self.create_publisher(String,'/competition/dense_safety/state',10)
        footprint_qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.footprint_pub=self.create_publisher(Polygon,'/local_costmap/footprint',footprint_qos)
        self.last_footprint=None
        self.create_subscription(Odometry,'/odom',self.odom,20)
        self.create_subscription(ModelStates,'/gazebo/model_states',self.models,1)
        self.create_subscription(String,'/competition/dense_obstacles/state',self.obstacles,10)
        self.create_subscription(String,'/competition/mission/state',self.mission,
                                 QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(String,'/competition/gripper/state',self.gripper,10)
        self.create_subscription(Twist,'/cmd_vel_dense_input',self.velocity,1)
        self.create_timer(.05,self.watchdog_tick)

    def odom(self,m):
        # Encoder odometry can drift after contact. Actor coordinates in this
        # existing simulator safety channel are WORLD coordinates, so the base
        # must come from the same frame, not untransformed /odom.
        self.measured_velocity=m.twist.twist

    def models(self,m):
        if 'six_arm' not in m.name:
            self.robot=None
            return
        p=m.pose[m.name.index('six_arm')];q=p.orientation
        roll=math.atan2(2*(q.w*q.x+q.y*q.z),1-2*(q.x*q.x+q.y*q.y))
        pitch=math.asin(max(-1.,min(1.,2*(q.w*q.y-q.z*q.x))))
        self.planar_valid=all(math.isfinite(v) for v in (roll,pitch,p.position.z)) and max(abs(roll),abs(pitch))<.30
        self.robot=(p.position.x,p.position.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
        self.world_pose_time=time.monotonic()

    def obstacles(self,m):
        try:d=json.loads(m.data)
        except ValueError:return
        if not dense_snapshot_valid(d,self.names):
            self.received=None
            return
        self.actors=[]
        for a in d['actors']:
            rail=self.patrols[a['model']]
            self.actors.append(dict(a,patrol_start=rail['start'],patrol_end=rail['end'],
                                   speed_bound=rail['speed']))
        self.stamp=d['stamp_sec'];self.received=time.monotonic()

    def mission(self,m):
        try:d=json.loads(m.data)
        except ValueError:return
        # Terminal mission states do not release a physically attached load.
        if getattr(self,'attached',None) is None:
            self.payload=d.get('state') in ('navigating_departure','navigating_destination','placing')

    def gripper(self,m):
        if m.data.startswith('attached:'):
            self.attached=True;self.payload=True
        elif m.data.startswith('released:'):
            self.attached=False;self.payload=False

    def velocity(self,m):
        values=(m.linear.x,m.linear.y,m.linear.z,m.angular.x,m.angular.y,m.angular.z)
        self.command=m if all(math.isfinite(v) for v in values) else Twist()
        self.command_time=time.monotonic()
        # Check the complete original safety envelope on EVERY new command,
        # including zero. A second timer phase need not delay braking.
        DenseSafetyGuard.tick(self)

    def watchdog_tick(self):
        if time.monotonic()-getattr(self,'last_evaluation',0.)>=.04:
            DenseSafetyGuard.tick(self)

    def tick(self):
        now=time.monotonic()
        self.last_evaluation=now
        command=self.command
        if hasattr(self,'footprint_pub') and (getattr(self,'last_footprint',None)!=self.payload
                or now-getattr(self,'footprint_time',0)>1.):
            self.footprint_pub.publish(Polygon(points=[Point32(x=float(x),y=float(y))
                for x,y in navigation_footprint(self.payload)]))
            self.last_footprint=self.payload
            self.footprint_time=now
        if command is None or self.command_time is None or now-self.command_time>.35:
            self.publisher.publish(Twist())
            return
        stamp=self.get_clock().now().nanoseconds/1e9
        fresh=(getattr(self,'planar_valid',True) and self.robot is not None and now-getattr(self,'world_pose_time',0)<.5 and self.received is not None
               and now-self.received<.5 and self.stamp is not None and 0<=stamp-self.stamp<.5)
        actors=[dict(a,source_age=max(0.,stamp-a.get('stamp_sec',stamp))) for a in self.actors]
        fresh=fresh and all(0<=stamp-a.get('stamp_sec',stamp)<.5 for a in self.actors)
        scale=0. if not fresh else dense_velocity_scale(self.robot,
          (command.linear.x,command.linear.y,command.angular.z),actors,self.payload)
        turning_clear=fresh and dense_rotation_clear(self.robot,actors,self.payload)
        # Suppressing yaw alone changes a validated arc into a straight line.
        # A denied turn must stop the WHOLE moving arc. Straight retreat stays
        # available under its independent swept stopping check.
        if not turning_clear and abs(command.angular.z)>1e-5:
            scale=0.
        output=Twist();output.linear.x=command.linear.x*scale;output.linear.y=command.linear.y*scale
        # A moving arc must keep its curvature when slowed. Keeping yaw at
        # full speed turns a safe path into a pivot beside the actor/wall.
        yaw_scale=scale if math.hypot(command.linear.x,command.linear.y)>1e-5 else 1.
        output.angular.z=command.angular.z*yaw_scale if turning_clear else 0.
        self.publisher.publish(output)
        if now-self.last_diag>1.:
            self.last_diag=now
            self.status.publish(String(data=json.dumps(dict(stamp=stamp,fresh=fresh,scale=scale,actors=len(self.actors),robot_frame='world',turning_clear=turning_clear,planar_valid=getattr(self,'planar_valid',False),payload=self.payload))))


def main(args=None):
    rclpy.init(args=args);node=DenseSafetyGuard()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.publisher.publish(Twist());node.destroy_node();rclpy.shutdown()
