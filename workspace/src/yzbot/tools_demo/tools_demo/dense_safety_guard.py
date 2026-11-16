"""Independent final guard for the added experimental actors only."""
import json,math,time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from tools_demo.transport_motion import dense_velocity_scale, dense_rotation_clear, dense_snapshot_valid
from tools_demo.transport_scene import load_dense_scene


class DenseSafetyGuard(Node):
    def __init__(self):
        super().__init__('competition_dense_safety_guard')
        self.robot=None;self.actors=[];self.stamp=None;self.payload=False
        self.command=None;self.command_time=None;self.received=None;self.last_diag=0.
        self.names={a['name'] for a in load_dense_scene()['actors']}
        self.publisher=self.create_publisher(Twist,'/cmd_vel_safe',10)
        self.status=self.create_publisher(String,'/competition/dense_safety/state',10)
        self.create_subscription(Odometry,'/odom',self.odom,20)
        self.create_subscription(String,'/competition/dense_obstacles/state',self.obstacles,10)
        self.create_subscription(String,'/competition/mission/state',self.mission,10)
        self.create_subscription(Twist,'/cmd_vel_dense_input',self.velocity,10)
        self.create_timer(.05,self.tick)

    def odom(self,m):
        p=m.pose.pose;q=p.orientation
        self.robot=(p.position.x,p.position.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
        self.odom_time=time.monotonic()

    def obstacles(self,m):
        try:d=json.loads(m.data)
        except ValueError:return
        if not dense_snapshot_valid(d,self.names):
            self.received=None
            return
        self.actors=d['actors'];self.stamp=d['stamp_sec'];self.received=time.monotonic()

    def mission(self,m):
        try:d=json.loads(m.data)
        except ValueError:return
        self.payload=d.get('state') in ('navigating_departure','navigating_destination','placing')

    def velocity(self,m):
        values=(m.linear.x,m.linear.y,m.linear.z,m.angular.x,m.angular.y,m.angular.z)
        self.command=m if all(math.isfinite(v) for v in values) else Twist()
        self.command_time=time.monotonic()

    def tick(self):
        now=time.monotonic();command=self.command
        if command is None or self.command_time is None or now-self.command_time>.35:
            self.publisher.publish(Twist())
            return
        stamp=self.get_clock().now().nanoseconds/1e9
        fresh=(self.robot is not None and now-getattr(self,'odom_time',0)<.5 and self.received is not None
               and now-self.received<.5 and self.stamp is not None and 0<=stamp-self.stamp<.5)
        scale=0. if not fresh else dense_velocity_scale(self.robot,
          (command.linear.x,command.linear.y,command.angular.z),self.actors,self.payload)
        output=Twist();output.linear.x=command.linear.x*scale;output.linear.y=command.linear.y*scale
        # A moving arc must keep its curvature when slowed. Keeping yaw at
        # full speed turns a safe path into a pivot beside the actor/wall.
        yaw_scale=scale if math.hypot(command.linear.x,command.linear.y)>1e-5 else 1.
        output.angular.z=(command.angular.z*yaw_scale if fresh and
            dense_rotation_clear(self.robot,self.actors,self.payload) else 0.)
        self.publisher.publish(output)
        if now-self.last_diag>1.:
            self.last_diag=now
            self.status.publish(String(data=json.dumps(dict(stamp=stamp,fresh=fresh,scale=scale,actors=len(self.actors)))))


def main(args=None):
    rclpy.init(args=args);node=DenseSafetyGuard()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.publisher.publish(Twist());node.destroy_node();rclpy.shutdown()
