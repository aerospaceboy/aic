"""One lightweight simulator driver for audited distributed transport actors.

Motion is task-clock driven, independent of the robot's chosen plan. Proximity
freezes are physical backstops and are reported, never scored as robot actions.
"""
import json,math,time
from pathlib import Path
from ament_index_python.packages import get_package_share_directory, get_package_prefix
import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SpawnEntity
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Bool
from rclpy.qos import QoSProfile, DurabilityPolicy
from tools_demo.transport_motion import patrol_position


class DenseActors(Node):
    def __init__(self):
        super().__init__('competition_dense_actors')
        self.declare_parameter('scene_file','')
        p=self.get_parameter('scene_file').value
        if not p:p=str(Path(get_package_share_directory('bot_navigation'))/'config/dense_transport_scene.json')
        self.scene=json.loads(Path(p).read_text());self.actors=self.scene['actors']
        self.names={a['name'] for a in self.actors}
        self.actual={};self.previous={};self.velocity={};self.robot=None;self.model_stamp=None
        self.task=None;self.started=False;self.last=None
        self.elapsed={a['name']:0. for a in self.actors}
        self.holds={a['name']:0 for a in self.actors};self.paused={a['name']:False for a in self.actors}
        self.spawned=set();self.spawning=None;self.pending={};self.last_publish=0.
        self.spawner=self.create_client(SpawnEntity,'/spawn_entity')
        self.motion=self.create_publisher(Bool,'/competition/dense_motion_started',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.motion.publish(Bool(data=False))
        self.publisher=self.create_publisher(String,'/competition/dense_obstacles/state',10)
        self.individual={a['name']:self.create_publisher(String,'/competition/obstacles/'+a['name'],10) for a in self.actors}
        self.create_subscription(ModelStates,'/gazebo/model_states',self.models,10)
        self.create_subscription(Odometry,'/odom',self.odom,20)
        self.create_subscription(String,'/competition/task/plan',self.plan,10)
        self.create_subscription(String,'/competition/dense_obstacles/guards',self.guard,10)
        self.create_timer(.1,self.tick)
        self.get_logger().info(f'dense_scene actors={len(self.actors)} static_world_unchanged=true')

    def models(self,msg):
        now=self.get_clock().now().nanoseconds/1e9
        self.model_stamp=now
        for missing in self.names-set(msg.name):
            self.actual.pop(missing,None);self.previous.pop(missing,None);self.velocity.pop(missing,None)
        for n,p in zip(msg.name,msg.pose):
            if n not in self.names:continue
            xy=(p.position.x,p.position.y,p.position.z)
            old=self.previous.get(n)
            if old is None or now < old[0]:
                self.previous[n]=(now,*xy[:2])
            elif now-old[0]>=.19:
                self.velocity[n]=((xy[0]-old[1])/(now-old[0]),(xy[1]-old[2])/(now-old[0]))
                self.previous[n]=(now,*xy[:2])
            self.actual[n]=xy

    def odom(self,msg):
        p=msg.pose.pose.position;self.robot=(p.x,p.y)

    def plan(self,msg):
        try:d=json.loads(msg.data)
        except ValueError:return
        tid=d.get('task_id')
        if not tid or tid==self.task:return
        self.task=tid;self.started=True;self.last=self.get_clock().now().nanoseconds/1e9
        self.motion.publish(Bool(data=True))
        # No phase reset or jump: a later task resumes actual continuous phase.

    def guard(self,msg):
        try:d=json.loads(msg.data)
        except ValueError:return
        if d.get('model') in self.holds:
            self.holds[d['model']]=d['holds'];self.paused[d['model']]=d['active']

    def sdf(self,a):
        x,y=patrol_position(a,0.);r=a['radius'];h=a['height']
        plugin=str(Path(get_package_prefix('bot_navigation'))/'lib/libdense_patrol.so')
        return f'''<sdf version="1.6"><model name="{a['name']}"><static>true</static>
          <pose>{x} {y} {h/2} 0 0 0</pose><link name="body">
          <collision name="collision"><geometry><cylinder><radius>{r}</radius><length>{h}</length></cylinder></geometry></collision>
          <visual name="visual"><geometry><cylinder><radius>{r}</radius><length>{h}</length></cylinder></geometry>
          <material><ambient>0.1 0.7 0.6 1</ambient><diffuse>0.1 0.7 0.6 1</diffuse></material></visual>
          </link><plugin name="{a['name']}_driver" filename="{plugin}">
          <start_x>{a['start'][0]}</start_x><start_y>{a['start'][1]}</start_y>
          <end_x>{a['end'][0]}</end_x><end_y>{a['end'][1]}</end_y>
          <speed>{a['speed']}</speed><phase_fraction>{a['phase_fraction']}</phase_fraction>
          <height>{h}</height><radius>{r}</radius></plugin></model></sdf>'''

    def spawn(self,a):
        request=SpawnEntity.Request();request.name=a['name'];request.xml=self.sdf(a);request.reference_frame='world';request.initial_pose.orientation.w=1.
        f=self.spawner.call_async(request);self.spawning=f
        def done(future):
            result=future.result()
            if result and (result.success or 'already exists' in result.status_message):self.spawned.add(a['name'])
            else:self.get_logger().error('dense_spawn_failed '+a['name'])
            self.spawning=None
        f.add_done_callback(done)

    def tick(self):
        now=self.get_clock().now().nanoseconds/1e9
        if not self.spawner.service_is_ready():return
        if self.spawning is None:
            for a in self.actors:
                if a['name'] not in self.spawned:self.spawn(a);break
        dt=0. if self.last is None or not self.started else max(0.,min(.2,now-self.last))
        self.last=now
        states=[]
        for a in self.actors:
            n=a['name']
            if n not in self.spawned:continue
            if n not in self.actual or self.model_stamp is None or now-self.model_stamp>.35:continue
            x,y,z=self.actual[n];vx,vy=self.velocity.get(n,(0.,0.))
            state=dict(model=n,x=x,y=y,vx=vx,vy=vy,radius=a['radius'],stamp_sec=self.model_stamp,success=True,
                       physical_guard_holds=self.holds[n],physical_guard_active=self.paused[n])
            states.append(state)
            if now-self.last_publish>=.19:self.individual[n].publish(String(data=json.dumps(state)))
        if now-self.last_publish>=.19:
            self.last_publish=now
            self.publisher.publish(String(data=json.dumps(dict(task_id=self.task,stamp_sec=now,
                 expected_count=len(self.actors),observed_count=len(states),actors=states))))


def main(args=None):
    rclpy.init(args=args);node=DenseActors()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.shutdown()
