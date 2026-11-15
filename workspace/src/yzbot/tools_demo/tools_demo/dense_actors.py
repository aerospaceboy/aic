"""One lightweight simulator driver for audited distributed transport actors.

Motion is task-clock driven, independent of the robot's chosen plan. Proximity
freezes are physical backstops and are reported, never scored as robot actions.
"""
import json,math,time
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates,EntityState
from gazebo_msgs.srv import SpawnEntity,SetEntityState
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tools_demo.transport_motion import patrol_position


class DenseActors(Node):
    def __init__(self):
        super().__init__('competition_dense_actors')
        self.declare_parameter('scene_file','')
        p=self.get_parameter('scene_file').value
        if not p:p=str(Path(get_package_share_directory('bot_navigation'))/'config/dense_transport_scene.json')
        self.scene=json.loads(Path(p).read_text());self.actors=self.scene['actors']
        self.actual={};self.previous={};self.velocity={};self.robot=None;self.model_stamp=None
        self.task=None;self.started=False;self.last=None
        self.elapsed={a['name']:0. for a in self.actors}
        self.holds={a['name']:0 for a in self.actors};self.paused={a['name']:False for a in self.actors}
        self.spawned=set();self.spawning=None;self.pending={};self.last_publish=0.
        self.spawner=self.create_client(SpawnEntity,'/spawn_entity')
        self.setter=self.create_client(SetEntityState,'/gazebo/set_entity_state')
        self.publisher=self.create_publisher(String,'/competition/dense_obstacles/state',10)
        self.individual={a['name']:self.create_publisher(String,'/competition/obstacles/'+a['name'],10) for a in self.actors}
        self.create_subscription(ModelStates,'/gazebo/model_states',self.models,10)
        self.create_subscription(Odometry,'/odom',self.odom,20)
        self.create_subscription(String,'/competition/task/plan',self.plan,10)
        self.create_timer(.1,self.tick)
        self.get_logger().info(f'dense_scene actors={len(self.actors)} static_world_unchanged=true')

    def models(self,msg):
        now=self.get_clock().now().nanoseconds/1e9
        self.model_stamp=now
        for n,p in zip(msg.name,msg.pose):
            xy=(p.position.x,p.position.y,p.position.z)
            old=self.previous.get(n)
            if old and now>old[0]+.02:
                self.velocity[n]=((xy[0]-old[1])/(now-old[0]),(xy[1]-old[2])/(now-old[0]))
            self.previous[n]=(now,*xy[:2]);self.actual[n]=xy

    def odom(self,msg):
        p=msg.pose.pose.position;self.robot=(p.x,p.y)

    def plan(self,msg):
        try:d=json.loads(msg.data)
        except ValueError:return
        tid=d.get('task_id')
        if not tid or tid==self.task:return
        self.task=tid;self.started=True;self.last=self.get_clock().now().nanoseconds/1e9
        # No phase reset or jump: a later task resumes actual continuous phase.

    def sdf(self,a):
        x,y=patrol_position(a,0.);r=a['radius'];h=a['height']
        return f'''<sdf version="1.6"><model name="{a['name']}"><static>true</static>
          <pose>{x} {y} {h/2} 0 0 0</pose><link name="body">
          <collision name="collision"><geometry><cylinder><radius>{r}</radius><length>{h}</length></cylinder></geometry></collision>
          <visual name="visual"><geometry><cylinder><radius>{r}</radius><length>{h}</length></cylinder></geometry>
          <material><ambient>0.1 0.7 0.6 1</ambient><diffuse>0.1 0.7 0.6 1</diffuse></material></visual>
          </link></model></sdf>'''

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
        if not self.spawner.service_is_ready() or not self.setter.service_is_ready():return
        if self.spawning is None:
            for a in self.actors:
                if a['name'] not in self.spawned:self.spawn(a);break
        dt=0. if self.last is None or not self.started else max(0.,min(.2,now-self.last))
        self.last=now
        cubes=[p for n,p in self.actual.items() if '_cube_' in n and not n.endswith('_anchor') and p[2]>.04]
        states=[]
        for a in self.actors:
            n=a['name']
            if n not in self.spawned:continue
            old=patrol_position(a,self.elapsed[n]);candidate=patrol_position(a,self.elapsed[n]+dt)
            hold=(self.robot is not None and math.dist(candidate,self.robot)<.45)
            hold=hold or any(math.dist(candidate,p[:2])<a['radius']+.10 for p in cubes)
            if hold and not self.paused[n]:self.holds[n]+=1
            self.paused[n]=hold
            if not hold:self.elapsed[n]+=dt
            x,y=patrol_position(a,self.elapsed[n])
            pending=self.pending.get(n)
            if pending is None or pending.done():
                request=SetEntityState.Request();request.state=EntityState();request.state.name=n;request.state.reference_frame='world'
                request.state.pose.position.x=x;request.state.pose.position.y=y;request.state.pose.position.z=a['height']/2
                request.state.pose.orientation.w=1.
                self.pending[n]=self.setter.call_async(request)
            if n not in self.actual or self.model_stamp is None or now-self.model_stamp>.35:continue
            x,y,z=self.actual[n];vx,vy=self.velocity.get(n,(0.,0.))
            state=dict(model=n,x=x,y=y,vx=vx,vy=vy,radius=a['radius'],stamp_sec=self.model_stamp,success=True,
                       physical_guard_holds=self.holds[n],physical_guard_active=hold)
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
