"""Read-only check of settled map geometry before a benchmark may start."""
import json
import math
from pathlib import Path
import time

from gazebo_msgs.srv import GetEntityState
import rclpy
from rclpy.node import Node

repo = Path(__file__).resolve().parents[2]
positions = json.loads((repo/'config/rocks_settled.json').read_text())
yaws = json.loads((repo/'config/rocks_settled_yaw.json').read_text())
rclpy.init()
node = Node('benchmark_static_map_truth_check')
client = node.create_client(GetEntityState, '/gazebo/get_entity_state')
passed = client.wait_for_service(timeout_sec=8.)
samples = []
deadline = time.monotonic()+15.
try:
    for name, expected in positions.items():
        if not passed or time.monotonic() >= deadline:
            passed = False
            break
        request = GetEntityState.Request()
        request.name, request.reference_frame = 'Untitled::'+name, 'world'
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=2.)
        response = future.result() if future.done() else None
        if response is None or not response.success:
            passed = False
            break
        p,q = response.state.pose.position,response.state.pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        xy_error = math.hypot(p.x-expected[0],p.y-expected[1])
        angle_error = abs(math.atan2(math.sin(yaw-yaws[name]),math.cos(yaw-yaws[name])))
        valid = xy_error <= .040 and angle_error <= .025
        passed = passed and valid
        samples.append({'link':name,'xyz':[p.x,p.y,p.z],'yaw':yaw,
                        'xy_error':xy_error,'yaw_error':angle_error,'valid':valid})
finally:
    node.destroy_node()
    rclpy.shutdown()
print(json.dumps({'static_map_truth_pass':passed and len(samples)==len(positions),
                  'rocks':samples},ensure_ascii=False))
raise SystemExit(0 if passed and len(samples)==len(positions) else 2)
