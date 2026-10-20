#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/lib.sh"
load_ros
python3 - <<'PY'
import socket,urllib.request,json
with socket.create_connection(('127.0.0.1',21274),timeout=3):print('coBridge TCP 21274: 可连接（尚不证明CoStudio订阅成功）')
s=json.load(urllib.request.urlopen('http://127.0.0.1:8080/api/state',timeout=5))
print('mission:',s.get('mission'));print('navigation:',s.get('navigation'))
print('avoidance:',{k:s.get('avoidance',{}).get(k) for k in ['obstacles_seen','original_pair_demonstrated','collision_detected','minimum_clearance']})
PY
timeout 8 ros2 action list
timeout 8 ros2 topic info /competition/status
timeout 8 ros2 topic info /competition/task/problem_chunk
timeout 8 ros2 topic info /competition/scene_markers
echo '还需在GUI核对地图、实时黄/青路线、双相机和任务状态。该检查不提交任务。'
