#!/usr/bin/env bash
# Motion-only benchmark. Every invocation retains a fresh evidence directory.
set -eo pipefail
repo=$(git -C "$(dirname "${BASH_SOURCE[0]}")" rev-parse --show-toplevel)
workspace=$(cd "$repo/../.." && pwd)
source "$repo/tools_demo/scripts/verify_runtime_sources.sh"
verify_yzbot_runtime_sources "$repo" "$workspace"
question=${1:?Usage: bash run_ordered_case.sh 'Move ...' [output-root]}
root=${2:-"$workspace/experiments/ordered-cases"}
# A stale ROS stack silently breaks the next run: the new Nav2 lifecycle
# manager talks to the old node's services and aborts bringup, so the global
# costmap never activates and every leg fails at 60 s with elapsed 0.0
# (2026-10-06, three validation cases in a row). gzserver alone is not a
# sufficient signal - the launch, nav2 and move_group processes also survive
# a killed campaign.
if pgrep -f '[r]os2 launch tools_demo' >/dev/null \
   || pgrep -x gzserver >/dev/null \
   || pgrep -f '[t]ools_demo/lib/tools_demo/(task_navigator|mission_coordinator|task_parser|moving_obstacle|safety_limiter|velocity_selector|avoidance_monitor)([[:space:]]|$)|[m]ybot/lib/mybot/pick_place_moveit([[:space:]]|$)|[n]av2_(controller|planner|collision_monitor)/' >/dev/null; then
  echo 'STALE_ROS_STACK: stop all children of the owning launch before this run; no automatic broad kill' >&2
  exit 2
fi
mkdir -p "$root"
out=$(mktemp -d "$root/$(date -u +%Y%m%dT%H%M%SZ)-XXXXXX")
printf '%s\n' "$out" > "$root/latest-run.txt"
printf '%s\n' "$question" > "$out/question.txt"
git -C "$repo" rev-parse HEAD > "$out/commit.txt"
git -C "$repo" status --short > "$out/worktree-status.txt"
git -C "$repo" diff HEAD --binary > "$out/worktree.patch"
find "$repo/mybot" "$repo/tools_demo/tools_demo" "$repo/tools_demo/config" "$repo/bot_navigation/config" \
  "$repo/bot_navigation/param" "$repo/bot_navigation/maps" "$repo/config" \
  "$repo/mybot_description/worlds" "$repo/mybot_description/urdf" \
  "$repo/mybot_description/mybot_description" "$repo/tools_demo/launch" \
  "$repo/tools_demo/scripts" "$repo/bot_navigation/launch" \
  "$repo/bot_navigation/src" "$repo/bot_navigation/include" "$repo/IFRA_LinkAttacher" \
  -type f ! -path '*/__pycache__/*' -print0 | sort -z | xargs -0 sha256sum > "$out/source-config.sha256"
printf '%s\n' 'motion-only; headless=true; LLM=true; require_llm=true; dashboard=false; moving_obstacle=true' > "$out/mode.txt"
cd "$workspace"
source /opt/ros/humble/setup.bash
source install/setup.bash
# YZBOT_RMW lets a campaign select another RMW without editing the campaign.
# This machine crashes repeatedly inside libfastrtps.so.2.6.5 (dmesg: segfaults
# in static_transformer / moving_obstacle / dynamic_obstacle, and SIGABRT in
# gzserver + controller_server), so the coverage matrix runs on CycloneDDS.
source "$repo/tools_demo/scripts/ros_transport.sh"
configure_yzbot_transport "$workspace"
# YZBOT credentials: the motion questions are English one-liners that the
# deterministic solver does not parse, so the model parser is required.
if [ -r "$HOME/.config/yzbot/llm.env" ]; then set -a; . "$HOME/.config/yzbot/llm.env"; set +a; fi
export DISPLAY=${DISPLAY:-:0} QT_X11_NO_MITSHM=1
cleanup() {
  for pid in ${stats_pid:-} ${launch_pid:-}; do kill -INT -- "-$pid" 2>/dev/null || true; done
  sleep 8
  for pid in ${stats_pid:-} ${launch_pid:-}; do kill -KILL -- "-$pid" 2>/dev/null || true; done
}
trap cleanup EXIT
setsid ros2 launch tools_demo competition_system.launch.py headless:=True generate_question:=False use_llm:=True require_llm:=True start_llm_server:=False moving_obstacle:=True dashboard:=False slam_evidence:=False run_acceptance:=False \
  subsystem_delay_sec:=25.0 navigation_delay_sec:=75.0 mission_delay_sec:=105.0 acceptance_delay_sec:=110.0 > "$out/launch.log" 2>&1 &
launch_pid=$!
ready=false
for i in $(seq 1 90); do
  if grep -aq 'persistent pick-place serving' "$out/launch.log"; then ready=true; break; fi
  if ! kill -0 "$launch_pid" 2>/dev/null; then break; fi
  sleep 2
done
if [ "$ready" != true ]; then echo STARTUP_FAILED | tee "$out/status.txt"; exit 1; fi
# pick_place serving only proves the manipulator is up; the mission must not
# start before Nav2 is actually serving, or task_navigator fails the first leg
# with "navigation prerequisites never became ready within 60s".
nav_ready=false
for i in $(seq 1 60); do
  if ros2 action list 2>/dev/null | grep -q 'navigate_to_pose'; then
    nav_ready=true
    break
  fi
  if ! kill -0 "$launch_pid" 2>/dev/null; then break; fi
  sleep 2
done
if [ "$nav_ready" != true ]; then
  echo NAV_NOT_READY | tee "$out/status.txt"
  exit 1
fi
# The simulation itself must be alive before a case is submitted. A dead
# gazebo process still leaves Nav2 actions present (observed 20261012T130052),
# which would otherwise be recorded as a navigation failure instead of a
# startup failure.
#
# The probe subscribes with sensor QoS and waits for real messages: `ros2 topic
# info` through the CLI daemon reports stale zero publisher counts here, and
# /clock is published BEST_EFFORT, so a default RELIABLE subscription never
# receives it.
sim_ready=false
for i in $(seq 1 90); do
  if timeout 25 python3 - <<'PY' 2>/dev/null
import sys, time, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
rclpy.init()
node = Node("ordered_case_sim_probe")
seen = {"clock": False, "scan": False}
node.create_subscription(Clock, "/clock", lambda _m: seen.__setitem__("clock", True), qos_profile_sensor_data)
node.create_subscription(LaserScan, "/scan", lambda _m: seen.__setitem__("scan", True), qos_profile_sensor_data)
deadline = time.time() + 8
while time.time() < deadline and not all(seen.values()):
    rclpy.spin_once(node, timeout_sec=0.5)
node.destroy_node()
rclpy.shutdown()
sys.exit(0 if all(seen.values()) else 1)
PY
  then
    sim_ready=true
    break
  fi
  if ! kill -0 "$launch_pid" 2>/dev/null; then break; fi
  sleep 1
done
if [ "$sim_ready" != true ]; then
  echo SIM_NOT_READY | tee "$out/status.txt"
  exit 1
fi
if ! python3 "$repo/tools_demo/scripts/verify_static_map_truth.py" > "$out/static-map-truth.json" 2>&1; then
  echo STATIC_MAP_TRUTH_MISMATCH | tee "$out/status.txt"
  exit 1
fi
sleep 8
setsid gz stats -p > "$out/gz_stats.csv" 2>&1 &
stats_pid=$!
set +e
rlc_timeout=$(awk "BEGIN{printf \"%.1f\", ${RLC_TIMEOUT_SEC:-480.0}+0}")
ros2 run tools_demo competition_case_runner --ros-args -p "question:=$question" -p timeout_sec:=$rlc_timeout > "$out/result.log" 2>&1
result_code=$?
set -e
printf '%s\n' "$result_code" > "$out/runner-exit-code.txt"
# The global recorder path can be stale after startup failure; copy only here.
python3 - "$out" <<'PY'
import json, pathlib, shutil, sys
out = pathlib.Path(sys.argv[1])
source = pathlib.Path('/tmp/competition_speed_profile.json')
status = {'copied': False, 'reason': 'missing_or_stale'}
try:
    terminal = {}
    for line in (out / 'result.log').read_text().splitlines():
        if 'MISSION_TERMINAL=' in line:
            terminal = json.loads(line.split('MISSION_TERMINAL=', 1)[1])
    profile = json.loads(source.read_text())
    if (source.stat().st_mtime >= (out / 'question.txt').stat().st_mtime
            and profile.get('task_id') == terminal.get('task_id')
            and profile.get('mission', {}).get('state') == terminal.get('state')):
        shutil.copy2(source, out / 'speed_profile.json')
        status = {'copied': True, 'reason': 'task_and_freshness_match'}
except (OSError, ValueError):
    pass
(out / 'profile_status.json').write_text(json.dumps(status))
PY
python3 -m tools_demo.evidence_audit "$out" "$question"
echo "EVIDENCE_DIRECTORY=$out"
exit "$result_code"
