#!/bin/bash
set -eo pipefail

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
repo=$(git -C "$script_dir" rev-parse --show-toplevel)
workspace=$(cd "$repo/../.." && pwd)

LAUNCH_PID_FILE=/tmp/yzbot_competition_system.pid

if [ -s "$LAUNCH_PID_FILE" ]; then
    read -r launch_pid < "$LAUNCH_PID_FILE"
    if [[ "$launch_pid" =~ ^[0-9]+$ ]] && kill -0 "$launch_pid" 2>/dev/null; then
        kill -TERM -- "-$launch_pid" 2>/dev/null || kill -TERM "$launch_pid"
        sleep 1
        kill -KILL -- "-$launch_pid" 2>/dev/null || true
    fi
    rm -f "$LAUNCH_PID_FILE"
fi

while read -r orphan_pid; do
    [ -n "$orphan_pid" ] && [ "$orphan_pid" != "$$" ] || continue
    kill -9 "$orphan_pid" 2>/dev/null
done < <(
    ps -eo pid=,args= | awk '
        /[r]os2 launch tools_demo competition_system\.launch\.py/ ||
        /[g]azebo_ros\/spawn_entity\.py/ {print $1}'
)
# velocity_selector has no --params-file argument, so the launch_params
# sweep below does not reap it after its launch parent exits. Two surviving
# selectors then publish into the same velocity chain on the next launch.
# Match the project executable, not a generic Python or shell process.
while read -r selector_pid; do
    [ -n "$selector_pid" ] && [ "$selector_pid" != "$$" ] || continue
    kill -TERM "$selector_pid" 2>/dev/null || true
done < <(
    ps -eo pid=,args= | awk '
        /[t]ools_demo\/lib\/tools_demo\/velocity_selector([[:space:]]|$)/ {
            print $1
        }'
)

for name in gzserver gzclient robot_state_pub static_transfor \
    spawn_entity.py cartographer_no cartographer_oc component_conta \
    move_group moveit_ros_move_group llama-server; do
    pkill -9 -x "$name" 2>/dev/null || true
done
for pid in $(pgrep -f "launch_param[s]_" || true); do
    [ "$pid" = "$$" ] && continue
    kill -9 "$pid" 2>/dev/null || true
done
sleep 2
# Local UDP transport needs no shared-memory deletion. Never remove other
# desktop applications' semaphores or locks during robot teardown.

if [ "${CLEAN_ONLY:-False}" = "True" ]; then
    exit 0
fi

export DISPLAY=${DISPLAY:-:0}
export QT_X11_NO_MITSHM=1
export YZBOT_GAZEBO_GUI_RENDER_RATE="${YZBOT_GAZEBO_GUI_RENDER_RATE:-15}"
export YZBOT_GAZEBO_GUI_WIDTH="${YZBOT_GAZEBO_GUI_WIDTH:-1100}"
export YZBOT_GAZEBO_GUI_HEIGHT="${YZBOT_GAZEBO_GUI_HEIGHT:-700}"
cd "$workspace"
source /opt/ros/humble/setup.bash
source install/setup.bash
source "$script_dir/ros_transport.sh"
configure_yzbot_transport "$workspace"
rm -f /tmp/run.log

headless_mode="${HEADLESS:-False}"
debug_gui_mode="${GAZEBO_DEBUG_GUI:-True}"
setsid nohup ros2 launch tools_demo competition_system.launch.py \
  headless:="$headless_mode" debug_gui:="$debug_gui_mode" \
  generate_question:=False use_llm:=False require_llm:=False \
  run_acceptance:=False moving_obstacle:=True dashboard:=True \
  start_llm_server:=False > /tmp/run.log 2>&1 &
launch_pid=$!
echo "$launch_pid" > "$LAUNCH_PID_FILE"
echo "LAUNCHED_PID=$launch_pid"
