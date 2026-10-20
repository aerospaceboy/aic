#!/usr/bin/env bash
# Start the current competition system for the legacy coStudio 1.4.3 panel.
set -eo pipefail

# A second manual start would tear down the first round halfway through its
# launch and leave CoStudio attached to a changing ROS graph.
exec 9>"${TMPDIR:-/tmp}/yzbot_manual_problem_mode.lock"
if ! flock -n 9; then
    echo "手动 CoStudio 模式正在启动；请等待现有启动完成。" >&2
    exit 1
fi
# HARNESS PATCH for the sway-bisect experiment (not part of 7ca1ba3).
# The readiness loop runs many `ros2` CLI calls; the first one auto-starts the
# per-user ROS 2 daemon, and a daemon spawned while fd 9 is open inherits the
# lock for the rest of the session. Every later start then dies with
# '手动 CoStudio 模式正在启动' even though nothing is running (observed
# 2026-10-03: lock held by ros2-daemon pid 261890). Start the daemon here with
# fd 9 closed so it can never hold the lock. Harness only - no effect on robot
# behaviour, controller tuning, physics or acceptance gates.

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
repo=$(git -C "$script_dir" rev-parse --show-toplevel)
workspace=$(cd -- "$repo/../.." && pwd)
project_root=$(cd -- "$workspace/.." && pwd)
source "$script_dir/verify_runtime_sources.sh"
verify_yzbot_runtime_sources "$repo" "$workspace"
run_dir="$project_root/runs/manual-$(date +%Y%m%dT%H%M%S)"
if [[ -n "${YZBOT_LLM_ENV_FILE:-}" ]]; then
    credentials=$YZBOT_LLM_ENV_FILE
elif [[ -r "$HOME/.config/yzbot/llm.env" ]]; then
    credentials="$HOME/.config/yzbot/llm.env"
else
    # Keep the legacy manual-mode credential location working during migration.
    credentials="$HOME/.config/yzbot/qwen.env"
fi

# Resolve the active workspace directly; do not rely on a portable-package
# helper in the user's home directory or a workspace symlink.
source /opt/ros/humble/setup.bash
source "$workspace/install/setup.bash"
source "$script_dir/ros_transport.sh"
configure_yzbot_transport "$workspace"
export DISPLAY=${DISPLAY:-:0}
if [[ -z "${XAUTHORITY:-}" ]]; then
    desktop_auth=$(find "/run/user/$(id -u)" -maxdepth 1 \
        -name '.mutter-Xwaylandauth.*' -type f 2>/dev/null | sort | tail -1)
    if [[ -n "$desktop_auth" ]]; then export XAUTHORITY="$desktop_auth"; fi
fi
set -u

# One-key start opens Gazebo GUI and RViz by default for operator demos.
# Set COSTUDIO_RVIZ=False to skip RViz and/or COSTUDIO_HEADLESS=True to skip
# the Gazebo client (scoring / low-load runs).
start_rviz="${COSTUDIO_RVIZ:-True}"
rviz_config="$repo/bot_navigation/rviz/navigation.rviz"
case "${start_rviz,,}" in
    true|1|yes|on)
        if ! command -v rviz2 >/dev/null 2>&1 || [[ ! -r "$rviz_config" ]]; then
            echo "RViz2 或导航配置不可用；安装 RViz2 或不要设置 COSTUDIO_RVIZ=True。" >&2
            exit 1
        fi
        ;;
esac

if ! command -v xdpyinfo >/dev/null 2>&1 || ! DISPLAY="${DISPLAY:-:0}" timeout 4 xdpyinfo >/dev/null 2>&1; then
    echo "Gazebo 图形桌面不可用：${DISPLAY:-:0}" >&2
    echo "请先登录当前用户的 Ubuntu 图形桌面，再重试。" >&2
    exit 1
fi
if [[ ! -r "$credentials" ]]; then
    echo "缺少私有大模型凭据：$credentials" >&2
    echo "先运行 scripts/03_configure_api.sh，或设置 YZBOT_LLM_ENV_FILE。" >&2
    exit 1
fi
set -a
source "$credentials"
set +a
if [[ -z "${DASHSCOPE_API_KEY:-}" ]]; then
    echo "凭据文件未设置 DASHSCOPE_API_KEY：$credentials" >&2
    exit 1
fi
if ! ros2 pkg prefix cobridge >/dev/null 2>&1; then
    echo "未安装 coBridge；请先安装 ros-humble-cobridge。" >&2
    exit 1
fi

mkdir -p "$run_dir"
echo "停止上一轮竞赛进程..."
CLEAN_ONLY=True bash "$script_dir/clean_launch.sh"
if pgrep -x cobridge >/dev/null 2>&1; then
    pkill -TERM -x cobridge
    for _ in {1..20}; do
        pgrep -x cobridge >/dev/null 2>&1 || break
        sleep 0.25
    done
fi

# The previous round's gzserver / Nav2 / mission processes can outlive the
# teardown by a few seconds.  Launching into that overlap made the new
# collision monitor miss its lifecycle handshake, which left
# /cmd_vel_collision_safe silent, so the safety limiter never published
# and the base stayed frozen for the whole round (round BC1 2026-09-09:
# 0.00 m/s for 276 s, pickup_navigation_failed).
settle_deadline=$((SECONDS + 25))
while (( SECONDS < settle_deadline )); do
    if ! pgrep -f \
        "gzserver|gzclient|competition_system\.launch\.py|competition_navigation\.launch\.py|mission\.launch\.py|collision_monitor|controller_server|planner_server|mission_coordinator" \
        >/dev/null 2>&1; then
        break
    fi
    sleep 0.5
done
# YZBOT_SKIP_OVERLAP_GUARD=1 bypasses this check. The pgrep pattern can match
# this script's own cleanup helpers (a pkill whose command line carries the
# pattern) even when no robot process is left, which blocked a fresh start on a
# just-rebooted machine (2026-10-15: refusal with gzserver=0 and rviz2=0). The
# guard stays active by default; bypass it only after checking by hand.
if [ "${YZBOT_SKIP_OVERLAP_GUARD:-0}" != "1" ] \
    && pgrep -f "[g]zserver|[c]ollision_monitor|[m]ission_coordinator" >/dev/null; then
    echo "旧机器人进程未退出；禁止重叠启动。" >&2
    exit 1
fi
# Select the daemon only AFTER the previous robot participants have exited.
# Close fd 9 so a detached daemon cannot retain the manual-start lock.
9>&- timeout -k 2 20 ros2 daemon stop >/dev/null 2>&1 || true
9>&- timeout -k 2 20 ros2 daemon start >/dev/null 2>&1 || true
sleep 3
printf 'ROS transport: %s; domain=%s\n' "$RMW_IMPLEMENTATION" "$ROS_DOMAIN_ID" | tee "$run_dir/transport.txt"

# The two coStudio vision panels are part of the required display, so the
# camera feeds are on by default. Set COSTUDIO_CAMERA_STREAMS=False for a
# bandwidth-lean session that shows telemetry only.
camera_streams="${COSTUDIO_CAMERA_STREAMS:-True}"
setsid nohup ros2 launch tools_demo costudio_bridge.launch.py \
    address:=0.0.0.0 port:=21274 camera_streams:="$camera_streams" \
    9>&- \
    >"$run_dir/cobridge.log" 2>&1 &
bridge_pid=$!
setsid nohup bash "$script_dir/run_competition_api.sh" \
    headless:="${COSTUDIO_HEADLESS:-False}" \
    debug_gui:="${COSTUDIO_DEBUG_GUI:-True}" \
    generate_question:=False dashboard:="${COSTUDIO_DASHBOARD:-False}" \
    moving_obstacle:=True slam_evidence:="${COSTUDIO_SLAM_EVIDENCE:-True}" run_acceptance:=False \
    slam_delay_sec:="${COSTUDIO_SLAM_DELAY_SEC:-100.0}" \
    9>&- \
    >"$run_dir/launch.log" 2>&1 &
launch_pid=$!
printf '%s\n' "$bridge_pid" >"$run_dir/cobridge.pid"
printf '%s\n' "$launch_pid" >"$run_dir/launch.pid"

# Start RViz right away (parallel with Gazebo / Nav2 bringup). Waiting for
# the full readiness gate made the window appear minutes late. RViz only
# needs /robot_description + use_sim_time and fills TF/map as they arrive.
case "${start_rviz,,}" in
    true|1|yes|on)
        if ! pgrep -x rviz2 >/dev/null 2>&1; then
            DISPLAY="${DISPLAY:-:0}" setsid nohup rviz2 \
                -d "$rviz_config" --ros-args -p use_sim_time:=true \
                9>&- >"$run_dir/rviz.log" 2>&1 &
            printf '%s\n' "$!" >"$run_dir/rviz.pid"
            echo "RViz: 提前启动导航视图（与仿真并行加载）"
        fi
        ;;
esac

# HARNESS PATCH for the sway bisect experiment (not part of 7ca1ba3).
# 240 s was tuned headless. With the Gazebo GUI + RViz + the coStudio bridge
# plus the 200 Hz ground-truth probe the graph needs longer; a measured GUI
# cycle reached a fully healthy graph (all lifecycle nodes active, TF
# connected) right at the old deadline. 330 s still fits inside the runner's
# own 420 s restart budget. Harness only.
deadline=$((SECONDS + ${STARTUP_TIMEOUT_SEC:-330}))
ready_checks=0
while (( SECONDS < deadline )); do
    if grep -aqE 'process has died|failed to send response: client will not receive response' "$run_dir/launch.log" "$run_dir/cobridge.log"; then
        echo "启动子进程故障；禁止出题。日志：$run_dir/launch.log" >&2
        exit 1
    fi
    if ! kill -0 "$launch_pid" 2>/dev/null; then
        echo "主启动进程提前退出；日志：$run_dir/launch.log" >&2
        exit 1
    fi
    if ! kill -0 "$bridge_pid" 2>/dev/null; then
        echo "coBridge 提前退出；查看 $run_dir/cobridge.log" >&2
        exit 1
    fi
    nodes=$(timeout 8 ros2 node list 2>/dev/null || true)
    status_info=$(timeout 8 ros2 topic info /competition/status 2>/dev/null || true)
    chunk_info=$(timeout 8 ros2 topic info -v \
        /competition/task/problem_chunk 2>/dev/null || true)
    # Readiness probes must not call a service on the node they probe.
    #
    # These five used `timeout -k 2 8 ros2 lifecycle get /<node>`. That opens a
    # get_state client against the node itself and has two failure modes, both
    # already met in this project:
    #   * the CLI is killed while the call is in flight, so the node's
    #     send_response fails and rclcpp throws out of its executor -
    #     'terminate called after throwing rclcpp::exceptions::RCLError:
    #     failed to send response: client will not receive response'. The node
    #     aborts with SIGABRT and exit -6. nav2_collision_monitor died exactly
    #     this way in 3 of the last 12 starts (2026-10-04); this gate then
    #     waits forever for a node that is already gone, reports
    #     '启动超时，未就绪', never submits the problem, and the robot simply
    #     stands still - which reads as the system being stuck. The same
    #     signature appears in 177 of 300 run logs.
    #   * the CLI hangs and SIGTERM does not reap it (see the /amcl note below).
    #
    # `ros2 action list` and `ros2 topic info` only read the graph; they never
    # open a service against the target, so probing can no longer break what it
    # probes. The replacements carry the same meaning: an action server exists
    # only while its lifecycle node is active, /map is only published by an
    # active map_server, and the collision chain's output topic is only
    # published by an active collision_monitor.
    actions=$(timeout 8 ros2 action list 2>/dev/null || true)
    collision_out=$(timeout 8 ros2 topic info \
        /cmd_vel_collision_safe 2>/dev/null || true)
    safe_out=$(timeout 8 ros2 topic info /cmd_vel_safe \
        2>/dev/null || true)
    map_info=$(timeout 8 ros2 topic info /map 2>/dev/null || true)
    monitor_state=$(
        grep -Fxq /collision_monitor <<<"$nodes" \
            && grep -Eq "Publisher count: [1-9]" <<<"$collision_out" \
            && echo active || echo unknown
    )
    navigator_state=$(
        grep -Fxq /navigate_to_pose <<<"$actions" \
            && echo active || echo unknown
    )
    controller_state=$(
        grep -Fxq /follow_path <<<"$actions" \
            && echo active || echo unknown
    )
    planner_state=$(
        grep -Fxq /compute_path_to_pose <<<"$actions" \
            && echo active || echo unknown
    )
    # Localization stack lives in a separate lifecycle manager. When it
    # fails to activate, the global costmap never receives the static map
    # layer, the navigator defers every goal forever, and the round ends
    # as a 0/5 watchdog abort. Observed 20261001T174220 round 27 (2rB_3bC):
    # map_server logged a /map_server/change_state response timeout,
    # 'Activating map_server' never appeared, and 367 'no map received'
    # warnings followed. Require the map to actually be published.
    map_server_state=$(
        grep -Eq "Publisher count: [1-9]" <<<"$map_info" \
            && echo active || echo unknown
    )
    # NOTE: /amcl is deliberately NOT required. map->odom is published by
    # the static competition_map_to_odom broadcaster, and amcl's get_state
    # service can stop answering (observed: 'failed to send response to
    # /amcl/get_state (timeout)'). The ros2 CLI then hangs and the SIGTERM
    # from `timeout` does not reap that Python process, so the readiness
    # loop blocked forever and kept this script's launch flock held.
    # The coStudio problem panel proves a successful parse from this topic;
    # gate readiness on it so the display chain is live before submission.
    parse_info=$(timeout 8 ros2 topic info -v \
        /competition/task/parse_result 2>/dev/null || true)
    # The simulation itself must be proved alive. Previously the gate accepted
    # a stack whose `gazebo` process had already died (observed 20261012T130052:
    # gazebo aborted with an RCLError ~60 s into the launch, the gate still
    # printed MANUAL_PROBLEM_MODE=PASS, and the base could never move).
    clock_info=$(timeout 8 ros2 topic info /clock 2>/dev/null || true)
    scan_info=$(timeout 8 ros2 topic info /scan 2>/dev/null || true)
    simulation_state=$(
        grep -Eq "Publisher count: [1-9]" <<<"$clock_info" \
            && grep -Eq "Publisher count: [1-9]" <<<"$scan_info" \
            && echo active || echo unknown
    )
    processes_ready=True
    for process_pattern in \
        "[g]zserver" \
        "[c]ollision_monitor" \
        "tools_demo/task_parser.*competition_task_parser" \
        "tools_demo/mission_coordinator.*competition_mission_coordinator" \
        "mybot/pick_place_moveit.*competition_pick_place" \
        "tools_demo/legacy_costudio_status.*legacy_costudio_status"; do
        if ! pgrep -f "$process_pattern" >/dev/null 2>&1; then
            processes_ready=False
            break
        fi
    done
    if grep -Fxq /competition_task_parser <<<"$nodes" \
        && grep -Fxq /competition_mission_coordinator <<<"$nodes" \
        && grep -Fxq /competition_pick_place <<<"$nodes" \
        && grep -Fxq /collision_monitor <<<"$nodes" \
        && grep -Fxq /legacy_costudio_status <<<"$nodes" \
        && grep -Fxq /cobridge <<<"$nodes" \
        && grep -Eq "Publisher count: [1-9]" <<<"$status_info" \
        && grep -Fq "Node name: competition_task_parser" <<<"$chunk_info" \
        && grep -q "^active" <<<"$monitor_state" \
        && grep -q "^active" <<<"$navigator_state" \
        && grep -q "^active" <<<"$controller_state" \
        && grep -q "^active" <<<"$planner_state" \
        && grep -q "^active" <<<"$map_server_state" \
        && grep -Eq "Publisher count: [1-9]" <<<"$collision_out" \
        && grep -Eq "Publisher count: [1-9]" <<<"$safe_out" \
        && grep -Fq "Node name: competition_task_parser" <<<"$parse_info" \
        && grep -q "^active" <<<"$simulation_state" \
        && [[ "$processes_ready" == True ]]; then
        ready_checks=$((ready_checks + 1))
    else
        ready_checks=0
    fi
    # DDS can retain departed nodes briefly.  Require three consecutive
    # observations of live, current-run processes before accepting a task.
    if (( ready_checks >= 3 )); then
        if ! 9>&- timeout -k 2 15 ros2 run tools_demo simulation_probe \
            >>"$run_dir/simulation-probe.jsonl" 2>&1; then
            ready_checks=0
            sleep 2
            continue
        fi
        # ROS graph discovery alone does not prove the HTTP submission or
        # websocket listener works. Never tell the video script to publish
        # into an endpoint which has not actually opened.
        if ! YZBOT_CHECK_DASHBOARD="${COSTUDIO_DASHBOARD:-False}" python3 - <<'PY'
import json
import os
import socket
import urllib.request
with socket.create_connection(("127.0.0.1", 21274), timeout=3):
    pass
if os.environ["YZBOT_CHECK_DASHBOARD"].lower() in {"true", "1", "yes", "on"}:
    with urllib.request.urlopen("http://127.0.0.1:8080/api/state", timeout=3) as reply:
        state = json.load(reply)
    assert state.get("mission", {}).get("state") == "idle", "mission not idle"
PY
        then
            ready_checks=0
            sleep 2
            continue
        fi
        case "${start_rviz,,}" in
            true|1|yes|on)
                # RViz is started at launch begin; only revive it if it died.
                if ! pgrep -x rviz2 >/dev/null 2>&1; then
                    DISPLAY="${DISPLAY:-:0}" setsid nohup rviz2 \
                        -d "$rviz_config" --ros-args -p use_sim_time:=true \
                        9>&- >"$run_dir/rviz.log" 2>&1 &
                    printf '%s\n' "$!" >"$run_dir/rviz.pid"
                    echo "RViz: 就绪后补启导航视图"
                else
                    echo "RViz: 导航视图（map 坐标系，仿真时间）"
                fi
                ;;
        esac
        # Prove the model path before declaring ready. The credential file is
        # only checked for presence earlier; a revoked key, a renamed model or
        # an unreachable endpoint would otherwise surface only when the
        # operator clicks submit, i.e. after the round clock has started.
        if [[ "${YZBOT_SKIP_LLM_PREFLIGHT:-False}" =~ ^([Tt]rue|1|[Yy]es|[Oo]n)$ ]]; then
            echo "大模型预检: 已按 YZBOT_SKIP_LLM_PREFLIGHT 跳过"
        else
            llm_preflight_log="$run_dir/llm-preflight.log"
            if ROS_DOMAIN_ID=93 timeout 90 python3 \
                    "$project_root/scripts/check_api.py" \
                    "$repo" "$project_root/examples/练习题.txt" \
                    "$project_root/examples/规则1.txt" \
                    >"$llm_preflight_log" 2>&1; then
                echo "大模型预检: 通过 $(grep -oE '用时 [0-9.]+ 秒' \
                    "$llm_preflight_log" | head -1)"
            else
                echo "启动未就绪：大模型预检失败，禁止出题。" >&2
                echo "原因见 $llm_preflight_log（常见：密钥失效或额度用尽、" >&2
                echo "model 或 base_url 不对、网络不通）。" >&2
                echo "修好后执行 scripts/08_api_check.sh 复核；离线练习可设" >&2
                echo "YZBOT_SKIP_LLM_PREFLIGHT=True 跳过本项。" >&2
                echo "查看 $run_dir/launch.log 和 $run_dir/cobridge.log；重启前请保留失败日志。" >&2
                exit 1
            fi
        fi
        vm_ip=$(hostname -I | awk '{print $1}')
        echo "MANUAL_PROBLEM_MODE=PASS"
        echo "CoStudio（本机）: ws://127.0.0.1:21274"
        echo "CoStudio（其他电脑）: ws://$vm_ip:21274"
        echo "在旧版“题目与大模型解析”面板填写题目和映射规则后提交。"
        if [[ "${COSTUDIO_DASHBOARD:-False}" =~ ^([Tt]rue|1|[Yy]es|[Oo]n)$ ]]; then
            echo "网页状态面板: http://$vm_ip:8080"
        else
            echo "网页面板未启用；需要时设置 COSTUDIO_DASHBOARD=True 后启动。"
        fi
        echo "日志: $run_dir/launch.log"
        exit 0
    fi
    sleep 2
done

echo "启动超时，未就绪；禁止出题。bt_navigator=$navigator_state controller=$controller_state planner=$planner_state simulation=$simulation_state；必须同时通过真实时钟/扫描探针。" >&2
echo "查看 $run_dir/launch.log 和 $run_dir/cobridge.log；重启前请保留失败日志。" >&2
exit 1
