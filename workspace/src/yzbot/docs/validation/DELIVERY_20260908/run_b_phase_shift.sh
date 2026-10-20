#!/usr/bin/env bash
# Motion-only benchmark. Every invocation retains a fresh evidence directory.
set -eo pipefail
repo="/home/ros/dev_ws/src/yzbot"
workspace=$(cd "$repo/../.." && pwd)
question=${1:?Usage: bash run_ordered_case.sh 'Move ...' [output-root]}
root=${2:-"$workspace/experiments/ordered-cases"}
if pgrep -x gzserver >/dev/null; then
  echo 'STALE_SIM_DETECTED: stop the owning launch before this run' >&2
  exit 2
fi
mkdir -p "$root"
out=$(mktemp -d "$root/$(date -u +%Y%m%dT%H%M%SZ)-XXXXXX")
printf '%s\n' "$out" > "$root/latest-run.txt"
printf '%s\n' "$question" > "$out/question.txt"
git -C "$repo" rev-parse HEAD > "$out/commit.txt"
git -C "$repo" status --short > "$out/worktree-status.txt"
git -C "$repo" diff HEAD --binary > "$out/worktree.patch"
find "$repo/tools_demo/tools_demo" "$repo/tools_demo/config" "$repo/bot_navigation/config" \
  -type f ! -path '*/__pycache__/*' -print0 | sort -z | xargs -0 sha256sum > "$out/source-config.sha256"
printf '%s\n' 'motion-only; headless=true; LLM=false; dashboard=false; moving_obstacle=true' > "$out/mode.txt"
cd "$workspace"
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$workspace/install/tools_demo/share/tools_demo/config/fastdds_udp.xml"
export DISPLAY=${DISPLAY:-:0} QT_X11_NO_MITSHM=1
cleanup() {
  for pid in ${stats_pid:-} ${launch_pid:-}; do kill -TERM -- "-$pid" 2>/dev/null || true; done
  sleep 3
  for pid in ${stats_pid:-} ${launch_pid:-}; do kill -KILL -- "-$pid" 2>/dev/null || true; done
}
trap cleanup EXIT
setsid ros2 launch tools_demo competition_system.launch.py headless:=True generate_question:=False use_llm:=False require_llm:=False start_llm_server:=False moving_obstacle:=True dashboard:=False slam_evidence:=False run_acceptance:=False > "$out/launch.log" 2>&1 &
launch_pid=$!
ready=false
for i in $(seq 1 90); do
  if grep -aq 'persistent pick-place serving' "$out/launch.log"; then ready=true; break; fi
  if ! kill -0 "$launch_pid" 2>/dev/null; then break; fi
  sleep 2
done
if [ "$ready" != true ]; then echo STARTUP_FAILED | tee "$out/status.txt"; exit 1; fi
sleep 10
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
