#!/usr/bin/env bash
# Run this project's exact historical runtime, without changing its checkout.
set -eo pipefail
workspace=${1:?Usage: bash run_frozen48_gui.sh WORKSPACE [COMMIT] [--check]}
expected=${2:-b21f8adbb8e76d07164ed6c403cc0cf0d7feb8bf}
mode=${3:-run}
case "$mode" in run|--check) ;; *) echo "Invalid mode: $mode" >&2; exit 2 ;; esac
workspace=$(cd -- "$workspace" && pwd)
repo="$workspace/src/yzbot"
actual=$(git -C "$repo" rev-parse HEAD)
resolved=$(git -C "$repo" rev-parse "$expected^{commit}")
if [[ "$actual" != "$resolved" ]]; then
    echo "Frozen-version mismatch: actual=$actual expected=$resolved" >&2
    exit 1
fi
# Include untracked files: a frozen run must not silently import new code.
if [[ -n "$(git -C "$repo" status --porcelain)" ]]; then
    echo "Working tree is not clean; refusing a mixed-version campaign" >&2
    exit 1
fi
source /opt/ros/humble/setup.bash
source "$workspace/install/setup.bash"
cd -- "$workspace"
export DISPLAY=${DISPLAY:-:0}
export COSTUDIO_HEADLESS=False
export COSTUDIO_RVIZ=True
export STARTUP_TIMEOUT_SEC=360
export CLEAN_ONLY=True
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$workspace/install/tools_demo/share/tools_demo/config/fastdds_udp.xml"
export PYTHONUNBUFFERED=1
export POLICY_CASES
POLICY_CASES=$(python3 "$repo/tools_demo/scripts/gen48_questions.py" | cut -d' ' -f1 | paste -sd, -)
python3 -c 'import os; t=os.environ["POLICY_CASES"].split(","); assert len(t)==len(set(t))==48; print("48 distinct cases; Gazebo GUI=True; RViz=True", flush=True)'
echo "Frozen runtime: $actual; workspace: $workspace"
if [[ "$mode" == --check ]]; then
    exit 0
fi
if pgrep -f '[p]ython3 .*run_zone9_campaign.py|[p]ython3 .*full48' >/dev/null; then
    echo "An existing campaign is active; refusing a second one" >&2
    exit 1
fi
# Preserve the chosen version's runner/watchdog/scoring. Do not override budgets.
exec python3 "$repo/tools_demo/scripts/run_zone9_campaign.py"
