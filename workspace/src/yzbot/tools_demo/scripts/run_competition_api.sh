#!/usr/bin/env bash
# Keep cloud credentials outside both the repository and command arguments.
set -eo pipefail
repo=$(cd -- "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
workspace=$(cd "$repo/../.." && pwd)
if [ -z "${DASHSCOPE_API_KEY:-}" ]; then
  credentials=${YZBOT_LLM_ENV_FILE:-"$HOME/.config/yzbot/llm.env"}
  if [ ! -r "$credentials" ]; then echo 'Missing private LLM credentials' >&2; exit 2; fi
  source "$credentials"
fi
if [ -z "${DASHSCOPE_API_KEY:-}" ]; then echo 'DASHSCOPE_API_KEY is unset' >&2; exit 2; fi
export DASHSCOPE_API_KEY
source /opt/ros/humble/setup.bash
source "$workspace/install/setup.bash"
source "$(dirname "${BASH_SOURCE[0]}")/ros_transport.sh"
configure_yzbot_transport "$workspace"
exec ros2 launch tools_demo competition_system.launch.py \
  start_llm_server:=False use_llm:=True require_llm:=True "$@"
