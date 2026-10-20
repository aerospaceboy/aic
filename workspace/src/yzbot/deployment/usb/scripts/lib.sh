#!/usr/bin/env bash
# Shared environment for the native-Linux delivery, independent of VM user/IP.
set -eo pipefail
USB_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
WORKSPACE="$USB_ROOT/workspace"
REPO="$WORKSPACE/src/yzbot"
load_ros() {
    [[ -r /opt/ros/humble/setup.bash ]] || { echo '缺少 ROS 2 Humble。' >&2; return 2; }
    source /opt/ros/humble/setup.bash
    [[ -r "$WORKSPACE/install/setup.bash" ]] || { echo '先执行 01_deploy.sh 完成构建。' >&2; return 2; }
    source "$WORKSPACE/install/setup.bash"
    export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    export FASTRTPS_DEFAULT_PROFILES_FILE="$WORKSPACE/install/tools_demo/share/tools_demo/config/fastdds_udp.xml"
    export YZBOT_LLM_ENV_FILE=${YZBOT_LLM_ENV_FILE:-"$HOME/.config/yzbot/llm.env"}
}
