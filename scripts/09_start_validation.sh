#!/usr/bin/env bash
# Full competition backend for optimization runs, without opening a CoStudio client.
set -eo pipefail
validation_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
if (( $# )); then
    echo '用法：bash scripts/09_start_validation.sh（不接收参数）' >&2
    echo '可设 YZBOT_VALIDATION_HEADLESS=True、YZBOT_VALIDATION_RVIZ=False。' >&2
    exit 2
fi
if pgrep -x gzserver >/dev/null || pgrep -x gzclient >/dev/null \
    || ps -eo args= | awk '/[r]os2 launch tools_demo competition_system\.launch\.py/ {found=1} END {exit !found}'; then
    echo '已有机器人/Gazebo版本运行；请先保存结果并正常结束该轮。' >&2
    exit 2
fi
[[ -r "$validation_root/workspace/install/setup.bash" ]] || {
    echo '缺少构建，请先执行部署本版本.sh。' >&2
    exit 2
}
exec env -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH -u COLCON_PREFIX_PATH \
    -u PYTHONPATH -u PYTHONHOME -u LD_LIBRARY_PATH -u ROS_DISTRO \
    -u ROS_VERSION -u ROS_PYTHON_VERSION -u GAZEBO_PLUGIN_PATH \
    -u GAZEBO_MODEL_PATH -u GAZEBO_RESOURCE_PATH -u CYCLONEDDS_URI \
    -u FASTRTPS_DEFAULT_PROFILES_FILE \
    PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
    COSTUDIO_HEADLESS="${YZBOT_VALIDATION_HEADLESS:-False}" \
    COSTUDIO_DEBUG_GUI=True COSTUDIO_RVIZ="${YZBOT_VALIDATION_RVIZ:-True}" \
    COSTUDIO_DASHBOARD=True COSTUDIO_CAMERA_STREAMS=True \
    COSTUDIO_SLAM_EVIDENCE=True \
    YZBOT_SKIP_LLM_PREFLIGHT="${YZBOT_VALIDATION_SKIP_LLM_PREFLIGHT:-False}" \
    YZBOT_GAZEBO_GUI_RENDER_RATE="${YZBOT_GAZEBO_GUI_RENDER_RATE:-60}" \
    bash --noprofile --norc \
    "$validation_root/workspace/src/yzbot/tools_demo/scripts/start_manual_problem_mode.sh"
