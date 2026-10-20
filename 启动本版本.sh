#!/usr/bin/env bash
# Snapshot-only convenience wrapper. Never tear down another version.
set -eo pipefail
version_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
if pgrep -x gzserver >/dev/null || pgrep -x gzclient >/dev/null \
    || ps -eo args= | awk '/[r]os2 launch tools_demo competition_system\.launch\.py/ {found=1} END {exit !found}'; then
    echo '已有机器人/Gazebo版本运行；请先正常结束它，本入口不会替你清理。' >&2
    exit 2
fi
[[ -r "$version_root/workspace/install/setup.bash" ]] || {
    echo "请先执行：bash $version_root/部署本版本.sh" >&2
    exit 2
}
exec env -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH -u COLCON_PREFIX_PATH \
    -u PYTHONPATH -u PYTHONHOME -u LD_LIBRARY_PATH -u ROS_DISTRO \
    -u ROS_VERSION -u ROS_PYTHON_VERSION -u GAZEBO_PLUGIN_PATH \
    -u GAZEBO_MODEL_PATH -u GAZEBO_RESOURCE_PATH -u CYCLONEDDS_URI \
    -u FASTRTPS_DEFAULT_PROFILES_FILE \
    PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
    bash --noprofile --norc "$version_root/scripts/05_start_competition.sh" "$@"
