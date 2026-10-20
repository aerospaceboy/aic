#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/lib.sh"
source /etc/os-release
printf '系统: %s\n架构: %s\n用户: %s\n' "$PRETTY_NAME" "$(uname -m)" "$(id -un)"
df -h "$HOME"
findmnt -T "$HOME" -o TARGET,SOURCE,FSTYPE,OPTIONS
[[ "$ID" == ubuntu && "$VERSION_ID" == 22.04 ]] || { echo '本交付仅验证 Ubuntu 22.04；不要在其他发行版强行安装 Humble 包。' >&2; exit 2; }
[[ $(uname -m) == x86_64 ]] || { echo '当前机器人交付仅验证 x86_64；ARM 出题器不代表机器人栈已支持 ARM。' >&2; exit 2; }
[[ -r /opt/ros/humble/setup.bash ]] || { echo '缺少 ROS 2 Humble，参考部署说明先安装，随后重跑检查。' >&2; exit 2; }
source /opt/ros/humble/setup.bash
printf 'ROS_DISTRO=%s\n桌面 DISPLAY=%s\n' "$ROS_DISTRO" "${DISPLAY:-未设置}"
for pkg in gazebo_ros nav2_bringup moveit_ros_move_group rviz2 cobridge; do
    ros2 pkg prefix "$pkg" 2>/dev/null || echo "待安装: $pkg"
done
echo '注意：系统检查不能证明 Live USB 可持久化。关机重启后必须核对文件仍存在。'
