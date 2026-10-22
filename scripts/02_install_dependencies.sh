#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/lib.sh"
bash "$USB_ROOT/scripts/00_system_check.sh"
sudo apt-get update
sudo apt-get install -y build-essential git curl gnupg python3-colcon-common-extensions python3-rosdep python3-pytest python3-yaml python3-requests python3-numpy python3-opencv libgazebo-dev qtbase5-dev x11-utils fonts-noto-cjk \
    ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-moveit ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-cv-bridge ros-humble-image-transport ros-humble-cartographer-ros ros-humble-rviz2 ros-humble-rmw-fastrtps-cpp ros-humble-rmw-cyclonedds-cpp
if [[ -z $(find /etc/ros/rosdep/sources.list.d -name '*.list' -print -quit 2>/dev/null) ]]; then sudo rosdep init; fi
rosdep update --rosdistro humble
rosdep install --from-paths "$REPO" --ignore-src --rosdistro humble -r -y
source /opt/ros/humble/setup.bash
if ! ros2 pkg prefix cobridge >/dev/null 2>&1; then
    shopt -s nullglob
    bridge_packages=("$USB_ROOT"/vendor/linux/ros-humble-cobridge_*.deb)
    if (( ${#bridge_packages[@]} == 0 )); then
        echo '缺少 coBridge：请准备匹配 Ubuntu 22.04/Humble 的安装包到 vendor/linux，再重新执行。' >&2
        exit 2
    fi
    sudo apt-get install -y "${bridge_packages[@]}"
fi
echo '已安装机器人依赖与 coBridge。首次安装依赖需要联网；不是完全离线系统镜像。'
