#!/usr/bin/env bash
# Build this snapshot in its own workspace, not on another version's overlay.
set -eo pipefail
version_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
exec env -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH -u COLCON_PREFIX_PATH \
    -u PYTHONPATH -u PYTHONHOME -u LD_LIBRARY_PATH -u ROS_DISTRO \
    -u ROS_VERSION -u ROS_PYTHON_VERSION -u GAZEBO_PLUGIN_PATH \
    -u GAZEBO_MODEL_PATH -u GAZEBO_RESOURCE_PATH \
    PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
    bash --noprofile --norc "$version_root/scripts/01_deploy.sh" "$version_root"
