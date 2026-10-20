#!/usr/bin/env bash
set -eo pipefail

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
repo=$(git -C "$script_dir" rev-parse --show-toplevel)
export HEADLESS=False
export GAZEBO_DEBUG_GUI=True
export YZBOT_GAZEBO_GUI_RENDER_RATE="${YZBOT_GAZEBO_GUI_RENDER_RATE:-15}"
export YZBOT_GAZEBO_GUI_WIDTH="${YZBOT_GAZEBO_GUI_WIDTH:-1100}"
export YZBOT_GAZEBO_GUI_HEIGHT="${YZBOT_GAZEBO_GUI_HEIGHT:-700}"

source "$repo/../../../scripts/00_env.sh"
exec bash "$script_dir/clean_launch.sh"
