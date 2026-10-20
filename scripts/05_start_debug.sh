#!/usr/bin/env bash
# Bounded-GUI debug entrypoint for the competition stack.
#
# Same stack as 05_start_competition.sh, but it launches the launch file
# directly so the operator can opt into the bounded Gazebo GUI explicitly.
# Kept separate from the competition entrypoint because that one also starts
# the CoStudio client and waits for MANUAL_PROBLEM_MODE=PASS.
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/lib.sh"
load_ros
export COSTUDIO_HEADLESS=False COSTUDIO_RVIZ=True COSTUDIO_DASHBOARD=True
export COSTUDIO_CAMERA_STREAMS=True COSTUDIO_SLAM_EVIDENCE=False
export YZBOT_GAZEBO_GUI_RENDER_RATE=${YZBOT_GAZEBO_GUI_RENDER_RATE:-60}
exec ros2 launch tools_demo competition_system.launch.py \
  headless:=False debug_gui:=True generate_question:=False \
  dashboard:=True moving_obstacle:=True \
  start_llm_server:=False use_llm:=False require_llm:=False "$@"
