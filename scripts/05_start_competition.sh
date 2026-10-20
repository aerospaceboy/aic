#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/lib.sh"
load_ros
export YZBOT_GAZEBO_GUI_RENDER_RATE=${YZBOT_GAZEBO_GUI_RENDER_RATE:-60}
export COSTUDIO_HEADLESS=False COSTUDIO_DEBUG_GUI=True COSTUDIO_RVIZ=True COSTUDIO_DASHBOARD=True
export COSTUDIO_CAMERA_STREAMS=True COSTUDIO_SLAM_EVIDENCE=True
echo '系统启动后不会自动出题或移动；等待MANUAL_PROBLEM_MODE=PASS，再打开CoStudio填写正式题目与规则。'
bash "$REPO/tools_demo/scripts/start_manual_problem_mode.sh"
mkdir -p "$USB_ROOT/runs"
if command -v costudio >/dev/null; then
    setsid nohup env -u ELECTRON_RUN_AS_NODE costudio > "$USB_ROOT/runs/costudio-client.log" 2>&1 &
elif command -v coStudio >/dev/null; then
    setsid nohup env -u ELECTRON_RUN_AS_NODE coStudio > "$USB_ROOT/runs/costudio-client.log" 2>&1 &
elif command -v honeybee >/dev/null; then
    setsid nohup env -u ELECTRON_RUN_AS_NODE honeybee > "$USB_ROOT/runs/costudio-client.log" 2>&1 &
elif [[ -x /opt/coStudio/honeybee ]]; then
    setsid nohup env -u ELECTRON_RUN_AS_NODE /opt/coStudio/honeybee > "$USB_ROOT/runs/costudio-client.log" 2>&1 &
else
    echo '系统已就绪；CoStudio客户端未找到，请先执行04_install_costudio.sh或从应用菜单打开。'
fi
