#!/usr/bin/env bash
# CMake PROGRAMS/world installs may be copies even with --symlink-install.
# Run before launching or terminating any existing round.
verify_yzbot_runtime_sources() {
    local runtime_repo=${1:?repository required}
    local runtime_workspace=${2:?workspace required}
    local runtime_pair runtime_source runtime_installed
    local runtime_pairs=(
        'mybot/pick_place_moveit.py|mybot/lib/mybot/pick_place_moveit'
        'mybot_description/worlds/offic_room.world|mybot_description/share/mybot_description/worlds/offic_room.world'
    )
    for runtime_pair in "${runtime_pairs[@]}"; do
        runtime_source="$runtime_repo/${runtime_pair%%|*}"
        runtime_installed="$runtime_workspace/install/${runtime_pair#*|}"
        if ! cmp -s "$runtime_source" "$runtime_installed"; then
            echo "运行程序/场景未构建或与源码不一致：$runtime_installed" >&2
            echo "请在 $runtime_workspace 执行：colcon build --symlink-install --packages-select mybot mybot_description，然后重新 source install/setup.bash。" >&2
            return 2
        fi
    done
    echo 'runtime_source_consistency:PASS (mybot executable and Gazebo world)'
}
