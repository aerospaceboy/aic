#!/usr/bin/env bash
# CMake PROGRAMS/world installs may be copies even with --symlink-install.
# Run before launching or terminating any existing round.
verify_yzbot_runtime_sources() {
    local runtime_repo=${1:?repository required}
    local runtime_workspace=${2:?workspace required}
    local runtime_pair runtime_source runtime_installed runtime_package runtime_directory runtime_relative
    local runtime_pairs=(
        'mybot/pick_place_moveit.py|mybot/lib/mybot/pick_place_moveit'
        'mybot_description/worlds/offic_room.world|mybot_description/share/mybot_description/worlds/offic_room.world'
    )
    # Python package data can also be copied by --symlink-install. Check every
    # runtime tree/profile, not just the arm executable and world.
    for runtime_package in bot_navigation tools_demo; do
        for runtime_directory in config param launch maps; do
            [[ -d "$runtime_repo/$runtime_package/$runtime_directory" ]] || continue
            while IFS= read -r -d '' runtime_source; do
                runtime_relative=${runtime_source#"$runtime_repo/"}
                runtime_pairs+=("$runtime_relative|$runtime_package/share/$runtime_relative")
            done < <(find "$runtime_repo/$runtime_package/$runtime_directory" -type f \
                \( -name '*.xml' -o -name '*.yaml' -o -name '*.yml' -o -name '*.json' \
                   -o -name '*.py' -o -name '*.pgm' -o -name '*.png' \) -print0)
        done
    done
    for runtime_source in "$runtime_repo"/bot_navigation/*plugins.xml; do
        [[ -f "$runtime_source" ]] || continue
        runtime_relative=${runtime_source#"$runtime_repo/"}
        runtime_pairs+=("$runtime_relative|bot_navigation/share/$runtime_relative")
    done
    for runtime_pair in "${runtime_pairs[@]}"; do
        runtime_source="$runtime_repo/${runtime_pair%%|*}"
        runtime_installed="$runtime_workspace/install/${runtime_pair#*|}"
        if ! cmp -s "$runtime_source" "$runtime_installed"; then
            echo "运行程序/场景未构建或与源码不一致：$runtime_installed" >&2
            echo "请在 $runtime_workspace 执行：colcon build --symlink-install --packages-select bot_navigation tools_demo mybot mybot_description，然后重新 source install/setup.bash。" >&2
            return 2
        fi
    done
    # A completed build must not be paired with an older installed library.
    for runtime_relative in libdense_dynamic_controller.so libdynamic_prediction_layer.so \
        libdynamic_prediction_path_clear.so libdense_patrol.so; do
        runtime_source="$runtime_workspace/build/bot_navigation/$runtime_relative"
        runtime_installed="$runtime_workspace/install/bot_navigation/lib/$runtime_relative"
        if [[ -f "$runtime_source" ]] && ! cmp -s "$runtime_source" "$runtime_installed"; then
            echo "已构建插件与安装插件不一致：$runtime_installed；请重新构建 bot_navigation。" >&2
            return 2
        fi
    done
    echo 'runtime_source_consistency:PASS (program, world, navigation profiles/trees and installed plugins)'
}
