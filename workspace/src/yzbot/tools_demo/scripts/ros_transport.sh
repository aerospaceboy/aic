#!/usr/bin/env bash
# Source after ROS and the workspace, before creating any participant.
configure_yzbot_transport() {
    local transport_workspace=${1:?workspace required}
    local transport_config="$transport_workspace/install/tools_demo/share/tools_demo/config"
    export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
    export RMW_IMPLEMENTATION="${YZBOT_RMW:-rmw_cyclonedds_cpp}"
    case "$RMW_IMPLEMENTATION" in
        rmw_cyclonedds_cpp)
            [[ -r "$transport_config/cyclonedds_local.xml" ]] || return 2
            unset FASTRTPS_DEFAULT_PROFILES_FILE
            export CYCLONEDDS_URI="file://$transport_config/cyclonedds_local.xml"
            ;;
        rmw_fastrtps_cpp)
            [[ -r "$transport_config/fastdds_udp.xml" ]] || return 2
            unset CYCLONEDDS_URI
            export FASTRTPS_DEFAULT_PROFILES_FILE="$transport_config/fastdds_udp.xml"
            ;;
        *) echo "不支持的 YZBOT_RMW: $RMW_IMPLEMENTATION" >&2; return 2 ;;
    esac
    ros2 pkg prefix "$RMW_IMPLEMENTATION" >/dev/null 2>&1 || {
        echo "缺少 $RMW_IMPLEMENTATION；先安装项目通信依赖。" >&2
        return 2
    }
}
