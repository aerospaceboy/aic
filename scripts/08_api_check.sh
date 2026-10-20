#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/lib.sh"
load_ros
set -a
source "$YZBOT_LLM_ENV_FILE"
set +a
export ROS_DOMAIN_ID=93
python3 "$USB_ROOT/scripts/check_api.py" "$REPO" "$USB_ROOT/examples/练习题.txt" "$USB_ROOT/examples/规则1.txt"
