#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/lib.sh"
[[ $(uname -m) == x86_64 ]] || { echo '本入口只适用于x86_64。' >&2; exit 2; }
binary="$USB_ROOT/vendor/question_generator/TMSCQtest_x86_x64.bin"
chmod u+x "$binary"
mkdir -p "$USB_ROOT/runs"
if [[ ${1:-} == --practice ]]; then
    "$binary" | tee "$USB_ROOT/runs/practice-question-$(date +%Y%m%dT%H%M%S).txt"
else
    "$binary" 11470507 | tee "$USB_ROOT/runs/official-question-$(date +%Y%m%dT%H%M%S).txt"
fi
echo '只复制“题目:”后的题目正文到面板。此脚本不发布任务、不启动机器人。正式题目每30分钟变化，提交前与裁判核对。'
