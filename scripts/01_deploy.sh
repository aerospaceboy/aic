#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/lib.sh"
bash "$USB_ROOT/scripts/00_system_check.sh"
cd "$USB_ROOT"
if [[ -f SHA256SUMS ]]; then
    sha256sum --quiet -c SHA256SUMS
else
    echo '当前为源码工作区；发布压缩包中的 SHA256SUMS 由打包脚本生成。'
fi
destination=${1:-"$HOME/yzbot_usb"}
[[ "$destination" = /* ]] || { echo '目标必须是绝对路径。' >&2; exit 2; }
if [[ "$destination" != "$USB_ROOT" ]]; then
    [[ ! -L "$destination" ]] || { echo '目标不能是符号链接。' >&2; exit 2; }
    if [[ -e "$destination" ]] && [[ -n $(find "$destination" -mindepth 1 -maxdepth 1 -print -quit) ]]; then
        echo "目标已包含文件，为避免覆盖而停止：$destination" >&2; exit 2
    fi
    mkdir -p "$destination"
    cp -a "$USB_ROOT/." "$destination/"
    # Frozen baselines are intentionally read-only. The new practice checkout
    # must be writable without changing the original archive/directory.
    chmod -R u+w "$destination"
fi
USB_ROOT=$(cd -- "$destination" && pwd)
WORKSPACE="$USB_ROOT/workspace"; REPO="$WORKSPACE/src/yzbot"
case "$(stat -f -c %T "$WORKSPACE")" in
    fuseblk|ntfs*|exfat*|vfat*|msdos*) echo '请复制到 Linux 持久化文件系统再构建，不在 NTFS/exFAT 中建立 colcon 符号链接。' >&2; exit 2 ;;
esac
source /opt/ros/humble/setup.bash
cd "$WORKSPACE"
colcon build --symlink-install --parallel-workers "${YZBOT_BUILD_WORKERS:-2}"
source install/setup.bash
python3 -m pytest -q -p no:cacheprovider -m 'not linter' src/yzbot/tools_demo/test src/yzbot/mybot/test
echo "源码构建和行为测试通过。"
echo "后续命令位置：$USB_ROOT/scripts/"
