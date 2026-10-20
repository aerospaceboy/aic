#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/lib.sh"
[[ $(id -u) != 0 ]] || { echo '请以比赛桌面用户执行，不能把扩展装到 root HOME。' >&2; exit 2; }
if ! command -v costudio >/dev/null && ! command -v coStudio >/dev/null && ! command -v honeybee >/dev/null && [[ ! -x /opt/coStudio/honeybee ]]; then
    sudo apt-get install -y "$USB_ROOT"/vendor/linux/coStudio_*linux_amd64.deb
fi
extensions="$HOME/.coStudio/extensions"
backup="$HOME/.local/share/yzbot/costudio-extension-backup-$(date +%Y%m%dT%H%M%S)"
mkdir -p "$extensions" "$backup"
for old in "$extensions"/aerospaceboy.yzbot-competition-*; do
    [[ -d "$old" ]] && mv -- "$old" "$backup/"
done
mkdir -p "$extensions/aerospaceboy.yzbot-competition-1.4.5"
cp -a "$USB_ROOT/costudio/extension/." "$extensions/aerospaceboy.yzbot-competition-1.4.5/"
echo '扩展安装完成，旧扩展已备份。关闭并重新打开CoStudio，再导入costudio/YZBot比赛布局.json。'
echo '原生Linux同机连接：coScene WebSocket，ws://127.0.0.1:21274。首次GUI兼容性仍需在U盘系统确认。'
