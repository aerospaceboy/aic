#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/lib.sh"
[[ $(id -u) != 0 ]] || { echo '请以比赛桌面用户执行，不能把扩展装到 root HOME。' >&2; exit 2; }
if ! command -v costudio >/dev/null && ! command -v coStudio >/dev/null && ! command -v honeybee >/dev/null && [[ ! -x /opt/coStudio/honeybee ]]; then
    sudo apt-get install -y "$USB_ROOT"/vendor/linux/coStudio_*linux_amd64.deb
fi
source_dir="$USB_ROOT/costudio/extension"
# coStudio 0.4.0 只在启动时扫描 $HOME/.coStudio/extensions/<publisher.name>-<version>，
# 读取 package.json 的 main（dist/extension.js）；src/test/node_modules 都不会被加载。
[[ -f "$source_dir/package.json" ]] || { echo "缺少扩展清单：$source_dir/package.json" >&2; exit 2; }
[[ -f "$source_dir/dist/extension.js" ]] || { echo "缺少扩展构建产物：$source_dir/dist/extension.js（开发机先执行 npm ci && npm run build -- --mode production）" >&2; exit 2; }
version=$(sed -n 's/.*"version": *"\([^"]*\)".*/\1/p' "$source_dir/package.json" | head -n 1)
[[ -n "$version" ]] || { echo '无法从 package.json 读取 version' >&2; exit 2; }
extensions="$HOME/.coStudio/extensions"
target="$extensions/aerospaceboy.yzbot-competition-$version"
backup="$HOME/.local/share/yzbot/costudio-extension-backup-$(date +%Y%m%dT%H%M%S)"
mkdir -p "$extensions" "$backup"
for old in "$extensions"/aerospaceboy.yzbot-competition-*; do
    [[ -d "$old" && "$old" != "$target" ]] && mv -- "$old" "$backup/"
done
rm -rf -- "$target"
mkdir -p "$target"
cp -a "$source_dir/." "$target/"
grep -q '"main": *"\./dist/extension\.js"' "$target/package.json" \
    || { echo '扩展 package.json 的 main 不是 ./dist/extension.js，coStudio 将无法加载' >&2; exit 2; }
echo "扩展已安装到 $target（版本 $version）。"
echo '必须完全退出并重新打开 coStudio（扩展只在启动时扫描，刷新页面不足以加载新面板）。'
echo '打开后导入 costudio/YZBot比赛布局.json；四个面板应正常渲染，不再显示 “Unknown panel type”。'
echo '若仍显示 “Unknown panel type”，请确认该目录存在、coStudio 与安装脚本使用同一个用户。'
echo '原生Linux同机连接：coScene WebSocket，ws://127.0.0.1:21274。'
