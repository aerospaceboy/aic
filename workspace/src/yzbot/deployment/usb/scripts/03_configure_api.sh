#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/lib.sh"
directory="$HOME/.config/yzbot"
mkdir -p "$directory"; chmod 700 "$directory"
printf 'OpenAI兼容根地址（含 /compatible-mode/v1，回车保留原配置）: '
read -r endpoint
printf '模型名（回车保留原配置 qwen3.8-max）: '
read -r model
read -r -s -p 'API Key（不回显）: ' secret; printf '\n'
[[ -n "$secret" ]] || { echo '未输入 Key；没有保存或修改。' >&2; exit 2; }
umask 077
if [[ -f "$directory/llm.env" ]]; then cp -p "$directory/llm.env" "$directory/llm.env.backup-$(date +%Y%m%dT%H%M%S)"; fi
printf 'export DASHSCOPE_API_KEY=%q\n' "$secret" > "$directory/llm.env"
unset secret
python3 - "$REPO/tools_demo/config/task_parser.yaml" "$endpoint" "$model" <<'PY'
import sys,yaml
from pathlib import Path
p=Path(sys.argv[1]);data=yaml.safe_load(p.read_text());params=data['competition_task_parser']['ros__parameters']
if sys.argv[2]:
    from urllib.parse import urlparse
    endpoint=sys.argv[2].rstrip('/')
    u=urlparse(endpoint)
    if u.scheme!='https' or not u.hostname or u.username or u.password or u.query or u.fragment:
        raise SystemExit('云 API 地址必须是无凭据、无查询参数的 https 根地址')
    params['base_url']=endpoint
if sys.argv[3]:params['model']=sys.argv[3]
params['llm_enable_thinking']=False
p.write_text(yaml.safe_dump(data,allow_unicode=True,sort_keys=False))
print('API地址和模型为非秘密配置；Key仅保存到当前用户私有目录。')
PY
source /opt/ros/humble/setup.bash
cd "$WORKSPACE"
colcon build --packages-select tools_demo
echo '配置后请执行 08_api_check.sh；不要把私有 llm.env 加入源码、Git、网盘作品或录屏。'
