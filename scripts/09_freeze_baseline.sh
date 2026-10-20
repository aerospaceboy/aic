#!/usr/bin/env bash
# Freeze a verified source delivery; never overwrite a previous baseline.
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/lib.sh"
out=${1:-"$HOME/桌面/224/基准版本"}
name=${2:-"YZBot-224-FROZEN-$(date +%Y%m%dT%H%M%S)"}
evidence=${3:-}
[[ "$out" = /* && "$name" =~ ^YZBot-224-FROZEN-[0-9T-]+$ ]] || {
    echo '输出须为绝对目录，版本名须为 YZBot-224-FROZEN-<日期时间>。' >&2; exit 2;
}
out=$(realpath -m "$out")
case "$out/" in
    "$USB_ROOT/"*) echo '冻结输出不能放在实践项目内部，以免递归打包。' >&2; exit 2 ;;
esac
[[ -d "$evidence" ]] || {
    echo '第三参数必须是已通过验证的证据目录，包含 rounds/R1-result.json 等。' >&2; exit 2;
}
python3 - "$evidence" <<'PY'
import json
import pathlib
import sys
root = pathlib.Path(sys.argv[1])
for tag in ('R1', 'R2', 'R3'):
    result = json.loads((root / 'rounds' / f'{tag}-result.json').read_text())
    assert result['pass'] is True, tag
    assert result['competition_seconds'] < 300, tag
print('三条映射规则的验收证据已通过。')
communication = json.loads((root / 'communication_acceptance.json').read_text())
assert communication['pass'] is True, 'communication acceptance'
assert communication['rmw_implementation'] == 'rmw_cyclonedds_cpp'
assert communication['clean_startups_passed'] >= 3
assert communication['final_response_errors'] == 0
for relative in communication['required_full_flow_results']:
    assert json.loads((root / relative).read_text())['pass'] is True, relative
PY
[[ -f "$evidence/validated_sources.sha256" ]] || {
    echo '缺少经验证源码的哈希收据 validated_sources.sha256，禁止冻结。' >&2; exit 2;
}
(cd "$USB_ROOT" && sha256sum --quiet -c "$evidence/validated_sources.sha256")
mkdir -p "$out"
[[ ! -e "$out/$name" && ! -e "$out/$name.zip" ]] || {
    echo "版本已存在，禁止覆盖：$out/$name" >&2; exit 2;
}
stage=$(mktemp -d "$out/.freeze-staging-XXXXXXXX")
bundle="$stage/$name"
mkdir -p "$bundle/workspace/src" "$bundle/acceptance"
rsync -a --exclude=.git --exclude=__pycache__ --exclude='*.pyc' \
    --exclude=.pytest_cache --exclude=node_modules --exclude='.env' \
    "$REPO/" "$bundle/workspace/src/yzbot/"
for directory in scripts costudio vendor rules examples; do
    rsync -a --exclude=node_modules --exclude=__pycache__ \
        "$USB_ROOT/$directory/" "$bundle/$directory/"
done
for file in README.md SOURCE_VERSION.json SOURCE_FILENAME_MAP.json \
    SOURCE_USER_CHANGES.patch LOCAL_CHANGES_20261012.md 版本冻结流程.md; do
    [[ ! -f "$USB_ROOT/$file" ]] || cp -a "$USB_ROOT/$file" "$bundle/"
done
rsync -a --exclude=costudio-client.log "$evidence/" "$bundle/acceptance/"
git -C "$REPO" diff --binary > "$bundle/acceptance/source_changes.patch"
git -C "$REPO" status --short > "$bundle/acceptance/source_status.txt"
# Refuse recognizable embedded credentials, printing filenames only.
if rg -l --hidden 'sk-[A-Za-z0-9]{20,}|AKIA[0-9A-Z]{16}|BEGIN (RSA |OPENSSH |EC )?PRIVATE KEY' "$bundle"; then
    echo "发现疑似凭据，保留临时目录以便检查，不发布：$stage" >&2; exit 2;
fi
python3 - "$bundle" "$name" "$REPO" <<'PY'
import datetime
import json
import pathlib
import subprocess
import sys
root, name, repo = pathlib.Path(sys.argv[1]), sys.argv[2], sys.argv[3]
subject = subprocess.check_output(['git', '-C', repo, 'show', '-s', '--format=%s', 'HEAD'], text=True).strip()
metadata = {
    'baseline': name, 'frozen_at': datetime.datetime.now().astimezone().isoformat(),
    'source_checkout_commit': subprocess.check_output(['git', '-C', repo, 'rev-parse', 'HEAD'], text=True).strip(),
    'source_checkout_commit_is_import_snapshot': subject.startswith('Imported delivery source snapshot'),
    'source_has_local_changes': bool(subprocess.check_output(['git', '-C', repo, 'status', '--porcelain'], text=True).strip()),
    'source_changes': 'acceptance/source_changes.patch',
    'private_credentials_included': False, 'build_install_runtime_included': False,
    'costudio_version': json.loads((root / 'costudio/extension/package.json').read_text())['version'],
    'clock_contract': 'valid confirm-click through matching mission terminal wall-clock endpoint',
    'acceptance': [f'acceptance/rounds/R{i}-result.json' for i in (1, 2, 3)],
    'usb_hardware_tested': False,
    'communication': json.loads((root / 'acceptance/communication_acceptance.json').read_text()),
}
(root / 'FROZEN_BASELINE.json').write_text(json.dumps(metadata, ensure_ascii=False, indent=2) + '\n')
PY
cd "$bundle"
find . -type f ! -name SHA256SUMS -print0 | sort -z | xargs -0 sha256sum > SHA256SUMS
sha256sum --quiet -c SHA256SUMS
chmod -R a-w "$bundle"
cd "$stage"
zip -q -r -X "$name.zip" "$name"
sha256sum "$name.zip" > "$name.zip.sha256"
sha256sum -c "$name.zip.sha256"
# Reparenting a directory requires write access to that directory on Linux.
# Restore only its root briefly, then lock it again at the final location.
# Children and the archived permissions remain read-only throughout.
chmod u+w "$bundle"
mv "$bundle" "$out/$name"
chmod a-w "$out/$name"
chmod a-w "$stage/$name.zip" "$stage/$name.zip.sha256"
mv "$stage/$name.zip" "$stage/$name.zip.sha256" "$out/"
rmdir "$stage"
echo "已冻结（目录只读、ZIP/哈希配套）：$out/$name.zip"
echo '后续实践在本机项目目录；改变后的版本使用新的日期时间名称，不能改写本基准。'
