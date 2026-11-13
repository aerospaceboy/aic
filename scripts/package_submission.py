#!/usr/bin/env python3
"""Package competition sources without local history or credentials."""

import argparse
import hashlib
from pathlib import Path
import re
import sys
import zipfile


ROOT = Path(__file__).resolve().parents[1]
ROOT_FILES = ('README.md', '.gitignore', '启动本版本.sh', '部署本版本.sh')
SOURCE_DIRS = ('scripts', 'examples', 'rules', 'costudio', 'workspace/src')
SKIP_DIRS = {
    '.git', '__pycache__', '.pytest_cache', '.mypy_cache', '.ruff_cache',
    'node_modules', 'build', 'install', 'log', '.ros', '.idea', '.vscode',
    'world_backups', 'runs', 'acceptance',
}
PRIVATE = re.compile(
    rb'sk-[A-Za-z0-9_-]{20,}|AKIA[0-9A-Z]{16}|'
    rb'-----BEGIN (?:RSA |OPENSSH |EC )?PRIVATE KEY-----')
REQUIRED = (
    'workspace/src/yzbot/LICENSE',
    'workspace/src/yzbot/mybot/launch/gazebo_world.launch.py',
    'workspace/src/yzbot/mybot_description/worlds/offic_room.world',
    'workspace/src/yzbot/bot_navigation/maps/mapn3.pgm',
    'workspace/src/yzbot/tools_demo/config/mission.yaml',
    'workspace/src/yzbot/tools_demo/config/task_parser.yaml',
    'costudio/extension/package.json', 'costudio/extension/dist/extension.js',
)


def excluded(path):
    """Exclude generated outputs and development records, not robot assets."""
    relative = path.relative_to(ROOT)
    name = path.name
    return (bool(set(relative.parts) & SKIP_DIRS)
            or any(part.startswith('.git.broken-') for part in relative.parts)
            or any(part.endswith(('.egg-info', '.dist-info'))
                   for part in relative.parts)
            or name.startswith('.env') or name.endswith(('.env', '~', '.pem'))
            or '.bak' in name or '.pre_' in name
            or name.endswith(('.orig', '.pyc', '.pyo', '.log', '.gguf',
                              '.db3', '.mcap', '.bag', '.patch'))
            or name in {'AGENTS.md', 'PROJECT_HANDOFF.md'}
            or name.startswith('HANDOFF_PROMPT')
            or (name.startswith('codex_') and name.endswith('.md')))


def collect_files(with_installers):
    files = {ROOT / name for name in ROOT_FILES}
    for directory in SOURCE_DIRS:
        files.update(p for p in (ROOT / directory).rglob('*')
                     if p.is_file() and not excluded(p))
    optional = [
        ROOT / 'vendor/question_generator/TMSCQtest_x86_x64.bin',
        ROOT / 'vendor/question_generator/出题程序使用说明.pdf',
        ROOT / 'vendor/linux/installer-origin.json',
    ]
    optional.extend((ROOT / 'vendor/linux').glob('ros-humble-cobridge_*.deb'))
    if with_installers:
        clients = list((ROOT / 'vendor/linux').glob(
            'coStudio_*linux_amd64.deb'))
        if not clients:
            raise ValueError('--with-installers requires the CoStudio deb')
        optional.extend(clients)
    for path in optional:
        if path.is_file():
            files.add(path)
        else:
            print(f'可选资源未包含：{path.relative_to(ROOT)}', file=sys.stderr)
    for name in REQUIRED:
        if ROOT / name not in files or not (ROOT / name).is_file():
            raise ValueError(f'Missing required source/resource: {name}')
    for path in files:
        if not path.resolve().is_relative_to(ROOT):
            raise ValueError(f'External symlink cannot be shipped: {path}')
        if '\n' in str(path.relative_to(ROOT)):
            raise ValueError('Newlines are not supported in release filenames')
    return sorted(files)


def build(output, with_installers=False):
    """Scan, build one portable ZIP and include its content checksums."""
    output = output.expanduser().resolve()
    if output.suffix != '.zip' or output.exists():
        raise ValueError('Output must be a new .zip path')
    files = collect_files(with_installers)
    # Print only affected filenames; never echo suspected secret contents.
    suspects = [str(p.relative_to(ROOT)) for p in files
                if p.suffix not in ('.deb', '.bin', '.pdf', '.jpg', '.png')
                and PRIVATE.search(p.read_bytes())]
    if suspects:
        raise ValueError('Suspected credentials in: ' + ', '.join(suspects))
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_suffix('.zip.partial')
    if temporary.exists():
        raise ValueError(f'Previous partial output exists: {temporary}')
    checksums = []
    try:
        with zipfile.ZipFile(temporary, 'x', zipfile.ZIP_DEFLATED) as archive:
            for path in files:
                name = path.relative_to(ROOT).as_posix()
                data = path.read_bytes()
                digest = hashlib.sha256(data).hexdigest()
                checksums.append(f'{digest}  ./{name}')
                archive.writestr(zipfile.ZipInfo.from_file(path, name), data,
                                 compress_type=zipfile.ZIP_DEFLATED)
            archive.writestr('SHA256SUMS', '\n'.join(checksums) + '\n')
        temporary.rename(output)
    except BaseException:
        temporary.unlink(missing_ok=True)
        raise
    digest = hashlib.sha256(output.read_bytes()).hexdigest()
    output.with_suffix('.zip.sha256').write_text(
        f'{digest}  {output.name}\n', encoding='utf-8')
    print(f'已生成：{output}\n文件数：{len(files) + 1}\n'
          f'大小：{output.stat().st_size / 1024 / 1024:.2f} MiB')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--with-installers', action='store_true')
    args = parser.parse_args()
    try:
        build(args.output, args.with_installers)
    except (OSError, ValueError) as error:
        parser.exit(2, f'{error}\n')


if __name__ == '__main__':
    main()
