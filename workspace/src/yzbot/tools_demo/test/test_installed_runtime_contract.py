"""A stale copied arm executable must never masquerade as tested source."""

from pathlib import Path
import subprocess

REPO = Path(__file__).resolve().parents[2]
GUARD = REPO/'tools_demo/scripts/verify_runtime_sources.sh'


def test_guard_refuses_stale_and_missing_installed_program(tmp_path):
    repo, workspace = tmp_path/'repo', tmp_path/'workspace'
    pairs = [('mybot/pick_place_moveit.py','mybot/lib/mybot/pick_place_moveit'),
             ('mybot_description/worlds/offic_room.world',
              'mybot_description/share/mybot_description/worlds/offic_room.world')]
    # Test-owned fixtures, not real project edits.
    for source,installed in pairs:
        src,dst=repo/source,workspace/'install'/installed
        src.parent.mkdir(parents=True,exist_ok=True)
        dst.parent.mkdir(parents=True,exist_ok=True)
        src.write_text('new')
        dst.write_text('old')
    command=['bash','-c','source "$1"; verify_yzbot_runtime_sources "$2" "$3"',
             'guard',str(GUARD),str(repo),str(workspace)]
    assert subprocess.run(command,capture_output=True).returncode == 2
    for source,installed in pairs:
        (workspace/'install'/installed).write_text((repo/source).read_text())
    assert subprocess.run(command,capture_output=True).returncode == 0
    (workspace/'install'/pairs[0][1]).unlink()
    assert subprocess.run(command,capture_output=True).returncode == 2


def test_both_entrypoints_check_before_teardown_or_launch():
    for script in ('start_manual_problem_mode.sh','run_ordered_case.sh'):
        text=(REPO/'tools_demo/scripts'/script).read_text()
        assert text.index('verify_yzbot_runtime_sources "$repo" "$workspace"') \
            < text.index('ros2 launch tools_demo')
