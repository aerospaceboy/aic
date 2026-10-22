"""Exercise server ownership and GUI startup against launch conditions."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace

from launch import LaunchContext
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.utilities import perform_substitutions
import pytest


WORLD_LAUNCH = (Path(__file__).resolve().parents[2]
                / 'mybot/launch/gazebo_world.launch.py')


@pytest.fixture
def world_launch():
    spec = importlib.util.spec_from_file_location('tested_world', WORLD_LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize('headless,debug_gui,expected', [
    ('True', 'True', []), ('True', 'False', []),
    ('False', 'True', ['debug']), ('False', 'False', ['standard']),
])
def test_client_waits_for_spawn_and_respects_gui_modes(
    world_launch, headless, debug_gui, expected,
):
    description = world_launch.generate_launch_description()
    context = LaunchContext()
    context.launch_configurations.update(
        headless=headless, debug_gui=debug_gui)
    # The server is independent of GUI selection; no client starts in parallel.
    processes = [entity for entity in description.entities
                 if type(entity) is ExecuteProcess]
    assert len(processes) == 1
    assert perform_substitutions(context, processes[0].cmd[0]) == 'gzserver'
    assert processes[0].condition is None
    handlers = [entity.event_handler for entity in description.entities
                if isinstance(entity, RegisterEventHandler)]
    spawn_handler = handlers[-1]
    event = SimpleNamespace(returncode=0)
    clients = spawn_handler.handle(event, context)
    enabled = [client for client in clients
               if client.condition.evaluate(context)]
    modes = []
    for client in enabled:
        command = [perform_substitutions(context, part) for part in client.cmd]
        assert command[:2] == ['gzclient', '--verbose']
        assert '-s' not in command  # Server ROS plugins stay on gzserver.
        assert not any(arg.endswith('.world') for arg in command)
        mode = 'debug' if '--gui-client-plugin' in command else 'standard'
        modes.append(mode)
    assert modes == expected


@pytest.mark.parametrize('returncode', [1, -9])
def test_failed_spawn_does_not_open_a_misleading_empty_window(
    world_launch, returncode,
):
    assert world_launch.client_after_spawn(
        SimpleNamespace(returncode=returncode), None, clients=[object()]) == []


@pytest.mark.parametrize('headless,debug_gui,rendered,ready', [
    ('False', 'True', False, False), ('False', 'True', True, True),
    ('True', 'True', False, True), ('False', 'False', False, True),
])
def test_one_click_gate_cannot_accept_only_a_running_splash(
    tmp_path, headless, debug_gui, rendered, ready,
):
    import os
    import subprocess

    path = (WORLD_LAUNCH.parents[2]
            / 'tools_demo/scripts/start_manual_problem_mode.sh')
    source = path.read_text()
    setup = source.split('gui_ready=False\n', 1)[1].split(
        'while (( SECONDS < deadline )); do', 1)[0]
    check = source.split('    if [[ "$gui_ready" != True ]]; then', 1)[1]
    check = check.split('    nodes=$(timeout', 1)[0]
    (tmp_path / 'launch.log').write_text(
        'YZBot low-load GUI active: scene/camera ready\n' if rendered else '')
    # Execute the real bounded-window gate with its deadline already elapsed.
    # No ROS graph or simulation health can replace this rendering proof.
    script = ('gui_ready=False\n' + setup
              + 'if [[ "$gui_ready" != True ]]; then' + check
              + '\n[[ "$gui_ready" == True ]]')
    result = subprocess.run(
        ['bash', '-c', 'run_dir="$1"\n' + script, 'gate-test', str(tmp_path)],
        env=dict(os.environ, COSTUDIO_HEADLESS=headless,
                 COSTUDIO_DEBUG_GUI=debug_gui, GAZEBO_GUI_TIMEOUT_SEC='0'),
        capture_output=True, text=True, timeout=3)
    assert (result.returncode == 0) is ready
    if not ready:
        assert 'Gazebo 图形窗口未就绪' in result.stderr
