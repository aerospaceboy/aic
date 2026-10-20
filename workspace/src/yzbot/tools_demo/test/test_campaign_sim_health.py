"""Contract tests for the campaign's simulator-health probe.

sim_dead() decides whether a round relaunches the simulator or burns its full
900 s budget re-publishing a question into a dead graph.
"""

import ast
import importlib.util
import pathlib

REPOSITORY = pathlib.Path(__file__).resolve().parents[2]
CAMPAIGN = REPOSITORY / "tools_demo" / "scripts" / "run_zone9_campaign.py"


def load_sim_dead():
    """Exec just the sim_dead function, without importing the campaign."""
    source = CAMPAIGN.read_text(encoding="utf-8")
    tree = ast.parse(source)
    function = next(
        node
        for node in tree.body
        if isinstance(node, ast.FunctionDef) and node.name == "sim_dead"
    )
    module = ast.Module(body=[function], type_ignores=[])
    namespace = {"pathlib": pathlib, "re": __import__("re")}
    exec(compile(module, str(CAMPAIGN), "exec"), namespace)
    return namespace["sim_dead"]


def death_line(name):
    return (
        f"[ERROR] [{name}]: process has died "
        "[pid 445534, exit code 255, cmd 'gazebo --verbose world.world']"
    )


def test_sim_dead_detects_the_gazebo_wrapper(monkeypatch, tmp_path):
    """The launcher starts `gazebo`, so that is the name in the log.

    Matching only gzserver-\\d+ is why a 2026-10-04 startup crash went
    unnoticed: the round re-published its question every 60 s for eight
    minutes while the mission sat in mission_start_waiting_for_odom.
    """
    sim_dead = load_sim_dead()
    log = tmp_path / "launch.log"
    log.write_text(death_line("gazebo-1"), encoding="utf-8")
    assert sim_dead(str(log)) is True


def test_sim_dead_detects_gzserver(monkeypatch, tmp_path):
    """gzserver-N is how ros2 launch reports a direct gzserver bringup."""
    sim_dead = load_sim_dead()
    log = tmp_path / "launch.log"
    log.write_text(death_line("gzserver-1"), encoding="utf-8")
    assert sim_dead(str(log)) is True


def test_sim_dead_ignores_gzclient(tmp_path):
    """The GUI dying does not stop the physics; relaunching would waste a round."""
    sim_dead = load_sim_dead()
    log = tmp_path / "launch.log"
    log.write_text(death_line("gzclient-1"), encoding="utf-8")
    assert sim_dead(str(log)) is False


def test_sim_dead_ignores_a_healthy_log_and_missing_file(tmp_path):
    sim_dead = load_sim_dead()
    healthy = tmp_path / "ok.log"
    healthy.write_text(
        "[INFO] [gzserver-1]: process started with pid [12]\n", encoding="utf-8"
    )
    assert sim_dead(str(healthy)) is False
    assert sim_dead(str(tmp_path / "absent.log")) is False
    assert sim_dead(None) is False


def test_sim_dead_ignores_a_dead_nav_node(tmp_path):
    """amcl and collision_monitor deaths are not simulator deaths.

    Those are handled by the node-level gates, not by relaunching Gazebo.
    """
    sim_dead = load_sim_dead()
    log = tmp_path / "launch.log"
    log.write_text(
        death_line("amcl-10") + "\n" + death_line("collision_monitor-24"),
        encoding="utf-8",
    )
    assert sim_dead(str(log)) is False


def test_relaunch_rebuilds_the_node_and_clears_the_baseline():
    """Relaunching the stack is useless if the runner stays blind to it.

    Measured 2026-10-04 (verify_lethal, round 4rB_1bC): the campaign's
    participant kept its discovery state across the restart and never re-paired
    with the replacement stack's /competition/mission/state publisher. It went
    on re-publishing the question every 60 s for nine minutes while that stack
    ran the mission to mission_terminal:completed=5,skipped=0, so the round
    would have burned its budget with the task already done.

    The baseline has to be cleared too: it records the previous round's latched
    status, and task_id is derived deterministically from the question, so a
    re-run of the same case can carry the id recorded as the baseline - which
    makes the "task_id not in (None, baseline)" test unsatisfiable for the rest
    of the round.
    """
    source = CAMPAIGN.read_text(encoding="utf-8")
    block = source.split("relaunches += 1", 1)[1].split("continue", 1)[0]
    assert "node.destroy_node()" in block
    assert "node = RoundNode()" in block
    assert "baseline = {}" in block
    assert 'record["baseline_task"] = None' in block
    rebuild = block.index("node = RoundNode()")
    submit = block.index("node.submit(question)")
    assert rebuild < submit
    assert block.index("baseline = {}") < submit
    # The stale status must not be re-captured as the new baseline.
    assert "dict(node.mission or {})" not in block
