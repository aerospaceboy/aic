"""Actual data, not stale ROS graph metadata, gates startup."""

from pathlib import Path

from tools_demo.simulation_probe import SimulationEvidence


def test_frozen_clock_is_not_ready_even_with_scans():
    evidence = SimulationEvidence()
    evidence.clock(1)
    evidence.clock(1)
    evidence.scan([1.0], 0.1, 10.0)
    evidence.scan([1.0], 0.1, 10.0)
    assert not evidence.ready()


def test_progressing_clock_requires_real_valid_scans():
    evidence = SimulationEvidence()
    evidence.clock(1)
    evidence.clock(2)
    for ranges in ([], [float('nan')], [float('inf')], [0.01], [11.0]):
        evidence.scan(ranges, 0.1, 10.0)
    assert not evidence.ready()
    evidence.scan([1.0], 0.1, 10.0)
    assert not evidence.ready()
    evidence.scan([1.0], 0.1, 10.0)
    assert evidence.ready()


def test_scan_without_clock_is_not_ready():
    evidence = SimulationEvidence()
    evidence.scan([1.0], 0.1, 10.0)
    evidence.scan([1.0], 0.1, 10.0)
    assert not evidence.ready()


def test_manual_launch_fails_fast_and_probes_before_pass():
    repository = Path(__file__).resolve().parents[2]
    source = (repository / 'tools_demo/scripts/start_manual_problem_mode.sh').read_text()
    assert 'process has died|failed to send response' in source
    assert '"[g]zserver"' in source
    assert source.index('simulation_probe') < source.index('echo "MANUAL_PROBLEM_MODE=PASS"')
    assert source.index('CLEAN_ONLY=True bash') < source.index('ros2 daemon stop')
    clean = (repository / 'tools_demo/scripts/clean_launch.sh').read_text()
    assert 'rm -f /dev/shm/' not in clean


def test_costudio_input_is_reliable_volatile_with_a_chunk_queue():
    from rclpy.qos import DurabilityPolicy, ReliabilityPolicy
    from tools_demo.task_question_node import input_chunk_qos, task_qos

    qos = input_chunk_qos()
    assert qos.durability == DurabilityPolicy.VOLATILE
    assert qos.reliability == ReliabilityPolicy.RELIABLE
    assert qos.depth >= 128
    assert task_qos().durability == DurabilityPolicy.TRANSIENT_LOCAL
    repository = Path(__file__).resolve().parents[2]
    parser = (repository / 'tools_demo/tools_demo/task_parser_node.py').read_text()
    block = parser.split('self.problem_chunk_subscription =', 1)[1].split(
        'self.last_problem:', 1,
    )[0]
    assert 'input_chunk_qos()' in block
