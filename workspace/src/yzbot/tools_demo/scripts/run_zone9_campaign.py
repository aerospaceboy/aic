#!/usr/bin/env python3
"""Nine-round AB/AC/BC zone regression with restart-per-round and pass/fail gates.

Pass gate per round: terminal state completed, completed=5, skipped=0,
mission elapsed <= 290 s, no collision evidence, no pick abort. Every round is recorded to
disk with its failure reason; the campaign continues regardless.
"""
import json
import os
import pathlib
import re
import subprocess
import time

from tools_demo.runtime_environment import configure_campaign_transport

# restart_system runs a child shell: its DDS exports cannot reach this parent.
# Bootstrap before rclpy loads the middleware, not merely before the restart.
configure_campaign_transport(pathlib.Path(__file__).resolve().parents[4])

import rclpy  # noqa: E402
from tools_demo.acceptance_contract import validated_avoidance  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import DurabilityPolicy  # noqa: E402
from rclpy.qos import QoSProfile  # noqa: E402
from rclpy.qos import ReliabilityPolicy  # noqa: E402
from std_msgs.msg import String  # noqa: E402

WORKSPACE = pathlib.Path("/home/ros/huxuyang/YZBot-224/workspace")
START = "src/yzbot/tools_demo/scripts/start_manual_problem_mode.sh"
STAMP = time.strftime("%Y%m%dT%H%M%S")
AB4_SELECTION = os.environ.get("AB4_CASES", "").strip()
AC4_SELECTION = os.environ.get("AC4_CASES", "").strip()
BC4_SELECTION = os.environ.get("BC4_CASES", "").strip()
# Any of the 48 legal tasks in the pick-order policy table can be run by its
# policy tag (e.g. 1bA_4rB = 1 blue to zone A, then 4 red to zone B); this is
# the same space tools_demo/scripts/gen48_questions.py emits.
POLICY_CASES = [
    case.strip()
    for case in os.environ.get("POLICY_CASES", "").split(",")
    if case.strip()
]
selected_four = [
    pair for pair, value in (
        ("AB", AB4_SELECTION), ("AC", AC4_SELECTION), ("BC", BC4_SELECTION)
    ) if value
]
if len(selected_four) > 1:
    raise ValueError("Only one four-case pair can be selected")
if POLICY_CASES:
    RUN_PREFIX = "policy"
elif selected_four:
    RUN_PREFIX = f"{selected_four[0].lower()}4"
else:
    RUN_PREFIX = "zone9"
OUT = pathlib.Path("/home/ros/huxuyang/YZBot-224/runs") / f"{RUN_PREFIX}_{STAMP}"
OUT.mkdir(parents=True, exist_ok=True)
PROGRESS = OUT / "progress.txt"
RESULTS = OUT / "results.jsonl"
SUMMARY = OUT / "summary.csv"

TASKS = {
    "AB": "请先将3个红色方块送到A区，再将2个蓝色方块送到B区。",
    "AC": "请先将3个红色方块送到A区，再将2个蓝色方块送到C区。",
    "BC": "请先将3个红色方块送到B区，再将2个蓝色方块送到C区。",
}
AB4_QUESTIONS = {
    "14": "请先将1个红色方块送到A区，再将4个蓝色方块送到B区。",
    "41": "请先将4个蓝色方块送到B区，再将1个红色方块送到A区。",
    "23": "请先将2个红色方块送到A区，再将3个蓝色方块送到B区。",
    "32": "请先将3个蓝色方块送到B区，再将2个红色方块送到A区。",
    "R14": "请先将1个蓝色方块送到B区，再将4个红色方块送到A区。",
    "R41": "请先将4个红色方块送到A区，再将1个蓝色方块送到B区。",
    "R23": "请先将2个蓝色方块送到B区，再将3个红色方块送到A区。",
    "R32": "请先将3个红色方块送到A区，再将2个蓝色方块送到B区。",
}
AB4_EXPECTED_ORDERS = {
    "14": "1:redx1->A | 2:bluex4->B",
    "41": "1:bluex4->B | 2:redx1->A",
    "23": "1:redx2->A | 2:bluex3->B",
    "32": "1:bluex3->B | 2:redx2->A",
    "R14": "1:bluex1->B | 2:redx4->A",
    "R41": "1:redx4->A | 2:bluex1->B",
    "R23": "1:bluex2->B | 2:redx3->A",
    "R32": "1:redx3->A | 2:bluex2->B",
}
AC4_QUESTIONS = {
    "14": "请先将1个红色方块送到A区，再将4个蓝色方块送到C区。",
    "41": "请先将4个蓝色方块送到C区，再将1个红色方块送到A区。",
    "23": "请先将2个红色方块送到A区，再将3个蓝色方块送到C区。",
    "32": "请先将3个蓝色方块送到C区，再将2个红色方块送到A区。",
    "R14": "请先将1个蓝色方块送到C区，再将4个红色方块送到A区。",
    "R41": "请先将4个红色方块送到A区，再将1个蓝色方块送到C区。",
    "R23": "请先将2个蓝色方块送到C区，再将3个红色方块送到A区。",
    "R32": "请先将3个红色方块送到A区，再将2个蓝色方块送到C区。",
}
AC4_EXPECTED_ORDERS = {
    "14": "1:redx1->A | 2:bluex4->C",
    "41": "1:bluex4->C | 2:redx1->A",
    "23": "1:redx2->A | 2:bluex3->C",
    "32": "1:bluex3->C | 2:redx2->A",
    "R14": "1:bluex1->C | 2:redx4->A",
    "R41": "1:redx4->A | 2:bluex1->C",
    "R23": "1:bluex2->C | 2:redx3->A",
    "R32": "1:redx3->A | 2:bluex2->C",
}
BC4_QUESTIONS = {
    "14": "请先将1个红色方块送到B区，再将4个蓝色方块送到C区。",
    "41": "请先将4个蓝色方块送到C区，再将1个红色方块送到B区。",
    "23": "请先将2个红色方块送到B区，再将3个蓝色方块送到C区。",
    "32": "请先将3个蓝色方块送到C区，再将2个红色方块送到B区。",
    "R14": "请先将1个蓝色方块送到C区，再将4个红色方块送到B区。",
    "R41": "请先将4个红色方块送到B区，再将1个蓝色方块送到C区。",
    "R23": "请先将2个蓝色方块送到C区，再将3个红色方块送到B区。",
    "R32": "请先将3个红色方块送到B区，再将2个蓝色方块送到C区。",
}
BC4_EXPECTED_ORDERS = {
    "14": "1:redx1->B | 2:bluex4->C",
    "41": "1:bluex4->C | 2:redx1->B",
    "23": "1:redx2->B | 2:bluex3->C",
    "32": "1:bluex3->C | 2:redx2->B",
    "R14": "1:bluex1->C | 2:redx4->B",
    "R41": "1:redx4->B | 2:bluex1->C",
    "R23": "1:bluex2->C | 2:redx3->B",
    "R32": "1:redx3->B | 2:bluex2->C",
}
FOUR_CASE_QUESTIONS = {
    "AB": AB4_QUESTIONS, "AC": AC4_QUESTIONS, "BC": BC4_QUESTIONS,
}
FOUR_CASE_ORDERS = {
    "AB": AB4_EXPECTED_ORDERS, "AC": AC4_EXPECTED_ORDERS,
    "BC": BC4_EXPECTED_ORDERS,
}
CLAUSE_RE = re.compile(r"^(\d+)([rb])([ABC])$")
CLAUSE_COLORS = {"r": "红色", "b": "蓝色"}
CLAUSE_WORDS = {"r": "red", "b": "blue"}


def policy_case_clauses(tag):
    """Split a policy tag into its ordered (count, colour, zone) clauses."""
    parts = tag.split("_")
    if len(parts) != 2:
        raise ValueError(f"Policy case {tag} must be <clause>_<clause>")
    clauses = []
    for part in parts:
        match = CLAUSE_RE.fullmatch(part)
        if not match:
            raise ValueError(f"Policy case clause {part} is malformed")
        count, colour, zone = match.groups()
        clauses.append((int(count), colour, zone))
    return clauses


def policy_case_question(tag):
    """Return the Chinese question and the expected parser order for a tag."""
    first, second = policy_case_clauses(tag)
    question = (
        f"请先将{first[0]}个{CLAUSE_COLORS[first[1]]}方块送到{first[2]}区，"
        f"再将{second[0]}个{CLAUSE_COLORS[second[1]]}方块送到{second[2]}区。"
    )
    order = (
        f"1:{CLAUSE_WORDS[first[1]]}x{first[0]}->{first[2]} | "
        f"2:{CLAUSE_WORDS[second[1]]}x{second[0]}->{second[2]}"
    )
    return question, order


POLICY_CASE_QUESTIONS = {
    case: policy_case_question(case) for case in POLICY_CASES
}
DEFAULT_ROUNDS = (
    [("AB", i) for i in range(1, 4)]
    + [("AC", i) for i in range(1, 4)]
    + [("BC", i) for i in range(1, 4)]
)
requested_tags = [
    tag.strip().upper()
    for tag in os.environ.get("ZONE9_TAGS", "").split(",")
    if tag.strip()
]
if POLICY_CASES:
    if selected_four or requested_tags:
        raise ValueError("POLICY_CASES cannot be combined with other selectors")
    # Duplicates are allowed: POLICY_CASES=tag,tag runs the same task twice
    # in one campaign (consecutive double-test under a time gate).
    ROUNDS = [(case, i + 1) for i, case in enumerate(POLICY_CASES)]
elif selected_four:
    four_pair = selected_four[0]
    selection = {
        "AB": AB4_SELECTION, "AC": AC4_SELECTION, "BC": BC4_SELECTION,
    }[four_pair]
    cases = [case.strip() for case in selection.split(",") if case.strip()]
    if (
        not cases
        or len(cases) != len(set(cases))
        or not set(cases).issubset(FOUR_CASE_QUESTIONS[four_pair])
    ):
        raise ValueError("Four-case selection has duplicate or unknown cases")
    if requested_tags:
        raise ValueError("Four-case selection and ZONE9_TAGS cannot be combined")
    ROUNDS = [(f"{four_pair}{case}", 1) for case in cases]
else:
    ROUNDS = [
        round_spec
        for round_spec in DEFAULT_ROUNDS
        if f"{round_spec[0]}{round_spec[1]}" in requested_tags
    ] if requested_tags else DEFAULT_ROUNDS
    if requested_tags and len(ROUNDS) != len(set(requested_tags)):
        raise ValueError("ZONE9_TAGS must list distinct valid AB/AC/BC round tags")
ROUND_COUNT = len(ROUNDS)
TERMINAL = {"completed", "failed", "canceled", "timed_out"}
ROUND_BUDGET_SEC = 900.0
# No-progress watchdog: after the task is accepted, if completed/index/state
# stay frozen this long the round is stuck (nav abort loops, perception dead
# ends). Abort instead of burning the full 900 s budget.
NO_PROGRESS_SEC = 150.0
# A startup race (leftover processes, a lifecycle handshake that
# times out) is not a mission failure: retry the round before
# recording it.
RESTART_ATTEMPTS = 3
QUESTION_RETRY_SEC = 60.0
SIM_HEALTH_PERIOD_SEC = 15.0


def log(message):
    line = f"[{time.strftime('%H:%M:%S')}] {message}"
    print(line, flush=True)
    with PROGRESS.open("a", encoding="utf-8") as handle:
        handle.write(line + "\n")


def retained_qos():
    return QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


class RoundNode(Node):
    def __init__(self):
        super().__init__("zone9_runner")
        self.mission = None
        self.avoidance = None
        qos = retained_qos()
        self.publisher = self.create_publisher(
            String, "/competition/task/question", qos
        )
        self.create_subscription(
            String, "/competition/mission/state", self._mission, qos
        )
        self.create_subscription(
            String, "/competition/avoidance/state", self._avoidance, qos
        )

    def _mission(self, message):
        try:
            self.mission = json.loads(message.data)
        except (TypeError, json.JSONDecodeError):
            pass

    def _avoidance(self, message):
        try:
            self.avoidance = json.loads(message.data)
        except (TypeError, json.JSONDecodeError):
            pass

    def submit(self, question):
        self.publisher.publish(String(data=question))
        rclpy.spin_once(self, timeout_sec=0.5)

    def spin_for(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.3)


def restart_system():
    started = time.monotonic()
    try:
        env = dict(os.environ)
        # Scoring gates use wall clock; keep Gazebo GUI / RViz off unless the
        # operator explicitly opts in. One-key start defaults to GUI+RViz.
        env.setdefault("COSTUDIO_HEADLESS", "True")
        env.setdefault("COSTUDIO_RVIZ", "False")
        result = subprocess.run(
            ["bash", START],
            cwd=str(WORKSPACE),
            capture_output=True,
            text=True,
            timeout=420,
            env=env,
        )
    except subprocess.TimeoutExpired:
        return False, None, "restart timeout"
    elapsed = time.monotonic() - started
    match = re.search(r"日志: (\S+)", result.stdout)
    log_path = match.group(1) if match else None
    ok = "MANUAL_PROBLEM_MODE=PASS" in result.stdout
    log(f"restart ok={ok} in {elapsed:.0f}s log={log_path}")
    return ok, log_path, None if ok else result.stdout[-400:]


def sim_dead(log_path):
    """True when the simulator process died, e.g. segfault while spawning.

    policy_20260917T143854 round 2 (1rB_4bA) lost 900 s to a gzserver
    SIGSEGV (exit code -11) raised by the spawn_entity call: with no
    robot there is no odom -> base_footprint transform, the task manager
    never accepts the question, and the round can only time out. Detect
    the crash so the campaign relaunches instead of burning the budget.

    The name has to cover BOTH launch process names. This check originally
    matched only ``gzserver-N``, which is how ros2 launch reports the server
    when the world is brought up as gzserver directly. The competition
    launcher starts the ``gazebo`` wrapper instead (it spawns gzserver and
    gzclient itself), so its death is reported as ``[gazebo-1]`` and the old
    pattern never matched it. Measured on 2026-10-04: a startup crash with

        [ERROR] [gazebo-1]: process has died [exit code 255, cmd 'gazebo ...']
        terminate called ... RCLError: failed to send response

    left the round spinning in the re-submit loop for eight minutes,
    re-publishing the question every 60 s while the mission waited forever on
    ``mission_start_waiting_for_odom``, because the health probe kept
    answering "simulation alive". gzclient is deliberately NOT included: the
    GUI dying does not stop the physics, and relaunching for it would throw
    away a healthy round.
    """
    if not log_path:
        return False
    path = pathlib.Path(log_path)
    if not path.exists():
        return False
    try:
        text = path.read_text(encoding="utf-8", errors="ignore")
    except OSError:
        return False
    return bool(
        re.search(r"\[ERROR\] \[(?:gzserver|gazebo)-\d+\]: process has died", text)
    )


def log_metrics(log_path):
    text = ""
    if log_path and pathlib.Path(log_path).exists():
        text = pathlib.Path(log_path).read_text(
            encoding="utf-8", errors="ignore"
        )
    parsed_plans = re.findall(
        r"task_id=[0-9a-f]+ source=(\w+) order=([^\r\n]+)", text
    )
    return {
        "plan_source": parsed_plans[-1][0] if parsed_plans else None,
        "plan_order": parsed_plans[-1][1].strip() if parsed_plans else None,
        # Count the manipulator's own failure results, not just the
        # 'Pick failed while attached' log line: campaign 20260912T192818
        # AC3 lost four picks (no IK solution, attachment never
        # stabilised, two grasp-window misses) and the old pattern scored
        # the round as failure-free while it burned 120 s of rework.
        "pick_failed": len(
            re.findall(r"persistent_result id=\d+ success=False", text)
        ),
        "lethal_start": len(
            re.findall(r"Starting point in lethal space", text)
        ),
        "carry_invalid": len(re.findall(r"carry_window .*valid=False", text)),
        "carry_abort": len(
            re.findall(r"Attached cube left the safe carry window", text)
        ),
        "nav_retry": len(re.findall(r"nav_retry", text)),
        "nav_timeout": len(re.findall(r"nav_result state=timed_out", text)),
        "delivery_valid": len(re.findall(r"delivery_truth.*valid=True", text)),
        "delivery_invalid": len(
            re.findall(r"delivery_truth.*valid=False", text)
        ),
        "collision_fault": len(re.findall(r"collision_fault", text, re.I)),
        "collision_detected": len(
            re.findall(r"collision_detected[^,}]*true", text, re.I)
        ),
    }


def run_round(index, pair, iteration):
    policy_case = pair in POLICY_CASE_QUESTIONS
    four_pair = (
        pair[:2]
        if not policy_case
        and pair[:2] in FOUR_CASE_QUESTIONS
        and pair[2:] in FOUR_CASE_QUESTIONS[pair[:2]]
        else None
    )
    case = pair[2:] if four_pair else None
    if policy_case:
        question, expected_order = POLICY_CASE_QUESTIONS[pair]
    elif case:
        question = FOUR_CASE_QUESTIONS[four_pair][case]
        expected_order = FOUR_CASE_ORDERS[four_pair][case]
    else:
        question, expected_order = TASKS[pair], None
    tag = pair if (case or policy_case) else f"{pair}{iteration}"
    log(f"=== round {index}/{ROUND_COUNT} {tag} ===")
    record = {
        "round": index,
        "tag": tag,
        "pair": four_pair if case else pair,
        "question": question,
        "expected_order": expected_order,
        "started_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
    }

    ok, log_path, error = False, None, None
    for attempt in range(1, RESTART_ATTEMPTS + 1):
        ok, log_path, error = restart_system()
        if ok:
            break
        log(f"restart attempt {attempt}/{RESTART_ATTEMPTS} failed: {error}")
        time.sleep(10.0)
    record["log_path"] = log_path
    record["restart_attempts"] = attempt
    record["sim_relaunches"] = 0
    if not ok:
        record.update(passed=False, reason=f"restart failed: {error}")
        return record

    watchdog_tripped = False
    last_progress_key = None
    last_progress_at = None
    rclpy.init()
    node = RoundNode()
    try:
        node.spin_for(6.0)
        baseline = dict(node.mission or {})
        record["baseline_task"] = baseline.get("task_id")
        node.submit(question)
        log("submitted: " + question)

        deadline = time.monotonic() + ROUND_BUDGET_SEC
        plan_seen = False
        last_submit = time.monotonic()
        relaunches = 0
        next_health = time.monotonic() + SIM_HEALTH_PERIOD_SEC
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.3)
            mission = node.mission or {}
            if not plan_seen and mission.get("task_id") not in (
                None,
                baseline.get("task_id"),
            ):
                plan_seen = True
                log(f"task accepted task_id={mission.get('task_id')}")
                last_progress_key = (
                    mission.get("completed"),
                    mission.get("current_index"),
                    mission.get("state"),
                    mission.get("cube_name"),
                )
                last_progress_at = time.monotonic()
            elif not plan_seen and time.monotonic() - last_submit > QUESTION_RETRY_SEC:
                # zone9_20260912T232649 BC2 sat in waiting_for_task for
                # 700 s and was canceled with completed=0 while all eight
                # rounds that did start passed 5/5: the question is
                # published before the parser has subscribed in some
                # restarts. Re-publish until the mission picks it up.
                node.submit(question)
                last_submit = time.monotonic()
                log("question re-submitted (no task yet)")
            if (
                mission.get("state") in TERMINAL
                and mission.get("total") == 5
                and mission.get("task_id") not in (None, baseline.get("task_id"))
            ):
                break
            if plan_seen and last_progress_at is not None:
                progress_key = (
                    mission.get("completed"),
                    mission.get("current_index"),
                    mission.get("state"),
                    mission.get("cube_name"),
                )
                if progress_key != last_progress_key:
                    last_progress_key = progress_key
                    last_progress_at = time.monotonic()
                elif time.monotonic() - last_progress_at > NO_PROGRESS_SEC:
                    watchdog_tripped = True
                    log(
                        "progress watchdog: no change in "
                        f"{progress_key} for {NO_PROGRESS_SEC:.0f}s; aborting"
                    )
                    break
            if time.monotonic() >= next_health:
                next_health = time.monotonic() + SIM_HEALTH_PERIOD_SEC
                if relaunches < RESTART_ATTEMPTS and sim_dead(log_path):
                    relaunches += 1
                    record["sim_relaunches"] = relaunches
                    log(
                        "gzserver died; relaunching simulation "
                        f"({relaunches}/{RESTART_ATTEMPTS})"
                    )
                    ok2, log_path2, error2 = restart_system()
                    if ok2 and log_path2:
                        log_path = log_path2
                        record["log_path"] = log_path
                    else:
                        log(f"relaunch failed: {error2}")
                    # A relaunch replaces every publisher this round was
                    # talking to, so rebuild the node instead of reusing it.
                    # Measured 2026-10-04 (verify_lethal batch, round 4rB_1bC):
                    # the campaign's participant kept its discovery state across
                    # the restart and never re-paired with the replacement
                    # stack's /competition/mission/state publisher. It went on
                    # re-publishing the question every 60 s for nine minutes
                    # ("question re-submitted (no task yet)") while that stack
                    # ran the mission to mission_terminal:completed=5,skipped=0.
                    # The round would have burned its whole budget with the task
                    # already done.
                    #
                    # The baseline is cleared for the same reason it exists: it
                    # records the PREVIOUS round's latched status so that status
                    # is not mistaken for the new task. After a relaunch anything
                    # still cached belongs to the dead stack, and task_id is
                    # derived deterministically from the question, so a re-run of
                    # the same case can even carry the id recorded as the
                    # baseline - which would make the "task_id not in (None,
                    # baseline)" test unsatisfiable for the rest of the round.
                    node.destroy_node()
                    node = RoundNode()
                    node.spin_for(6.0)
                    baseline = {}
                    record["baseline_task"] = None
                    plan_seen = False
                    last_progress_key = None
                    last_progress_at = None
                    watchdog_tripped = False
                    node.submit(question)
                    last_submit = time.monotonic()
                    log("re-submitted after relaunch: " + question)
                    continue
        mission = dict(node.mission or {})
        avoidance = dict(node.avoidance or {})
    finally:
        node.destroy_node()
        rclpy.shutdown()

    record["mission"] = {
        key: mission.get(key)
        for key in ("state", "completed", "skipped", "total", "elapsed_sec",
                    "task_id", "detail")
    }
    record["progress_watchdog"] = bool(watchdog_tripped)
    if watchdog_tripped:
        record.setdefault("watchdog_reason", "no_progress")
    record["avoidance"] = {
        key: avoidance.get(key)
        for key in ("collision_detected", "minimum_clearance", "safe",
                    "obstacles_seen", "minimum_event", "required_clearance",
                    "sample_count")
    }
    record["metrics"] = log_metrics(log_path)

    reasons = []
    if (case or policy_case) and (
        record["metrics"]["plan_source"] != "llm"
        or record["metrics"]["plan_order"] != record["expected_order"]
    ):
        reasons.append(
            "LLM plan order mismatch: "
            f"source={record['metrics']['plan_source']} "
            f"order={record['metrics']['plan_order']}"
        )
    if watchdog_tripped and mission.get("state") != "completed":
        reasons.append(
            f"progress_watchdog detail={mission.get('detail')}"
        )
    if mission.get("state") != "completed":
        reasons.append(f"state={mission.get('state')} detail={mission.get('detail')}")
    if mission.get("completed") != 5:
        reasons.append(f"completed={mission.get('completed')}")
    if mission.get("skipped") not in (0, None):
        reasons.append(f"skipped={mission.get('skipped')}")
    elapsed = mission.get("elapsed_sec")
    if not isinstance(elapsed, (int, float)):
        reasons.append("no elapsed time")
    elif elapsed > 290.0:
        reasons.append(f"elapsed={elapsed:.1f}s > 290s")
    if avoidance.get("collision_detected") is True:
        reasons.append("avoidance collision_detected")
    if not validated_avoidance(avoidance):
        reasons.append(
            "avoidance unsafe "
            f"minimum_clearance={avoidance.get('minimum_clearance')} "
            f"required_clearance={avoidance.get('required_clearance')}"
        )
    if record["metrics"]["collision_fault"]:
        reasons.append(f"collision_fault x{record['metrics']['collision_fault']}")
    if record["metrics"]["pick_failed"]:
        reasons.append(f"pick failures x{record['metrics']['pick_failed']}")
    # The raw carry_window samples are pre-decision readings: the settled
    # recheck deliberately samples the post-retract entity-state lag before
    # it decays, and campaign 20260912T151705 logged 14 such samples with
    # zero picks actually lost. Only an abort - the pick giving up on a cube
    # that stayed out of the window - is a round failure; the raw count
    # stays in the record as a diagnostic.
    if record["metrics"]["carry_abort"]:
        reasons.append(
            f"carry_window abort x{record['metrics']['carry_abort']}"
        )
    if record["metrics"]["lethal_start"]:
        reasons.append(
            f"lethal start x{record['metrics']['lethal_start']}"
        )
    if record["metrics"]["delivery_invalid"]:
        reasons.append(
            f"delivery truth invalid x{record['metrics']['delivery_invalid']}"
        )

    record["passed"] = not reasons
    record["reason"] = "; ".join(reasons) if reasons else "ok"
    record["finished_at"] = time.strftime("%Y-%m-%dT%H:%M:%S")
    log(
        f"round {tag} passed={record['passed']} state={mission.get('state')} "
        f"completed={mission.get('completed')} skipped={mission.get('skipped')} "
        f"elapsed={elapsed} reason={record['reason']}"
    )
    return record


def main():
    log(f"campaign start dir={OUT}")
    header = (
        "round,tag,passed,state,completed,skipped,elapsed_sec,"
        "collision,min_clearance,pick_failed,carry_invalid,carry_abort,"
        "lethal_start,nav_retry,reason\n"
    )
    if not SUMMARY.exists():
        SUMMARY.write_text(header, encoding="utf-8")

    for index, (pair, iteration) in enumerate(ROUNDS, start=1):
        record = run_round(index, pair, iteration)
        with RESULTS.open("a", encoding="utf-8") as handle:
            handle.write(json.dumps(record, ensure_ascii=False) + "\n")
        mission = record.get("mission", {})
        avoidance = record.get("avoidance", {})
        metrics = record.get("metrics", {})
        with SUMMARY.open("a", encoding="utf-8") as handle:
            handle.write(
                ",".join(
                    str(value)
                    for value in (
                        record["round"], record["tag"], record["passed"],
                        mission.get("state"), mission.get("completed"),
                        mission.get("skipped"), mission.get("elapsed_sec"),
                        avoidance.get("collision_detected"),
                        avoidance.get("minimum_clearance"),
                        metrics.get("pick_failed"),
                        metrics.get("carry_invalid"),
                        metrics.get("carry_abort"),
                        metrics.get("lethal_start"),
                        metrics.get("nav_retry"),
                        '"' + str(record["reason"]).replace('"', "'") + '"',
                    )
                )
                + "\n"
            )

    passed = sum(
        1
        for line in RESULTS.read_text(encoding="utf-8").splitlines()
        if json.loads(line).get("passed")
    )
    log(f"CAMPAIGN_DONE passed={passed}/{ROUND_COUNT} dir={OUT}")


if __name__ == "__main__":
    main()
