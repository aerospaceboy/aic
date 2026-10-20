#!/usr/bin/env python3
"""Generate one official problem, submit once, and retain matching evidence."""

import argparse
import base64
import fcntl
import json
from pathlib import Path
import subprocess
import time
import urllib.request
import uuid


def request_json(url, payload=None):
    """Use bounded HTTP requests without retrying a potentially accepted POST."""
    data = None if payload is None else json.dumps(payload, ensure_ascii=False).encode()
    request = urllib.request.Request(
        url, data=data, headers={"Content-Type": "application/json"}
    )
    with urllib.request.urlopen(request, timeout=10) as response:
        return json.load(response)


def decode_question(raw):
    """Accept UTF-8 and the legacy generator's GB18030 output."""
    for encoding in ("utf-8-sig", "gb18030"):
        try:
            question = raw.decode(encoding).strip()
        except UnicodeDecodeError:
            continue
        if question:
            return question
    raise ValueError("官方程序未输出有效题目；未提交")


def require_idle(snapshot, now):
    """Reject old rounds, running missions, and disconnected simulation data."""
    mission = snapshot.get("mission", {})
    if mission.get("state") != "idle" or mission.get("task_id"):
        raise ValueError("系统不是新一轮空闲状态；请先重置场景，未提交")
    stamp = snapshot.get("robot_updated_at", 0)
    if not 0 <= now - stamp <= 8:
        raise ValueError("机器人位置数据失效，仿真可能已停止；未提交")
    if snapshot.get("parse_result", {}).get("state") == "parsing":
        raise ValueError("已有题目正在解析；未重复提交")


def matching_result(snapshot, task_id):
    """Never accept an earlier task's model success as this request's result."""
    result = snapshot.get("parse_result", {})
    if result.get("request_id") != task_id:
        return None
    if result.get("state") == "failed":
        raise ValueError("本题 API 解析失败：" + str(result.get("error", "unknown")))
    if result.get("state") != "published":
        return None
    if result.get("source") != "llm" or result.get("task_id") != task_id:
        raise ValueError("本题没有匹配的真实大模型解析证据")
    return result


def main():
    """Run only after launch readiness; generation-only mode never publishes."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--mapping-b64", default="")
    parser.add_argument("--generate-only", action="store_true")
    parser.add_argument("--watch", action="store_true")
    parser.add_argument("--url", default="http://127.0.0.1:8080")
    parser.add_argument("--generator", type=Path,
                        default=Path.home() / "dev_ws/TMSCQtest_x86_x64.bin")
    args = parser.parse_args()
    rules = base64.b64decode(args.mapping_b64, validate=True).decode("utf-8").strip()
    if not args.generate_only and not rules:
        raise ValueError("本轮映射规则不能为空")
    with open("/tmp/yzbot_video_submit.lock", "w") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        if not args.generate_only:
            require_idle(request_json(args.url + "/api/state"), time.time())
        started = time.monotonic()
        task_id = "video-" + uuid.uuid4().hex
        folder = Path.home() / "dev_ws/experiments/manual-submissions" / task_id
        folder.mkdir(parents=True)
        (folder / "timing.json").write_text(json.dumps({
            "start_wall_unix": time.time(), "start_monotonic": started,
            "start_event": "before_official_generator", "task_id": task_id,
            "git_head": subprocess.check_output(
                ["git", "-C", str(Path(__file__).resolve().parents[2]),
                 "rev-parse", "HEAD"], text=True).strip(),
        }, indent=2))
        completed = subprocess.run([str(args.generator)], capture_output=True,
                                   timeout=15, check=True)
        (folder / "official-generator.stdout").write_bytes(completed.stdout)
        (folder / "official-generator.stderr").write_bytes(completed.stderr)
        question = decode_question(completed.stdout)
        print("\n官方原始题目：\n" + question, flush=True)
        print("\n本轮映射：\n" + rules, flush=True)
        if args.generate_only:
            print("GENERATOR_ONLY=PASS（未发布、未调用 API）", flush=True)
            return
        # Validate unsupported rules before a paid request; never publish this
        # deterministic candidate or substitute it for the required model result.
        from tools_demo.problem_solver import problem_to_plan
        problem_to_plan(question, rules, task_id=task_id)
        payload = {"question": question, "mapping_rules": rules, "task_id": task_id}
        (folder / "request.json").write_text(json.dumps(payload, ensure_ascii=False, indent=2))
        print("任务编号：" + task_id + "\n证据目录：" + str(folder), flush=True)
        try:
            response = request_json(args.url + "/api/task", payload)
        except Exception:
            print("提交结果不确定；不要直接重发，先核对该任务编号。", flush=True)
            raise
        (folder / "http-response.json").write_text(json.dumps(response, ensure_ascii=False))
        print("题目已发送，正在等待本题 API 解析证据……", flush=True)
        deadline = time.monotonic() + 90
        while time.monotonic() < deadline:
            snapshot = request_json(args.url + "/api/state")
            (folder / "latest-state.json").write_text(json.dumps(snapshot, ensure_ascii=False))
            result = matching_result(snapshot, task_id)
            if result:
                (folder / "parse-result.json").write_text(
                    json.dumps(result, ensure_ascii=False, indent=2))
                print("API_PARSE=PASS model=" + result.get("model", "")
                      + " source=llm", flush=True)
                print(json.dumps(result.get("plan"), ensure_ascii=False), flush=True)
                break
            time.sleep(1)
        else:
            raise TimeoutError("90 秒内未获得本题解析成功证据；禁止自动重发")
        if not args.watch:
            return
        last_report = -10.0
        signature = None
        warned = False
        while time.monotonic() - started < 600:
            snapshot = request_json(args.url + "/api/state")
            elapsed = time.monotonic() - started
            mission = snapshot.get("mission", {})
            if mission.get("task_id") == task_id:
                key = (mission.get("state"), mission.get("completed"), mission.get("detail"))
                if key != signature or elapsed - last_report >= 10:
                    with (folder / "mission-observations.jsonl").open("a") as output:
                        output.write(json.dumps(
                            {"wall_elapsed_sec": elapsed, **snapshot},
                            ensure_ascii=False) + "\n")
                    print("总墙钟 %.1fs | %s | 完成 %s/5 | 跳过 %s | %s" % (
                        elapsed, mission.get("state"), mission.get("completed"),
                        mission.get("skipped"), mission.get("detail")), flush=True)
                    signature, last_report = key, elapsed
                if mission.get("state") in {"completed", "failed", "canceled"}:
                    terminal = {"wall_elapsed_sec": elapsed, **snapshot}
                    (folder / "terminal.json").write_text(
                        json.dumps(terminal, ensure_ascii=False, indent=2))
                    if (mission.get("state") != "completed"
                            or mission.get("completed") != 5
                            or mission.get("skipped") != 0):
                        raise RuntimeError("本轮未完成五块零跳过；失败证据已保留")
                    print("本轮搬运结束；核对避障与评分证据。总墙钟 %.1fs（含出题与解析）" % elapsed, flush=True)
                    return
            if elapsed >= 300 and not warned:
                print("已超过 300 秒；后续仅用于故障诊断，不能计入限时比赛成绩。", flush=True)
                warned = True
            time.sleep(1)
        raise TimeoutError("观察超过 600 秒，未获得终态；没有自动重试或伪报成功")


if __name__ == "__main__":
    try:
        main()
    except Exception as error:
        print(type(error).__name__ + ": " + str(error), flush=True)
        raise SystemExit(1)
