#!/usr/bin/env python3
"""Run sequential 5-cube trials with a system restart between each."""
import json
from pathlib import Path
import subprocess
import sys
import time
import urllib.request

COUNT = int(sys.argv[1]) if len(sys.argv) > 1 else 3
PREFIX = sys.argv[2] if len(sys.argv) > 2 else "batch"
CLEAN_LAUNCH = Path(__file__).resolve().with_name("clean_launch.sh")
FULL_Q = ("每个衣柜有5件衣物;有T恤、裤子两种衣柜;小美需要2件T恤,1条裤子;"
          "小帅、小丽都需要7件T恤;小娜需要2件T恤,1条裤子;小杰需要1条裤子。"
          "设T恤衣柜数量为x,裤子衣柜数量为y,计算x、y分别为多少。")
RULES = "x代表红色,y代表蓝色,数量大于等于3的去A区,其他颜色去C区"


def launch():
    subprocess.run(
        ["bash", str(CLEAN_LAUNCH)],
        check=True,
        timeout=60,
    )
    deadline = time.time() + 300
    while time.time() < deadline:
        try:
            d = json.loads(urllib.request.urlopen(
                "http://127.0.0.1:8080/api/state", timeout=5).read())
            robot = d.get("robot") or {}
            ready = (
                d.get("mission", {}).get("state") == "idle"
                and robot.get("x") is not None
            )
            if ready:
                return True
        except Exception:
            pass
        time.sleep(5)
    return False


def post(task_id):
    payload = {"question": FULL_Q, "mapping_rules": RULES, "task_id": task_id}
    req = urllib.request.Request(
        "http://127.0.0.1:8080/api/task",
        data=json.dumps(payload).encode("utf-8"),
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    return urllib.request.urlopen(req, timeout=10).status


def wait_terminal(task_id, timeout=340):
    start = time.time()
    while time.time() - start < timeout:
        try:
            d = json.loads(urllib.request.urlopen(
                "http://127.0.0.1:8080/api/state", timeout=5).read())
            m = d.get("mission", {})
            if (m.get("task_id") == task_id
                    and m.get("state") in ("completed", "failed", "canceled")):
                return m, d.get("avoidance", {})
        except Exception:
            pass
        time.sleep(3)
    return None, None


results = []
for i in range(1, COUNT + 1):
    tid = f"{PREFIX}_{i:02d}"
    print(f"=== trial {i}/{COUNT} task={tid} ===", flush=True)
    ok = launch()
    if not ok:
        print("  system NOT ready", flush=True)
        results.append("not_ready")
        continue
    try:
        post(tid)
    except Exception as e:
        print(f"  POST failed: {e}", flush=True)
        results.append("post_failed")
        continue
    m, av = wait_terminal(tid)
    if m is None:
        print("  TIMEOUT", flush=True)
        results.append("timeout")
        continue
    line = (f"  result: state={m.get('state')} completed={m.get('completed')} "
            f"skipped={m.get('skipped')} total={m.get('total')} "
            f"elapsed={round(m.get('elapsed_sec', 0), 1)}s")
    print(line, flush=True)
    if av:
        min_clr = av.get("minimum_clearance")
        print(
            f"  avoidance: collision={av.get('collision_detected')} "
            f"min_clr={round(min_clr, 3) if min_clr is not None else 'n/a'}",
            flush=True,
        )
    results.append(
        f"{m.get('completed')}/{m.get('total')} "
        f"collision={av.get('collision_detected')}"
    )


print("BATCH DONE", flush=True)
for r in results:
    print("RESULT:", r)
