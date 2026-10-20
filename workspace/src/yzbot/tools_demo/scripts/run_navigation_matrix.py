#!/usr/bin/env python3
"""Run 10 clean cube-exit starts against all three delivery zones."""

import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import yaml


REPO = Path(__file__).resolve().parents[2]


def _cases():
    zones = yaml.safe_load(
        (REPO / "bot_navigation/config/competition_zones.yaml")
        .read_text(encoding="utf-8")
    )["competition_task_navigator"]["ros__parameters"]
    risk = yaml.safe_load(
        (REPO / "bot_navigation/config/navigation_risk_zones.yaml")
        .read_text(encoding="utf-8")
    )["zone_strategies"]
    for color in ("red", "blue"):
        for index in range(1, 6):
            cube = f"{color}_cube_{index}"
            key = f"{cube}_entry_aisle" if color == "red" else f"{cube}_entry"
            start = zones[key]
            allowed = zones.get("cube_approach_allowlist", [])
            if allowed:
                dock = zones[cube]
                side = next(side for side in ("west", "east")
                            if f"{cube}:{side}" in allowed)
                sign = -1 if side == "west" else 1
                start = [dock[0] + .36 + sign * .9, dock[1],
                         0.0 if sign == -1 else 3.141592653589793]
            for zone in ("A", "B", "C"):
                yield cube, zone, start, risk[zone]["place_pose"]


def _terminate(process):
    def signal_group(value):
        try:
            os.killpg(process.pid, value)
        except ProcessLookupError:
            pass

    signal_group(signal.SIGINT)
    if process.poll() is None:
        try:
            process.wait(timeout=6)
        except subprocess.TimeoutExpired:
            pass
    # ros2 launch may exit before a composable container completes teardown.
    signal_group(signal.SIGTERM)
    time.sleep(1.0)
    signal_group(signal.SIGKILL)
    if process.poll() is None:
        try:
            process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            pass


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--start", type=int, default=1)
    parser.add_argument("--limit", type=int, default=30)
    parser.add_argument("--match", default="")
    parser.add_argument("--timeout", type=float, default=90.0)
    parser.add_argument("--domain", type=int, default=41)
    parser.add_argument("--moving-obstacle", action="store_true")
    parser.add_argument("--output", default="/tmp/navigation_matrix_report.json")
    args = parser.parse_args()
    reports = []

    for number, (cube, zone, start, target) in enumerate(_cases(), 1):
        if number < args.start:
            continue
        if args.match and args.match not in f"{cube}_to_{zone}":
            continue
        if number > args.limit:
            break
        domain = ((args.domain + number - 1) % 200) + 1
        environment = dict(
            os.environ,
            ROS_DOMAIN_ID=str(domain),
            GAZEBO_MASTER_URI=f"http://127.0.0.1:{11345 + number}",
        )
        case = f"{number:02d}_{cube}_to_{zone}"
        log_path = Path(f"/tmp/navigation_matrix_{case}.log")
        command = [
            "ros2", "launch", "tools_demo", "navigation_matrix_case.launch.py",
            f"robot_x:={start[0]}", f"robot_y:={start[1]}",
            f"robot_yaw:={start[2]}",
            f"moving_obstacle:={'True' if args.moving_obstacle else 'False'}",
        ]
        with log_path.open("w", encoding="utf-8") as log:
            launch = subprocess.Popen(
                command, env=environment, stdout=log, stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            try:
                time.sleep(2.0)
                if launch.poll() is not None:
                    result = {
                        "status": "launch_failed",
                        "returncode": launch.returncode,
                    }
                    raise RuntimeError("navigation launch exited early")
                probe = subprocess.run(
                    [
                        "ros2", "run", "tools_demo", "navigation_matrix_probe",
                        "--x", str(target[0]), "--y", str(target[1]),
                        "--yaw", str(target[2]), "--timeout", str(args.timeout),
                    ],
                    env=environment,
                    capture_output=True,
                    text=True,
                    timeout=args.timeout + 90.0,
                )
                marker = next(
                    (
                        line
                        for line in probe.stdout.splitlines()
                        if line.startswith("MATRIX_RESULT=")
                    ),
                    None,
                )
                result = json.loads(marker.split("=", 1)[1]) if marker else {
                    "status": "probe_error", "stdout": probe.stdout[-1000:],
                    "stderr": probe.stderr[-1000:],
                }
            except subprocess.TimeoutExpired:
                result = {"status": "harness_timeout"}
            except RuntimeError:
                pass
            finally:
                _terminate(launch)
                time.sleep(3.0)
        result.update(
            {
                "case": case,
                "cube": cube,
                "zone": zone,
                "domain": domain,
                "log": str(log_path),
            }
        )
        reports.append(result)
        print(json.dumps(result, sort_keys=True), flush=True)
        Path(args.output).write_text(json.dumps(reports, indent=2), encoding="utf-8")

    failures = [item for item in reports if item.get("status") != "succeeded"]
    print(f"matrix: {len(reports) - len(failures)}/{len(reports)} succeeded")
    raise SystemExit(1 if failures else 0)


if __name__ == "__main__":
    main()
