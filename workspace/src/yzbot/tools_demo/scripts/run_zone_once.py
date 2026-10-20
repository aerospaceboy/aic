#!/usr/bin/env python3
"""Run one zone round (AB/AC/BC) as a regression gate.

    cd /home/ros/huxuyang/YZBot-224 && source scripts/00_env.sh
    python3 workspace/src/yzbot/tools_demo/scripts/run_zone_once.py AC 1

Set COSTUDIO_HEADLESS=False and DISPLAY=:0 to keep the Gazebo GUI open for
the round (the runner restarts the whole stack, so the GUI has to be enabled
through the restart).
"""
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import run_zone9_campaign as campaign  # noqa: E402

pair = sys.argv[1]
iteration = sys.argv[2] if len(sys.argv) > 2 else "1"
record = campaign.run_round(1, pair, iteration)
print("RECORD=" + json.dumps(record, ensure_ascii=False))
