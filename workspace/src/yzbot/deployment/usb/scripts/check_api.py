"""Verify one real model answer in an isolated ROS domain, without a mission."""
from pathlib import Path
import sys
import time
import rclpy
from tools_demo.task_parser_node import TaskParserNode

repo, question, rules = map(Path, sys.argv[1:])
rclpy.init(args=["--ros-args", "--params-file", str(repo / "tools_demo/config/task_parser.yaml"), "-p", "use_sim_time:=false", "-p", "require_llm:=true"])
node = TaskParserNode()
try:
    start = time.monotonic()
    plan = node.parse_problem(question.read_text(), rules.read_text(), require_llm=True)
    print("真实API校验成功，用时", round(time.monotonic() - start, 3), "秒，计划", plan.to_dict())
    assert plan.source == "llm"
finally:
    node.destroy_node()
    rclpy.shutdown()
