# YZBot 交接提示词 V13（2026-09-06，agent50）

## 当前权威状态

- VM 权威源码：`/home/ros/dev_ws/src/yzbot`，HEAD `af1d2ab`，实际运行环境为 ROS 2 Humble + Gazebo。
- 本轮未合入导航候选；`task_navigator.py` 已恢复默认 `obstacle_max_reverses=1`，候选改动与测试文件均已回退。
- 保留既有用户工作区差异：`tools_demo/config/cube_order_policy.json` 与历史未跟踪文档，不要覆盖或 reset。

## 本轮证据

- 4rB_1bA 失败复现：`/home/ros/dev_ws/experiments/ordered-cases/20260906T062829Z-oWbhdT`，red4 已取到，B 东侧镜像投放目标导航约 148.51s 后 abort，随后级联 skip。
- 同题 bag 复现基线：`/home/ros/dev_ws/experiments/targeted-4rB_1bA-20260906T092243Z`，271.8s，5/5，`CASE_ACCEPTED=true`。
- 单变量 B 南侧中间点候选：`/home/ros/dev_ws/experiments/targeted-4rB_1bA-20260906T095654Z`，258.7s，completed=5/skipped=0，但 `CASE_ACCEPTED=false`；安全证据 `avoidance_demonstrated=false`、`minimum_clearance=0.0832`，因此已回退，不能宣传为修复。

## 下一步

1. 优先处理 B 东侧进场与 obstacle_1 运动轨迹的安全证据耦合；不要再只增加 reverse 次数或直接追加未验证 waypoint。
2. 任何候选必须同时满足 5/5、`CASE_ACCEPTED=true`、双障碍证据与净空阈值；只看 `completed=5` 不够。
3. 每个候选保留独立 launch/result/rosbag，失败立即回退；先做离线动态轨迹与时间窗分析，再进行一次单变量完整回归。
4. 未完成项仍包括 4bC 族 300s、真实 API+CoStudio 全墙钟验收、arm 首用 CONTROL_FAILED 根因和 Windows CoStudio 验收。

## 复现命令

```bash
cd /home/ros/dev_ws/src/yzbot
bash tools_demo/scripts/run_ordered_case.sh 'Move 4 red cubes to zone B and 1 blue cube to zone A.' /home/ros/dev_ws/experiments/next-4rB_1bA
```
