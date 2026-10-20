# 月面仓储智能作业机器人

本仓库是 ROS 2 Humble + Gazebo Classic 竞赛项目。机器人由移动底盘、六自由度
机械臂、夹爪、RGB-D 相机和激光雷达组成，可解析自然语言分拣任务，在动态障碍
环境中导航到红/蓝取料架，识别并抓取方块，再按顺序投放到 A/B/C 目标区。

## 已实现能力

- 随机中英文任务生成；竞赛模式强制使用 OpenAI 兼容大模型，
  严格规则解析仅用于开发调试。
- RGB-D 颜色分割、深度中值、相机投影和 TF 三维坐标变换。
- Nav2 分区导航、取料架分段进出路线、移动障碍和任务取消。
- MoveIt 规划、真实 Gazebo 控制器执行、固定关节吸附、释放和落区真值验证。
- 最多 5 个方块的有序任务状态机、超时、重试、跳过与失败恢复。
- 浏览器评分面板：题目、顺序、红蓝数量、进度、状态、路径、机器人位姿。
- v2 实时路径画布：实际行驶轨迹、直接停靠指令路径、Nav2 全局/局部路径和障碍位置。
- 出生点移动障碍避障：两个反相位板子（obstacle_1/3）经静态 sweep ghost 绕行，
  全局路径绕开 x=13.4 扫带，避免高速穿板子。
- 一条 launch 命令启动整套系统，并自动检查节点、接口类型、发布者、动作和 TF。

## 环境

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Classic 11
- MoveIt 2、Nav2、ros2_control

当前默认赛事环境为 `mybot_description/worlds/offic_room.world` 和
`bot_navigation/maps/mapn3.yaml`。`room.world`、`map.yaml` 等只作历史参考。

当前唯一规则依据为 `大模型技术创新赛比赛规则2026.pdf`（2026-06-17，7 页）。

当前权威布局（`bot_navigation/config/competition_zones.yaml`）：

- 机器人出生点 `(15.0, -2.0)`，yaw π（面朝 -X）。
- Zone：A `(-2.41, 2.99)`（中心 `(-2.01, 2.99)`，本轮未移动）、B `(-0.40, -12.00)`（中心 `(0, -12)`）、C `(-9.40, -7.00)`（中心 `(-9, -7)`）。
- 三个移动障碍物：`obstacle_1`/`obstacle_3` 为两块 0.06×0.80×0.60 m 板子，在
  出生点走廊 x=13.4 沿 y 反相位摆动（周期 12 s）；`obstacle_2` 为半径 0.18 m 圆柱
  横穿主走廊（周期 20 s）。

导航为 **per-cube**：每个物块有自己的 dock 和 entry，Nav2 规划到 entry 后由有界
直靠控制器完成最后一段。出生点板子避障见下文「出生点移动障碍避障」。

按 2026 规则，抓取与放置**逐件各 4 分、分别计分**，优化目标是 5 分钟内期望总分最高。

源码工作区示例为 `/home/ros/dev_ws/src/yzbot`。

## 构建

```bash
cd ~/dev_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 一键启动

```bash
ros2 launch tools_demo competition_system.launch.py
```

默认行为：

- `start_llm_server:=True`：自动启动虚拟机内置的 llama.cpp 与
  Qwen2.5-Coder 0.5B GGUF 模型，无需另开终端；
- `headless:=False`：打开 Gazebo 图形客户端，可以直接观察机器人和方块。
- `generate_question:=True`：系统就绪后生成一轮随机任务。
- `use_llm:=True`、`require_llm:=True`：竞赛模式要求本地 OpenAI 兼容服务返回有效计划。
- `moving_obstacle:=True`、`dashboard:=True`、`run_acceptance:=True`。

仅在不需要观察窗口时切换为无界面模式：

```bash
ros2 launch tools_demo competition_system.launch.py headless:=True
```

宿主机打开 `http://192.168.153.128:8080/` 查看实时评分面板。短距离 v2 航段会直接闭环
停靠，此时画布显示指令路径和实际轨迹；Nav2 介入时同时显示全局与局部避障路径。

## 常用比赛模式

无本地大模型服务时，只能进入不计大模型验收的开发调试模式：

```bash
ros2 launch tools_demo competition_system.launch.py \
  use_llm:=False require_llm:=False run_acceptance:=False
```

手动发布指定题目前，关闭随机出题：

```bash
ros2 launch tools_demo competition_system.launch.py \
  generate_question:=False use_llm:=False \
  require_llm:=False run_acceptance:=False
```

详细操作、验收、故障恢复和评分点对应关系见
[`docs/competition_operator_guide.md`](docs/competition_operator_guide.md) 与
[`docs/codex_final_report.md`](docs/codex_final_report.md)。

## PDF 满分验收模式

当前竞赛模式要求真实大模型、5 个方块全部完成、移动障碍物安全避障和
实时 SLAM 证据同时成立。规则解析只保留为开发调试能力，不再被竞赛验收视为
大模型成功。

```bash
ros2 launch tools_demo competition_system.launch.py \
  use_llm:=True require_llm:=True \
  moving_obstacle:=True slam_evidence:=True
```

成功验收必须同时满足：

- 任务计划来源为 `llm`；
- `completed == total == 5` 且 `skipped == 0`；
- 三个动态障碍物均被观测且未检测到碰撞（`obstacles_seen>=2`、`safe=true`）；
- `/competition/slam/map` 存在真实发布者；
- 任务在 300 秒内结束。

提交技术文档和赛前清单分别见
[`docs/competition_submission_document.md`](docs/competition_submission_document.md)
与 [`docs/submission_checklist.md`](docs/submission_checklist.md)。

当前进度与待办见 [`PROJECT_HANDOFF.md`](PROJECT_HANDOFF.md)（出生点板子避障采用
sweep ghost 绕行，300 s 目标冲刺中）。
