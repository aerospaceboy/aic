# 竞赛操作手册

## 1. 赛前准备

进入虚拟机终端：

```bash
cd ~/dev_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

确认没有旧的 Gazebo、MoveIt 或 Nav2 实例占用资源：

```bash
ps -eo pid,pgid,stat,cmd | grep -E \
  'gzserver|gazebo|move_group|nav2|competition_system' \
  | grep -v grep
```

如果输出了旧实例，先在启动它的终端按 `Ctrl+C`，等待其正常退出。不要在不核对
PID/PGID 的情况下使用宽泛的 `killall`。

## 2. 一键启动

比赛推荐命令：

```bash
ros2 launch tools_demo competition_system.launch.py
```

默认 `headless:=False`，Gazebo 窗口会显示在 Ubuntu 桌面中。

启动过程按以下顺序自动执行：

1. 立即启动 Gazebo、机器人、控制器和 LinkAttacher。
2. 15 秒后并行启动 MoveIt 和 Nav2。
3. 45 秒后启动任务解析、RGB-D 识别、任务协调器和面板。
4. 48 秒后启动非破坏性赛前自检。
5. 默认在任务层启动后生成一轮随机题目。

看到以下日志表示启动契约全部通过：

```text
COMPETITION_ACCEPTANCE=PASS
```

自检证据默认写入：

```text
/tmp/competition_acceptance.json
```

本虚拟机在冷启动测试中约 54 秒达到 PASS。

## 3. 显示模式和 RGB-D

默认 `headless:=False`，同时显示 Gazebo 图形客户端。统一启动文件会在 SSH
环境中自动发现当前 Ubuntu 桌面的 `DISPLAY=:0` 与 Xauthority，因此从远程
终端启动时窗口也会显示在虚拟机桌面中。

仅在不需要观察窗口时：

```bash
ros2 launch tools_demo competition_system.launch.py headless:=True
```

如果 RGB-D 没有输出，检查：

```bash
ros2 topic info /camera_rgbd/camera_rgbd/image_raw --verbose
ros2 topic info /camera_rgbd/camera_rgbd/depth/image_raw --verbose
```

两者的 `Publisher count` 都应至少为 1。自检也会检查该条件，不会把只有订阅者、
没有真实传感器发布者的空话题误判为成功。

## 4. 面板

在 Windows 宿主机浏览器打开：

```text
http://192.168.153.128:8080/
```

健康检查和机器可读状态：

```bash
curl http://127.0.0.1:8080/healthz
curl http://127.0.0.1:8080/api/state
```

面板只读，不提供任何控制接口。它显示：

- 自然语言题目和解析来源；
- 执行顺序、红/蓝数量和目标区域；
- 5 分钟剩余时间、完成数和跳过数；
- 当前导航、视觉、抓取、夹具和任务状态；
- Nav2 全局路径及机器人实时位姿。

## 5. 题目与解析

统一启动默认用 `start_llm_server:=True` 自动运行虚拟机内置的
`llama-server` 和 Qwen2.5-Coder 0.5B GGUF 模型，并通过
`http://localhost:8081/v1/chat/completions` 提供 OpenAI 兼容服务。
如需连接已经单独启动的模型服务，传入 `start_llm_server:=False`。
竞赛模式要求模型输出通过结构与题面交叉校验；服务不可用、超时或 JSON
校验失败时，本轮任务会被拒绝，避免规则解析冒充大模型成功。

没有模型服务时只能用于开发调试，并关闭竞赛验收：

```bash
ros2 launch tools_demo competition_system.launch.py \
  use_llm:=False require_llm:=False run_acceptance:=False
```

需要手动测试题目时，先用 `generate_question:=False` 启动，再持续发布 3 秒，
让所有 transient-local 订阅者完成发现：

```bash
timeout 3 ros2 topic pub \
  --qos-durability transient_local \
  --rate 2 \
  /competition/task/question \
  std_msgs/msg/String \
  "{data: 'Move 1 blue cube to zone C.'}"
```

## 6. 五分钟端到端验收

系统运行后，在另一个终端执行：

```bash
source /opt/ros/humble/setup.bash
source ~/dev_ws/install/setup.bash
ros2 run tools_demo competition_acceptance \
  --ros-args \
  --params-file \
  ~/dev_ws/install/tools_demo/share/tools_demo/config/competition.yaml \
  -p monitor_mission:=true \
  -p evidence_path:=/tmp/competition_timed_acceptance.json
```

该模式先检查启动契约，再等待任务终态；`mission_timeout_sec` 被限制在
`(0, 300]` 秒。只有任务状态为 `completed` 且完成数大于 0 才返回成功。

2026-07-30 的单块历史基线实测（不等同于当前五块满分验收）：

- 题目：`Move 1 blue cube to zone C.`
- 任务耗时：139.7 秒。
- 验收总耗时：143.128 秒。
- 结果：`completed=1`、`skipped=0`、`COMPETITION_ACCEPTANCE=PASS`。
- Gazebo 真值：`blue_cube_1=(2.7585,-0.8803,0.0155)`，静止且位于 C 区。

## 7. 常用检查

```bash
ros2 node list
ros2 action list -t
ros2 service list -t
ros2 control list_controllers
ros2 lifecycle get /bt_navigator
ros2 topic echo /competition/mission/state \
  --qos-durability transient_local --once
```

关键服务：

- `/ATTACHLINK`、`/DETACHLINK`
- `/gazebo/get_entity_state`
- `/controller_manager/list_controllers`
- `/compute_ik`

关键动作：

- `/navigate_to_pose`
- `/arm_controller/follow_joint_trajectory`
- `/gripper_controller/follow_joint_trajectory`
- `/move_action`

## 8. 故障恢复

| 现象 | 检查与处理 |
| --- | --- |
| 面板打不开 | 检查 `competition_dashboard`、8080 端口和 `/healthz` |
| 自检缺少 RGB-D publisher | 确认 Ubuntu 桌面已登录；检查 DISPLAY/Xauthority 和 Gazebo 相机日志 |
| LLM 连接拒绝 | 启动并预热 OpenAI 兼容模型服务；竞赛模式不得使用规则结果替代 |
| 找不到方块 | 确认已到取料架、RGB/深度均有发布者、感知话题 targets 非空 |
| 返回 already attached | 先调用 `/DETACHLINK`，再核对模型名与 link 名 |
| 导航超时/取消 | 查看 `/competition/navigation/state` 和 error 话题，重新启动该轮 |
| 任务失败 | 查看 mission 的 `detail`、`skipped` 和 `/competition/mission/error` |
| Gazebo 退出慢 | 先等待 `Ctrl+C` 正常退出，再精确核对进程组后处理残留 |

Gazebo 的 pose/contact queue limit 警告通常只打印一次，表示可视化传输队列丢弃
旧消息，不代表物理任务失败。MoveIt 的 OctoMap 3D 传感器插件提示也是一次性
非致命信息；当前抓取仍使用规划场景、几何碰撞检查和真实 Gazebo 执行。
