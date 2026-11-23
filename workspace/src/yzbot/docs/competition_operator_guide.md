# 比赛操作说明

完整部署命令见项目根目录 [README](../../../../README.md)。

1. 在 Ubuntu 图形桌面进入项目目录，执行 `bash 启动本版本.sh`。
2. 等待 `MANUAL_PROBLEM_MODE=PASS`，确认 Gazebo 主窗口、RViz、CoStudio 和网页面板正常。
3. CoStudio 使用 coScene WebSocket 连接 `ws://127.0.0.1:21274`，导入根目录 `costudio/YZBot比赛布局.json`。
4. 正式出题执行 `bash scripts/06_generate_question.sh`，练习时追加 `--practice`。
5. 填写题目正文及对应映射规则，检查后确认提交。API 校验失败、题目不一致或安全链未就绪时，应处理故障后重新开始。
6. 观察视觉目标、导航路径、夹取、运输和区域投放；机器人自动选择合法物块与任务顺序。
7. 结束后检查五件物块全部完成、无跳过、真实模型解析、视觉取件来源、实际落区、用时和避障记录。没有自然触发的有效避障指标应如实保留。

网页面板地址：`http://127.0.0.1:8080/`。

任务执行中不要重复启动仿真。查看当前日志使用启动输出中的路径；环境连接检查使用 `bash scripts/07_check_connection.sh`。停止本项目栈可在项目目录、正确加载环境后执行：

```bash
source scripts/lib.sh
load_ros
CLEAN_ONLY=True bash workspace/src/yzbot/tools_demo/scripts/clean_launch.sh
```

该清理脚本会终止机器人仿真栈，不能在其他任务执行中使用。

API Key 位于用户私有目录，不应出现在源码、提交 ZIP、截图或视频中。发布包不包含 Key，目标机器需单独配置。

## 移动障碍物（部署默认 17 个）

部署与验收默认开启 dense 分布式障碍（`YZBOT_DENSE_SCENE` 默认 `1`，由 `tools_demo/transport_scene.py` 的 `dense_enabled()` 读取）：

- 世界文件 `mybot_description/worlds/offic_room.world` 内的 4 个原有障碍：`competition_moving_obstacle_1/2`（橙色圆柱）与 `crossing_cart_1/2`（紫色横穿小车）；
- 运行时由 `competition_dense_actors` 通过 `/spawn_entity` 逐个生成 13 个分布式障碍 `transport_mover_01..13`，参数在 `bot_navigation/config/dense_transport_scene.json`（`new_actor_count: 13`、`original_actor_count: 4`）；它们不在 world 文件里，SDF 的 `<static>true</static>` 只表示不参与动力学，位置由插件每帧写入；
- 分布式障碍由 Gazebo 插件 `libdense_patrol.so` 在 `WorldUpdateBegin` 中写位姿，按 A↔B 三角波往返（0.28–0.36 m/s），遇到机器人或物块会反向退让；收到 `/competition/task/plan` 后才开始巡逻；
- dense 模式把 `bot_navigation/config/dense_scene_overrides.yaml` 合并进 `competition_avoidance_monitor` / `competition_dynamic_obstacles`，其中 `expected_obstacle_count: 17`；`competition_zones.yaml` 里的 `expected_obstacle_count: 4` 只是回退基准值，不要删除；
- 启动日志会打印 `dense_scene actors=13 static_world_unchanged=true`。运行期核对：`ros2 param get /competition_avoidance_monitor expected_obstacle_count` 应为 17，`ros2 topic list | grep /competition/obstacles/` 应有 17 个话题。

验收必须在默认 dense 场景下进行，不得设置 `YZBOT_DENSE_SCENE=0` 关闭分布式障碍来通过测试；该开关只用于 4 障碍物回退回归。

## 出生通道参数

两个圆柱障碍物的参数位于 `bot_navigation/config/competition_zones.yaml`。它们持续往返运动，任务开始时对齐初始相位；不得通过停止障碍物或更改避障计数制造通过结果。

| 参数 | 障碍物 1 | 障碍物 2 |
|---|---|---|
| 固定 X / Y 运动范围（m） | 8.50 / [-0.80, 0.60] | 5.00 / [-0.80, 0.60] |
| 指令移动速度（m/s） | 0.225 | 0.225 |
| 往返周期（s） | 12.4444 | 12.4444 |
| 任务起始相位偏移（s） | 7.7 | 11.9 |

小车在完成车道对齐后，进场与穿行速度上限为 1.10 m/s；接近等待点时仍保留减速、停车和位置精度检查。出生通道内侧前方的障碍物使用前半平面保护，准入预测与实时限流共用同一计算；其他路线继续使用普通保护。第二个障碍物的任务相位使小车通常在原安全位置等待约 2～3 秒；实际放行仍以新鲜观测和完整通行窗口为准，不设置固定停车延时。

修改障碍物周期时，必须同步 `tools_demo/navigation_math.py` 中的速度、周期常量并重新验证完整穿行。有效避障仍要求对两个不同障碍物实际产生运动响应，未触发时如实记录。
