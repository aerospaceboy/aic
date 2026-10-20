# Codex 项目持续执行计划

更新日期：2026-07-30

工作区：`/home/ros/dev_ws`

源码仓库：`/home/ros/dev_ws/src/yzbot`

目标分支：`origin/aerospaceboy`

执行状态（2026-07-30）：Step 1–8 已完成并推送；Step 9 的统一启动、赛前自检、
300 秒任务验收和最终文档已实现，正在执行最终全工作区回归与发布审计。

## 1. PDF 最终目标摘要

竞赛任务要求在 ROS 2 与 Gazebo 室内仿真中，使用搭载六自由度机械臂、
RGB-D 相机和激光雷达的移动机器人完成多轮自主分拣：

1. 从题目程序取得随机自然语言任务，使用大模型解析红、蓝方块的数量、
   执行顺序和目标区域。
2. 使用视觉识别目标颜色、位置和姿态。
3. 使用 SLAM/Nav2 在 A、B、C 区域之间自主导航，并能绕开静态和移动障碍。
4. 使用 MoveIt 控制机械臂抓取、搬运和释放方块，按照题目顺序放入指定区域。
5. 通过文字或语音播报当前任务、导航、抓取、投放和完成状态。
6. 通过统一 ROS 2 话题和可视化界面展示题目、顺序、数量、路径、工作状态、
   夹爪状态、进度和轮次完成状态。
7. 单轮完整流程需要在 5 分钟内完成，并保留可复现的代码、启动说明、
   测试记录和演示资料。

PDF 评分点同时覆盖系统完整启动、大模型解析与交互、抓取、准确投放、
移动障碍避障、状态展示、场景复杂度以及项目文档。

## 2. 当前项目结构

`/home/ros/dev_ws` 是 colcon 工作区，`build/`、`install/`、`log/` 为生成目录。
真正的 Git 仓库位于 `src/yzbot/`；工作区根目录的 `.git/` 为空目录，不能用于
版本控制。因此项目文档在仓库内跟踪为 `src/yzbot/docs/`，同时使用
`/home/ros/dev_ws/docs -> src/yzbot/docs` 链接满足工作区固定文档路径。

主要功能包如下：

| 包 | 当前职责 |
| --- | --- |
| `mybot_description` | 移动底盘、六轴机械臂、RGB-D、激光雷达 Xacro；Gazebo 世界 |
| `mybot` | MoveIt/SRDF、ros2_control、控制器、Gazebo 与 RViz 启动；安全吸附脚本 |
| `bot_navigation` | Nav2 参数、地图、Cartographer/RTAB-Map 与导航启动 |
| `tools_demo` | 现有本地 Llama HTTP 聊天示例和 ROS 字符串发布示例 |
| `linkattacher_msgs` | Gazebo 吸附和释放服务接口 |
| `ros2_linkattacher` | Gazebo 固定关节吸附插件 |

## 3. 已实现并核对的能力

- 组合机器人 Xacro 已包含差速底盘、六轴机械臂、夹爪、RGB-D 相机和雷达。
- Gazebo `room.world`、`cloister.world` 及多张 Nav2 地图已存在。
- Gazebo ros2_control 可加载 `joint_state_broadcaster`、`arm_controller` 和
  `gripper_controller`。
- MoveIt 规划组、KDL IK、控制器映射和 RViz 配置已存在；用户已经验证
  RViz 的 Plan + Execute 能驱动 Gazebo 中的真实机械臂模型。
- `/ATTACHLINK` 与 `/DETACHLINK` 已可用；插件已改为固定关节，并能在释放时
  删除关节记录，先前的单自由度 damping 越界问题已修复。
- `auto_grasp_moveit.py` 已实现方块安全窗口检查、吸附、小幅关节搬运、
  相对漂移验证、释放和机械臂返回。
- Nav2、Cartographer、Gazebo、MoveIt、OpenCV、NumPy、`cv_bridge` 和
  `requests` 均已安装，可在不增加系统依赖的条件下继续开发。
- Git 分支为 `aerospaceboy`，初始检查时与 `origin/aerospaceboy` 同步。

## 4. 当前未提交修改

开始执行时检测到两项用户已有修改，均予以保留：

1. `mybot/config/kinematics.yaml`：KDL 求解超时由 0.005 秒增加到 0.05 秒。
2. `mybot_description/worlds/room.world`：增加 `gazebo_ros_state` 插件，并整理
   LinkAttacher 插件格式。

这两项不会加入 Step 1 文档提交；将在 Step 2 完成构建和接口验证后单独提交。

## 5. 目标差距

当前代码仍缺少以下闭环能力：

- 没有统一任务协议、随机题目节点、结构化数量/顺序/区域输出以及可靠解析测试。
- 现有 Llama 示例是阻塞式终端程序，提示词与任务无关，缺少超时、JSON 校验和
  无模型服务时的明确降级路径。
- 没有从 RGB-D 图像生成带颜色标签的三维目标位姿。
- 现有抓取脚本只验证吸附后小幅关节运动，不会基于目标位姿执行完整
  “预抓取—下降—吸附—抬升—放置—释放”流程。
- 地面方块在当前机械臂固定朝下姿态下不可达；需要在场景中设置窄型取放台，
  或计算新的可达姿态，同时避免夹爪和台面碰撞。
- Nav2 能力尚未与任务区域、目标选择和机械臂状态机连接。
- 没有可重复的移动障碍节点与绕障验收流程。
- 没有统一状态话题、进度数据、播报节点、任务面板和轮次计时。
- 没有单命令集成启动、失败恢复、5 分钟端到端验证脚本和最终验收报告。
- `vision_msgs`、`ai_msgs`、`sound_play` 和 `pyttsx3` 当前不可用，不能把实现
  建立在这些缺失依赖上；应优先使用现有标准消息、OpenCV 和系统已有命令。

## 6. 风险与控制策略

| 风险 | 控制策略 |
| --- | --- |
| 吸附位置错误导致模型飞散 | 吸附前校验相对位姿和速度；使用固定关节；失败时立即释放 |
| 机械臂 IK 分支跳变或碰撞 | 使用当前状态作为种子，分预抓取/笛卡尔下降，检查规划比例与碰撞 |
| 机械臂无法触及地面 | 使用窄型静态取放台提高方块，同时保留足够夹爪净空 |
| Gazebo/MoveIt 启动时序不稳定 | 使用事件处理、服务/动作就绪检查和有界超时 |
| Nav2 地图与世界坐标不一致 | 逐个校验 `map -> odom -> base_link`、区域坐标和代价地图 |
| 视觉深度噪声或目标遮挡 | HSV 连通域过滤、深度中值、相机内参投影和 TF 时间检查 |
| 本地 LLM 服务未启动 | LLM 为首选解析器；提供严格规则解析作为可测试降级，并明确发布解析来源 |
| 语音依赖缺失 | 统一发布文本播报；仅在检测到系统已有 TTS 命令时启用语音 |
| 现有进程被测试干扰 | 启动前检查节点/进程；只停止本步骤自行启动的进程 |
| 远程分支新增提交 | fetch 后普通 merge，解决冲突并重测；禁止 rebase/强推 |

## 7. 编号实施步骤

### Step 1：基线审计与执行计划

目标：完成 PDF、工作区、Git、ROS 包、依赖和现有能力审计，建立计划与日志。

涉及文件：

- `docs/codex_execution_plan.md`
- `docs/codex_execution_log.md`

验证：

- 检查文档章节和目标覆盖情况。
- `git diff --check`
- 确认只暂存两份文档。
- 提交并推送到 `origin/aerospaceboy`。

### Step 2：稳定 Gazebo 状态接口与 MoveIt 基础配置

目标：验证并固化现有 `gazebo_ros_state` 和 KDL 超时修改，确认仿真、控制器、
MoveIt IK/规划服务和吸附服务可重复启动。

涉及文件：

- `mybot/config/kinematics.yaml`
- `mybot_description/worlds/room.world`
- `docs/codex_execution_log.md`

验证：

- `colcon build --symlink-install --packages-select mybot_description mybot`
- 启动 Gazebo/MoveIt 测试实例。
- 检查控制器、`/gazebo/get_entity_state`、`/compute_ik`、`/ATTACHLINK`、
  `/DETACHLINK`。
- 运行可达与不可达 IK 用例，记录实际结果。

### Step 3：完整 MoveIt 抓取—搬运—放置原语

目标：加入可达的窄型取放台和可参数化方块，基于 MoveIt 服务实现完整运动序列，
吸附前后验证方块相对位姿，释放后验证落点。

计划文件：

- `mybot_description/worlds/room.world`
- `mybot/pick_place_moveit.py`
- `mybot/CMakeLists.txt`
- `mybot/package.xml`
- `mybot/test/test_pick_place_math.py`
- `docs/codex_execution_log.md`

验证：

- Python 编译与单元测试。
- 构建 `mybot_description`、`mybot`。
- 对 Gazebo 实体运行一次完整抓取和投放。
- 读取开始、吸附、搬运、释放后的实体位姿并检查误差。

### Step 4：任务协议、大模型适配和随机题目

目标：定义稳定的任务 JSON 协议；生成 PDF 风格随机题目；使用可配置的
OpenAI 兼容本地 Llama 服务解析，并提供严格、可追踪的规则降级解析。

计划文件：

- `tools_demo/tools_demo/task_protocol.py`
- `tools_demo/tools_demo/task_question_node.py`
- `tools_demo/tools_demo/task_parser_node.py`
- `tools_demo/config/task_parser.yaml`
- `tools_demo/setup.py`
- `tools_demo/package.xml`
- `tools_demo/test/test_task_protocol.py`
- `docs/codex_execution_log.md`

验证：

- 单元测试覆盖颜色、数量、顺序、区域、非法题目和 LLM 非法 JSON。
- ROS 话题测试：
  `/competition/task/question`、`/competition/task/plan`、
  `/competition/task/counts`、`/competition/task/order`。
- 本地 LLM 可用时验证模型路径；不可用时验证降级来源被明确标记。

### Step 5：RGB-D 颜色目标识别与三维定位

目标：从现有 RGB-D 相机识别红/蓝方块，使用相机内参和有效深度估计三维位置，
转换到机器人或地图坐标系并发布调试图像与目标列表。

计划文件：

- `tools_demo/tools_demo/color_target_detector.py`
- `tools_demo/tools_demo/perception_math.py`
- `tools_demo/config/perception.yaml`
- `tools_demo/launch/perception.launch.py`
- `tools_demo/test/test_perception_math.py`
- `tools_demo/setup.py`
- `tools_demo/package.xml`
- `docs/codex_execution_log.md`

验证：

- HSV 分割、连通域、深度中值和投影数学单元测试。
- 检查 RGB、深度、CameraInfo 频率与时间戳。
- Gazebo 中对已知红/蓝方块比较视觉位姿与实体真值误差。

### Step 6：Nav2 分区导航与移动障碍

目标：定义 A/B/C 区域位姿，封装 `NavigateToPose`，加入可重复移动障碍，
验证路径、到达误差、超时、取消和绕障行为。

计划文件：

- `bot_navigation/config/competition_zones.yaml`
- `bot_navigation/param/originbot_nav2.yaml`
- `bot_navigation/launch/competition_navigation.launch.py`
- `tools_demo/tools_demo/task_navigator.py`
- `tools_demo/tools_demo/moving_obstacle.py`
- `tools_demo/test/test_navigation_contract.py`
- `tools_demo/setup.py`
- `tools_demo/package.xml`
- `docs/codex_execution_log.md`

验证：

- 构建相关包并启动 Nav2。
- 校验生命周期节点、TF、地图、代价地图和 `NavigateToPose`。
- 依次导航 A/B/C，记录到达误差与耗时。
- 打开移动障碍，确认局部路径发生调整且无碰撞。

### Step 7：任务状态机与导航—感知—抓放集成

目标：按任务顺序选择目标并执行“导航至目标—识别—抓取—导航至区域—投放”，
加入超时、重试、跳过不可用目标和安全释放。

计划文件：

- `tools_demo/tools_demo/mission_coordinator.py`
- `tools_demo/config/mission.yaml`
- `tools_demo/launch/mission.launch.py`
- `tools_demo/test/test_mission_state_machine.py`
- `mybot/pick_place_moveit.py`
- `tools_demo/setup.py`
- `tools_demo/package.xml`
- `docs/codex_execution_log.md`

验证：

- 使用模拟接口单元测试正常、导航失败、感知丢失、抓取失败和重试路径。
- Gazebo 中执行至少一红一蓝的真实完整流程。
- 从实体最终位置核对颜色、数量、顺序和区域。

### Step 8：状态话题、播报与可视化面板

目标：统一发布任务、路径、工作状态、夹爪状态、进度、倒计时和轮次完成；
提供零新增依赖的浏览器面板和文字播报，可用时启用系统 TTS。

计划文件：

- `tools_demo/tools_demo/status_dashboard.py`
- `tools_demo/tools_demo/announcer.py`
- `tools_demo/web/index.html`
- `tools_demo/config/dashboard.yaml`
- `tools_demo/launch/dashboard.launch.py`
- `tools_demo/test/test_dashboard_state.py`
- `tools_demo/setup.py`
- `tools_demo/package.xml`
- `docs/codex_execution_log.md`

验证：

- 检查所有 `/competition/...` 话题类型、QoS 和更新频率。
- 使用 HTTP 请求验证面板状态接口和静态页面。
- 检查播报文本；仅在已有 TTS 可用时执行语音冒烟测试。

### Step 9：集成启动、5 分钟验收与最终文档

目标：提供单命令启动与验收脚本，完成全工作区构建、回归测试和 PDF 逐项核对。

计划文件：

- `tools_demo/launch/competition_system.launch.py`
- `tools_demo/config/competition.yaml`
- `tools_demo/test/run_competition_acceptance.py`
- `readme.md`
- `docs/competition_operator_guide.md`
- `docs/codex_execution_plan.md`
- `docs/codex_execution_log.md`
- `docs/codex_final_report.md`

验证：

- `colcon build --symlink-install`
- `colcon test` 与 `colcon test-result --verbose`
- 独立启动完整系统，检查所有必要节点、服务、动作、控制器和 TF。
- 在 5 分钟计时内完成随机任务，保存任务、路径、抓取、投放和面板状态证据。
- 检查无持续关键报错、无误提交文件，确认远程分支包含全部提交。

## 8. 统一接口初稿

优先使用当前环境已有标准消息：

| 话题 | 类型 | 用途 |
| --- | --- | --- |
| `/competition/task/question` | `std_msgs/msg/String` | 原始自然语言题目 |
| `/competition/task/plan` | `std_msgs/msg/String` | 带 schema/version/source 的 JSON 任务 |
| `/competition/task/order` | `std_msgs/msg/String` | 人可读执行顺序 |
| `/competition/task/counts` | `std_msgs/msg/UInt32MultiArray` | 红/蓝目标数量 |
| `/competition/perception/targets` | `std_msgs/msg/String` | 带颜色和 Pose 的 JSON 目标列表 |
| `/competition/mission/state` | `std_msgs/msg/String` | 状态机当前阶段 |
| `/competition/mission/progress` | `std_msgs/msg/Float32` | 0 到 1 的完成进度 |
| `/competition/gripper/state` | `std_msgs/msg/String` | 打开、吸附、搬运、释放、故障 |
| `/competition/announcement` | `std_msgs/msg/String` | 文字/语音播报源 |
| `/competition/round/complete` | `std_msgs/msg/Bool` | 本轮完成 |
| `/plan` | `nav_msgs/msg/Path` | Nav2 全局路径 |

最终类型以实现和运行验证为准，变更会记录在执行日志和最终报告中。

## 9. 最终验收方法

最终验收以一次全新启动的独立仿真实例为准：

1. 全工作区构建和测试均通过。
2. Gazebo 场景、机器人、红蓝方块、区域和移动障碍存在。
3. 控制器、TF、RGB-D、雷达、MoveIt、Nav2、吸附服务均正常。
4. 随机题目被解析成合法结构化任务，解析来源可追踪。
5. 视觉目标位置与 Gazebo 真值误差在配置阈值内。
6. 机器人按题目顺序完成导航、抓取和分区投放，实体最终位置验证通过。
7. 移动障碍存在时仍能安全到达，日志中无碰撞事件。
8. 面板与标准话题完整反映题目、数量、顺序、路径、夹爪、进度和完成状态。
9. 流程总耗时不超过 300 秒；失败恢复至少覆盖一次可控故障。
10. 所有提交已推送至 `origin/aerospaceboy`，工作区仅保留明确记录的用户改动，
    最终报告如实说明自动与人工验证边界。
