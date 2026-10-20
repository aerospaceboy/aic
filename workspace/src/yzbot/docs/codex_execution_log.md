# Codex 项目执行日志

本日志记录每个编号开发步骤的目标、修改、构建、测试、提交和推送结果。
Git 提交无法在自身内容中包含自己的最终哈希，因此每一步的实际哈希会在
提交后记录到下一次日志更新和最终报告中，同时可由对应的唯一 commit message
直接查询。

## Step 1：基线审计与执行计划

- 日期：2026-07-30
- 目标：分析竞赛 PDF、远程工作区、Git、ROS 2 能力与目标差距，建立可执行计划。
- 修改文件：
  - `docs/codex_execution_plan.md`
  - `docs/codex_execution_log.md`
- 关键实现：
  - 汇总 PDF 最终目标和评分覆盖点。
  - 记录六个现有 ROS 包、已有 Gazebo/MoveIt/Nav2/LinkAttacher 能力。
  - 识别任务协议、视觉、完整抓放、导航集成、移动障碍、面板和验收缺口。
  - 形成九个可独立验证、提交和推送的编号步骤。
  - 由于 `/home/ros/dev_ws/.git` 为空，确定以 `src/yzbot` 为实际仓库，并将
    工作区根 `docs` 链接到仓库内受版本控制的 `docs/`。
- 基线 Git 检查：
  - 分支：`aerospaceboy`
  - 远程：`origin = ssh://git@ssh.github.com:443/aerospaceboy/mybot.git`
  - 开始提交：`06470ae Use fixed joints for link attachments`
  - `git fetch origin`：成功，本地与 `origin/aerospaceboy` 同步。
  - 保留的用户修改：`mybot/config/kinematics.yaml`、
    `mybot_description/worlds/room.world`。
- 环境检查：
  - Ubuntu 22.04、ROS 2 Humble、工作区 `/home/ros/dev_ws`。
  - 包：`bot_navigation`、`linkattacher_msgs`、`mybot`、
    `mybot_description`、`ros2_linkattacher`、`tools_demo`。
  - 已有依赖：Nav2、Cartographer、Gazebo ROS、MoveIt、OpenCV、
    `cv_bridge`、NumPy、requests、PyYAML。
  - 缺失且不安装的可选依赖：`vision_msgs`、`ai_msgs`、`sound_play`、
    `pyttsx3`。
  - ROS 运行态审计时没有用户业务节点；仅发现 ROS CLI 临时基础话题。
- 构建命令：本步骤仅修改 Markdown 文档，无需构建 ROS 包。
- 测试命令：
  - `git diff --check`
  - 文档章节与计划覆盖人工静态检查
  - 提交前 `git status --short`、`git diff`、`git diff --cached`
- 测试结果：文档路径、必需章节和 `git diff --check` 均通过；提交前确认仅两份
  新文档进入 Step 1 暂存范围，用户的两项已有修改保持未暂存。
- 遇到的问题：
  - 工作区根 `.git/` 是空目录，不能提交根 `docs/`。
  - 初次 ROS 审计命令受 shell 引号和无界 ROS CLI 等待影响。
- 解决方法：
  - 在真实源码仓库内跟踪 `docs/`，根目录建立相对符号链接。
  - 使用 `/tmp` 中的只读审计脚本和每条 ROS CLI 命令超时保护。
- Commit message：`step-1: add project execution plan`
- Commit hash：`f9b79e7e3052009f10ad8d1afdddb248ef660da4`
- 推送目标：`origin/aerospaceboy`
- 推送结果：成功；本地 `HEAD` 与 `origin/aerospaceboy` 均为 `f9b79e7`。
- 剩余任务：执行 Step 2 至 Step 9。

## Step 2：稳定 Gazebo 状态接口与 MoveIt 基础配置

- 日期：2026-07-30
- 目标：验证并固化用户已有的 Gazebo 状态插件和 KDL IK 超时修改。
- 修改文件：
  - `mybot/config/kinematics.yaml`
  - `mybot_description/worlds/room.world`
  - `docs/codex_execution_log.md`
- 关键实现：
  - 将 KDL IK 超时从 0.005 秒调整为 0.05 秒，减少仿真中正常可达目标的偶发超时。
  - 在 `room.world` 加载 `libgazebo_ros_state.so`，命名空间为 `/gazebo`，
    更新率为 20 Hz，使标准实体状态服务稳定可用。
  - 保持 LinkAttacher 插件加载，并修正新增 XML 块的行尾和尾部制表符。
- 静态检查：
  - Python `xml.etree.ElementTree` 解析 `room.world`：通过。
  - PyYAML 解析 `kinematics.yaml` 并断言 timeout 为 0.05：通过。
  - `git diff --check`：通过。
- 构建命令：
  - `colcon build --symlink-install --packages-select mybot_description mybot`
- 构建结果：2 个包成功，0 个失败。
- 测试命令：
  - `colcon test --packages-select mybot_description mybot`
  - `colcon test-result --verbose --test-result-base build/mybot_description`
  - 独立 headless Gazebo 启动，随后启动 `mybot move_group.launch.py`。
  - `ros2 node list`
  - `ros2 action list -t`
  - `ros2 control list_controllers`
  - `ros2 control list_hardware_interfaces`
  - 调用 `/gazebo/get_entity_state` 查询 `six_arm`。
  - 检查 `/ATTACHLINK`、`/DETACHLINK`、`/compute_ik`。
  - 使用当前关节状态对 9 个目标执行 `/compute_ik`。
- 测试结果：
  - `mybot_description`：3 个测试、0 错误、0 失败、1 跳过；`mybot` 无测试。
  - `joint_state_broadcaster`、`arm_controller`、`gripper_controller` 全部
    `active`，7 个位置 command interface 均已 claimed。
  - `/gazebo/get_entity_state` 成功返回 `six_arm` 世界位姿。
  - MoveIt 提供 `/move_action`、`/execute_trajectory`、`/compute_ik`；
    机械臂和夹爪轨迹动作存在。
  - 当前姿态、前方 0.37/0.40 米以及取放台候选姿态可求解；
    低于机械臂工作空间的目标稳定返回 `NO_IK (-31)`，符合预期。
  - 测试使用独立 PID/进程组启动，验证后只清理本步骤进程。
- 遇到的问题：
  - 第一次 `git diff --check` 检出新增 world 插件块的 CRLF 尾随空白。
  - headless `gzserver` 无显示设备，RGB-D 渲染传感器未启用。
  - 全局 `colcon test-result` 同时报告 2026-07-28 遗留的
    `bot_navigation` lint 失败，不属于本步骤运行结果。
- 解决方法：
  - 仅重写新增插件块行尾，未格式化或改动大型 world 的其他内容。
  - 本步骤验证状态、控制和 IK；视觉运行测试留到 Step 5，并使用虚拟机现有 X11。
  - 使用包级 test result base 核对本次目标包，结果为 0 失败；历史导航 lint
    问题将在导航步骤处理。
- Commit message：`step-2: stabilize Gazebo state and MoveIt IK`
- Commit hash：`9745153579a11c193ce71b8ba58813c388e0372c`
- 推送目标：`origin/aerospaceboy`
- 推送结果：成功；本地 `HEAD` 与 `origin/aerospaceboy` 均为 `9745153`。
- 剩余任务：执行 Step 3 至 Step 9。

## Step 3：完整 MoveIt 抓取—搬运—放置原语

- 日期：2026-07-30
- 目标：在 Gazebo 中实现可重复的完整“预抓取—下降—吸附—抬升—搬运—
  放置—释放—验证—回零”流程，并消除吸附时底盘飞散和 MoveIt 仿真时间不同步。
- 修改文件：
  - `IFRA_LinkAttacher/ros2_LinkAttacher/src/gazebo_link_attacher.cpp`
  - `mybot/CMakeLists.txt`
  - `mybot/launch/move_group.launch.py`
  - `mybot/package.xml`
  - `mybot/pick_place_moveit.py`
  - `mybot/test/test_pick_place_math.py`
  - `mybot_description/worlds/room.world`
  - `docs/codex_execution_log.md`
- 关键实现：
  - 在场景中增加 30 mm 宽、124 mm 高的取放台，使 30 mm 方块进入机械臂稳定
    工作空间，同时保留夹爪净空。
  - 新增 `pick_place_moveit` 节点，组合 `/compute_ik`、`/move_action`、
    `/compute_cartesian_path`、`/execute_trajectory`、夹爪轨迹动作、
    Gazebo 实体状态服务和 LinkAttacher 服务。
  - 任务开始时临时将 `six_arm::base_footprint` 固定到地面，结束或异常时必定释放；
    吸附前检查方块位于 link6 安全窗口，吸附后检查相对漂移不超过 2 mm。
  - LinkAttacher 从全局单吸附布尔锁改为逐链接占用检查，允许底盘—地面与
    夹爪—方块两组无关固定关节同时存在，仍拒绝同一链接的重复吸附。
  - MoveIt 启动统一设置 `use_sim_time=true`，消除控制器使用仿真时钟而
    MoveGroup 使用系统时钟导致的 `CONTROL_FAILED`。
  - 对取放接触区使用短距离受控笛卡尔路径；远距离运动仍使用 MoveGroup
    碰撞规划。每轮先进入确定的六关节零位，避免从异常停止姿态产生不同 IK 分支。
  - 默认在 0.11 m 工具高度释放，使放置下降避开低位奇异区；释放后以支撑台
    顶面加方块半高作为真实期望落点，而不是释放瞬间悬空位置。
  - 发布 `/competition/mission/state` 与 `/competition/gripper/state`，
    为后续任务状态机和面板提供接口。
- 静态检查：
  - `python3 -m py_compile mybot/pick_place_moveit.py`：通过。
  - `ament_flake8 mybot/pick_place_moveit.py mybot/test/test_pick_place_math.py`：
    通过。
  - Python `xml.etree.ElementTree` 解析 `room.world` 和 `package.xml`：通过。
  - `git diff --check`：通过。
- 构建命令：
  - `colcon build --symlink-install --packages-select ros2_linkattacher mybot_description mybot`
- 构建结果：3 个包成功，0 个失败。
- 测试命令：
  - `colcon test --packages-select mybot`
  - `colcon test-result --test-result-base build/mybot`
  - 独立 headless Gazebo 与 MoveIt 实例中运行 `ros2 run mybot pick_place_moveit`。
  - 检查 Gazebo 日志中的 ODE damping 越界、NaN、进程崩溃和模型飞散特征。
- 测试结果：
  - 数学单元测试 4 项全部通过；CTest 汇总 5 个测试、0 错误、0 失败、0 跳过。
  - 最终默认参数运行完整成功，四段笛卡尔路径比例均为 `1.000`。
  - 抓取窗口：link6 相对方块约
    `(0.00260, 0.00000, 0.04124) m`，判定安全。
  - 搬运后吸附相对漂移为 `0.000139495 m`，约 0.139 mm。
  - 放置实际位置约 `(0.3298, 0.1394, 0.0391) m`，期望位置约
    `(0.3302, 0.1400, 0.0356) m`，三维误差 `0.00354 m`。
  - 节点发布 `pick_place_complete:red_cube_1`，机械臂回零并发布
    `base_released`；Gazebo 日志未发现 ODE damping 越界、NaN、崩溃或飞散。
- 遇到的问题：
  - MoveGroup 初始使用系统时钟，控制器使用仿真时钟，首次执行返回
    `CONTROL_FAILED (-4)`。
  - 未固定底盘时，机械臂动作造成底盘漂移约 31 mm；固定底盘后旧插件的全局锁
    又阻止夹爪同时吸附方块。
  - 从低位异常停止姿态继续运行时，IK 选择了无法连续抬升的分支，笛卡尔比例为 0。
  - 放置工具高度 0.08 m 时，低位下降在关节空间中断，只完成约 58% 至 62%。
- 解决方法：
  - MoveGroup 使用仿真时间；任务期间按 PID 范围临时固定底盘。
  - LinkAttacher 改为支持无关链接并发吸附，并重新启动 Gazebo 加载新 `.so`。
  - 每轮先回到确定工作零位；放置释放高度提高到 0.11 m，方块短距离落到台面。
  - 接触区临时移除对应 MoveIt 支撑台碰撞对象，离开接触区后恢复。
- Commit message：`step-3: implement MoveIt pick and place`
- Commit hash：`543a1ee64e142475497921ba977b2fea80047091`
- 推送目标：`origin/aerospaceboy`
- 推送结果：成功；本地 `HEAD` 与 `origin/aerospaceboy` 均为 `543a1ee`。
- 剩余任务：执行 Step 4 至 Step 9。

## Step 4：任务协议、大模型适配和随机题目

- 日期：2026-07-30
- 目标：建立稳定任务 JSON 协议、可复现随机题目、OpenAI 兼容本地模型解析，
  并在模型不可用、JSON 非法或内容与题面冲突时明确降级到确定性规则解析。
- 修改文件：
  - `tools_demo/tools_demo/task_protocol.py`
  - `tools_demo/tools_demo/task_question_node.py`
  - `tools_demo/tools_demo/task_parser_node.py`
  - `tools_demo/config/task_parser.yaml`
  - `tools_demo/test/test_task_protocol.py`
  - `tools_demo/setup.py`
  - `tools_demo/package.xml`
  - `tools_demo/tools_demo/chat_interactive.py`
  - `tools_demo/tools_demo/chat_interactive_ros.py`
  - `docs/codex_execution_log.md`
- 关键实现：
  - 定义版本为 `1.0` 的任务计划，严格校验任务 ID、来源、红/蓝颜色、1 至 10
    数量、A/B/C 区域和执行顺序，并派生红蓝总数与逐方块执行序列。
  - 支持 PDF 风格中文题目、中文数字/阿拉伯数字和对应英文题目的规则解析；
    随机节点每轮生成两个颜色、两个不同区域和 1 至 3 个方块的可解析题目。
  - 模型适配器调用本机 OpenAI 兼容 `/v1/chat/completions`，温度为 0、
    请求超时 2 秒；只接受无额外字段的严格 JSON。
  - 当题目本身可由规则解析时，模型结果必须与题面颜色、数量、区域和顺序完全一致，
    否则拒绝模型结果并发布 `source=rules` 的降级计划。
  - 使用可靠、transient-local QoS 发布 `/competition/task/question`、
    `/competition/task/plan`、`/competition/task/counts` 和
    `/competition/task/order`。
  - 移除未安装且未使用的 `ai_msgs` 依赖，补充 `std_msgs`、requests 和 NumPy
    的系统依赖键；保留原有聊天入口并清理其历史 lint 问题。
- 静态与单元测试：
  - 新协议测试 14 项全部通过，覆盖中英文题目、随机生成、稳定 JSON、
    合法模型响应、非法 JSON、非法字段、越界值、模型幻觉和规则降级。
  - 新文件 `ament_flake8` 与 `ament_pep257`：通过。
  - `colcon test --packages-select tools_demo`：17 个测试、0 错误、0 失败、
    1 个版权模板检查跳过。
  - `git diff --check`：通过。
- 构建命令：
  - `colcon build --symlink-install --packages-select tools_demo`
- 构建结果：1 个包成功，0 个失败。
- ROS 运行测试：
  - 未启动 Llama 服务且 `use_llm=true` 时，连接在有界时间内失败，日志明确记录
    `using rules`；种子题目被发布为
    `source=rules`、`blue×1→C | red×3→A`，counts 为 `[3, 1]`。
  - 使用 `/tmp` 中 Python 标准库模拟 OpenAI 兼容端点，模型返回严格合法 JSON；
    节点发布 `source=llm`、`red×2→B | blue×1→A`，counts 为 `[2, 1]`，
    execution order 为 `red, red, blue`。
  - 两种路径均通过真实 ROS 话题订阅核对，测试节点和模拟服务随后按独立 PID 清理。
- 遇到的问题：
  - 初次从仓库根运行 pytest 时，外层 ROS 包目录遮蔽内部 Python 包。
  - 随机模板中的“搬运方块到X区”没有命中最初只包含“搬运到”的目的地正则。
  - 整包回归暴露原有聊天示例的 lint/docstring 问题。
  - 首次在消息发布后启动 CLI 订阅未取得 transient-local 缓存消息。
- 解决方法：
  - 从 `tools_demo` 包根运行协议单测，并以 colcon 整包测试作最终依据。
  - 增加独立“到”目的地形式，50 次种子随机生成回归全部可解析。
  - 对原聊天示例做不改变入口与交互行为的最小 lint 清理。
  - 运行态测试先启动订阅再发布题目，并另用独立测试话题验证模型成功路径。
- Commit message：`step-4: add task protocol and LLM fallback`
- Commit hash：`709555b5456156c863a4e36f9b1338028712cbb1`
- 推送目标：`origin/aerospaceboy`
- 推送结果：成功；本地 `HEAD` 与 `origin/aerospaceboy` 均为 `709555b`。
- 剩余任务：执行 Step 5 至 Step 9。

## Step 5：RGB-D 颜色目标识别与三维定位

- 日期：2026-07-30
- 目标：从腕部 RGB-D 相机识别红蓝方块，使用有效深度与相机内参生成三维点，
  转换到 `base_link` 并通过 Gazebo 实体真值验证误差。
- 修改文件：
  - `tools_demo/tools_demo/perception_math.py`
  - `tools_demo/tools_demo/color_target_detector.py`
  - `tools_demo/config/perception.yaml`
  - `tools_demo/launch/perception.launch.py`
  - `tools_demo/test/test_perception_math.py`
  - `tools_demo/setup.py`
  - `tools_demo/package.xml`
  - `docs/codex_execution_log.md`
- 关键实现：
  - 支持 `32FC1` 米制深度与 `16UC1` 毫米深度，过滤 NaN、零值和范围外样本，
    对颜色连通域使用深度中值。
  - 实现针孔相机投影、四元数归一化/旋转和刚体点变换的可测试纯函数。
  - 红色使用 0–10 与 170–179 两段 HSV 色相，蓝色使用 95–135；
    开闭形态学后提取、面积过滤和排序连通域。
  - 使用 ROS `message_filters.ApproximateTimeSynchronizer` 配对 RGB 与深度，
    避免低仿真帧率下“最新帧”错配和日志刷屏。
  - 查询 `base_link ← camera_rgbd_frame_optical` TF，将检测点转换到机器人坐标，
    发布 `/competition/perception/targets` JSON 与
    `/competition/perception/debug_image`。
  - 目标 JSON 包含 schema、源/目标 frame、时间戳、颜色、像素、连通域面积、
    深度和三维位置；输出按颜色和距离稳定排序。
  - 参数采用运行审计得到的真实双层 Gazebo topic：
    `/camera_rgbd/camera_rgbd/image_raw`、
    `/camera_rgbd/camera_rgbd/depth/image_raw` 和
    `/camera_rgbd/camera_rgbd/camera_info`。
- 传感器实况：
  - RGB/深度尺寸为 640×480，光学帧为 `camera_rgbd_frame_optical`。
  - CameraInfo：`fx=fy=554.254691`，`cx=320.5`，`cy=240.5`。
  - 深度编码为 `32FC1`；当前虚拟机负载下 RGB 约 2–3 Hz，深度约 5–7 Hz。
- 静态与单元测试：
  - 新增 8 项测试，覆盖两种深度编码、无效深度中值、针孔投影、刚体变换、
    红色 hue wrap、蓝色分割、连通域与非法编码。
  - 新文件 `ament_flake8`、`ament_pep257` 和 Python 编译：全部通过。
  - `colcon test --packages-select tools_demo`：25 个测试、0 错误、0 失败、
    1 个版权模板检查跳过。
  - `git diff --check`：通过。
- 构建命令：
  - `colcon build --symlink-install --packages-select tools_demo`
- 构建结果：1 个包成功，0 个失败。
- Gazebo 真值验证：
  - 独立 `DISPLAY=:0` 的 `gzserver` 加载 room.world 和机器人，检测器稳定发布
    `frame_id=base_link` 的结构化消息，空视野正确输出 `targets=[]`。
  - 临时验证器从 TF 计算两个相机前方测试点，将红蓝方块放置到世界坐标并分别
    固定到两个静态取放台；检测完成后将 Gazebo 世界真值反算回 `base_link`。
  - 红色检测：深度约 0.435 m，面积 1591 px，三维误差 `0.015008 m`。
  - 蓝色检测：深度约 0.535 m，面积 1043 px，三维误差 `0.015161 m`。
  - 两项均低于 0.05 m 阈值；约 15 mm 偏差与 30 mm 方块前表面到中心的半尺寸
    一致。测试固定关节自动释放，测试 Gazebo/检测进程按核对 PID 清理。
- 遇到的问题：
  - Xacro 同时设置 namespace 与 camera_name，实际 topic 比注释多一层
    `camera_rgbd`。
  - 初版“最新 RGB + 最新深度”在低帧率下产生 0.14–0.43 秒错配。
  - Gazebo state 插件只接受 `world` reference_frame，不接受 scoped robot link。
  - 第一次审计退出遗留独立 `gzserver`，导致下一实例端口占用。
- 解决方法：
  - 以 `ros2 topic list -t`、CameraInfo 和 Image 实测结果作为默认配置依据。
  - 改用 ApproximateTimeSynchronizer；重复告警按原因键去重。
  - 通过 TF 和机器人世界位姿精确换算相机测试点/实体真值，不依赖 scoped
    reference_frame。
  - 所有运行脚本记录并核对父子 PID；清理残留后重新独立启动并完成真值测试。
- Commit message：`step-5: add RGB-D color target detection`
- Commit hash：本条记录所属提交，提交后由 `git rev-parse HEAD` 验证。
- 推送目标：`origin/aerospaceboy`
- 推送结果：待提交与推送后补充。
- 剩余任务：执行 Step 6 至 Step 9。

## Step 6：Nav2 分区导航、动态避障与底盘稳定化

- 日期：2026-07-30
- 目标：建立 A/B/C 分区导航接口，在 Gazebo 中验证 Nav2 闭环与移动障碍物，
  同时消除机械臂运动或底盘组合速度导致的漂移、失速和飞散。
- 修改文件：
  - `bot_navigation/config/competition_zones.yaml`
  - `bot_navigation/launch/competition_navigation.launch.py`
  - `bot_navigation/param/originbot_nav2.yaml`
  - `bot_navigation/package.xml`
  - `tools_demo/tools_demo/navigation_math.py`
  - `tools_demo/tools_demo/task_navigator.py`
  - `tools_demo/tools_demo/moving_obstacle.py`
  - `tools_demo/test/test_navigation_math.py`
  - `tools_demo/test/test_navigation_contract.py`
  - `tools_demo/setup.py`
  - `tools_demo/package.xml`
  - `mybot_description/urdf/large_base.xacro`
  - `mybot_description/urdf/large_base_gazebo.xacro`
  - `mybot_description/urdf/arm.xacro`
  - `mybot_description/package.xml`
  - `mybot_description/worlds/room.world`
  - `docs/codex_execution_log.md`
- 关键实现：
  - 配置 A、B、C 三个地图目标及 120 秒超时，通过
    `/competition/navigation/goal` 和 `/competition/navigation/cancel`
    接收任务控制。
  - 封装 Nav2 `NavigateToPose`，以 transient-local JSON 发布
    `ready/sending/active/canceling/timing_out` 和终态，并输出距离、恢复次数、
    状态码和明确错误。
  - 在导航走廊加入可碰撞圆柱体，以 10 Hz 三角波横穿路线，并发布
    `/competition/obstacle/state` 真值。
  - Nav2 使用前进型 DWB 速度包络；AMCL 不重复广播 map 到 odom，由竞赛启动
    文件提供唯一固定变换。
  - 修正轮距、轮轴高度、转动惯量、关节限位与接触参数；机械臂关节由无效
    `joint_properties` 改用标准 URDF `dynamics`。
  - Gazebo 底盘改用官方 planar move 插件，根链接设置为 kinematic，使插件
    唯一负责平面位姿，锁定 z/roll/pitch；保留 ros2_control 对机械臂的真实
    关节执行，避免 ODE 轮胎接触与机械臂反力导致整车飞散。
- 物理与控制验证：
  - 组合指令 `linear.x=0.15`、`angular.z=-0.36` 持续 10 秒，底盘完成明显
    平移和转向，z 保持 0。
  - `arm_controller` 将 joint2 移至 0.1 rad，实际值约 0.100000829 rad，
    底盘世界位姿保持不变。
  - 直接 Nav2 目标 `(0.5, 0, 0)` 返回 `SUCCEEDED`，机器人由原点到达
    `(0.3525, -0.0006, 0)`。
  - 正式接口导航 B 区由 `(0.3542, -0.0005)` 到
    `(1.8767, 0.0636)`，状态为 `sending -> active -> succeeded`，
    状态码 4，无错误。
  - 导航期间移动障碍物成功更新 116 次，y 方向覆盖 1.5 m，证明动态障碍节点
    持续运行。
  - 非法区域 D 被拒绝并发布可读错误；C 区目标取消时状态为
    `active -> canceling -> canceled`，状态码 5。
- 静态检查与构建：
  - 新增 Python 文件 `py_compile`、`ament_flake8` 和 `ament_pep257` 全部通过。
  - `git diff --check` 通过。
  - `colcon build --symlink-install --packages-select mybot_description
    bot_navigation tools_demo`：3 个包成功，0 个失败。
  - 完整机器人 Xacro 展开后由 `check_urdf` 成功解析。
- 测试结果：
  - `tools_demo`：43 项，0 错误，0 失败，1 项版权模板检查跳过。
  - `mybot_description`：3 项，0 错误，0 失败，1 项版权模板检查跳过。
  - 契约测试固定 planar 插件、kinematic 根链接、插件依赖和标准关节 dynamics，
    防止后续修改重新引入飞散问题。
- 遇到的问题：
  - ODE 差速轮模型无法稳定跟随组合线速度/角速度，轮胎接触与上装反力会造成
    失速、漂移，严重时整车飞散。
  - 仅降低速度、调整轮距、摩擦或改用 planar 插件但保留动态根链接，仍会受到
    物理解算与插件速度设置冲突。
- 解决方法：
  - 将底盘平面运动建模为 kinematic，并由 planar move 插件独占；
    机械臂关节继续由 Gazebo/ros2_control 解算。
  - 使用实际 ROS action、Gazebo 实体位姿和竞赛公开 topic 三层验证，
    而不是仅以 action 返回成功作为依据。
- Commit message：`step-6: add stable Nav2 competition navigation`
- Commit hash：本条记录所属提交，提交后由 `git rev-parse HEAD` 验证。
- 推送目标：`origin/aerospaceboy`
- 推送结果：待提交与推送后补充。
- 剩余任务：执行 Step 7 至 Step 9。

## Step 7：自主竞赛任务状态机与红蓝端到端闭环

- 日期：2026-07-30
- 关键实现：
  - 将任务计划、分区导航、RGB-D 目标选择、MoveIt 抓取/释放和 Gazebo 落区
    真值验证接入同一有序状态机。
  - 支持最多 5 个方块、颜色顺序、重试、跳过、取消和 295 秒任务预算。
  - 为红/蓝取料架分别实现安全进场、对接与离架路线，并优化机械臂执行速度。
- 运行验证：
  - 蓝色方块到 C 区：177.192 秒，`completed=1, skipped=0`。
  - 优化后红色方块到 A 区：137.073 秒，`completed=1, skipped=0`。
  - LinkAttacher 吸附/释放压力测试 20/20 通过。
  - `mybot` 10/10 测试通过；`tools_demo` 66/67，1 项按设计跳过。
- Commit：`e65bf47 step-7: integrate autonomous competition mission`
- 推送结果：已推送至 `origin/aerospaceboy`。

## Step 8：竞赛评分可视化面板

- 日期：2026-07-30
- 关键实现：
  - 新增只读 HTTP 面板和线程安全状态聚合器。
  - 展示题目、解析来源、顺序、数量、任务进度、5 分钟剩余时间、导航、
    视觉、夹具、Nav2 路径和机器人位姿。
  - `/healthz`、`/api/state` 和宿主机 `10.88.65.175:8080` 均实测可达。
  - 兼容 transient-local 与 volatile 任务题目发布者。
- 验证：
  - Windows 纯逻辑测试 12/12。
  - 虚拟机 `tools_demo` 71 通过、1 跳过、0 失败。
  - 实时 API 同步返回题目、计划、任务、导航、路径和机器人位姿。
- Commit：`5be10c4 step-8: add competition scoring dashboard`
- 推送结果：已推送至 `origin/aerospaceboy`。

## Step 9：一键启动、赛前自检与五分钟验收

- 日期：2026-07-30
- 关键实现：
  - 新增 `competition_system.launch.py`，分阶段启动 Gazebo/控制器、
    MoveIt/Nav2、任务层和自检器。
  - `gazebo_world.launch.py` 新增 headless 模式；统一启动在 SSH 环境自动复用
    已登录 Ubuntu 桌面的 DISPLAY/Xauthority，使 RGB-D 相机仍可渲染。
  - 新增自检器，检查节点、接口精确类型、服务、动作、关键 TF、面板健康状态，
    并要求 RGB、深度、雷达、关节和时钟存在真实发布者。
  - 可选任务监控将端到端验收限制在 300 秒内，并保存 JSON 证据。
- 发现并修复：
  - 首次 headless 冷启动虽然 ROS 话题名称存在，但 RGB-D 发布者为 0；
    Gazebo 日志显示 rendering disabled，导致任务在视觉阶段失败。
  - 统一启动补充显示环境后，RGB 和深度发布者均为 1，视觉结果恢复；
    自检增加 publisher 约束，消除同类假 PASS。
- 运行验证：
  - 从空项目进程冷启动，约 54 秒达到 `COMPETITION_ACCEPTANCE=PASS`。
  - 统一实例完成 `Move 1 blue cube to zone C.`：任务 139.7 秒，
    验收总耗时 143.128 秒，`completed=1, skipped=0`。
  - Gazebo 真值 `blue_cube_1=(2.7585,-0.8803,0.0155)`，静止且位于 C 区。
  - 关键日志无 ODEJoint damping 越界、进程崩溃、Traceback 或 FATAL。
- 文档：
  - 更新 `readme.md`。
  - 新增 `docs/competition_operator_guide.md`。
  - 新增 `docs/codex_final_report.md`。
- 最终构建与测试：
  - 全工作区 `colcon build --symlink-install`：6 个包成功、0 失败。
  - `mybot`：10/10 通过。
  - `tools_demo`：78 通过、1 跳过、0 失败。
  - 全量 `colcon test` 另检出 139 项仓库原有 lint 失败：
    `bot_navigation` 旧 launch 文件的 flake8/版权格式，以及 IFRA
    LinkAttacher 的 CMake 空白、package.xml 元素顺序、版权头和 C++ 格式。
    这些失败不涉及本阶段运行逻辑，已独立记录，未通过关闭测试来掩盖。
- 最终差异审计与提交哈希将在本步骤提交前完成。

## 2026-08-25：物理碰撞、导航振荡与夹放恢复修复

- 现场复现：原版本完整任务仅完成 2/5，日志包含 184 次 RPP
  `collision ahead`、24 次 controller patience exceeded，并会重复选择已投放
  方块；kinematic 底盘不受 Gazebo 接触约束，存在穿墙风险。
- 导航与任务修复：
  - 底盘改为动态物理模型，使墙体、托盘和岩石接触真实生效；验证静置无发散。
  - 方块接近开放东西两侧并优先选择小车同侧，禁止为缩短配送路线横穿托盘。
  - 起点跨越 `x=13.4` 双动态障碍门时，从轨迹端点外侧确定性绕行。
  - 配送目标位于托盘另一侧时，经近侧角、远侧角两段绕行；同侧不增加绕路。
  - 导航恢复倒车次数由 3 限制为 1，controller failure tolerance 调整为 8 秒，
    避免短暂障碍立即失败，同时消除连续前后振荡。
  - 新任务重置候选库存；候选耗尽直接跳过当前作业，不再回到已用实体或陈旧视觉点。
  - 用实测路线风险避开红3的 32 秒碰撞预测停滞，并扩大 C 出口静态路径标定
    命中范围，使第二个蓝块选择真实较短路线。
- 夹放恢复修复：
  - 动态底盘固定关节毫米级沉降纳入 carry window，仍拒绝 Z=-4 cm 或横向 8 cm
    的真实掉落。
  - LinkAttacher 错误锚点会自动拆除、恢复附着前世界姿态并重建，最多 3 次；
    每次仍需连续两帧稳定窗口才能抬升。
  - 首次 place 遇到短暂控制器超时时保留方块附着供状态机重试；仅重试耗尽后
    执行统一 safety release。
- 验证证据：
  - 导航/任务相关回归 183/183 通过；夹放回归 25/25 通过。
  - `mybot`、`mybot_description`、`bot_navigation`、`tools_demo` 构建成功。
  - 动态物理底盘版本曾完成完整任务 5/5、0 跳过，全部 Gazebo 真值投放通过，
    用时 376.5 秒；该结果建立了可靠性基线，尚未达到 300 秒目标。
  - 方向感知绕行聚焦验收完成红4到 A：1/1、0 跳过、113.8 秒；动态障碍
    最小中心距 0.7729 m，高于 0.56 m 要求，无碰撞监控故障。
  - 最新附着锚点恢复轮中红4完成长距离携带和 A 区真值投放；该轮 VM 调度冻结，
    A 导航被拉长到 117 秒，因此不作为性能验收数据。
- 遗留：在资源状态稳定的 VM 上重跑最新版本完整 5/5 计时；可靠性修复优先，
  300 秒提速继续以阶段耗时证据推进。


## 2026-09-22 比赛视频启动、出题与 API 联调修复

- 用户反馈主机 Start-CompetitionVideo.ps1 出题异常。复现 Windows PowerShell 5.1 经 plink 丢失嵌套引号，改用 -m 命令文件；PS5.1 和 PS7 均实测官方中文出题通过。
- 新增 video_submit.py，只生成一次、提交一次、按 request/task ID 核验真实 API，再观察本轮终态。状态接口保留 parse_result；增加旧题/陈旧场景/非 LLM/重复提交等行为测试。
- 模型改为根据题目和映射独立解题，不再接收预先算好的候选答案；保留本地验算及强制 LLM。沿用本分支 qwen3.7-plus，密钥不入库。
- 第一次真实联调 Nav2 配置失败，未完成任务。已保留失败记录，不补造 terminal。修复启动假就绪：核心 Nav2 生命周期须全部 active，桥接进程、端口及 dashboard 必须就绪；SLAM 延后启动以降低负载，但不声称根治底层 DDS 偶发故障。
- 第二次官方随机题：4 红 A、1 蓝 C，5/5，零跳过；墙钟 241.807s（出题前至终态观察），任务 elapsed 233.2s，LLM latency 7.733s。两个障碍 yield，最小净空 0.2854m。未运行独立 acceptance，未伪造 CASE_ACCEPTED；未重跑 48 题。
- 最终功能回归 565 passed（3 个 linter 排除）；完整回归 565 passed/1 skipped/2 failed，失败为历史 flake8/pep257，清洁基线也失败。工作区额外 105 flake8 是 install 下旧 launch 重复扫描，无新增源错误。7 包构建通过。
- 手册 docs/guide/VIDEO_WORKFLOW_20260922.md，审查 docs/guide/VIDEO_AUDIT_20260922.md，证据 docs/validation/VIDEO_START_20260922/；主机中文交付文件与 VM 桌面同步。既有 GUI 默认刷新改动和未跟踪历史文档保留、未纳入提交。
- 未完成：跨冷启动稳定性、多题/跨障碍相位严格验收、历史风格债务、正式拍摄与赛区要求核对。仍须以完整裁判墙钟为准，不把单题成功推广为全部达标。
