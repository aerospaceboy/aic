> 历史文档归档：内容来源365c0af:PROJECT_HANDOFF.md；仅将本机绝对路径替换成占位符，原件可从该Git版本获取。当前状态请读根PROJECT_HANDOFF。

# YZBot-224 项目交接快照

> 本文件是**当前状态快照**（最后更新：2026-09-21，策略性能优化轮：5 个高延迟任务集合优化并实测通过）。
> 本轮主题：**对 audit 高收益且墙钟贴近门限的任务做单键选块/接近侧优化，降低偶发超时概率**；前序同日主题为 coStudio 可视化与解析证据链。
> 历史过程见 `docs/AGENT_HISTORY.md`（按 Agent 追加，不删除）；coStudio 相关细节见 `docs/costudio/`。
> 快照只描述**当前代码与实测状态**，历史轮次细节以 `docs/AGENT_HISTORY.md` 为准。

---

## 1. 项目背景和最终目标

- 平台：ROS 2 Humble + Gazebo Classic 11 竞赛机器人 YZBot-224（移动底盘 + 六轴臂 + 夹爪 + 底盘前视相机 + 腕部 RGB-D + 激光）。
- 任务：中文计算题 + 赛前映射规则 → 大模型解题得到红/蓝抓取数量 → Nav2 导航取料 → MoveIt 2 + IFRA 吸附抓取 → 送 A/B/C 区并做真值校验 → 面板实时显示任务与状态。
- **合法任务空间 = 48 例**：6 个有序区域对 × 4 种数量分配 × 2 种子句顺序；`tools_demo/scripts/gen48_questions.py` 是唯一权威定义。
- **比赛规则（`大模型技术创新赛比赛规则2026.pdf`）**：仅 1 轮、限时 5 分钟；总分 110 分、8 个评分项（见 §9 规则对照）。
- 单轮验收门槛（`run_zone9_campaign.py`）：`state=completed`、`completed=5`、`skipped=0`、`elapsed_sec ≤ 290`、`safe=true` 且 `minimum_clearance ≥ 0.10 m`、无碰撞/抓取失败/搬运中止/致命起步/导航重试、`plan_source=llm`。
  - 比赛硬门槛是 **300 s**；290 s 是**工程批次**预留余量，两者不要混用。

---

## 2. 当前整体架构

| 层 | 组成 |
| --- | --- |
| 仿真/底盘 | `gzserver` + `ros2_control` + `planar_move`（只订阅 `/cmd_vel_safe`）+ IFRA LinkAttacher 吸附 |
| 题目/大模型 | `/competition/task/problem`(coStudio 分片重组) → `task_parser`（qwen3.8-max，要求 llm 且与确定性解逐项一致）→ `/competition/task/plan` + `/competition/task/parse_result` |
| 导航 | Nav2（Smac + RotationShim + RPP）；`task_navigator` 预转/单环停靠/离区/走廊/有界恢复；per-cube dock 与 entry |
| 避障 | Nav2 代价地图 + Collision Monitor（`/cmd_vel`→`/cmd_vel_collision_safe`）+ 真值兜底限速 `safety_limiter`（→`/cmd_vel_safe`）+ 证据节点 `avoidance_monitor` |
| 机械臂 | `pick_place_moveit` 持久请求；反升余弦重定时、非接触夹爪、净空抬升、放置抬离 |
| 任务层 | `mission_coordinator` 状态机；离线 `cube_order_policy.json` 优先，运行时 `cube_delivery_cost` 兜底 |
| 可视化 | coStudio（Windows）← coBridge(21274) ← 标准 ROS 2 话题；浏览器面板 :8080 备用 |
| 场景标记 | `scene_markers` 读 `/gazebo/model_states` → `/competition/scene_markers`（物块/区域/小车/障碍） |

---

## 3. 核心目录、模块与文件

| 路径 | 作用 |
| --- | --- |
| `mybot/pick_place_moveit.py` | 抓放执行；`GRASP_XY_LIMIT=0.035`、`CARRY_X/Y/Z`、运输包络 0.35 m/0.15 rad；下降前 `refine_pick_target_before_descend` |
| `tools_demo/tools_demo/task_parser_node.py` | 大模型解题 + 确定性校验；发布 plan/counts/order/llm_status/**parse_result** |
| `tools_demo/tools_demo/legacy_costudio_status.py` | 汇总为 `/competition/status`：题目、解析证据、指令、执行顺序、**逐轮完成、识别结果**、进度、夹爪、避障 |
| `tools_demo/tools_demo/scene_markers.py` | 三维场景标记（本轮新增） |
| `tools_demo/tools_demo/task_navigator.py` | 分区路线、Nav2 调度、直接对接、有界恢复 |
| `tools_demo/tools_demo/mission_coordinator.py` | 任务状态机、选块策略装载与运行时分 |
| `tools_demo/tools_demo/color_target_detector.py` | RGB-D 颜色分割 + 深度中值 + TF 三维定位 → `/competition/perception/targets` |
| `tools_demo/tools_demo/avoidance_monitor.py` | 最小净空与碰撞锁存证据 |
| `tools_demo/costudio_extension/yzbot-competition/` | **coStudio 扩展源码（本轮纳入仓库）**：题目面板、状态卡、双相机画面、开始按钮 |
| `tools_demo/config/costudio_reference_layout.json` | coStudio 参考布局（两个视觉面板分别绑定两路相机） |
| `tools_demo/config/cube_order_policy.json` | 48 条离线选块顺序与停靠侧（性能核心） |
| `tools_demo/scripts/run_zone9_campaign.py` | 批量回归运行器（用例选择、失败重试、仿真死亡自动重启） |
| `tools_demo/scripts/start_manual_problem_mode.sh` | 一键手动竞赛入口（自检 + 桥接 + 仿真 + 面板数据） |
| `tools_demo/scripts/clean_launch.sh` | 清理仿真与残留进程 |
| `docs/costudio/README.md` / `PARSE_DISPLAY.md` / `SCENE_MARKERS.md` | coStudio 连接、解析显示契约、场景标记说明 |
| `docs/AGENT_HISTORY.md` | 逐 Agent 历史（只追加） |

---

## 4. 本轮之前已经完成的内容（基线）

- 平台与闭环：ROS 2 竞赛栈、LLM 题目解析、per-cube 导航、MoveIt 抓放、双层避障、评分面板。
- **48 例合法任务全部有通过记录**（`coverage48_summary.py`：legal 48 / verified 48），策略归属核对 `verify48_policy_match.py` 为 matched 48/48。
- 48 轮连续批次 `runs/policy_20260918T125937` 为 44/48，其余经定向重跑与修复闭合（**不是**单次连续 48/48）。
- 九轮 9/9 证据：`runs/zone9_20260914T015654`（最长 286.6 s）。
- 工具链：覆盖总表、证据归属核对、实测腿耗时表、按侧表、逐腿余量、单轮分段、批量报告、全表审计等脚本。
- 下降前目标复核（`refine_pick_target_before_descend`）与自身膨胀格有界恢复（0.30 m 豁免 / 0.50 m 上限）。

---

## 5. 本轮新增完成的内容

### 5.0 策略性能优化（2026-09-21）

- 方法：`audit_policy` 实测腿耗时 + `coverage48` 墙钟排序；单键改集合/接近侧 → 无 GUI 单轮验证。
- **已应用并验证 26+ 键**（含 2026-09-22 A 类补完）（含中等收益批：3rA_2bC/2rA_3bC/4rA_1bC/3bB_2rA/1bC_4rA/1rB_4bC；墙钟降幅约 14–68 s），全部 `5/5 completed`、零抓取失败、`verify48` matched 48/48：
  - `2rB_3bC` 282→214.4 · `3bC_2rA` 262→205.7 · `3rB_2bC` 264→220.9 · `4bC_1rA` 267→226.2 · `4rB_1bC` 274→250.9
  - `1bB_4rA` 256→194.7 · `2bC_3rA` 238→192.8 · `2rA_3bB` 245→207.1 · `2bC_3rB` 268→232.1 · `3bC_2rB` 262→227.5 · `4bB_1rA` 272→248.3 · `4rA_1bB` 250→194.3
- `4rA_1bB`：blue5 用 east 会 lethal start，**west 通过**——接近侧影响投放安全。
- **`1rA_4bB` 回滚**：blue→B 导航失败 + lethal start，属运行时问题。
- audit ≥8 s 条目 27→**9**（其中 1rA_4bB 失败已回滚）；高墙钟低 gain 任务转导航运行时分析。
- 备份：`cube_order_policy.json.bak_before_opt_20260921`；明细见 `docs/POLICY_OPT_PENDING_20260921.md` 与 AGENT_HISTORY。

### 5.1 coStudio 双相机面板（修复“两个画面是同一台相机”）

- 原状：两个视觉面板分别订阅 RGB-D 原图与 `debug_image`，本质是同一台腕部相机的两路派生流，且桥接白名单里没有底盘相机。
- 现状：面板一 = 底盘前视相机 `/camera/image_raw`；面板二 = 机械臂腕部 RGB-D `/camera_rgbd/camera_rgbd/image_raw`；`debug_image` 仅作可选诊断流。
- 桥接 `CAMERA_STREAM_TOPICS` 同时放行上述两路；契约测试 `test_costudio_camera_contract.py` 防止再次绑定到同一相机。

### 5.2 三维地图实时标记（物块 / ABC 区 / 小车 / 移动障碍）

- 新增 `scene_markers` 节点：读 `/gazebo/model_states`（20 Hz 发布）→ 10 Hz 输出 `/competition/scene_markers`（`MarkerArray`），先 DELETEALL 再 ADD，2 s 无数据自动清除残留。
- 内容：红蓝物块各 5 个（跟随抓取/运输/投放后的真实位置，带名称）、A/B/C 区域矩形与字母、小车位置与朝向箭头、两个移动障碍圆柱与编号。
- 实测：5 s 内收到 23 帧、16 个实体，平面坐标与 Gazebo 真值误差 0.000 m；同时提供 `/competition/map` 底图别名。

### 5.3 大模型解析证据链与面板判定（对应评分项 2）

- `task_parser_node` 新增 `/competition/task/parse_result`：每次提交带唯一 `request_id`，按 `parsing → published / failed` 发布模型名、题目、映射规则、完整计划与失败原因；所有拒绝路径（非法 JSON、空题目、模型错误、与确定性解不一致）都发布 `failed`。
- `legacy_costudio_status` 消费该证据；面板显示“大模型解析成功 · 指令已发布 / 解析失败 · 未发布本题指令 / 大模型解析中”，并给出模型名与任务编号；**解析失败时清空上一题的指令与执行顺序**。
- 面板实时模式不再用演示数据补齐缺失字段（此前缺字段会显示示例数字“红3蓝2”）。

### 5.4 面板逐轮完成情况与识别结果（对应评分项 6）

- 新增逐轮追踪：按物块记录每次完成事件，输出 `actual_grasp_order`（含 cube_name）、`current_grasp_step`、`current_rule`、`grasp_rule_compliance`（`IN_PROGRESS / RULE_MATCH / RULE_VIOLATION`），新任务自动清零。
- 新增识别结果：订阅 `/competition/perception/targets`，把当前视野内颜色写入 `visible_targets`（2 s 过期），面板“识别”字段实时显示。
- 实测：面板“当前执行：1/5 · 红→A · 实际目标：red_cube_5”；载荷 `actual_grasp_order=[red_cube_5→A, red_cube_1→A]`；识别话题实测出现非空样本 `['red','red']`。

### 5.5 模型切换为 qwen3.8-max（原 qwen3.7-plus → 2026-09-22 更新）

- `tools_demo/config/task_parser.yaml` 的 `model` 由 `qwen3.7-flash-2026-07-15` 改为 `qwen3.8-max`；同步更新操作手册、交付说明与包根 README。
- 2026-09-22：`llm_enable_thinking` 改为 false（thinking 约 20 s → 关闭后约 2 s）。
- 验证：隔离域真实调用 `state=published / source=llm / model=qwen3.8-max`；运行中系统 `ros2 param get /competition_task_parser model` = `qwen3.8-max`；coStudio 面板显示 `模型：qwen3.8-max`。

### 5.6 一键启动脚本调整

- **RViz 默认关闭**（`COSTUDIO_RVIZ=True` 才打开，仅在 ROS 图就绪后启动）；一键启动不再附带第二个 GUI。
- **相机流默认开启**（`COSTUDIO_CAMERA_STREAMS=False` 可关闭）；此前默认关闭导致两个相机画面一直“等待图片信息中”。
- 就绪自检新增 `/competition/task/parse_result` 发布者检查，提交前即确认解析显示链路可用。

### 5.7 扩展源码纳入仓库

- coStudio 扩展此前只存在于旧仓库与 Windows 安装目录，新主仓库没有源码，面板能力无法复现；现纳入 `tools_demo/costudio_extension/yzbot-competition`（源码 + 生产构建 + README + .gitignore，不含 node_modules）。

### 5.8 技术报告优化与规则对照

- 生成代码对齐版技术报告 `YZBot月面仓储移动操作机器人系统技术报告-代码对齐优化版.docx`（Windows 工作区）：修正双相机描述、补 48/48 与测试证据、区分 300/290 s、去掉“待补”、修正标题层级与表格、补图片替代文本，26 页逐页渲染检查通过。
- 按 2026 规则逐条核对 8 个评分项，形成缺口清单（见 §9 与 §11）。

---

## 6. 本轮新增或修改的文件及其作用

| 文件 | 类型 | 作用 |
| --- | --- | --- |
| `tools_demo/tools_demo/scene_markers.py` | 新增 | 三维场景标记节点（物块/区域/小车/障碍） |
| `tools_demo/tools_demo/task_parser_node.py` | 改 | 新增 `/competition/task/parse_result` 证据与全部失败路径上报 |
| `tools_demo/tools_demo/legacy_costudio_status.py` | 改 | 消费解析证据、逐轮完成追踪、识别结果、规则一致性 |
| `tools_demo/launch/costudio_bridge.launch.py` | 改 | 放行 `/camera/image_raw`；随启动 `scene_markers`；放行 `/competition/scene_markers` |
| `tools_demo/scripts/start_manual_problem_mode.sh` | 改 | RViz 默认关闭、相机流默认开启、就绪自检加入 parse_result |
| `tools_demo/setup.py` / `package.xml` | 改 | 注册 `scene_markers` 入口与 `visualization_msgs` 依赖 |
| `tools_demo/config/task_parser.yaml` | 改 | 模型改为 `qwen3.8-max` |
| `tools_demo/config/costudio_reference_layout.json` | 新增 | 双相机 coStudio 参考布局 |
| `tools_demo/costudio_extension/yzbot-competition/**` | 新增 | coStudio 扩展源码与生产构建 |
| `tools_demo/test/test_parse_result_evidence.py` | 新增 | 解析证据 + 逐轮进度 + 识别结果契约测试（9 项） |
| `tools_demo/test/test_costudio_camera_contract.py` | 新增 | 双相机话题与面板绑定契约（3 项） |
| `tools_demo/test/test_scene_markers.py` | 新增 | 场景标记几何/标签/清理契约（3 项） |
| `tools_demo/test/test_startup_race_contract.py` | 改 | 固定“RViz 默认关闭 + 三处后台启动释放锁” |
| `tools_demo/test/test_navigation_contract.py` | 改 | 固定相机流默认开启 |
| `tools_demo/test/test_parser_mode_callbacks.py` | 改 | 桩补齐 `begin_parse/publish_parse_result` |
| `docs/costudio/PARSE_DISPLAY.md` | 新增 | 解析显示契约与实测证据 |
| `docs/costudio/SCENE_MARKERS.md` | 新增 | 三维标记说明 |
| `docs/costudio/README.md` | 改 | 新增 parse_result / scene_markers / 双相机 / 相机开关说明 |
| `docs/guide/ROS2竞赛系统命令手册.md`、`docs/guide/项目交付说明_20260908.md`、包根 `README.md`、`docs/*.md` | 改 | 模型名更新为 qwen3.8-max |
| `PROJECT_HANDOFF.md` | 改 | 本快照 |
| `docs/AGENT_HISTORY.md` | 改 | 追加本轮记录 |

---

## 7. 当前已验证正常运行的功能

| 证据 | 结果 |
| --- | --- |
| 编译 | `colcon build --symlink-install --packages-select mybot tools_demo bot_navigation` → 3 packages finished |
| 单元/契约测试 | `555 passed, 1 skipped, 2 deselected`（`-k 'not test_flake8 and not test_pep257'`） |
| 一键启动 | `MANUAL_PROBLEM_MODE=PASS`；节点/话题/生命周期/控制器/解析证据齐备 |
| 大模型解析 | 真实模型 `qwen3.8-max`：`state=published`、`source=llm`、红 4→A / 蓝 1→C；失败路径不发布指令 |
| 面板显示 | 实测 DOM：`大模型解析成功 · 指令已发布`、`模型：qwen3.8-max · 任务：...`、`指令内容`、`执行顺序`、`本轮完成情况 已完成 x/5`、`当前执行 N/5 · 色→区` |
| 用户实跑一轮 | 宠物店题（红 2→B、蓝 3→A）：5/5 完成、235.3 s、`navigation=succeeded` |
| 双相机 | 两个面板均收到图像（占位文本消失），桥接日志确认订阅两路；话题各有 1 个发布者 |
| 三维标记 | 16 个实体、坐标与真值一致（误差 0.000 m），约 5.5 Hz |
| 逐轮进度 | 载荷 `actual_grasp_order` 随完成递增并带物块名；新任务清零 |
| 识别结果 | 订阅 `/competition/perception/targets`；实测抓到非空样本 `['red','red']` |
| 场景设计（规则） | 货物最小间距 **2.25 m**；三对区域之间均有墙（19/25、25/25、20/25 条平行线穿墙）；2 个有效移动障碍 |

---

## 8. 当前编译、启动、运行和测试命令

```bash
# 0. 环境（每个新 shell 都要）
cd <HOME>/huxuyang/YZBot-224 && source scripts/00_env.sh

# 1. 构建
cd workspace && source <ROS_HUMBLE>/setup.bash
colcon build --symlink-install --packages-select mybot tools_demo bot_navigation
source install/setup.bash

# 2. 测试（工作集；全量 colcon test 会命中历史 lint 失败）
cd <HOME>/huxuyang/YZBot-224/workspace/src/yzbot
python3 -m pytest -q tools_demo/test mybot/test -k 'not test_flake8 and not test_pep257'

# 3. 一键手动竞赛模式（含 coStudio 桥接、双相机、解析证据；**默认打开 Gazebo GUI + RViz**）
cd <HOME>/huxuyang/YZBot-224
DISPLAY=:0 COSTUDIO_HEADLESS=False bash \
  workspace/src/yzbot/tools_demo/scripts/start_manual_problem_mode.sh
#   省图形负载（跑分）：COSTUDIO_HEADLESS=True COSTUDIO_RVIZ=False
#   省带宽：COSTUDIO_CAMERA_STREAMS=False

# 4. 单独重启桥接（不重启仿真时用）
ros2 launch tools_demo costudio_bridge.launch.py address:=0.0.0.0 port:=21274 camera_streams:=True

# 5. 批量回归（示例：指定用例）
cd <HOME>/huxuyang/YZBot-224/workspace
CLEAN_ONLY=True bash src/yzbot/tools_demo/scripts/clean_launch.sh
POLICY_CASES=2bA_3rC,4rB_1bA setsid nohup \
  python3 src/yzbot/tools_demo/scripts/run_zone9_campaign.py > /tmp/run.out 2>&1 &

# 6. 结果核对
python3 src/yzbot/tools_demo/scripts/coverage48_summary.py
python3 src/yzbot/tools_demo/scripts/verify48_policy_match.py | tail -2
python3 src/yzbot/tools_demo/scripts/audit_policy.py --min-gain 8
```

coStudio 侧：桥接地址 `ws://<VM_IP>:21274`（coBridge 协议）；扩展源码构建 `cd tools_demo/costudio_extension/yzbot-competition && npm ci && npm run build -- --mode production`，把 `dist/extension.js` 覆盖到 `%USERPROFILE%\.coStudio\extensions\aerospaceboy.yzbot-competition-<ver>\dist\extension.js` 后重启 coStudio。

凭据：`<PRIVATE_CONFIG>/llm.env`（优先）或 `qwen.env`，内容为 `DASHSCOPE_API_KEY`；不入库、不进日志。

---

## 9. 按 2026 比赛规则的对照结果（总分 110，8 项）

| # | 评分项 | 状态 | 依据 |
| --- | --- | --- | --- |
| 1 | 系统启动加载（10） | 满足 | 一键启动自检 PASS；`COSTUDIO_HEADLESS=False` 时仿真界面显示 |
| 2 | 大模型解析并显示（10） | 满足 | `parse_result=published/source=llm`；面板显示指令内容与执行顺序；失败即清空 |
| 3 | 货物抓取（20，每件 4） | 能力满足 | 48/48 有通过记录；本轮实跑 5/5 |
| 4 | 搬运放置（20，每件 4） | 能力满足 | 实跑 235.3 s 完成 5/5，落区真值有效 |
| 5 | 机器人避障（10，每个 5） | 满足 | 2 个移动障碍在出生点走廊（x=8.5 / x=5.0）；监测最小净空约 0.28 m、无碰撞 |
| 6 | 状态反馈（10） | 本轮补齐后满足 | 路径 `/plan`、`/local_plan`、`/competition/map`；进度与逐轮完成、机械臂状态、识别结果均上板 |
| 7 | 场景设计（10） | 满足 | 货物最小间距 2.25 m；区域间有墙；2 个有效障碍 |
| 8 | 技术文档（20） | **有缺口** | 报告已优化，但首页缺必填项（见 §11） |

规则正文另有两项流程要求：**大模型生成语音/文本实时播报作业状态**（未实现）、**双机位录屏与作品提交**（待人工完成）。

---

## 10. 当前仍存在的问题、Bug 与风险

1. **比赛单轮不可重试**是最大风险：一次感知退化或导航卡死即整轮结束（2026-09-21 15:09 一轮即 `mission_terminal:target_unavailable`）。历史上方两类偶发故障仍未根治。
2. **技术文档首页缺必填信息**：赛道名称（算法应用赛）、参赛编号、团队名称、团队成员；当前只有标题与空的“队伍：____”。
3. **大模型语音/文本播报未实现**（规则正文要求，非评分表项）。
4. **提交物未就绪**：双机位录屏视频、算法源码压缩包（现有 portable 包是较早快照）。
5. **手动竞赛模式 `slam_evidence:=False`**：规则把 SLAM 写进技术路线，未开 Cartographer，裁判追问时缺证据（可选补强）。
6. 环境为 4 核 VM：整场回归期间 load 常 3–7、高峰 15–19，会放大超时与感知失败；跑分前先看 `uptime`。
7. 策略 audit：≥8 s 改进空间 27→22；其中 5 个高延迟任务本轮已应用并验证。剩余补丁见 AGENT_HISTORY（1bB_4rA/4rA_1bB/2rA_3bB 等）。
8. 随机题（`generate_question:=True`）+ 面板流程、GUI 跑分、大模型失败降级路径尚未做端到端自动化覆盖。
9. `build/` 下 `legacy_costudio_status.py` 等是硬链接/副本行为，改 `.py` 后务必重新 `colcon build`（`mybot` 为复制安装）。
10. coStudio 扩展需重启客户端才加载新版本；重启后数据源需重新连接（coStudio 会自动重连，必要时点“打开连接”）。

---

## 11. 尚未完成的任务

1. 技术文档首页补齐四项必填信息（需要队伍提供编号/队名/成员）。
2. 大模型播报文本（可复用现有 LLM 客户端，状态变化时生成一行播报，避免占用比赛时间）。
3. 双机位录屏与省赛作品提交（技术文档 + 源码压缩包 + 视频）。
4. 赛前按当前代码重新导出便携源码包。
5. 逐条验证审计发现的高收益策略改动（每条一轮）。
6. 低负载下的连续验收（≥20 轮）。
7. 偶发故障（导航卡死、感知退化）根因定位与运行器看门狗。
8. 规则第 5 项若想拿满分，可在确认现有两个障碍有效性的基础上评估是否再加一个移动物体（每个 5 分，上限 10 分）。

---

## 12. 需要继续优化或重构的内容

1. ~~运行器看门狗~~ **已完成**（2026-09-22）：`NO_PROGRESS_SEC=150`，无进展中止；回归通过。
2. `legacy_costudio_status` 逐轮追踪目前依赖 `mission/state` 的 `current_index/cube_name/completed` 组合；若协调器状态语义变化需同步更新（已有测试保护）。
3. 面板“识别”字段依赖探测器实时发布；扫描阶段之外显示“暂无目标”属正常，但可考虑保留最近一次识别结果并标注时间。
4. coStudio 布局随面板标题/绑定变更需重新导入；建议把布局版本写入 `docs/costudio/README.md` 以便对账。
5. `readme.md` 的环境段（出生点/障碍坐标）与当前配置仍不一致，需按 `competition_zones.yaml` 修订。

---

## 13. 下一位 Agent 应优先完成的任务

1. **继续策略补丁单轮验证**（1bB_4rA、4rA_1bB、2rA_3bB、4bB_1rA…），并分析高墙钟低 gain 的 C 族任务。
2. **先跑现网自检**：一键启动 → 面板提交一题 → 确认“大模型解析成功 · 指令已发布”、双相机有画面、三维地图有物块/区域/小车/障碍、逐轮进度随完成递增；异常先查 `uptime` 与 `ps -eo pcpu --sort=-pcpu | head`。
2. **补齐技术文档首页四项信息**（拿到队伍信息后直接改 DOCX 首页）。
3. **做一次低负载连续验收**（≥20 轮），记录成功轮数与失败分类。
4. **给运行器加看门狗**（§12.1），后续所有验证都会更快。
5. 需要时补大模型播报文本（§11.2）。

---

## 14. 推荐的后续实施顺序

```
现网自检（1 轮） → 技术文档首页补齐 → 低负载连续验收（≥20 轮）
      → 运行器看门狗 → 按审计清单逐条优化（每条一轮验证）
      → 大模型播报文本 → 双机位录屏与作品提交 → readme 环境段修订
```

---

## 15. 开发过程中必须遵守的限制和注意事项

- **不要改场景坐标**（已验收布局：托盘、A/B/C 锚点、障碍中心）；规则允许自行设计位置，但改动需重新跑覆盖验证。
- **不要删除** `base_footprint→ground_plane` 地面关节；场景既有元素不得删减。
- **不要放宽** `GRASP_*` / `CARRY_*` / `TRANSPORT_*` / BodyStop / NearSlow 比例；**禁止**为了让失败轮通过而放松验收门槛（290 s、净空 0.10 m 等）。
- 改 `.py` 后必须 `colcon build`（`mybot` 复制安装；`tools_demo` 亦需重建才能生效）。
- 每次跑分前 `source scripts/00_env.sh`；**跑分必须无 GUI**；GUI 演示用一键手动模式。
- 策略表必须**单题实测**，禁止整表重写；任何降低抓取成功率的改动立即回滚。
- 证据口径：`runs/*/results.jsonl` 是唯一权威；不要用单轮结论替代复跑。
- 桥接白名单是**安全边界**：新增话题要显式加白名单；客户端只允许发布 `/competition/task/problem_chunk`。
- **默认关闭大模型 thinking**（`llm_enable_thinking: false`）：本路径只做候选计划复核，thinking 会把 ~2 s 拖到 ~20 s；未经明确要求不得打开。
- 提交前不要带入 `build/`、`install/`、`log/`、`runs/`、`__pycache__`、`node_modules`。

---

## 16. Git 分支、最新提交与远程仓库

- 工作区：`<HOME>/huxuyang/YZBot-224/workspace/src/yzbot`
- 分支：**`huxuyang`**（跟踪 `origin/huxuyang`）。**本仓库没有 `aerospaceboy` 分支**——那是旧仓库 `aerospaceboy/mybot` 的名字。
- 远程：`origin` = `https://github.com/cty050422/yzbot-224.git`
- 推送目标：**`origin/huxuyang`**，普通 `git push`，**禁止 force**。
- 本轮之前最新提交：`9fe19e1 docs: record the handoff commit id`
- 本轮交接提交：`chore: update project progress and handoff`（提交 ID 以 `git log -1` 为准）

---

## 17. 给下一位 Agent 的接手说明

1. 从 `<HOME>/huxuyang/YZBot-224` 开始，先 `source scripts/00_env.sh`；再读本文件 §7（已验证）、§9（规则对照）、§10（风险）、§15（限制）。
2. **第一件事是自检**（§13.1）：一键启动 → 面板提交一题 → 依次确认解析状态、双相机、三维标记、逐轮进度。
3. coStudio 侧关注三处：桥接白名单（相机随 `camera_streams`）、`/competition/task/parse_result`（解析判定）、`/competition/scene_markers`（三维标记）；细节见 `docs/costudio/`。
4. 改面板显示要同时改 `tools_demo/tools_demo/legacy_costudio_status.py` 与 `tools_demo/costudio_extension/yzbot-competition/src/`，构建后覆盖已安装扩展并重启 coStudio。
5. 判断“某任务该不该改”的流水线：`leg_slack_report.py` → `leg_cost_table.py --optimize` → `audit_policy.py` → 改**一个**键 → 跑一轮 → `full48_report.py` 复核 → `verify48_policy_match.py` 核对证据归属。
6. 长跑前先 `CLEAN_ONLY=True bash tools_demo/scripts/clean_launch.sh`；跑完也清理；`/competition/mission/state` 是 transient-local，重启后不要把旧状态当当前任务。
7. 提交信息沿用简短祈使句；本轮为 `chore: update project progress and handoff`。


## 2026-09-21 部署补充（本节覆盖前文过期的连接操作）

VM 当前使用 huxuyang；入口为 tools_demo/scripts/start_manual_problem_mode.sh，已去除 home/scripts 环境依赖。
Windows 扩展使用仓库配套 1.4.4；1.5.0 单面板版本与此布局不兼容。
推荐导入 tools_demo/config/costudio_deployment_layout.json，题目面板“确定并执行”是实际入口，旧 mission/start 无后端订阅，已从部署布局移除。
未解析目标区显示等待；小面板可滚动到题目输入顶部。ROS 555 passed, 1 skipped，连接与启动已验证，未重跑完整比赛。
详见 docs/validation/DEPLOYMENT_20260921/REPORT.md 与 docs/costudio/README.md。历史证据仍保留；清理的旧副本已归档主机。
