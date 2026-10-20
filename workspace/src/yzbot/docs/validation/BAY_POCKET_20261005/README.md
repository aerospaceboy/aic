# 蓝4口袋航线修复与 1rA4bB 安全 5/5（2026-10-05 第二轮接手）

**一句话结论**：按证据把「B bay → blue4 口袋」这条腿的走廊车道从 NavFn 自选的 y=-2.98 钉到 y=-2.50 后，
`Move 1 red cube to zone A, then move 4 blue cubes to zone B.` 首次取得 **安全 5/5（CASE_ACCEPTED=true，exit 0）**，
但**运动墙钟 359.1 s，未达 300 s**，且这是**单次通过**，不是稳定结论。48 题矩阵、四蓝→C、真实 API/CoStudio 本轮**未测**。

## 1. 环境与基线事实（已核对）

- 主机 Windows，工作区 `D:/user/Desktop/AIC`；VM 为 Ubuntu 22.04 / ROS 2 Humble，`ros@192.168.153.128`。
- 仓库 `/home/ros/dev_ws/src/yzbot`，分支 `master`，`HEAD=ab09ac3669c3ea91f3f0558d75a224b75b471d1f` = `origin/master` = 交接基线。
- 接手时**无 gzserver / 无他人的仿真**（pgrep 为空），不存在抢占他人运行的问题。
- 用户未提交改动**原样保留**、未提交：`bot_navigation/launch/competition_navigation.launch.py`（支撑轮静态 TF，+35 行）与
  `gazebo_debug_gui/src/gazebo_debug_gui_plugin.cpp`（+1/-1）；`stash@{0} preserve-preexisting-gui-and-wheel-tf` 保留。
- 本轮**未改动** world（sha256 `51f23aa6…fdc5d4` 未变）、静态地图、`competition_zones.yaml`、`mission.yaml`、allowlist、
  障碍轨迹、安全链与验收阈值。
- 主机原有 `scripts/Invoke-VM.ps1` 在本机 plink 0.74 下引号会被吃掉（`base64.b64decode` 语法错误），
  本轮改用 `scripts/takeover/vmrun.sh`（单文件）与 `vmrun2.sh`（目录 tar.gz 上传）经 `plink -m` 执行，凭据仍取本机既有配置。

## 2. 已确认根因（两处卡停同源，均为事实级证据）

| 项 | 1rA4bB / 蓝4取件腿 | 3rA2bB / red3→A 腿 |
|---|---|---|
| 停滞点 | (9.07,-3.29)，静止 28.9 s | (-8.43,-2.37)，约 27.6 s |
| 自身格代价 | `nav_costmap_probe start_cost=71`（膨胀，非致死） | 同 71 |
| 全局代价图 | y=-2.99 行全 0；y=-3.29 行 99/71/99；**y=-3.58 行致死** | 静态块南界约 y≈-2.2，机器人落在其外缘 0.17–0.20 m |
| 规划车道 | NavFn 自选 y≈-2.98（距致死行仅 0.60 m） | 路径经 (-8.68,-2.72)，机器人偏出约 0.4 m |
| 控制器 | RPP `detected collision ahead`×244、`Controller patience exceeded`×6 | ×214、×4 |
| 规划器 | `Starting point in lethal space`×2 | ×2 |
| 无效重试 | `nav_retry` 2 次（从同一格重发同一目标） | 2 次 + 1 次 bounded recovery |
| 脱困 | bounded recovery 倒车 0.5 m → (9.18,-2.80) → 重规划成功 | 倒车 0.5 m → (-8.55,-2.84) → 成功 |
| 动态障碍 | 卡停点 2.5 m 内无任何命名移动体 | 4.8 m 内无 |
| 速度链 | `/cmd_vel_nav`=`/selected`=`/cmd_vel_safe`=0（RPP 主动零速，非安全层截断） | 同 |

关键补充（读 bag 实测）：机器人**不是**沿错误路径走，而是**沿正确车道摆动**——走廊段 `/cmd_vel_nav` 在 1.65 m/s 下
0.4 s 内从 +1.83 rad/s 甩到 -1.50 rad/s，横向偏差峰值 0.31 m；正是这 0.31 m 把机器人推进墙的膨胀带，
使其自身格 cost=71、footprint 覆盖致死格，RPP 随即锁死零速；而 `_lethal_start_cost()` 只拦 253/254，
拦不住 71，于是两次重试都在同一格原地重发，直到有界逃逸才解开。
`hazard=recent_trail` 只是 `recovery_label` 的默认值（该点不在已配置风险区内），**不代表识别到轨迹假障碍**。

## 3. 已落地修复（小改，一个主变量）

仅一处功能性改动 + 相应测试，全部在 `tools_demo`：

- `tools_demo/tools_demo/navigation_math.py`：新增 `bay_pocket_waypoints(start, target)`。仅当
  **起点在 B bay 内**（x≥8.0 且 -7.6≤y≤-3.6）**且目标在 blue4 口袋内**（3.4≤x≤6.2 且 y≤-4.5）时，
  返回两个走廊锚点 `bay_pocket_gate(12.40,-2.50)` 与 `bay_pocket_corridor(4.40,-2.50)`；
  把该腿的走廊车道钉在 y=-2.50（距墙致死行 1.08 m > 实测摆动 0.31 m + 车体半径 0.28 m）。
  其他任何取件/投放腿都不受影响。
- `tools_demo/tools_demo/task_navigator.py`：非 legacy 路径下，`dynamic_gate_waypoints` 为空时改用该锚点；
  `bay_pocket_*` 航点沿用 0.08 m 紧交接距离（与既有 `universal_*`/`A_SWEEP_*` 同一先例，避免提前交接导致斜切）。
- `tools_demo/test/test_navigation_math.py`：新增 3 个测试（锚点坐标与净空断言、其他腿返回空、非有限输入报错）。

## 4. 已验证效果（单次，600 s 诊断上限，headless + moving_obstacle + bag）

证据目录：`/home/ros/dev_ws/experiments/verify-20261005b-1rA4bB/20261005T112835Z-iKHWNf`
（`result.log`、`launch.log`、`bag/`、`audit.json`、`worktree.patch`、`source-config.sha256`）

| 指标 | 本轮 | 参考（最终修复前 final2-1rA4bB） |
|---|---|---|
| CASE_ACCEPTED | **true**（exit 0） | false（420 s 截断） |
| 完成/跳过 | **5/5，skipped=0** | 3/5，skipped=0 |
| 颜色顺序/真值 | order_verified=true、truth_precise=true、5 条 delivery_truth 全 valid+precise | 3 条 |
| 蓝4取件腿耗时 | **25.44 s** | **103.43 s**（含 28.9 s 静止） |
| 该腿 nav_retry / patience / bounded recovery / lethal-start | **0 / 0 / 0 / 0** | 2 / 6 / 1 / 2 |
| 运动墙钟 | 359.1 s（`motion_under_300=false`） | 420 s 截断 |
| 最小真值净空 | 0.1142 m（>0.10） | 0.269 m |
| 四移动体响应 | 全部 yield；original_pair=true、two_dynamic_responses=true、collision_detected=false | 原始双响应 true |

本轮 19 条导航腿合计 288.7 s，13 次 manip；最贵腿为 B 投放腿 67.33 s（自 (-11.38,2.30)）与 36.66 s（自 (-1.14,-5.99)）。

### 第二题确认（同版本，600 s 上限）

证据：`/home/ros/dev_ws/experiments/verify-20261005b-3rA2bB/20261005T114046Z-gIr7jc`

| 指标 | 本轮 | 参考（final2-3rA2bB） |
|---|---|---|
| CASE_ACCEPTED | **true**（exit 0） | false（420 s 截断） |
| 完成/跳过 | **5/5，skipped=0**（red1/red3/red2→A、blue1/blue4→B） | 4/5 |
| 运动墙钟 | **516.9 s**（`motion_under_300=false`） | 420 s 截断 |
| red3→A 腿 | **37.26 s**，0 retry / 0 bounded / 0 lethal-start | **86.49 s**（2 retry + 1 bounded + 2 lethal-space） |
| 蓝4取件腿（该腿即本修复作用对象） | **26.85 s** | 35.34 s（当次未卡停） |
| 最小真值净空 | 0.2232 m | 0.3846 m |
| 移动体响应 | 3 个有响应（obstacle_1/2 + partition_1）、original_pair=true、collision=false | 原始双响应 true |

⚠ 口径：**red3→A 本轮没有复现卡停，不代表该缺陷已修**——本轮修复只作用于「B bay → blue4 口袋」这条腿，
A 腿的规划/控制路径完全未变；参考 run 与本周期的差异说明该卡停是**间歇性**的（由 ±0.31 m 摆动是否踩进膨胀带决定），
根因（走廊摆动）仍在。要判断它是否稳定消失，需要多轮重复。

## 5. 测试与构建

- `colcon build --symlink-install --packages-select tools_demo`：通过。
- 行为回归：**685 passed**（上一基线 682 + 新增 3），命令与交接一致（排除 3 个模板质量检查）。
- flake8（99 列）：新增代码**无新增问题**；`navigation_math.py:348 E302`、`task_navigator.py:3414/3415 F841`
  在 HEAD 版本同样存在（行号随改动偏移），属既有问题，未顺手修改。

## 6. 未完成 / 风险（不得当作已解决）

1. **单次通过，不是稳定通过**；300 s 未达（359.1 s），48 题矩阵未跑，四蓝→C、真实 API/CoStudio/完整墙钟未测。
2. 最小净空 0.1142 m 只比阈值高 0.0142 m，绑定项是 `preview_moving_partition_1`（(-6.093,1.9) 附近）。
3. `3rA2bB / red3→A` 的同源卡停本轮**未复现**（该腿 37.26 s，0 retry），但该腿的代码路径未被改动，
   卡停是间歇性的 → **不得**据此宣布该缺陷已修；需要多轮重复验证。
4. 走廊里 RPP 的 ±0.31 m 摆动**本身未修**：本次只是把车道钉离危险带。改速度/改布局后必须重算该锚点。
5. 抓取附着后 Gazebo 服务/夹爪间歇不响应：本轮未复现、未定位。
6. `3rA2bB` 的 bag 目录缺 `metadata.yaml`，rosbag2 不能按目录打开（只能用 sqlite 直读）。

## 7. 下一步（建议顺序）

1. 用本版本复跑 `3rA2bB`（600 s）确认 red3→A 不再出现同源卡停；再跑一道原本已通过的题做回归。
2. 若能复现「停滞且自身格 cost≥50」的模式，再评估把 `_retry_nav_goal()` 的守卫从 253/254 扩到「膨胀带 + 已停滞」
   （推迟的候选修复，见 worklog：停滞时从同一膨胀格重发同一目标必然无效，直接走已证明安全的有界恢复即可省 ~18 s/次）。
3. 300 s 优化先削最大腿（B 腿 67.3 s、36.7 s）与 manip 间隔，最后才动速度/加速度。
4. 48 题用**新 campaign 根目录**后台串行，按阶段读摘要。
