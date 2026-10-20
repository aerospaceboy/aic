> 历史中间记录，非当前运行指令。最新规则与48题复核见 [UNORDERED_290_20261006](../UNORDERED_290_20261006/README.md)；接手使用该目录的 HANDOFF_PROMPT.md。

# 下一位 Agent 提示词（DISTRIBUTED_LAYOUT_20261006 交接）

你没有此前聊天上下文。本文件是完整交接。**不要假设**连接方式、仓库位置或"已通过"的含义。

## 0. 三句话状态

1. 五个资源已按候选方案重新分散并全链同步（提交 `1718eaa640354d60521a47b94a74e81104885a30`，master 已推送）。world/地图/配置/默认值/界面/合同测试一致；
   `705 passed, 3 deselected`；静止地图差异只有 360 个托盘像素。
2. **反序 2bB_3rA 在最终构建上全门通过**（5/5、零跳过、顺序与真值 precise、双原始响应、净空 0.5287 m、257.8 s、
   `CASE_ACCEPTED=true`、`motion_functional_pass=true`、`motion_under_300=true`）。正序 3rA_2bB 在**仅布局**构建上同样全门通过（224.5 s），
   但在最终构建复跑时失败。
3. **48 题矩阵尚未运行**，red4/blue4/blue1-east 未做定向覆盖，GUI/API/CoStudio/完整墙钟未测。任务未完成，继续推进。

## 1. 环境与入口

- 主机 Windows，工作区 `D:/user/Desktop/AIC`；VM `ros@192.168.153.128`（Ubuntu 22.04 / ROS 2 Humble）。
- 仓库 `/home/ros/dev_ws/src/yzbot`（master）；本轮交付目录 `D:/user/Desktop/AIC/distributed-20261006/`
  （README/worklog/case-results.csv/scripts；`scripts/vmrun.sh` 单文件、`vmrun2.sh` 目录上传，均用 plink -m）。
- VM 运行脚本：`/home/ros/distributed-20261006/run-case-bag.sh '<question>' <root>`（headless + 移动障碍 + bag + evidence_audit；
  `RLC_TIMEOUT_SEC=600`）。48 题：`tools_demo/scripts/run48_v2.sh 1 48 ordered <root>` + `summarize48_v2.py`。
- 备份：`/home/ros/distributed-20261006-backup/`、本地 ref `backup/distributed-20261006`。
- **必须保留**用户未提交的 `competition_navigation.launch.py`（支撑轮 TF）与 `gazebo_debug_gui_plugin.cpp`，以及旧 stash；不要 reset --hard。

## 2. 已落地（提交 1718eaa）

- 布局：red1(-7.5,-2.8)、red2(1.0,-5.4)、red4(11.4,-9.8)、blue1(7.4,3.6)、blue4(-0.5,-8.8)（cube+tray 同移）；
  最小 cube 间距 3.059 m、最小 cube→区域 4.809 m；区域/墙/岩石/出生点/四个移动体未动。
- 同步链：`config/complex_layout.json` → `tools_demo/scripts/activate_complex_layout.py` →
  zones/mission/两个 py 默认表/final_mission_layout/地图/风险区；另手工改 `test_mission_contract.py`、`test_navigation_contract.py` 的硬编码坐标。
- 修复：① velocity_smoother 放开有界横向通道（纠偏实测生效）；② 穿越计时"起跑后计时"+12 s 等待上界，并与限速器共享
  `safe_crossing_phase`；③ hold 触发 1.30→1.05、standoff 1.05→0.85、entry lateral 1.20→1.45；④ 交接增加 0.45 m/s 速度门 + 2 s 宽限。

## 3. 当前最关键的未解决故障（有证据，务必先处理）

**东门（安全限速器 east gate）可能把机器人压住远超 12 s，导致腿失败。**

- 证据：r12（正序，最终构建）`experiments/distributed-layout-r12-forward/*/launch.log`：
  `mover_crossing_hold begin x=9.812` → 7.8 s 后 admitted → 12.05 s 后 `nav_result state=failed`（我的等待上界到期），
  而安全限速器直到 **admitted 之后 18.5 s** 才打印 `east mover gate released on southbound phase`。
- 机制：`safety_limiter.east_gate_blocked` 只在**观测到** mover_1 南向下行（`safe_crossing_phase`：当前 y ≤ -0.65 且 prev-current ≥ 0.01）时放行，
  且机器人停在 x∈[9.3,9.75] 带内；巡逻周期 8 s，正常应在一个周期内出现该相位。等 18.5 s 说明
  **mover 真值话题可能短时陈旧**（`actor_fresh` 为假时永远不会满足相位），或机器人在带内反复重锁。
- 相邻证据：r3/r5/r10 也是同类早退（0 完成、28–43 s）；这些运行都保留了 bag 与 launch.log。

**建议修复方向（按优先级）**
1. 先查真值陈旧：对 r12 的 bag 检查 `/competition/obstacles/obstacle_1` 的发布间隔与 `success` 字段，
   以及 `moving_obstacle` 的 service 调用失败日志（历史上有 `Moving obstacle service call failed`）。
2. 把"等待上界到期"从**腿失败**改成**可恢复**：回到 holding 状态并重置一次有界预算（最多 2 次），
   或让上界由"真值新鲜度"决定（真值陈旧时不计入穿越预算）。任何改动都必须保持有限、不得放宽安全门。
3. 若确认是展示航点丢失导致响应数不足，再处理"重试后接回未完成展示段"（按 task_id/generation 保存）。

## 4. 本轮其它已知问题

- 反序双响应**相位敏感**：r4/r6/r7/r8 只记到 1 个原始响应（r11 才两个）。根因线索：展示路线在 x=5.0 车道上
  曾以 1.65 m/s 通过（bag 实测），mover 的 `response_speed` 0.80/0.85 未触发；hold 触发受"当前航点是否在车道另一侧"过滤。
- GUI 可视演示未复跑：交接减速修复（0.45 m/s 门）尚未在 GUI 负载下验证；上一轮 GUI 失败证据在
  `sprint300-20261005/visible-demo-20261006/`。
- `bay_pocket_waypoints` 现在不再命中任何新布局目标（blue4 已不在那个口袋），属死代码，可保留但不要据此加航点。
- 既有 flake8 债：`task_navigator.py` 两处 F841、若干长行；本轮新增代码无新增诊断。

## 5. 下一步顺序（建议）

1. 修 §3 的东门/等待问题，并保证正序+反序各连续两次全门通过（保留失败证据）。
2. 定向覆盖：4 红题验证 red4、4 蓝→C 题验证 blue4、blue1 的 east 侧；每例保留 bag。
3. 冻结版本（新提交）→ 后台串行 48 题（新 root）→ `summarize48_v2.py` 汇总，
   逐题判定必须同时满足 CASE_ACCEPTED=true、motion_functional_pass=true、motion_under_300=true。
4. 运动收敛后再做真实 API/CoStudio 与完整墙钟；不要把运动时间当完整比赛时间。
5. 更新 `PROJECT_HANDOFF.md`、`AGENT_HISTORY.md`、`docs/validation/DISTRIBUTED_LAYOUT_20261006/`。
