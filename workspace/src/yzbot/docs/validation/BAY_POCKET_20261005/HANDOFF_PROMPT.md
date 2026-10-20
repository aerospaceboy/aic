# 下一位 Agent 提示词（2026-10-05 第二轮接手交接）

你没有此前聊天上下文。本文件是完整交接。**不要假设**连接方式、仓库位置、成绩对应哪一版布局；下面给出已核实的事实与可复现命令。
上一轮提示词在 `hour-repair-20261005/下一位Agent提示词.md`（本轮之前的基线）；本文件只覆盖"在那之后发生了什么"。

## 0. 三句话状态

1. 已修一条确定性航线缺陷并**首次拿到该题安全 5/5**：`Move 1 red cube to zone A, then move 4 blue cubes to zone B.`
   在 600 s 诊断上限内 `CASE_ACCEPTED=true`、5/5、零跳过、零碰撞、最小净空 0.1142 m。
2. **仍未达 300 s**（运动墙钟 359.1 s），**只是单次通过**；48 题矩阵、blue→C、真实 API/CoStudio/完整墙钟**未测**。
3. 已推送 master：`4e04c0a7f92099a513ce0b1996e2c177ea6a5a07`（父提交 `ab09ac3`）。用户未提交的 GUI/RViz 与支撑轮 TF 改动**未提交、原样保留**。

## 1. 环境（已核实，别重新猜）

- 主机 Windows；工作区 `D:/user/Desktop/AIC`。VM：Ubuntu 22.04 / ROS 2 Humble，`ros@192.168.153.128`（本机 plink 0.74）。
- 仓库在 VM：`/home/ros/dev_ws/src/yzbot`（**不是**主机 AIC 目录），工作空间 `/home/ros/dev_ws`，分支 `master`。
- 主机 `hour-repair-20261005/scripts/Invoke-VM.ps1` 在本机 plink 下**引号会被吃掉**（会把 `python3 -c "..."` 拆坏）。
  本轮可用：`scripts/takeover/vmrun.sh <local.py> [timeout]`（单文件 base64 上传）与
  `vmrun2.sh <local_dir> [timeout] [entry]`（整目录 tar.gz 上传后跑 `main.py`）。二者都用 `plink -m` 命令文件，
  凭据取本机既有配置，**不要把密码/密钥写进任何交付物**。
- 单题带 bag 的运行脚本：`/home/ros/verify-20261005/run-case-bag.sh '<question>' <root>`，
  用法示例见 `scripts/takeover/payload_run2/main.py`（`RLC_TIMEOUT_SEC=600`，headless + 移动障碍 + bag + evidence_audit）。

## 2. 本轮改了什么（已提交）

提交 `4e04c0a`，仅 6 个文件：

- `tools_demo/tools_demo/navigation_math.py`：新增 `bay_pocket_waypoints()` + 常量。仅当**起点在 B bay 内**
  （x≥8.0 且 -7.6≤y≤-3.6）**且目标在 blue4 口袋内**（3.4≤x≤6.2 且 y≤-4.5）时，返回两个走廊锚点
  `(12.40,-2.50)`、`(4.40,-2.50)`；否则返回空。
- `tools_demo/tools_demo/task_navigator.py`：非 legacy 路径下 `dynamic_gate_waypoints` 为空时改用该锚点；
  `bay_pocket_*` 航点使用 0.08 m 紧交接距离（与既有 `universal_*`/`A_SWEEP_*` 同先例）。
- `tools_demo/test/test_navigation_math.py`：3 个新测试（含"锚点离墙致死行 > 0.31+0.28 m"的净空断言）。
- `docs/validation/BAY_POCKET_20261005/{README.md,worklog.md,case-results.csv}`：报告、日志、逐题结果。

**未改动**：world（sha256 `51f23aa6…fdc5d4`）、静态地图、`competition_zones.yaml`、`tools_demo/config/mission.yaml`、
allowlist（红色仍 east-only）、障碍轨迹/周期、速度仲裁与安全链、验收阈值（0.10 m 与 CASE_ACCEPTED 判据）。

### 为什么改这里（证据，务必继承）

两处卡停**同源**：机器人沿**正确**车道摆动，横向峰值 0.31 m（走廊段 `/cmd_vel_nav` 在 1.65 m/s 下 0.4 s 内
+1.83→-1.50 rad/s），把自己推进墙的膨胀带 → 自身格 cost=71（膨胀、非致死）→ footprint 覆盖致死格 →
RPP 锁死零速（`collision ahead`×244/214，`patience exceeded`×6/4）→ 而 `_lethal_start_cost()` 只拦 253/254，
拦不住 71，于是 2 次 `nav_retry` 在同一格重发同一目标必然无效 → 最终靠 bounded recovery 倒车 0.5 m 脱困。

- 1rA4bB 蓝4取件腿：NavFn 自选车道 y≈-2.98，墙致死行 y=-3.58，余量仅 0.60 m → 蓝4腿 103.43 s（含 28.9 s 静止）。
- 3rA2bB red3→A：同型，卡在 (-8.43,-2.37)，该腿 86.49 s。
- `hazard=recent_trail` **只是 recovery_label 的默认值**（该点不在已配置风险区内），**不代表识别到假障碍**；
  两处卡停 2.5 m / 4.8 m 内都没有任何命名动态障碍物 → 卡停**不是**避障问题。
- 卡停期间 `/cmd_vel_nav`=`/selected`=`/cmd_vel_safe`=0：RPP 主动零速，不是安全层截断。

## 3. 已验证效果（**单次**）

证据：`/home/ros/dev_ws/experiments/verify-20261005b-1rA4bB/20261005T112835Z-iKHWNf`

| 项 | 本轮 | 参考（final2-1rA4bB） |
|---|---|---|
| CASE_ACCEPTED / exit | **true / 0** | false / 1（420 s 截断） |
| 完成 / 跳过 | **5/5 / 0** | 3/5 / 0 |
| 蓝4取件腿 | **25.44 s**，retry/patience/bounded/lethal 全 0 | 103.43 s，2/6/1/2 |
| 运动墙钟 | 359.1 s（`motion_under_300=false`） | — |
| 最小真值净空 | 0.1142 m（partition_1），>0.10 | 0.269 m |
| 四移动体响应 | 全 yield；original_pair / two_dynamic 均 true；collision=false | 原始双响应 true |

构建 `colcon build --symlink-install --packages-select tools_demo` 通过；行为回归 **685 passed**；
flake8 新增代码 0 新问题（`navigation_math.py:348 E302` 与 `task_navigator.py:3414/3415 F841` 在 HEAD 同样存在）。

## 4. 明确未完成 / 风险

1. 单次通过 ≠ 稳定；**300 s 未达**；48 题未跑；四蓝→C、真实 API/CoStudio/完整墙钟未测。
2. 净空 0.1142 m 只比阈值高 0.0142 m（绑定项 partition_1）——任何路线/速度改动都要重新验收。
3. 走廊里 RPP 的 ±0.31 m 摆动**本身没修**，只是把车道钉离危险带；改速度或布局后必须重算锚点。
4. 3rA2bB 的 red3→A 同源卡停**本轮未复测**（见下面 step 1）。
5. 附着后 Gazebo 服务/夹爪间歇不响应未复现、未定位；`3rA2bB` 的 bag 缺 `metadata.yaml`。
6. 用户未提交改动仍在工作树（`git status` 可见），**不要提交它们**，不要 `reset --hard`，旧 stash 保留。

## 5. 建议的下一步（按优先级）

1. **回归**：用当前 HEAD 复跑 `Move 3 red cubes to zone A, then move 2 blue cubes to zone B.`（600 s），
   确认 red3→A 是否仍是 86 s 级卡停（本轮修复**不覆盖**该点），并跑一道原本已通过的题。
2. **候选小修（已定位、本轮未实施）**：停滞且自身格 cost≥50 时 `_retry_nav_goal()` 仍会重发同一目标。
   改法：把"起点击穿膨胀带"与 253/254 并列视为不可重试，并在 `result_callback` 的 `progress_recovery_pending`
   分支补一次 `_try_bounded_recovery()`。预期每次卡停省 10–18 s，不新增无限重试、不放宽安全阈值。改完必须过 3rA2bB。
3. **300 s**：先削最大腿（B 投放腿 67.33 s、36.66 s；13 次 manip 之间的固定间隔），最后才动速度/加速度。
4. **48 题**：用**新 campaign 根目录**后台串行（`RLC_TIMEOUT_SEC=600`），按阶段读摘要；
   600 s 是诊断上限，不是达标线；修复改了版本就必须换新 root，不能把旧成功计入新版本。
5. 真实 API/CoStudio 与完整墙钟单独验收，不能与运动时间混算。

## 6. 交付物位置

- 主机：`D:/user/Desktop/AIC/hour-repair-20261005/takeover-2/`
  （`README.md` 报告、`worklog.md` 日志、`case-results.csv` 本轮结果、`evidence/` 两个 run 的台账与腿表挖掘）。
- 仓库：`docs/validation/BAY_POCKET_20261005/`（同一报吿/日志/结果的 repo 版本）。
- VM：`/home/ros/dev_ws/experiments/verify-20261005b-*/`（本轮完整日志 + bag + audit）、
  `/home/ros/verify-20261005/`（运行脚本、批次日志、COMMIT.txt）、
  `/home/ros/dev_ws/experiments/verify-20261005-1rA4bB/*/INTERRUPTED.md`（被主动中断的候选，失败证据保留）。
