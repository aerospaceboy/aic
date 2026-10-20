# 工作日志（2026-10-05 第二轮接手）

- 11:13 UTC 接手。核对 master / HEAD=ab09ac3=origin/master，工作树含用户未提交的 GUI 刷新与支撑轮 TF 改动（保留），
  旧 stash 保留，无 gzserver 残留（未抢占他人仿真）。世界哈希与交接一致，未改 world/地图/allowlist。
- 11:15–11:35 只读证据挖掘（两个后台子代理并行，未改仓库源码、未 kill 进程）：
  - `hour-repair-20261005-final2-1rA4bB/20261005T104007Z-GN15L4` → 表1 实体台账 + 表2 19 条导航腿；
  - `hour-repair-20261005-final2-3rA2bB/20261005T103122Z-mkRG5J` → 同两张表（该 run bag 在 t≈195.6 s 因 database is locked 中断）。
  - 结论文件：`scripts/takeover/evidence_mining_1rA4bB.md`、`evidence_mining_3rA2bB.md`。
- 定位两处卡停同源：机器人沿正确车道摆动 0.31 m → 自身格进入墙的膨胀带（cost 71）→ footprint 覆盖致死格 →
  RPP 零速 + 无意义重试；`_lethal_start_cost()` 只拦 253/254 拦不住 71。参考 run 该腿 103.43 s（含 28.9 s 静止）。
- 用 mapn3.pgm + 全局代价图实测：NavFn 自选走廊车道 y≈-2.98，距致死行 y=-3.58 仅 0.60 m → 车道本身余量不足。
- 第一个候选（单锚点 (9.20,-2.85)）实施后自评发现该锚点几乎与被卡车道重合，无法提供余量；已在 11:25 停止该候选的
  在跑仿真（记为 INTERRUPTED，见 `verify-20261005-1rA4bB/*/INTERRUPTED.md`），改为两个锚点方案，重新构建后重跑。
- 落地修复：`bay_pocket_waypoints()` 两个走廊锚点 (12.40,-2.50)/(4.40,-2.50)，仅作用于「B bay 内起点 → blue4 口袋目标」；
  `bay_pocket_*` 航点用 0.08 m 紧交接；新增 3 个测试。未改 world/地图/zones/mission/allowlist/障碍/安全阈值。
- 定向运行（600 s 上限，headless + moving_obstacle + bag）：
  `/home/ros/dev_ws/experiments/verify-20261005b-1rA4bB/20261005T112835Z-iKHWNf`
  → **CASE_ACCEPTED=true（exit 0），5/5、skipped=0、order_verified、truth_precise，运动墙钟 359.1 s**；
  蓝4取件腿 25.44 s（原 103.43 s），该腿 retry/patience/bounded/lethal-start 全为 0；
  最小真值净空 0.1142 m > 0.10，四个移动体全部 yield，original_pair/two_dynamic 均 true，无碰撞。
- 构建与测试：colcon build tools_demo 通过；行为回归 685 passed（+3）；flake8 新增代码 0 新增问题
  （两个既有 E302/F841 在 HEAD 复核同样存在）。
- 未做：48 题矩阵、四蓝→C、3rA2bB 复跑、真实 API/CoStudio、完整 300 s 墙钟、附着后服务不响应定位。
- 提交：见 COMMIT.txt（仅按清单提交 3 个 tools_demo 文件 + `docs/validation/BAY_POCKET_20261005/`；用户未提交文件未纳入）。

## 推迟的候选修复（本轮已定位但未实施，留给下一轮）

停滞（无位移）**且自身格 cost≥50（膨胀带）**时，`_retry_nav_goal()` 仍会从同一格重发同一目标，必然无效：
两处卡停各自因此多耗约 10–18 s。候选改法：在该函数里把「起点击穿膨胀带」与 253/254 一并视为不可重试，
并在 `result_callback` 的 `progress_recovery_pending` 分支补一次 `_try_bounded_recovery()`（现有有界恢复，已验证安全），
不新增无限重试、不放宽任何安全阈值。上线前需用 3rA2bB 复跑验证。

- 第二题确认（同版本、600 s 上限）：verify-20261005b-3rA2bB/20261005T114046Z-gIr7jc →
  CASE_ACCEPTED=true（exit 0）、5/5、skipped=0、运动墙钟 516.9 s、最小净空 0.2232 m、无碰撞；
  red3→A 腿 37.26 s（参考 run 86.49 s，本轮未复现卡停，0 retry / 0 bounded / 0 lethal-start）；
  蓝4取件腿 26.85 s。该 run 记录的 commit.txt 已是 4e04c0a，worktree 只剩用户未提交的两个文件。
- 明确口径：red3→A 未复现 ≠ 已修（该腿代码路径未改，卡停为间歇性），需多轮重复才能下结论。
- 本轮两次运行均 >300 s（359.1 / 516.9），300 s 目标未达；48 题矩阵未跑。
